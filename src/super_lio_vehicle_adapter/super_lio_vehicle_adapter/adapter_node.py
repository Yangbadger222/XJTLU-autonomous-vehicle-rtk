"""Adapt Super-LIO's world->IMU output without inventing vehicle geometry.

At the pinned upstream commit Super-LIO publishes an odometry message whose
world frame is ``world`` and whose child is ``imu``.  It is not the vehicle
``odom -> base_footprint`` contract.  Until a timestamped source-frame TF, a
measured IMU->base transform or the audited legacy IMU-origin navigation
convention is selected, the node withholds vehicle odometry. Source health
remains distinct from coordinate validity and is matched by measurement time.
This keeps the original RTK authority stop policy fail-closed.
"""
from __future__ import annotations
import time
import json
import math
import copy

try:
    import rclpy
    from rclpy.duration import Duration
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import TransformStamped
    from std_msgs.msg import String
    from rclpy.node import Node
    from tf2_ros import Buffer, StaticTransformBroadcaster, TransformBroadcaster, TransformException, TransformListener
except ImportError:  # allows static linting on the developer laptop
    rclpy = None
    Duration = Buffer = TransformException = TransformListener = object


def _qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def _qconj(q):
    return (-q[0], -q[1], -q[2], q[3])


def _qrotate(q, v):
    return _qmul(_qmul(q, (v[0], v[1], v[2], 0.0)), _qconj(q))[:3]


def _finite(values):
    import math
    return all(math.isfinite(float(value)) for value in values)


def _parameter_bool(value):
    if isinstance(value, bool):
        return value
    normalized = str(value).strip().lower()
    if normalized in {"1", "true", "yes", "on"}:
        return True
    if normalized in {"0", "false", "no", "off", ""}:
        return False
    raise ValueError(f"invalid boolean parameter: {value!r}")


def _stamp_is_set(stamp) -> bool:
    """Require a real acquisition stamp; zero must never mean latest TF."""
    try:
        sec = int(stamp.sec)
        nanosec = int(stamp.nanosec)
    except (AttributeError, TypeError, ValueError):
        return False
    return sec >= 0 and 0 <= nanosec < 1_000_000_000 and (sec > 0 or nanosec > 0)


REFERENCE_CONTRACT = "corridor_e54c6af_fast_imu_origin_v1"
SOURCE_CERTIFICATE = "fixed_extrinsic_observation_lower_bound_v1"
SOURCE_TWIST_CONVENTION = "imu_body_full_state_v1"


def _navigation_reference_valid(convention, contract_id, translation, quaternion):
    """Identity is a recorded navigation point convention, not a chassis measurement."""
    return (convention == "locked_fast_imu_origin" and contract_id == REFERENCE_CONTRACT and
            tuple(translation) == (0., 0., 0.) and tuple(quaternion) == (0., 0., 0., 1.))


def _source_certificate(text):
    """Validate the bound's identity/units and return a measurement-stamped decision."""
    try:
        if len(text) > 4096:
            return None
        data = json.loads(text)
        if (not isinstance(data, dict) or data.get("source") != "super_lio/f89f48dc" or
                data.get("certificate") != SOURCE_CERTIFICATE or
                data.get("twist_convention") != SOURCE_TWIST_CONVENTION):
            return None
        stamp = data["stamp_ns"]
        observed = float(data["minimum_observation_information"])
        bound = float(data["legacy_min_eig_lower_bound"])
        if type(data["iterations"]) is not int or type(data["effective_points"]) is not int:
            return None
        if type(stamp) is not int or stamp <= 0 or not _finite((observed, bound)) or observed < 0 or bound < 0:
            return None
        if abs(bound-min(observed, 100000.)) > 1e-6*max(1., bound):
            return None
        eligible = (data.get("status") == "OK" and int(data["iterations"]) >= 1 and
                    int(data["effective_points"]) >= 50 and bound >= 75.)
        return {"stamp_ns": stamp, "eligible": eligible, "reason": str(data.get("reason", "unknown"))}
    except (ValueError, TypeError, KeyError, OverflowError, RecursionError):
        return None


def _normalize_quaternion(q):
    import math
    if len(q) != 4 or not _finite(q):
        return None
    norm = math.sqrt(sum(float(value) * float(value) for value in q))
    if not math.isfinite(norm) or norm <= 1e-12:
        return None
    return tuple(float(value) / norm for value in q)


def _rotate_covariance(covariance, quaternion, angular_quaternion=None):
    """Rotate a 6x6 pose covariance by a source->target quaternion.

    Odometry pose covariance is ordered as xyz/rpy. The same 3x3 rotation is
    applied to both blocks; this keeps the covariance contract honest when a
    stamped ``world -> odom`` transform has a non-zero yaw. The optional
    second rotation handles the pinned source's mixed twist convention:
    linear velocity in world, angular velocity in IMU coordinates.
    """
    import math
    if len(covariance) != 36:
        return None
    q = _normalize_quaternion(quaternion)
    if q is None or not _finite(covariance):
        return None
    x, y, z, w = q
    rotation = (
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
        (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
        (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
    )
    angular_q = _normalize_quaternion(angular_quaternion) if angular_quaternion is not None else q
    if angular_q is None:
        return None
    columns = [_qrotate(angular_q, axis) for axis in
               ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))]
    angular_rotation = tuple(tuple(columns[col][row] for col in range(3)) for row in range(3))
    rotations = (rotation, angular_rotation)
    matrix = [list(float(value) for value in covariance[row * 6:(row + 1) * 6])
              for row in range(6)]
    output = [[0.0] * 6 for _ in range(6)]
    for block_row in (0, 1):
        for block_col in (0, 1):
            for row in range(3):
                for col in range(3):
                    output[3 * block_row + row][3 * block_col + col] = sum(
                        rotations[block_row][row][i] * matrix[3 * block_row + i][3 * block_col + j] *
                        rotations[block_col][col][j]
                        for i in range(3) for j in range(3))
    return tuple(value for row in output for value in row)


def _source_twist_to_base(q_imu_base, linear_imu, angular_imu, translation_imu_base=(0.,0.,0.)):
    """v_B=R_IB^T(v_I + omega_I cross r_IB).

    The patched native source supplies both twist blocks in IMU coordinates
    and includes estimated attitude uncertainty in its full-state covariance.
    """
    r=translation_imu_base;w=angular_imu
    cross=(w[1]*r[2]-w[2]*r[1],w[2]*r[0]-w[0]*r[2],w[0]*r[1]-w[1]*r[0])
    return (_qrotate(_qconj(q_imu_base),tuple(linear_imu[i]+cross[i] for i in range(3))),
            _qrotate(_qconj(q_imu_base),angular_imu))


def _transform_covariance(covariance, jacobian):
    if len(covariance)!=36 or not _finite(covariance):return None
    return tuple(sum(jacobian[i][k]*covariance[6*k+l]*jacobian[j][l]
                     for k in range(6) for l in range(6)) for i in range(6) for j in range(6))


def _lever_covariance(covariance,q_world_imu,q_imu_base,translation,*,pose=False,q_target_world=(0.,0.,0.,1.)):
    # First-order fixed-axis pose errors and native IMU/body twist convention
    # are pinned to ROSWrapper.cpp. Every cross block is retained.
    columns=[]
    for axis in range(6):
        v=[0.,0.,0.];w=[0.,0.,0.]
        (v if axis<3 else w)[axis%3]=1.
        if pose:
            lever=_qrotate(q_world_imu,translation)
            cross=(w[1]*lever[2]-w[2]*lever[1],w[2]*lever[0]-w[0]*lever[2],w[0]*lever[1]-w[1]*lever[0])
            dv=_qrotate(q_target_world,tuple(v[i]+cross[i] for i in range(3)))
            dw=_qrotate(q_target_world,w)
        else:dv,dw=_source_twist_to_base(q_imu_base,v,w,translation)
        columns.append(dv+dw)
    jacobian=[[columns[j][i] for j in range(6)] for i in range(6)]
    return _transform_covariance(covariance,jacobian)


def _covariance_is_known(covariance):
    """Reject ROS unknown-covariance sentinels and malformed matrices."""
    if len(covariance) != 72 or not _finite(covariance):
        return False
    for offset in (0, 36):
        matrix=[[float(covariance[offset+6*r+c]) for c in range(6)] for r in range(6)]
        scale=max(1.,max(abs(value) for row in matrix for value in row))
        tolerance=1e-10*scale
        if sum(matrix[i][i] for i in range(6))<=tolerance:return False
        if any(abs(matrix[r][c]-matrix[c][r])>tolerance for r in range(6) for c in range(6)):return False
        # Pivot-free LDL factorization handles PSD zero eigenvalues. A zero
        # pivot requires its residual column to be zero; otherwise indefinite.
        lower=[[0.]*6 for _ in range(6)];diagonal=[0.]*6
        for i in range(6):
            diagonal[i]=matrix[i][i]-sum(lower[i][k]**2*diagonal[k] for k in range(i))
            if diagonal[i]<-tolerance:return False
            lower[i][i]=1.
            for j in range(i+1,6):
                residual=matrix[j][i]-sum(lower[j][k]*lower[i][k]*diagonal[k] for k in range(i))
                if abs(diagonal[i])<=tolerance:
                    if abs(residual)>tolerance:return False
                else:lower[j][i]=residual/diagonal[i]
    return True



class SuperLioVehicleAdapter(Node if rclpy else object):
    def __init__(self):
        super().__init__("super_lio_vehicle_adapter")
        self.declare_parameter("input_topic", "/lio/odom")
        self.declare_parameter("vehicle_odom_topic", "/lio/odom_vehicle")
        self.declare_parameter("source_health_topic", "/lio/health")
        self.declare_parameter("health_topic", "/lio/vehicle_health")
        self.declare_parameter("source_frame", "world")
        self.declare_parameter("world_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("source_child_frame", "imu")
        self.declare_parameter("tf_timeout_s", 0.05)
        self.declare_parameter("imu_to_base_extrinsic_verified", False)
        self.declare_parameter("imu_to_base_translation_m", [0.0, 0.0, 0.0])
        self.declare_parameter("imu_to_base_quaternion_xyzw", [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter("require_source_health_ok", True)
        self.declare_parameter("require_covariance", True)
        self.declare_parameter("navigation_reference_convention", "measured_rigid_body")
        self.declare_parameter("navigation_reference_contract", "")
        self.declare_parameter("world_gauge_mode", "external_stamped_tf")
        self.declare_parameter("publish_unhealthy_odometry", False)
        self._verified = _parameter_bool(
            self.get_parameter("imu_to_base_extrinsic_verified").value)
        self._source_frame = str(self.get_parameter("source_frame").value)
        self._target_frame = str(self.get_parameter("world_frame").value)
        self._source_child_frame = str(self.get_parameter("source_child_frame").value)
        self._tf_timeout_s = max(0.0, float(self.get_parameter("tf_timeout_s").value))
        self._translation = tuple(float(x) for x in self.get_parameter("imu_to_base_translation_m").value)
        self._rotation = tuple(float(x) for x in self.get_parameter("imu_to_base_quaternion_xyzw").value)
        self._legacy_reference = _navigation_reference_valid(
            str(self.get_parameter("navigation_reference_convention").value),
            str(self.get_parameter("navigation_reference_contract").value), self._translation, self._rotation)
        if str(self.get_parameter("navigation_reference_convention").value) == "locked_fast_imu_origin" and not self._legacy_reference:
            raise ValueError("locked navigation reference contract/identity override rejected")
        self._own_gauge = str(self.get_parameter("world_gauge_mode").value) == "source_local_world"
        if self._own_gauge and (not self._legacy_reference or self._source_frame != "world" or self._target_frame != "odom"):
            raise ValueError("source local-world gauge requires the audited world/odom legacy reference")
        self._publish_unhealthy = _parameter_bool(self.get_parameter("publish_unhealthy_odometry").value)
        self._source_certificate_stamp = None
        self._pending_odom = None
        self._last_source_stamp = None
        self._last_source_pose = None
        self._reference_fault = ""
        self._require_source_health_ok = _parameter_bool(
            self.get_parameter("require_source_health_ok").value)
        self._require_covariance = _parameter_bool(
            self.get_parameter("require_covariance").value)
        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)
        self._vehicle_tf = TransformBroadcaster(self)
        if self._own_gauge:
            # odom is this estimator session's local world gauge. This is an
            # owned, declared rigid transform; map->odom remains RTK-owned.
            self._gauge_tf = StaticTransformBroadcaster(self)
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = "odom"
            transform.child_frame_id = "world"
            transform.transform.rotation.w = 1.
            self._gauge_tf.sendTransform(transform)
        self._source_health_ok = False
        self._source_health_received=0.
        self._health = self.create_publisher(String, str(self.get_parameter("health_topic").value), 10)
        self._odom = self.create_publisher(Odometry, str(self.get_parameter("vehicle_odom_topic").value), 10)
        self.create_subscription(String, str(self.get_parameter("source_health_topic").value), self._health_callback, 10)
        self.create_subscription(Odometry, str(self.get_parameter("input_topic").value), self._callback, 10)
        self._publish_health("UNKNOWN: Super-LIO source health/IMU-to-base equivalence is not verified")

    def _publish_health(self, text: str):
        self._health.publish(String(data=text))

    def _health_callback(self, msg):
        # Native observation/covariance qualification is independent of the
        # navigation point convention or a measured rigid-body extrinsic.
        certificate = _source_certificate(str(msg.data))
        self._source_certificate_stamp = certificate["stamp_ns"] if certificate else None
        self._source_health_ok = bool(certificate and certificate["eligible"])
        if not self._source_health_ok:
            self._reject("source observation certificate not eligible")
        self._source_health_received=time.monotonic()
        pending = self._pending_odom
        if pending is not None and self._source_certificate_stamp == pending.header.stamp.sec*1_000_000_000+pending.header.stamp.nanosec:
            self._pending_odom = None
            self._callback(pending)

    def _reject(self, reason: str):
        self._publish_health("UNKNOWN: " + reason)

    def _source_to_target_transform(self, stamp):
        """Return the stamped target<-source transform, or ``None``.

        A differing source frame needs timestamped TF; a static frame-id
        relabel is never accepted.
        """
        if self._own_gauge or self._source_frame == self._target_frame:
            return ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
        try:
            transform = self._buffer.lookup_transform(
                self._target_frame, self._source_frame, stamp,
                timeout=Duration(seconds=self._tf_timeout_s))
        except (TransformException, TypeError, ValueError, RuntimeError) as exc:
            self._reject(f"stamped TF unavailable: {exc}")
            return None
        translation = transform.transform.translation
        rotation = _normalize_quaternion((transform.transform.rotation.x,
                                          transform.transform.rotation.y,
                                          transform.transform.rotation.z,
                                          transform.transform.rotation.w))
        if rotation is None or not _finite((translation.x, translation.y, translation.z)):
            self._reject("stamped TF contained a non-finite transform")
            return None
        return ((float(translation.x), float(translation.y), float(translation.z)), rotation)

    def _callback(self, msg: Odometry):
        if not _stamp_is_set(msg.header.stamp):
            self._reject("source odometry has no acquisition timestamp")
            return
        if str(msg.header.frame_id) != self._source_frame:
            self._reject(f"source frame {msg.header.frame_id!r} != {self._source_frame!r}")
            return
        if str(msg.child_frame_id) != self._source_child_frame:
            self._reject(f"source child frame {msg.child_frame_id!r} != {self._source_child_frame!r}")
            return
        if not (self._verified or self._legacy_reference):
            self._reject("missing measured IMU-to-base extrinsic")
            return
        source_health_current=self._source_health_ok and 0<=time.monotonic()-self._source_health_received<=.50
        source_stamp = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        if self._reference_fault:
            self._reject(self._reference_fault)
            return
        if self._source_certificate_stamp != source_stamp:
            self._pending_odom = msg
            # Do not transiently fault a previously accepted pair merely
            # because DDS delivered the next odom before its certificate.
            # Without a matched pair, state/health freshness still expires.
            return
        if self._last_source_stamp is not None and source_stamp <= self._last_source_stamp:
            if source_stamp < self._last_source_stamp:
                self._reference_fault = "source clock regressed; restart with a new localization session"
                self._reject(self._reference_fault)
            return
        if self._require_source_health_ok and not source_health_current and not self._publish_unhealthy:
            self._reject("Super-LIO source health is not OK")
            return
        if len(self._translation) != 3 or not _finite(self._translation):
            self._reject("invalid IMU-to-base transform")
            return
        q_wi = _normalize_quaternion((msg.pose.pose.orientation.x,
                                      msg.pose.pose.orientation.y,
                                      msg.pose.pose.orientation.z,
                                      msg.pose.pose.orientation.w))
        q_ib = _normalize_quaternion(self._rotation)
        if q_wi is None or q_ib is None:
            self._reject("invalid IMU-to-base or source pose quaternion")
            return
        position = msg.pose.pose.position
        if not _finite((position.x, position.y, position.z)):
            self._reject("nonfinite source position")
            return
        if self._legacy_reference and self._last_source_pose is not None:
            previous_position, previous_rotation = self._last_source_pose
            delta = math.hypot(position.x-previous_position[0], position.y-previous_position[1])
            yaw_now = math.atan2(2*(q_wi[3]*q_wi[2]+q_wi[0]*q_wi[1]), 1-2*(q_wi[1]**2+q_wi[2]**2))
            yaw_previous = math.atan2(2*(previous_rotation[3]*previous_rotation[2]+previous_rotation[0]*previous_rotation[1]), 1-2*(previous_rotation[1]**2+previous_rotation[2]**2))
            if delta > .50 or abs(math.atan2(math.sin(yaw_now-yaw_previous), math.cos(yaw_now-yaw_previous))) > math.radians(15):
                self._reference_fault = "source pose step exceeds locked bridge guards; restart with a new session"
                self._reject(self._reference_fault)
                return
        covariance = tuple(msg.pose.covariance) + tuple(msg.twist.covariance)
        if self._require_covariance:
            if not _covariance_is_known(covariance):
                self._reject("Super-LIO covariance is unavailable or non-finite")
                return
        source_to_target = self._source_to_target_transform(msg.header.stamp)
        if source_to_target is None:
            return
        tf_translation, q_ts = source_to_target
        q_target_imu = _qmul(q_ts, q_wi)
        q_target_base = _qmul(q_target_imu, q_ib)
        if not _finite(q_wi + q_target_base + q_ts):
            self._reject("non-finite Super-LIO pose")
            return
        output = Odometry()
        output.header = copy.deepcopy(msg.header)
        output.header.frame_id = self._target_frame
        output.child_frame_id = str(self.get_parameter("base_frame").value)
        output.pose = copy.deepcopy(msg.pose)
        lever_arm_source = _qrotate(q_wi, self._translation)
        target_position = _qrotate(q_ts, (
            msg.pose.pose.position.x + lever_arm_source[0],
            msg.pose.pose.position.y + lever_arm_source[1],
            msg.pose.pose.position.z + lever_arm_source[2]))
        output.pose.pose.position.x = target_position[0] + tf_translation[0]
        output.pose.pose.position.y = target_position[1] + tf_translation[1]
        output.pose.pose.position.z = target_position[2] + tf_translation[2]
        output.pose.pose.orientation.x, output.pose.pose.orientation.y = q_target_base[0], q_target_base[1]
        output.pose.pose.orientation.z, output.pose.pose.orientation.w = q_target_base[2], q_target_base[3]
        rotated_pose_covariance = _lever_covariance(msg.pose.covariance,q_wi,q_ib,self._translation,pose=True,q_target_world=q_ts)
        if rotated_pose_covariance is None:
            self._reject("source pose covariance could not be rotated into target frame")
            return
        output.pose.covariance = list(rotated_pose_covariance)
        output.twist = copy.deepcopy(msg.twist)
        body_velocity, body_angular = _source_twist_to_base(
            q_ib, (msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z),
            (msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z),self._translation)
        rotated_twist_covariance = _lever_covariance(msg.twist.covariance,q_wi,q_ib,self._translation)
        if rotated_twist_covariance is None or not _finite(body_velocity + body_angular):
            self._reject("source twist could not be converted into base frame")
            return
        output.twist.covariance = list(rotated_twist_covariance)
        output.twist.twist.linear.x, output.twist.twist.linear.y, output.twist.twist.linear.z = body_velocity
        output.twist.twist.angular.x, output.twist.twist.angular.y, output.twist.twist.angular.z = body_angular
        self._odom.publish(output)
        self._last_source_stamp = source_stamp
        if self._legacy_reference:
            self._last_source_pose = ((position.x, position.y, position.z), q_wi)
        transform = TransformStamped()
        transform.header, transform.child_frame_id = output.header, output.child_frame_id
        transform.transform.translation.x = output.pose.pose.position.x
        transform.transform.translation.y = output.pose.pose.position.y
        transform.transform.translation.z = output.pose.pose.position.z
        transform.transform.rotation = output.pose.pose.orientation
        self._vehicle_tf.sendTransform(transform)
        self._publish_health("OK: measurement-matched observation bound and audited navigation reference applied" if source_health_current
                             else "UNKNOWN: Super-LIO source health has no validated equivalence")


def main(args=None):
    if rclpy is None:
        raise RuntimeError("ROS 2 rclpy is required on the Humble target; laptop replay is actuator-free")
    rclpy.init(args=args)
    node = SuperLioVehicleAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
