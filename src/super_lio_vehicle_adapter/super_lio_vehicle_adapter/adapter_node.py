"""Adapt Super-LIO's world->IMU output without inventing vehicle geometry.

At the pinned upstream commit Super-LIO publishes an odometry message whose
world frame is ``world`` and whose child is ``imu``.  It is not the vehicle
``odom -> base_footprint`` contract.  Until a timestamped source-frame TF, a
measured IMU->base transform, and source health are supplied, this node
publishes source-aware UNKNOWN health and intentionally does not publish
vehicle odometry.
This keeps the original RTK authority stop policy fail-closed.
"""
from __future__ import annotations

try:
    import rclpy
    from rclpy.duration import Duration
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from rclpy.node import Node
    from tf2_ros import Buffer, TransformException, TransformListener
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


def _normalize_quaternion(q):
    import math
    if len(q) != 4 or not _finite(q):
        return None
    norm = math.sqrt(sum(float(value) * float(value) for value in q))
    if not math.isfinite(norm) or norm <= 1e-12:
        return None
    return tuple(float(value) / norm for value in q)


def _rotate_covariance(covariance, quaternion):
    """Rotate a 6x6 pose covariance by a source->target quaternion.

    Odometry pose covariance is ordered as xyz/rpy. The same 3x3 rotation is
    applied to both blocks; this keeps the covariance contract honest when a
    stamped ``world -> odom`` transform has a non-zero yaw. Twist covariance is
    left in the child frame because the measured IMU->base transform is gated
    to the identity whenever covariance checking is enabled.
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
    matrix = [list(float(value) for value in covariance[row * 6:(row + 1) * 6])
              for row in range(6)]
    output = [[0.0] * 6 for _ in range(6)]
    for block_row in (0, 1):
        for block_col in (0, 1):
            for row in range(3):
                for col in range(3):
                    output[3 * block_row + row][3 * block_col + col] = sum(
                        rotation[row][i] * matrix[3 * block_row + i][3 * block_col + j] *
                        rotation[col][j]
                        for i in range(3) for j in range(3))
    return tuple(value for row in output for value in row)


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
        self._verified = _parameter_bool(
            self.get_parameter("imu_to_base_extrinsic_verified").value)
        self._source_frame = str(self.get_parameter("source_frame").value)
        self._target_frame = str(self.get_parameter("world_frame").value)
        self._source_child_frame = str(self.get_parameter("source_child_frame").value)
        self._tf_timeout_s = max(0.0, float(self.get_parameter("tf_timeout_s").value))
        self._translation = tuple(float(x) for x in self.get_parameter("imu_to_base_translation_m").value)
        self._rotation = tuple(float(x) for x in self.get_parameter("imu_to_base_quaternion_xyzw").value)
        self._require_source_health_ok = _parameter_bool(
            self.get_parameter("require_source_health_ok").value)
        self._require_covariance = _parameter_bool(
            self.get_parameter("require_covariance").value)
        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)
        self._source_health_ok = False
        self._health = self.create_publisher(String, str(self.get_parameter("health_topic").value), 10)
        self._odom = self.create_publisher(Odometry, str(self.get_parameter("vehicle_odom_topic").value), 10)
        self.create_subscription(String, str(self.get_parameter("source_health_topic").value), self._health_callback, 10)
        self.create_subscription(Odometry, str(self.get_parameter("input_topic").value), self._callback, 10)
        self._publish_health("UNKNOWN: Super-LIO source health/IMU-to-base equivalence is not verified")

    def _publish_health(self, text: str):
        self._health.publish(String(data=text))

    def _health_callback(self, msg):
        self._source_health_ok = str(msg.data).upper().startswith("OK")

    def _reject(self, reason: str):
        self._publish_health("UNKNOWN: " + reason)

    def _source_to_target_transform(self, stamp):
        """Return the stamped target<-source transform, or ``None``.

        A differing source frame needs timestamped TF; a static frame-id
        relabel is never accepted.
        """
        if self._source_frame == self._target_frame:
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
        if not self._verified:
            self._reject("missing measured IMU-to-base extrinsic")
            return
        if self._require_source_health_ok and not self._source_health_ok:
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
        # A non-zero lever arm requires a covariance/velocity transform that
        # includes angular-rate cross terms. Refuse until that measured path is
        # explicitly implemented; never relabel the frame or copy covariance.
        if any(abs(value) > 1e-9 for value in self._translation):
            self._reject("non-zero lever arm covariance transform is not verified")
            return
        covariance = tuple(msg.pose.covariance) + tuple(msg.twist.covariance)
        if self._require_covariance:
            if not _finite(covariance) or not any(abs(float(value)) > 0.0 for value in covariance):
                self._reject("Super-LIO covariance is unavailable or non-finite")
                return
            # The message covariance is still expressed in the IMU frame.
            # No 6x6 rotation/adjoint transform is implemented here, so a
            # non-identity verified rotation must remain motion-blocking.
            if any(abs(value) > 1e-9 for value in q_ib[:3]):
                self._reject("non-identity IMU-to-base covariance transform is not verified")
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
        output.header = msg.header
        output.header.frame_id = self._target_frame
        output.child_frame_id = str(self.get_parameter("base_frame").value)
        output.pose = msg.pose
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
        rotated_pose_covariance = _rotate_covariance(msg.pose.covariance, q_ts)
        if rotated_pose_covariance is None:
            self._reject("source pose covariance could not be rotated into target frame")
            return
        output.pose.covariance = list(rotated_pose_covariance)
        output.twist = msg.twist
        body_velocity = _qrotate(q_ib, (msg.twist.twist.linear.x,
                                        msg.twist.twist.linear.y,
                                        msg.twist.twist.linear.z))
        body_angular = _qrotate(q_ib, (msg.twist.twist.angular.x,
                                       msg.twist.twist.angular.y,
                                       msg.twist.twist.angular.z))
        output.twist.twist.linear.x, output.twist.twist.linear.y, output.twist.twist.linear.z = body_velocity
        output.twist.twist.angular.x, output.twist.twist.angular.y, output.twist.twist.angular.z = body_angular
        self._odom.publish(output)
        self._publish_health("OK: source covariance and zero-lever-arm transform verified")


def main(args=None):
    if rclpy is None:
        raise RuntimeError("ROS 2 rclpy is required on the Humble target; laptop replay is actuator-free")
    rclpy.init(args=args)
    node = SuperLioVehicleAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
