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
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from rclpy.node import Node
except ImportError:  # allows static linting on the developer laptop
    rclpy = None


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
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _normalize_quaternion(q):
    import math
    if len(q) != 4 or not _finite(q):
        return None
    norm = math.sqrt(sum(float(value) * float(value) for value in q))
    if not math.isfinite(norm) or norm <= 1e-12:
        return None
    return tuple(float(value) / norm for value in q)


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
        self.declare_parameter("imu_to_base_extrinsic_verified", False)
        self.declare_parameter("imu_to_base_translation_m", [0.0, 0.0, 0.0])
        self.declare_parameter("imu_to_base_quaternion_xyzw", [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter("require_source_health_ok", True)
        self.declare_parameter("require_covariance", True)
        self._verified = _parameter_bool(
            self.get_parameter("imu_to_base_extrinsic_verified").value)
        self._source_frame = str(self.get_parameter("source_frame").value)
        self._target_frame = str(self.get_parameter("world_frame").value)
        self._translation = tuple(float(x) for x in self.get_parameter("imu_to_base_translation_m").value)
        self._rotation = tuple(float(x) for x in self.get_parameter("imu_to_base_quaternion_xyzw").value)
        self._require_source_health_ok = _parameter_bool(
            self.get_parameter("require_source_health_ok").value)
        self._require_covariance = _parameter_bool(
            self.get_parameter("require_covariance").value)
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

    def _callback(self, msg: Odometry):
        if str(msg.header.frame_id) != self._source_frame:
            self._reject(f"source frame {msg.header.frame_id!r} != {self._source_frame!r}")
            return
        if self._source_frame != self._target_frame:
            self._reject(
                f"source frame {self._source_frame!r} needs timestamped TF to "
                f"{self._target_frame!r}; refusing frame relabel")
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
        q_wb = _qmul(q_wi, q_ib)
        if not _finite(q_wi + q_wb):
            self._reject("non-finite Super-LIO pose")
            return
        output = Odometry()
        output.header = msg.header
        output.header.frame_id = self._target_frame
        output.child_frame_id = str(self.get_parameter("base_frame").value)
        output.pose = msg.pose
        output.pose.pose.position.x += _qrotate(q_wi, self._translation)[0]
        output.pose.pose.position.y += _qrotate(q_wi, self._translation)[1]
        output.pose.pose.position.z += _qrotate(q_wi, self._translation)[2]
        output.pose.pose.orientation.x, output.pose.pose.orientation.y = q_wb[0], q_wb[1]
        output.pose.pose.orientation.z, output.pose.pose.orientation.w = q_wb[2], q_wb[3]
        output.twist = msg.twist
        q_bw = _qconj(q_wb)
        body_velocity = _qrotate(q_bw, (msg.twist.twist.linear.x,
                                        msg.twist.twist.linear.y,
                                        msg.twist.twist.linear.z))
        body_angular = _qrotate(q_bw, (msg.twist.twist.angular.x,
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
