"""Adapt Super-LIO's world->IMU output without inventing vehicle geometry.

At the pinned upstream commit Super-LIO publishes an odometry message whose
child is not the vehicle base and has no covariance/health equivalent. Until a
measured IMU->base transform is supplied, this node publishes source-aware
UNKNOWN health and intentionally does not publish a vehicle odometry command
input. This keeps the original RTK authority stop policy fail-closed.
"""
from __future__ import annotations

try:
    import rclpy
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from rclpy.node import Node
except ImportError:  # allows static linting on the developer laptop
    rclpy = None


class SuperLioVehicleAdapter(Node if rclpy else object):
    def __init__(self):
        super().__init__("super_lio_vehicle_adapter")
        self.declare_parameter("input_topic", "/lio/odom")
        self.declare_parameter("vehicle_odom_topic", "/lio/odom_vehicle")
        self.declare_parameter("health_topic", "/lio/health")
        self.declare_parameter("imu_to_base_extrinsic_verified", False)
        self._verified = bool(self.get_parameter("imu_to_base_extrinsic_verified").value)
        self._health = self.create_publisher(String, str(self.get_parameter("health_topic").value), 10)
        self._odom = self.create_publisher(Odometry, str(self.get_parameter("vehicle_odom_topic").value), 10)
        self.create_subscription(Odometry, str(self.get_parameter("input_topic").value), self._callback, 10)
        self._publish_health("UNKNOWN: Super-LIO health/IMU-to-base equivalence is not verified")

    def _publish_health(self, text: str):
        self._health.publish(String(data=text))

    def _callback(self, msg: Odometry):
        if not self._verified:
            self._publish_health("UNKNOWN: missing measured IMU-to-base extrinsic")
            return
        # A verified transform must be implemented here before enabling live.
        # Do not merely relabel child_frame_id: that would be a false frame.
        self._publish_health("FAIL: verified transform path is not implemented")


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
