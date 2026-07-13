#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from interface.srv import Relocalize
from rclpy.node import Node


def yaw_from_quaternion(quaternion):
    siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy_cosp = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(siny_cosp, cosy_cosp)


class InitialPoseRelocalizeBridge(Node):
    def __init__(self):
        super().__init__("initialpose_relocalize_bridge")

        self.declare_parameter("pcd_map", "")
        self.declare_parameter("initialpose_topic", "/initialpose")
        self.declare_parameter("amcl_initialpose_topic", "/amcl/initialpose")
        self.declare_parameter("relocalize_service", "/localizer/relocalize")
        self.declare_parameter("service_wait_s", 2.0)

        self.pcd_map = self.get_parameter("pcd_map").value
        initialpose_topic = self.get_parameter("initialpose_topic").value
        amcl_initialpose_topic = self.get_parameter("amcl_initialpose_topic").value
        relocalize_service = self.get_parameter("relocalize_service").value
        self.service_wait_s = float(self.get_parameter("service_wait_s").value)

        self.client = self.create_client(Relocalize, relocalize_service)
        self.amcl_initialpose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, amcl_initialpose_topic, 10
        )
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            initialpose_topic,
            self.on_initial_pose,
            10,
        )

        self.get_logger().info(
            f"Bridging {initialpose_topic} to {relocalize_service} with pcd_map={self.pcd_map}"
        )

    def on_initial_pose(self, msg):
        if msg.header.frame_id.lstrip("/") != "map":
            self.get_logger().error(
                "Initial pose must use frame_id=map; refusing ambiguous coordinates"
            )
            return
        if not self.pcd_map:
            self.get_logger().error("pcd_map is empty; cannot call /localizer/relocalize")
            return

        self.amcl_initialpose_publisher.publish(msg)

        if not self.client.wait_for_service(timeout_sec=self.service_wait_s):
            self.get_logger().error("/localizer/relocalize service is not available")
            return

        req = Relocalize.Request()
        req.pcd_path = self.pcd_map
        req.x = float(msg.pose.pose.position.x)
        req.y = float(msg.pose.pose.position.y)
        req.z = float(msg.pose.pose.position.z)
        req.yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        req.pitch = 0.0
        req.roll = 0.0

        self.get_logger().info(
            "Initial pose -> relocalize: "
            f"x={req.x:.3f}, y={req.y:.3f}, z={req.z:.3f}, yaw={req.yaw:.3f} rad"
        )
        future = self.client.call_async(req)
        future.add_done_callback(self.on_relocalize_done)

    def on_relocalize_done(self, future):
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f"Relocalize call failed: {exc}")
            return

        if response.success:
            self.get_logger().info(f"Relocalize accepted: {response.message}")
        else:
            self.get_logger().error(f"Relocalize rejected: {response.message}")


def main(args=None):
    rclpy.init(args=args)
    node = InitialPoseRelocalizeBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
