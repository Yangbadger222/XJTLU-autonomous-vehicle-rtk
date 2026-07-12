#!/usr/bin/env python3

import copy
import math

import rclpy
from geometry_msgs.msg import Twist
from interface.msg import LocalizationStatus
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


class LocalizationCmdGate(Node):
    def __init__(self):
        super().__init__("localization_cmd_gate")
        self.declare_parameter("cmd_vel_in", "/cmd_vel")
        self.declare_parameter("cmd_vel_out", "/cmd_vel_localized")
        self.declare_parameter("status_topic", "/localizer/status")
        self.declare_parameter("status_timeout_s", 0.5)
        self.declare_parameter("obstacle_topic", "/fastlio2/body_cloud_nav2")
        self.declare_parameter("obstacle_timeout_s", 0.5)
        self.declare_parameter("cmd_timeout_s", 0.25)
        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("in_place_linear_threshold", 0.02)
        self.declare_parameter("angular_zero_threshold", 0.01)
        self.declare_parameter("min_in_place_angular_speed", 0.20)

        cmd_vel_in = str(self.get_parameter("cmd_vel_in").value)
        cmd_vel_out = str(self.get_parameter("cmd_vel_out").value)
        status_topic = str(self.get_parameter("status_topic").value)
        obstacle_topic = str(self.get_parameter("obstacle_topic").value)
        self.status_timeout_s = float(self.get_parameter("status_timeout_s").value)
        self.obstacle_timeout_s = float(
            self.get_parameter("obstacle_timeout_s").value
        )
        self.cmd_timeout_s = float(self.get_parameter("cmd_timeout_s").value)
        self.in_place_linear_threshold = max(
            0.0,
            float(self.get_parameter("in_place_linear_threshold").value),
        )
        self.angular_zero_threshold = max(
            0.0,
            float(self.get_parameter("angular_zero_threshold").value),
        )
        self.min_in_place_angular_speed = max(
            0.0,
            float(self.get_parameter("min_in_place_angular_speed").value),
        )
        publish_hz = float(self.get_parameter("publish_hz").value)

        self.allowed = False
        self.last_status_time = None
        self.last_obstacle_time = None
        self.last_cmd_time = None
        self.publisher = self.create_publisher(Twist, cmd_vel_out, 10)
        self.create_subscription(Twist, cmd_vel_in, self.on_cmd_vel, 10)
        self.create_subscription(LocalizationStatus, status_topic, self.on_status, 10)
        self.create_subscription(
            PointCloud2,
            obstacle_topic,
            self.on_obstacle,
            qos_profile_sensor_data,
        )
        self.create_timer(1.0 / max(1.0, publish_hz), self.on_timer)

    def on_status(self, msg):
        self.allowed = bool(
            msg.localized
            and msg.sensors_ready
            and msg.state
            in (LocalizationStatus.LOCALIZED, LocalizationStatus.DEGRADED)
        )
        self.last_status_time = self.get_clock().now()
        if not self.allowed:
            self.publisher.publish(Twist())

    def on_cmd_vel(self, msg):
        self.last_cmd_time = self.get_clock().now()
        self.publisher.publish(
            self.apply_in_place_angular_floor(msg) if self.is_allowed() else Twist()
        )

    def apply_in_place_angular_floor(self, msg):
        angular_z = float(msg.angular.z)
        if (
            math.hypot(msg.linear.x, msg.linear.y)
            > self.in_place_linear_threshold
            or abs(angular_z) <= self.angular_zero_threshold
            or abs(angular_z) >= self.min_in_place_angular_speed
        ):
            return msg
        out = copy.deepcopy(msg)
        out.angular.z = math.copysign(self.min_in_place_angular_speed, angular_z)
        return out

    def on_obstacle(self, _msg):
        self.last_obstacle_time = self.get_clock().now()

    def is_fresh(self, stamp, timeout_s):
        if stamp is None:
            return False
        age = (self.get_clock().now() - stamp).nanoseconds * 1.0e-9
        return 0.0 <= age <= timeout_s

    def is_allowed(self):
        return bool(
            self.allowed
            and self.is_fresh(self.last_status_time, self.status_timeout_s)
            and self.is_fresh(self.last_obstacle_time, self.obstacle_timeout_s)
        )

    def on_timer(self):
        if not self.is_allowed() or not self.is_fresh(
            self.last_cmd_time, self.cmd_timeout_s
        ):
            self.publisher.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationCmdGate()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
