#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist
from interface.msg import LocalizationStatus
from rclpy.node import Node


class LocalizationCmdGate(Node):
    def __init__(self):
        super().__init__("localization_cmd_gate")
        self.declare_parameter("cmd_vel_in", "/cmd_vel")
        self.declare_parameter("cmd_vel_out", "/cmd_vel_localized")
        self.declare_parameter("status_topic", "/localizer/status")
        self.declare_parameter("status_timeout_s", 0.5)
        self.declare_parameter("publish_hz", 20.0)

        cmd_vel_in = str(self.get_parameter("cmd_vel_in").value)
        cmd_vel_out = str(self.get_parameter("cmd_vel_out").value)
        status_topic = str(self.get_parameter("status_topic").value)
        self.status_timeout_s = float(self.get_parameter("status_timeout_s").value)
        publish_hz = float(self.get_parameter("publish_hz").value)

        self.allowed = False
        self.last_status_time = None
        self.last_cmd = Twist()
        self.publisher = self.create_publisher(Twist, cmd_vel_out, 10)
        self.create_subscription(Twist, cmd_vel_in, self.on_cmd_vel, 10)
        self.create_subscription(LocalizationStatus, status_topic, self.on_status, 10)
        self.create_timer(1.0 / max(1.0, publish_hz), self.on_timer)

    def on_status(self, msg):
        self.allowed = bool(
            msg.localized
            and msg.sensors_ready
            and msg.state == LocalizationStatus.LOCALIZED
        )
        self.last_status_time = self.get_clock().now()
        if not self.allowed:
            self.publisher.publish(Twist())

    def on_cmd_vel(self, msg):
        self.last_cmd = msg
        self.publisher.publish(msg if self.is_allowed() else Twist())

    def is_allowed(self):
        if not self.allowed or self.last_status_time is None:
            return False
        age = (self.get_clock().now() - self.last_status_time).nanoseconds * 1.0e-9
        return age <= self.status_timeout_s

    def on_timer(self):
        if not self.is_allowed():
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
