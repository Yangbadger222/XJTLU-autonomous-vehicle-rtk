#!/usr/bin/env python3
from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool

from gps_waypoint_dispatcher.corridor_cmd_guard import CorridorCommandGuard


class CorridorCmdVelGuard(Node):
    def __init__(self) -> None:
        super().__init__("corridor_cmd_vel_guard")
        self.declare_parameter("input_topic", "/cmd_vel_nav")
        self.declare_parameter("motion_allowed_topic", "/localization_authority/motion_allowed")
        self.declare_parameter("stop_override_topic", "/gps_corridor/stop_override")
        self.declare_parameter("output_topic", "/cmd_vel")
        self.declare_parameter("straight_max_mps", 0.85)
        self.declare_parameter("turn_product_limit", 0.25)
        self.declare_parameter("min_turn_rate_radps", 0.05)
        self.declare_parameter("command_timeout_s", 0.25)
        self.declare_parameter("heartbeat_timeout_s", 0.50)

        self._guard = CorridorCommandGuard(
            straight_max_mps=float(self.get_parameter("straight_max_mps").value),
            turn_product_limit=float(self.get_parameter("turn_product_limit").value),
            min_turn_rate_radps=float(
                self.get_parameter("min_turn_rate_radps").value
            ),
            command_timeout_s=float(self.get_parameter("command_timeout_s").value),
            heartbeat_timeout_s=float(
                self.get_parameter("heartbeat_timeout_s").value
            ),
        )
        self._output_pub = self.create_publisher(
            Twist, str(self.get_parameter("output_topic").value), 10
        )
        self._command_sub = self.create_subscription(
            Twist,
            str(self.get_parameter("input_topic").value),
            self._command_callback,
            10,
        )
        self._authority_sub = self.create_subscription(
            Bool,
            str(self.get_parameter("motion_allowed_topic").value),
            self._authority_callback,
            10,
        )
        self._stop_sub = self.create_subscription(
            Bool,
            str(self.get_parameter("stop_override_topic").value),
            self._stop_callback,
            10,
        )
        self._timer = self.create_timer(0.05, self._timer_callback)

    def _command_callback(self, msg: Twist) -> None:
        self._guard.update_command(
            msg.linear.x,
            msg.angular.z,
            received_s=time.monotonic(),
        )

    def _authority_callback(self, msg: Bool) -> None:
        self._guard.update_authority(msg.data, received_s=time.monotonic())

    def _stop_callback(self, msg: Bool) -> None:
        self._guard.update_stop_override(msg.data, received_s=time.monotonic())

    def _timer_callback(self) -> None:
        guarded = self._guard.evaluate(now_s=time.monotonic())
        output = Twist()
        output.linear.x = guarded.linear_x
        output.angular.z = guarded.angular_z
        self._output_pub.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CorridorCmdVelGuard()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
