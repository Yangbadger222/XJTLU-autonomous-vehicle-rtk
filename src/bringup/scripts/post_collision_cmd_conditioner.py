#!/usr/bin/env python3

import copy
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class PostCollisionCmdConditioner(Node):
    def __init__(self):
        super().__init__("post_collision_cmd_conditioner")
        self.declare_parameter("cmd_vel_in", "/cmd_vel_safe_raw")
        self.declare_parameter("cmd_vel_out", "/cmd_vel_safe")
        self.declare_parameter("reference_cmd_vel", "/cmd_vel_localized")
        self.declare_parameter("reference_timeout_s", 0.20)
        self.declare_parameter("slowdown_ratio_threshold", 0.80)
        self.declare_parameter("linear_deadband", 0.14)
        self.declare_parameter("in_place_linear_threshold", 0.02)
        self.declare_parameter("turning_angular_threshold", 0.08)
        self.declare_parameter("angular_zero_threshold", 0.01)
        self.declare_parameter("min_in_place_angular_speed", 0.20)

        cmd_vel_in = str(self.get_parameter("cmd_vel_in").value)
        cmd_vel_out = str(self.get_parameter("cmd_vel_out").value)
        reference_cmd_vel = str(self.get_parameter("reference_cmd_vel").value)
        self.reference_timeout_s = max(
            0.0, float(self.get_parameter("reference_timeout_s").value)
        )
        self.slowdown_ratio_threshold = min(
            1.0,
            max(0.0, float(self.get_parameter("slowdown_ratio_threshold").value)),
        )
        self.linear_deadband = max(
            0.0, float(self.get_parameter("linear_deadband").value)
        )
        self.in_place_linear_threshold = max(
            0.0, float(self.get_parameter("in_place_linear_threshold").value)
        )
        self.turning_angular_threshold = max(
            0.0, float(self.get_parameter("turning_angular_threshold").value)
        )
        self.angular_zero_threshold = max(
            0.0, float(self.get_parameter("angular_zero_threshold").value)
        )
        self.min_in_place_angular_speed = max(
            0.0, float(self.get_parameter("min_in_place_angular_speed").value)
        )

        self.publisher = self.create_publisher(Twist, cmd_vel_out, 10)
        self.last_reference = None
        self.last_reference_time = None
        self.subscription = self.create_subscription(
            Twist, cmd_vel_in, self.on_cmd_vel, 10
        )
        self.reference_subscription = self.create_subscription(
            Twist, reference_cmd_vel, self.on_reference_cmd_vel, 10
        )

    def on_reference_cmd_vel(self, msg):
        self.last_reference = msg
        self.last_reference_time = self.get_clock().now()

    def on_cmd_vel(self, msg):
        self.publisher.publish(self.condition_command(msg))

    def condition_command(self, msg):
        if not self.is_slowdown_output(msg):
            return msg
        linear_speed = math.hypot(msg.linear.x, msg.linear.y)
        angular_speed = abs(msg.angular.z)
        angular_requested = angular_speed > self.angular_zero_threshold
        pure_rotation = linear_speed <= self.in_place_linear_threshold
        stalled_turn = (
            self.in_place_linear_threshold < linear_speed < self.linear_deadband
            and angular_speed >= self.turning_angular_threshold
        )
        if not angular_requested or not (pure_rotation or stalled_turn):
            return msg

        out = copy.deepcopy(msg)
        if stalled_turn:
            out.linear.x = 0.0
            out.linear.y = 0.0
        if angular_speed < self.min_in_place_angular_speed:
            out.angular.z = math.copysign(
                self.min_in_place_angular_speed, msg.angular.z
            )
        return out

    def is_slowdown_output(self, msg):
        output_linear = math.hypot(msg.linear.x, msg.linear.y)
        output_angular = abs(msg.angular.z)
        if output_linear <= 1.0e-6 and output_angular <= self.angular_zero_threshold:
            return False
        if self.last_reference is None or self.last_reference_time is None:
            return False
        age_s = (
            self.get_clock().now() - self.last_reference_time
        ).nanoseconds * 1.0e-9
        if age_s < 0.0 or age_s > self.reference_timeout_s:
            return False

        reference_linear = math.hypot(
            self.last_reference.linear.x, self.last_reference.linear.y
        )
        reference_angular = abs(self.last_reference.angular.z)
        linear_reduced = (
            reference_linear > self.in_place_linear_threshold
            and output_linear
            < reference_linear * self.slowdown_ratio_threshold
        )
        angular_reduced = (
            reference_angular > self.angular_zero_threshold
            and output_angular
            < reference_angular * self.slowdown_ratio_threshold
        )
        return linear_reduced or angular_reduced


def main(args=None):
    rclpy.init(args=args)
    node = PostCollisionCmdConditioner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
