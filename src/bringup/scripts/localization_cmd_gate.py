#!/usr/bin/env python3

import json
import math

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from interface.msg import LocalizationStatus
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String


class LocalizationCmdGate(Node):
    def __init__(self):
        super().__init__("localization_cmd_gate")
        self.declare_parameter("cmd_vel_in", "/cmd_vel")
        self.declare_parameter("cmd_vel_out", "/cmd_vel_localized")
        self.declare_parameter("status_topic", "/localizer/status")
        self.declare_parameter("status_timeout_s", 0.5)
        self.declare_parameter("authority_status_topic", "/travel/prior_map_tf/status")
        self.declare_parameter("authority_timeout_s", 0.6)
        self.declare_parameter("obstacle_topic", "/fastlio2/body_cloud_nav2")
        self.declare_parameter("obstacle_timeout_s", 0.5)
        self.declare_parameter("cmd_timeout_s", 0.40)
        self.declare_parameter("collision_output_topic", "/cmd_vel_safe_raw")
        self.declare_parameter("final_output_topic", "/cmd_vel_safe")
        self.declare_parameter("output_timeout_s", 0.25)
        self.declare_parameter("state_topic", "/travel/control_gate/status")
        self.declare_parameter("diagnostics_topic", "/diagnostics")
        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("status_publish_hz", 5.0)

        cmd_vel_in = str(self.get_parameter("cmd_vel_in").value)
        cmd_vel_out = str(self.get_parameter("cmd_vel_out").value)
        status_topic = str(self.get_parameter("status_topic").value)
        authority_status_topic = str(
            self.get_parameter("authority_status_topic").value
        )
        obstacle_topic = str(self.get_parameter("obstacle_topic").value)
        collision_output_topic = str(
            self.get_parameter("collision_output_topic").value
        )
        final_output_topic = str(self.get_parameter("final_output_topic").value)
        state_topic = str(self.get_parameter("state_topic").value)
        diagnostics_topic = str(self.get_parameter("diagnostics_topic").value)
        self.status_timeout_s = float(self.get_parameter("status_timeout_s").value)
        self.authority_timeout_s = float(
            self.get_parameter("authority_timeout_s").value
        )
        self.obstacle_timeout_s = float(
            self.get_parameter("obstacle_timeout_s").value
        )
        self.cmd_timeout_s = float(self.get_parameter("cmd_timeout_s").value)
        self.output_timeout_s = float(self.get_parameter("output_timeout_s").value)
        publish_hz = float(self.get_parameter("publish_hz").value)
        status_publish_hz = float(self.get_parameter("status_publish_hz").value)

        self.allowed = False
        self.localizer_state = "UNINITIALIZED"
        self.localizer_reason = "no_status"
        self.authority_active = False
        self.authority_state = "NO_STATUS"
        self.last_status_time = None
        self.last_authority_time = None
        self.last_obstacle_time = None
        self.last_cmd_time = None
        self.last_collision_time = None
        self.last_final_time = None
        self.last_input = Twist()
        self.last_gated_output = Twist()
        self.last_collision_output = Twist()
        self.last_final_output = Twist()
        self.last_reported_reason = None
        self.publisher = self.create_publisher(Twist, cmd_vel_out, 10)
        self.state_publisher = self.create_publisher(String, state_topic, 10)
        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray, diagnostics_topic, 10
        )
        self.create_subscription(Twist, cmd_vel_in, self.on_cmd_vel, 10)
        self.create_subscription(LocalizationStatus, status_topic, self.on_status, 10)
        self.create_subscription(String, authority_status_topic, self.on_authority, 10)
        self.create_subscription(
            PointCloud2,
            obstacle_topic,
            self.on_obstacle,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Twist, collision_output_topic, self.on_collision_output, 10
        )
        self.create_subscription(Twist, final_output_topic, self.on_final_output, 10)
        self.create_timer(1.0 / max(1.0, publish_hz), self.on_timer)
        self.create_timer(
            1.0 / max(1.0, status_publish_hz), self.publish_status
        )

    def on_status(self, msg):
        self.allowed = bool(
            msg.localized
            and msg.sensors_ready
            and msg.state
            in (LocalizationStatus.LOCALIZED, LocalizationStatus.DEGRADED)
        )
        self.localizer_state = msg.state_label or str(msg.state)
        self.localizer_reason = msg.reason or "unspecified"
        self.last_status_time = self.get_clock().now()
        if not self.allowed:
            self.publish_gated(Twist())

    def on_cmd_vel(self, msg):
        self.last_cmd_time = self.get_clock().now()
        self.last_input = msg
        self.publish_gated(msg if self.is_allowed() else Twist())

    def on_obstacle(self, _msg):
        self.last_obstacle_time = self.get_clock().now()

    def on_authority(self, msg):
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError):
            self.authority_active = False
            self.authority_state = "INVALID_STATUS"
        else:
            self.authority_active = bool(payload.get("tf_active", False))
            self.authority_state = str(payload.get("state", "UNKNOWN"))
        self.last_authority_time = self.get_clock().now()

    def on_collision_output(self, msg):
        self.last_collision_time = self.get_clock().now()
        self.last_collision_output = msg

    def on_final_output(self, msg):
        self.last_final_time = self.get_clock().now()
        self.last_final_output = msg

    def publish_gated(self, msg):
        self.last_gated_output = msg
        self.publisher.publish(msg)

    def is_fresh(self, stamp, timeout_s):
        if stamp is None:
            return False
        age = (self.get_clock().now() - stamp).nanoseconds * 1.0e-9
        return 0.0 <= age <= timeout_s

    def is_allowed(self):
        return bool(
            self.allowed
            and self.is_fresh(self.last_status_time, self.status_timeout_s)
            and self.authority_active
            and self.is_fresh(self.last_authority_time, self.authority_timeout_s)
            and self.is_fresh(self.last_obstacle_time, self.obstacle_timeout_s)
        )

    def on_timer(self):
        if not self.is_allowed() or not self.is_fresh(
            self.last_cmd_time, self.cmd_timeout_s
        ):
            self.publish_gated(Twist())

    @staticmethod
    def command_magnitude(msg):
        return max(math.hypot(msg.linear.x, msg.linear.y), abs(msg.angular.z))

    @staticmethod
    def command_reduced(reference, output, ratio=0.95):
        reference_components = (
            abs(reference.linear.x),
            abs(reference.linear.y),
            abs(reference.angular.z),
        )
        output_components = (
            abs(output.linear.x),
            abs(output.linear.y),
            abs(output.angular.z),
        )
        return any(
            reference_value > 1.0e-4
            and output_value < ratio * reference_value
            for reference_value, output_value in zip(
                reference_components, output_components
            )
        )

    def age_s(self, stamp):
        if stamp is None:
            return None
        return max(0.0, (self.get_clock().now() - stamp).nanoseconds * 1.0e-9)

    def control_reason(self):
        if not self.is_fresh(self.last_status_time, self.status_timeout_s):
            return "LOCALIZATION_STATUS_TIMEOUT"
        if not self.allowed:
            return "LOCALIZATION_BLOCKED"
        if not self.is_fresh(self.last_authority_time, self.authority_timeout_s):
            return "LOCALIZATION_AUTHORITY_TIMEOUT"
        if not self.authority_active:
            return "LOCALIZATION_AUTHORITY_BLOCKED"
        if not self.is_fresh(self.last_obstacle_time, self.obstacle_timeout_s):
            return "POINTCLOUD_TIMEOUT"
        if not self.is_fresh(self.last_cmd_time, self.cmd_timeout_s):
            if self.command_magnitude(self.last_input) <= 1.0e-4:
                return "IDLE_NO_COMMAND"
            return "COMMAND_TIMEOUT"

        requested = self.command_magnitude(self.last_input)
        gated = self.command_magnitude(self.last_gated_output)
        if requested <= 1.0e-4:
            return "IDLE_NO_COMMAND"

        if self.is_fresh(self.last_collision_time, self.output_timeout_s):
            collision = self.command_magnitude(self.last_collision_output)
            if gated > 1.0e-4 and collision <= 1.0e-4:
                return "COLLISION_STOP"
            if gated > 1.0e-4 and self.command_reduced(
                self.last_gated_output, self.last_collision_output
            ):
                return "COLLISION_SLOWDOWN"

        if self.is_fresh(self.last_final_time, self.output_timeout_s):
            final = self.command_magnitude(self.last_final_output)
            if gated > 1.0e-4 and final <= 1.0e-4:
                return "OUTPUT_BLOCKED"
        return "PASS"

    def publish_status(self):
        reason = self.control_reason()
        payload = {
            "reason": reason,
            "motion_requested": self.command_magnitude(self.last_input) > 1.0e-4,
            "localizer_state": self.localizer_state,
            "localizer_reason": self.localizer_reason,
            "authority_state": self.authority_state,
            "authority_active": self.authority_active,
            "status_age_s": self.age_s(self.last_status_time),
            "authority_age_s": self.age_s(self.last_authority_time),
            "pointcloud_age_s": self.age_s(self.last_obstacle_time),
            "command_age_s": self.age_s(self.last_cmd_time),
            "collision_output_age_s": self.age_s(self.last_collision_time),
        }
        self.state_publisher.publish(String(data=json.dumps(payload, sort_keys=True)))

        status = DiagnosticStatus()
        status.name = "travel/control_gate"
        status.hardware_id = "travel_control_chain"
        status.message = reason
        if reason in {
            "LOCALIZATION_STATUS_TIMEOUT",
            "LOCALIZATION_BLOCKED",
            "LOCALIZATION_AUTHORITY_TIMEOUT",
            "LOCALIZATION_AUTHORITY_BLOCKED",
            "POINTCLOUD_TIMEOUT",
        }:
            status.level = DiagnosticStatus.ERROR
        elif reason in {
            "COMMAND_TIMEOUT",
            "COLLISION_STOP",
            "COLLISION_SLOWDOWN",
            "OUTPUT_BLOCKED",
        }:
            status.level = DiagnosticStatus.WARN
        else:
            status.level = DiagnosticStatus.OK
        status.values = [
            KeyValue(key=key, value="null" if value is None else str(value))
            for key, value in payload.items()
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.diagnostics_publisher.publish(array)

        if reason != self.last_reported_reason:
            self.get_logger().info(f"Control gate state: {reason}")
            self.last_reported_reason = reason


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationCmdGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
