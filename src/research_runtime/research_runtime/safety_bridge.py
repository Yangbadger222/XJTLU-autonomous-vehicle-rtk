"""ROS edge placeholder with an actuator-free replay implementation.

The ROS node is intentionally refused when the required ROS transport is not
available. This avoids silently publishing commands from a developer laptop.
"""
from __future__ import annotations

import argparse
import sys
import time
from .replay_sim import main as replay_main
from .authority import AuthorityState, SafetyGate

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Bool, String
    from research_interfaces.msg import TimedTrajectory2D
except ImportError:
    rclpy = None


if rclpy:
    class SafetyBridgeNode(Node):
        """Timed trajectory edge; authority and health are fail-closed."""
        def __init__(self):
            super().__init__("research_safety_bridge")
            self.declare_parameter("mode", "replay")
            self.declare_parameter("actuator_enabled", False)
            self.declare_parameter("authority_timeout_s", 0.50)
            self._mode = str(self.get_parameter("mode").value)
            self._actuator_enabled = bool(self.get_parameter("actuator_enabled").value)
            self._gate = SafetyGate(float(self.get_parameter("authority_timeout_s").value))
            self._allowed = False
            self._allowed_stamp = 0.0
            self._health = "UNKNOWN"
            self._health_stamp = 0.0
            self._trajectory = None
            self._trajectory_stamp = 0.0
            self._pub = self.create_publisher(Twist, "/cmd_vel", 10)
            self.create_subscription(Bool, "/localization_authority/motion_allowed", self._authority, 10)
            self.create_subscription(String, "/lio/health", self._health_cb, 10)
            self.create_subscription(TimedTrajectory2D, "/research/ego_trajectory", self._trajectory_cb, 10)
            self.create_timer(0.05, self._tick)

        def _authority(self, msg):
            self._allowed = bool(msg.data)
            self._allowed_stamp = time.monotonic()

        def _health_cb(self, msg):
            self._health = str(msg.data).upper()
            self._health_stamp = time.monotonic()

        def _trajectory_cb(self, msg):
            self._trajectory = msg
            self._trajectory_stamp = time.monotonic()

        def _tick(self):
            now = time.monotonic()
            point = self._trajectory.points[0] if self._trajectory and self._trajectory.points else None
            valid = bool(self._actuator_enabled and self._mode == "live" and point is not None
                         and self._health.startswith("OK") and (now - self._trajectory_stamp) <= 0.25
                         and self._trajectory.status == TimedTrajectory2D.STATUS_OK)
            state = AuthorityState(self._allowed, self._allowed_stamp, now,
                                   "OK" if valid else "UNKNOWN",
                                   trajectory_ok=valid,
                                   map_ok=bool(self._trajectory and self._trajectory.map_version))
            command = self._gate.command(float(point.v) if point else 0.0,
                                         float(point.w) if point else 0.0, state)
            output = Twist()
            output.linear.x, output.angular.z = command.linear_x, command.angular_z
            self._pub.publish(output)


def main() -> int:
    if rclpy:
        rclpy.init()
        node = SafetyBridgeNode()
        try:
            rclpy.spin(node)
        finally:
            node.destroy_node()
            rclpy.shutdown()
        return 0
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--mode", choices=("replay",), default="replay")
    parser.add_argument("--output")
    args, _ = parser.parse_known_args()
    if args.mode != "replay":
        raise SystemExit("research_safety_bridge only permits --mode replay outside ROS/Jetson")
    # replay_sim owns the deterministic harness and its output formatting. It
    # parses only --output, so strip --mode before delegating rather than
    # letting the mode flag reach the second parser and fail the entry point.
    original_argv = sys.argv
    sys.argv = [original_argv[0]]
    if args.output:
        sys.argv.extend(["--output", args.output])
    try:
        return replay_main()
    finally:
        sys.argv = original_argv
