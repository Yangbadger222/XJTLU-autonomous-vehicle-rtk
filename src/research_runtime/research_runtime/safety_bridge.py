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
    import math
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool, String
    from research_interfaces.msg import TimedTrajectory2D
except ImportError:
    rclpy = None

from .trajectory import TimedPoint, TimedTrajectory, VehicleLimits
from .trajectory_tracker import TrackerState, TimedTrajectoryTracker


if rclpy:
    class SafetyBridgeNode(Node):
        """Timed trajectory edge; authority and health are fail-closed."""
        def __init__(self):
            super().__init__("research_safety_bridge")
            self.declare_parameter("mode", "replay")
            self.declare_parameter("actuator_enabled", False)
            self.declare_parameter("authority_timeout_s", 0.50)
            self.declare_parameter("health_timeout_s", 0.50)
            self.declare_parameter("health_topic", "/lio/vehicle_health")
            self.declare_parameter("odom_topic", "/lio/odom_vehicle")
            self.declare_parameter("tracker_longitudinal_gain", 0.8)
            self.declare_parameter("tracker_lateral_gain", 1.5)
            self.declare_parameter("tracker_heading_gain", 1.0)
            self._mode = str(self.get_parameter("mode").value)
            self._actuator_enabled = bool(self.get_parameter("actuator_enabled").value)
            self._gate = SafetyGate(float(self.get_parameter("authority_timeout_s").value))
            self._health_timeout_s = float(self.get_parameter("health_timeout_s").value)
            self._tracker = TimedTrajectoryTracker(
                VehicleLimits(),
                longitudinal_gain=float(self.get_parameter("tracker_longitudinal_gain").value),
                lateral_gain=float(self.get_parameter("tracker_lateral_gain").value),
                heading_gain=float(self.get_parameter("tracker_heading_gain").value))
            self._allowed = False
            self._allowed_stamp = 0.0
            self._health = "UNKNOWN"
            self._health_stamp = 0.0
            self._trajectory = None
            self._trajectory_contract = None
            self._trajectory_stamp = 0.0
            self._state = None
            self._state_stamp = 0.0
            self._pub = self.create_publisher(Twist, "/cmd_vel", 10)
            self.create_subscription(Bool, "/localization_authority/motion_allowed", self._authority, 10)
            self.create_subscription(String, str(self.get_parameter("health_topic").value), self._health_cb, 10)
            self.create_subscription(Odometry, str(self.get_parameter("odom_topic").value), self._odom_cb, 10)
            self.create_subscription(TimedTrajectory2D, "/research/ego_trajectory", self._trajectory_cb, 10)
            self.create_timer(0.05, self._tick)

        def _authority(self, msg):
            self._allowed = bool(msg.data)
            self._allowed_stamp = time.monotonic()

        def _health_cb(self, msg):
            self._health = str(msg.data).upper()
            self._health_stamp = time.monotonic()

        def _odom_cb(self, msg):
            q = msg.pose.pose.orientation
            sin_yaw = 2.0 * (q.w * q.z + q.x * q.y)
            cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(sin_yaw, cos_yaw)
            values = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
            if all(math.isfinite(float(value)) for value in values):
                self._state = TrackerState(*values)
                self._state_stamp = time.monotonic()

        @staticmethod
        def _time_seconds(stamp):
            return float(stamp.sec) + 1e-9 * float(stamp.nanosec)

        def _trajectory_cb(self, msg):
            self._trajectory = msg
            self._trajectory_stamp = time.monotonic()
            try:
                points = tuple(TimedPoint(point.t, point.x, point.y, point.yaw,
                                           point.v, point.w, point.a, point.alpha,
                                           point.curvature) for point in msg.points)
                self._trajectory_contract = TimedTrajectory.from_points(
                    msg.trajectory_id, msg.map_version, msg.header.frame_id,
                    self._time_seconds(msg.generated_at),
                    self._time_seconds(msg.valid_until), points)
            except (TypeError, ValueError):
                self._trajectory_contract = None

        def _tick(self):
            now = time.monotonic()
            ros_now = self.get_clock().now().nanoseconds * 1e-9
            tracked = None
            if self._trajectory_contract is not None and self._state is not None:
                tracked = self._tracker.command(self._trajectory_contract, self._state, now=ros_now)
            health_fresh = self._health_stamp > 0.0 and now - self._health_stamp <= self._health_timeout_s
            health_ok = health_fresh and self._health.startswith("OK")
            valid = bool(self._actuator_enabled and self._mode == "live"
                         and tracked is not None and health_ok
                         and (now - self._trajectory_stamp) <= 0.25
                         and (now - self._state_stamp) <= 0.25
                         and self._trajectory is not None
                         and self._trajectory.status == TimedTrajectory2D.STATUS_OK)
            state = AuthorityState(self._allowed, self._allowed_stamp, now,
                                   "OK" if health_ok else "UNKNOWN",
                                   trajectory_ok=valid,
                                   map_ok=bool(self._trajectory and self._trajectory.map_version))
            command = self._gate.command(tracked.linear_x if tracked else 0.0,
                                         tracked.angular_z if tracked else 0.0, state)
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
