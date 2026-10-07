"""ROS edge placeholder with an actuator-free replay implementation.

The ROS node is intentionally refused when the required ROS transport is not
available. This avoids silently publishing commands from a developer laptop.
"""
from __future__ import annotations
import math

import argparse
import sys
import time
from .replay_sim import main as replay_main
from .authority import AuthorityState, SafetyGate, SafetyCommand
from .operator_gate import OperatorGate, consent_from_message

try:
    import rclpy
    import math
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.clock import Clock, ClockType
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from nav_msgs.msg import OccupancyGrid
    from std_msgs.msg import Bool, String
    from research_interfaces.msg import TimedTrajectory2D, ResearchStatus, OperatorPermit
except ImportError:
    rclpy = None

from .trajectory import TimedPoint, TimedTrajectory, VehicleLimits
from .trajectory_tracker import TrackerState, TimedTrajectoryTracker
from .grid_map import LocalObstacleGrid
from .command_smoother import slew_command
from .physical_parameter_lock import PHYSICAL_LIMITS, LOCKED_FOOTPRINT, require_locked_motion_parameters


def _parameter_bool(value) -> bool:
    """Parse ROS launch substitutions without treating ``\"false\"`` as true."""
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    normalized = str(value).strip().lower()
    if normalized in {"true", "1", "yes", "on"}:
        return True
    if normalized in {"false", "0", "no", "off", ""}:
        return False
    raise ValueError(f"invalid boolean parameter: {value!r}")


def _valid_map_version(value: str) -> bool:
    normalized = str(value).strip()
    return bool(normalized and normalized.upper() != "UNKNOWN")


def rtk_mode_is_allowed(mode, age_s, timeout_s=.5):
    return (mode in ("RTK_AUTHORITATIVE","RTK_REACQUIRING") and
            math.isfinite(age_s) and 0 <= age_s <= timeout_s)


if rclpy:
    class SafetyBridgeNode(Node):
        """Timed trajectory edge; authority and health are fail-closed."""
        def __init__(self):
            super().__init__("research_safety_bridge")
            self.declare_parameter("mode", "replay")
            self.declare_parameter("actuator_enabled", False)
            self.declare_parameter("authority_timeout_s", 0.50)
            self.declare_parameter("health_timeout_s", 0.50)
            self.declare_parameter("map_timeout_s", 0.50)
            self.declare_parameter("grid_timeout_s", 0.50)
            self.declare_parameter("state_timeout_s", 0.20)
            for key, value in PHYSICAL_LIMITS.items():
                self.declare_parameter(key, value)
            require_locked_motion_parameters({key: self.get_parameter(key).value for key in PHYSICAL_LIMITS})
            self.declare_parameter("max_curvature_1pm", 0.0)
            self.declare_parameter("max_lateral_speed_mps", 0.0)
            self.declare_parameter("health_topic", "/lio/vehicle_health")
            self.declare_parameter("odom_topic", "/lio/odom_vehicle")
            self.declare_parameter("obstacle_grid_topic", "/research/local_obstacle_grid")
            self.declare_parameter("map_version_topic", "/research/map_version")
            self.declare_parameter("permission_grid_topic", "/research/permission_grid")
            footprint_parameter = self.declare_parameter("footprint_xy", Parameter.Type.DOUBLE_ARRAY)
            self.declare_parameter("tracker_longitudinal_gain", 0.8)
            self.declare_parameter("tracker_lateral_gain", 1.5)
            self.declare_parameter("tracker_heading_gain", 1.0)
            self.declare_parameter("tracker_preview_s", 0.10)
            self._normal_slew_fraction = float(self.declare_parameter("normal_slew_fraction", 0.50).value)
            if not math.isfinite(self._normal_slew_fraction) or not 0 < self._normal_slew_fraction <= 1:
                raise ValueError("normal_slew_fraction must lie within (0,1]")
            self._mode = str(self.get_parameter("mode").value)
            self._actuator_enabled = _parameter_bool(self.get_parameter("actuator_enabled").value)
            self._gate = SafetyGate(float(self.get_parameter("authority_timeout_s").value))
            self._health_timeout_s = float(self.get_parameter("health_timeout_s").value)
            self._map_timeout_s = float(self.get_parameter("map_timeout_s").value)
            self._grid_timeout_s = float(self.get_parameter("grid_timeout_s").value)
            self._state_timeout_s = float(self.get_parameter("state_timeout_s").value)
            self._last_command = (0.0, 0.0)
            self._last_tick = time.monotonic()
            max_curvature = float(self.get_parameter("max_curvature_1pm").value)
            max_lateral = float(self.get_parameter("max_lateral_speed_mps").value)
            self._limits_configured = (math.isfinite(max_curvature) and max_curvature > 0.0 and
                                       math.isfinite(max_lateral) and max_lateral > 0.0)
            self._tracker = TimedTrajectoryTracker(
                VehicleLimits(
                    max_speed_mps=float(self.get_parameter("max_speed_mps").value),
                    min_speed_mps=float(self.get_parameter("min_speed_mps").value),
                    max_yaw_rate_rps=float(self.get_parameter("max_yaw_rate_rps").value),
                    max_accel_mps2=float(self.get_parameter("max_accel_mps2").value),
                    max_decel_mps2=float(self.get_parameter("max_decel_mps2").value),
                    max_yaw_accel_rps2=float(self.get_parameter("max_yaw_accel_rps2").value),
                    max_yaw_decel_rps2=float(self.get_parameter("max_yaw_decel_rps2").value),
                    max_lateral_accel_mps2=float(self.get_parameter("max_lateral_accel_mps2").value),
                    max_curvature_1pm=max_curvature if self._limits_configured else None,
                    max_lateral_speed_mps=max_lateral),
                longitudinal_gain=float(self.get_parameter("tracker_longitudinal_gain").value),
                lateral_gain=float(self.get_parameter("tracker_lateral_gain").value),
                heading_gain=float(self.get_parameter("tracker_heading_gain").value),
                preview_s=float(self.get_parameter("tracker_preview_s").value))
            self._allowed = False
            self._operator_gate = OperatorGate()
            self._allowed_stamp = 0.0
            self._authority_mode,self._authority_mode_stamp = "UNKNOWN",0.0
            self._health = "UNKNOWN"
            self._health_stamp = 0.0
            self._trajectory = None
            self._trajectory_contract = None
            self._trajectory_stamp = 0.0
            self._state = None
            self._state_stamp = 0.0
            self._map_version = ""
            self._map_stamp = 0.0
            self._grid = None
            self._grid_stamp = 0.0
            self._footprint = self._parse_footprint(footprint_parameter.value)
            if self._footprint != LOCKED_FOOTPRINT:
                raise ValueError("protected corridor footprint override rejected")
            self._permission, self._permission_stamp = None, 0.0
            self._tf_valid, self._tf_stamp = False, 0.0
            self._pub = self.create_publisher(Twist, "/cmd_vel", 10)
            self._status_pub = self.create_publisher(ResearchStatus, "/research/status", 10)
            self.create_subscription(Bool, "/localization_authority/motion_allowed", self._authority, 10)
            self.create_subscription(String,"/localization_authority/mode",self._authority_mode_cb,10)
            self.create_subscription(String, str(self.get_parameter("health_topic").value), self._health_cb, 10)
            self.create_subscription(Odometry, str(self.get_parameter("odom_topic").value), self._odom_cb, 10)
            self.create_subscription(OccupancyGrid, str(self.get_parameter("obstacle_grid_topic").value), self._grid_cb, 10)
            self.create_subscription(String, str(self.get_parameter("map_version_topic").value), self._map_version_cb, 10)
            self.create_subscription(TimedTrajectory2D, "/research/ego_trajectory", self._trajectory_cb, 10)
            self.create_subscription(OccupancyGrid, str(self.get_parameter("permission_grid_topic").value),
                                     self._permission_cb, 10)
            self.create_subscription(Bool, "/research/tf_integrity", self._tf_cb, 10)
            self.create_subscription(OperatorPermit, "/research/operator_permit", self._operator_cb, 10)
            self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
            self.create_timer(0.05, self._tick, clock=self._steady_clock)

        def _operator_cb(self, msg):
            age = self.get_clock().now().nanoseconds*1e-9-self._time_seconds(msg.header.stamp)
            if msg.header.frame_id == "odom" and 0 <= age <= .5:
                self._operator_gate.receive(consent_from_message(msg), time.monotonic())

        def _authority(self, msg):
            self._allowed = bool(msg.data)
            self._allowed_stamp = time.monotonic()

        def _authority_mode_cb(self,msg):
            self._authority_mode,self._authority_mode_stamp=str(msg.data),time.monotonic()

        def _health_cb(self, msg):
            self._health = str(msg.data).upper()
            self._health_stamp = time.monotonic()

        def _odom_cb(self, msg):
            q = msg.pose.pose.orientation
            norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
            stamp = self._time_seconds(msg.header.stamp)
            age = self.get_clock().now().nanoseconds * 1e-9 - stamp
            if (msg.header.frame_id != "odom" or msg.child_frame_id != "base_footprint" or
                    not math.isfinite(norm) or norm <= 1e-12 or stamp <= 0 or
                    age < -0.10 or age > self._state_timeout_s):
                self._state = None
                self._state_stamp = 0.0
                return
            qx, qy, qz, qw = q.x/norm, q.y/norm, q.z/norm, q.w/norm
            sin_yaw = 2.0 * (qw * qz + qx * qy)
            cos_yaw = 1.0 - 2.0 * (qy * qy + qz * qz)
            yaw = math.atan2(sin_yaw, cos_yaw)
            values = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
            if all(math.isfinite(float(value)) for value in values):
                self._state = TrackerState(*values)
                self._state_stamp = time.monotonic()
            else:
                self._state = None
                self._state_stamp = 0.0

        @staticmethod
        def _parse_footprint(values):
            try:
                flat = [float(value) for value in values]
            except (TypeError, ValueError):
                return ()
            if len(flat) < 6 or len(flat) % 2 or not all(math.isfinite(value) for value in flat):
                return ()
            return tuple((flat[index], flat[index + 1]) for index in range(0, len(flat), 2))

        def _map_version_cb(self, msg):
            value = str(msg.data).strip()
            self._map_version = value if _valid_map_version(value) else "UNKNOWN"
            self._map_stamp = time.monotonic()

        def _grid_cb(self, msg):
            self._grid = None
            self._grid_stamp = 0.0
            try:
                stamp = self._time_seconds(msg.header.stamp)
                age = self.get_clock().now().nanoseconds * 1e-9 - stamp
                if (msg.header.frame_id != "odom" or not _valid_map_version(self._map_version) or
                        stamp <= 0 or age < -0.10 or age > self._grid_timeout_s):
                    self._grid = None
                    return
                origin_q = msg.info.origin.orientation
                if (abs(float(origin_q.x)) > 1e-9 or abs(float(origin_q.y)) > 1e-9 or
                        abs(float(origin_q.z)) > 1e-9 or abs(float(origin_q.w) - 1.0) > 1e-9):
                    self._grid = None
                    return
                cells = tuple(-1 if int(value) < 0 else 100 if int(value) >= 50 else 0
                              for value in msg.data)
                self._grid = LocalObstacleGrid(
                    frame_id=msg.header.frame_id,
                    map_version=self._map_version,
                    resolution_m=float(msg.info.resolution),
                    origin_x_m=float(msg.info.origin.position.x),
                    origin_y_m=float(msg.info.origin.position.y),
                    width=int(msg.info.width), height=int(msg.info.height),
                    cells=cells, unknown_is_occupied=True)
                self._grid_stamp = time.monotonic()
            except (TypeError, ValueError):
                self._grid = None

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

        def _permission_cb(self, msg):
            self._permission, self._permission_stamp = None, 0.0
            try:
                q = msg.info.origin.orientation
                age = self.get_clock().now().nanoseconds*1e-9-self._time_seconds(msg.header.stamp)
                if (msg.header.frame_id != "odom" or not 0 <= age <= self._grid_timeout_s or
                        abs(q.x)+abs(q.y)+abs(q.z) > 1e-9 or abs(q.w-1) > 1e-9):
                    return
                # Only an explicitly permitted cell is allowed. No-data and
                # every nonzero value stay forbidden; this is independent of
                # observed support and cannot open a legacy keepout area.
                self._permission = LocalObstacleGrid("odom", self._map_version,
                    msg.info.resolution, msg.info.origin.position.x, msg.info.origin.position.y,
                    msg.info.width, msg.info.height, tuple(0 if v == 0 else 100 for v in msg.data))
                self._permission_stamp = time.monotonic()
            except (ValueError, TypeError):
                pass

        def _tf_cb(self, msg):
            self._tf_valid, self._tf_stamp = bool(msg.data), time.monotonic()

        def _tf_ready(self, now):
            return self._tf_valid and now-self._tf_stamp <= self._state_timeout_s

        def _occupied_polygon(self, polygon, margin_m=0.0):
            return (self._grid.polygon_occupied(polygon, margin_m) or self._permission is None or
                    self._permission.polygon_occupied(polygon, margin_m))

        def _tick(self):
            now = time.monotonic()
            ros_now = self.get_clock().now().nanoseconds * 1e-9
            tracked = None
            map_fresh = self._map_stamp > 0.0 and now - self._map_stamp <= self._map_timeout_s
            grid_fresh = self._grid_stamp > 0.0 and now - self._grid_stamp <= self._grid_timeout_s
            grid_ready = bool(self._grid is not None and self._trajectory_contract is not None and
                              self._grid.map_version == self._map_version and
                              _valid_map_version(self._map_version) and map_fresh and grid_fresh and
                              self._map_version == self._trajectory_contract.map_version and
                              self._footprint and self._limits_configured and self._tf_ready(now) and
                              self._permission is not None and
                              now-self._permission_stamp <= self._grid_timeout_s)
            if self._trajectory_contract is not None and self._state is not None and grid_ready:
                tracked = self._tracker.command(
                    self._trajectory_contract, self._state, now=ros_now,
                    expected_map_version=self._map_version,
                    footprint=self._footprint, occupied=self._grid.occupied,
                    resolution=self._grid.resolution_m,
                    occupied_polygon=self._occupied_polygon)
            health_fresh = self._health_stamp > 0.0 and now - self._health_stamp <= self._health_timeout_s
            health_ok = health_fresh and self._health.startswith("OK")
            valid = bool(self._actuator_enabled and self._mode == "live"
                         and tracked is not None and health_ok
                         and (now - self._trajectory_stamp) <= 0.25
                         and (now - self._state_stamp) <= 0.25
                         and self._trajectory is not None
                         and self._trajectory.status == TimedTrajectory2D.STATUS_OK)
            operator_ok = self._operator_gate.allowed(now, mode=self._mode, map_version=self._map_version,
                sole_publisher=self.count_publishers("/research/operator_permit") == 1)
            valid = valid and operator_ok
            rtk_mode_ok=rtk_mode_is_allowed(self._authority_mode,now-self._authority_mode_stamp,
                float(self.get_parameter("authority_timeout_s").value))
            state = AuthorityState(self._allowed and rtk_mode_ok, self._allowed_stamp, now,
                                   "OK" if health_ok else "UNKNOWN",
                                   trajectory_ok=valid,
                                   map_ok=grid_ready)
            command = self._gate.command(tracked.linear_x if tracked else 0.0,
                                         tracked.angular_z if tracked else 0.0, state)
            if command.allowed:
                self._last_command = slew_command(self._last_command,
                    (command.linear_x, command.angular_z), now - self._last_tick, self._tracker.limits,
                    self._normal_slew_fraction)
                curvature_limit=self._tracker.limits.max_curvature_1pm
                wire_v,wire_w=(float(f"{value:.3f}") for value in self._last_command)
                if (abs(wire_v*wire_w)>self._tracker.limits.max_lateral_accel_mps2+1e-12 or
                    (curvature_limit is not None and abs(wire_w)>curvature_limit*abs(wire_v)+1e-12)):
                    self._last_command=(0.,0.)
                    command=SafetyCommand(0.,0.,False,"post_slew_curvature_rejected")
            else:
                # Authority and safety stops retain immediate stop precedence.
                self._last_command = (0.0, 0.0)
            self._last_tick = now
            output = Twist()
            output.linear.x, output.angular.z = self._last_command
            self._pub.publish(output)
            status = ResearchStatus()
            status.header.frame_id = "odom"
            status.header.stamp = self.get_clock().now().to_msg()
            status.mode, status.map_version, status.lio_health = self._mode, self._map_version, self._health
            status.state = "TRACKING" if command.allowed else "STOPPED"
            status.motion_allowed, status.actuator_enabled = self._allowed and rtk_mode_ok, self._actuator_enabled
            status.reason = command.reason
            if not operator_ok: status.reason += ";operator_consent_missing_stale_or_denied"
            if not rtk_mode_ok:status.reason+=";rtk_mode_or_age_denied:"+self._authority_mode
            if not grid_ready:
                detail=[]
                if not map_fresh:detail.append("map_stale")
                if not grid_fresh:detail.append("grid_stale")
                if not self._tf_ready(now):detail.append("tf_integrity_or_age")
                if self._permission is None or now-self._permission_stamp>self._grid_timeout_s:detail.append("permission_missing_or_age")
                if self._grid is not None and self._grid.map_version!=self._map_version:detail.append("grid_version")
                if self._trajectory_contract is not None and self._trajectory_contract.map_version!=self._map_version:detail.append("trajectory_version")
                if detail:status.reason+=":"+",".join(detail)
            if tracked is None and self._tracker.last_rejection:
                status.reason += ":tracker:" + self._tracker.last_rejection
            self._status_pub.publish(status)


def main() -> int:
    if rclpy:
        rclpy.init()
        node = SafetyBridgeNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            if rclpy.ok():
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
