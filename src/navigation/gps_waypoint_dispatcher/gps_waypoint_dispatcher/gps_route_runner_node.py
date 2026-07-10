#!/usr/bin/env python3
from __future__ import annotations

import math
import time
from dataclasses import dataclass
from pathlib import Path as FSPath

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Float64MultiArray, String
from tf2_ros import Buffer, TransformException, TransformListener

from gps_waypoint_dispatcher.nav2_lifecycle_ready import (
    DEFAULT_REQUIRED_NAV2_LIFECYCLE_NODES,
    normalize_lifecycle_node_names,
    summarize_lifecycle_states,
)
from gps_waypoint_dispatcher.route_safety import (
    ContinuousReadiness,
    GlobalCorrectionWatchdog,
    LocalOdomWatchdog,
    WatchdogDecision,
    summarize_nav2_success_progress,
    summarize_map_gps_consistency,
    summarize_tf_freshness,
)
from gps_waypoint_dispatcher.scene_runtime import (
    FixedENUProjector,
    default_route_file,
    haversine_m,
    quaternion_to_yaw,
    yaw_to_quaternion,
)


def valid_fix(msg: NavSatFix | None) -> bool:
    if msg is None:
        return False
    if msg.status.status < 0:
        return False
    if not math.isfinite(msg.latitude) or not math.isfinite(msg.longitude):
        return False
    return True


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


GOAL_STATUS_ALIGNMENT_SHIFT = -100
GOAL_STATUS_GLOBAL_CORRECTION_HOLD = -101


@dataclass
class Alignment2D:
    theta: float
    tx: float
    ty: float
    source: str
    revision: int


@dataclass
class RouteWaypoint:
    name: str
    lat: float
    lon: float
    alt: float
    enu_x: float
    enu_y: float


@dataclass
class SegmentPlan:
    waypoint: RouteWaypoint
    start_enu: tuple[float, float]
    end_enu: tuple[float, float]
    total_length_m: float
    total_subgoals: int
    dir_x: float
    dir_y: float


class GPSRouteRunner(Node):
    def __init__(self) -> None:
        super().__init__("gps_route_runner")
        self.declare_parameter("route_file", str(default_route_file()))
        self.declare_parameter("route_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("fix_topic", "/fix")
        self.declare_parameter("alignment_topic", "/gps_corridor/enu_to_map")
        self.declare_parameter("stop_override_topic", "/gps_corridor/stop_override")
        self.declare_parameter(
            "motion_allowed_topic", "/localization_authority/motion_allowed"
        )
        self.declare_parameter(
            "authority_status_topic", "/localization_authority/status"
        )
        self.declare_parameter("lio_odom_topic", "/fastlio2/lio_odom")
        self.declare_parameter("stop_override_publish_hz", 10.0)
        self.declare_parameter("cancel_ack_timeout_s", 2.0)
        self.declare_parameter("authority_ready_confirmation_s", 1.0)
        self.declare_parameter("global_hold_timeout_s", 15.0)
        self.declare_parameter("terminal_stop_hold_s", 1.2)
        self.declare_parameter("terminal_stop_publish_hz", 20.0)
        self.declare_parameter("startup_wait_timeout_s", 90.0)
        self.declare_parameter("enu_origin_lat", 0.0)
        self.declare_parameter("enu_origin_lon", 0.0)
        self.declare_parameter("enu_origin_alt", 0.0)
        self.declare_parameter("odom_watchdog_monitor_period_s", 0.1)
        self.declare_parameter("local_rate_abort_mps", 3.0)
        self.declare_parameter("local_yaw_rate_abort_radps", 3.0)
        self.declare_parameter("local_rate_abort_count", 3)
        self.declare_parameter("local_catastrophic_rate_mps", 10.0)
        self.declare_parameter("local_catastrophic_yaw_rate_radps", 10.0)
        self.declare_parameter("global_correction_rate_mps", 0.50)
        self.declare_parameter("global_correction_yaw_rate_degps", 5.0)
        self.declare_parameter("motion_authority_max_age_s", 0.50)
        self.declare_parameter("tf_pose_max_age_s", 3.0)
        self.declare_parameter("alignment_shift_cancel_threshold_m", 0.5)
        self.declare_parameter("alignment_shift_cooldown_s", 3.0)
        self.declare_parameter("map_gps_divergence_warn_m", 2.0)
        self.declare_parameter("map_gps_divergence_abort_m", 5.0)
        self.declare_parameter(
            "nav2_lifecycle_nodes",
            list(DEFAULT_REQUIRED_NAV2_LIFECYCLE_NODES),
        )

        self._route_file = FSPath(self.get_parameter("route_file").value).expanduser()
        self._route_frame = str(self.get_parameter("route_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._fix_topic = str(self.get_parameter("fix_topic").value)
        self._alignment_topic = str(self.get_parameter("alignment_topic").value)
        self._stop_override_topic = str(
            self.get_parameter("stop_override_topic").value
        )
        self._motion_allowed_topic = str(
            self.get_parameter("motion_allowed_topic").value
        )
        self._authority_status_topic = str(
            self.get_parameter("authority_status_topic").value
        )
        self._lio_odom_topic = str(self.get_parameter("lio_odom_topic").value)
        self._stop_override_publish_hz = float(
            self.get_parameter("stop_override_publish_hz").value
        )
        self._cancel_ack_timeout_s = float(
            self.get_parameter("cancel_ack_timeout_s").value
        )
        self._authority_ready_confirmation_s = float(
            self.get_parameter("authority_ready_confirmation_s").value
        )
        self._global_hold_timeout_s = float(
            self.get_parameter("global_hold_timeout_s").value
        )
        self._terminal_stop_hold_s = max(
            0.0, float(self.get_parameter("terminal_stop_hold_s").value)
        )
        self._terminal_stop_publish_hz = max(
            1.0, float(self.get_parameter("terminal_stop_publish_hz").value)
        )
        self._startup_wait_timeout_s = float(self.get_parameter("startup_wait_timeout_s").value)
        self._enu_origin_lat = float(self.get_parameter("enu_origin_lat").value)
        self._enu_origin_lon = float(self.get_parameter("enu_origin_lon").value)
        self._enu_origin_alt = float(self.get_parameter("enu_origin_alt").value)
        self._odom_watchdog_monitor_period_s = float(
            self.get_parameter("odom_watchdog_monitor_period_s").value
        )
        self._motion_authority_max_age_s = float(
            self.get_parameter("motion_authority_max_age_s").value
        )
        self._tf_pose_max_age_s = float(self.get_parameter("tf_pose_max_age_s").value)
        self._alignment_shift_cancel_threshold_m = float(
            self.get_parameter("alignment_shift_cancel_threshold_m").value
        )
        self._alignment_shift_cooldown_s = float(
            self.get_parameter("alignment_shift_cooldown_s").value
        )
        self._map_gps_divergence_warn_m = float(
            self.get_parameter("map_gps_divergence_warn_m").value
        )
        self._map_gps_divergence_abort_m = float(
            self.get_parameter("map_gps_divergence_abort_m").value
        )
        raw_lifecycle_nodes = self.get_parameter("nav2_lifecycle_nodes").value
        self._nav2_lifecycle_nodes = normalize_lifecycle_node_names(raw_lifecycle_nodes)
        self._last_alignment_shift_mono = -math.inf

        self._status_pub = self.create_publisher(String, "/gps_corridor/status", 10)
        self._goal_pub = self.create_publisher(PoseStamped, "/gps_corridor/goal_map", 10)
        self._path_pub = self.create_publisher(NavPath, "/gps_corridor/path_map", 10)
        self._stop_override_pub = self.create_publisher(
            Bool, self._stop_override_topic, 10
        )
        self._fix_sub = self.create_subscription(NavSatFix, self._fix_topic, self._fix_callback, 10)
        self._alignment_sub = self.create_subscription(
            Float64MultiArray, self._alignment_topic, self._alignment_callback, 10
        )
        self._motion_allowed_sub = self.create_subscription(
            Bool,
            self._motion_allowed_topic,
            self._motion_allowed_callback,
            10,
        )
        self._authority_status_sub = self.create_subscription(
            String,
            self._authority_status_topic,
            self._authority_status_callback,
            10,
        )
        self._lio_odom_sub = self.create_subscription(
            Odometry, self._lio_odom_topic, self._lio_odom_callback, 50
        )

        self._latest_fix: NavSatFix | None = None
        self._last_fix_key: tuple | None = None
        self._latest_alignment: Alignment2D | None = None
        self._alignment_revision = 0
        self._stop_override = True
        self._motion_allowed = False
        self._motion_allowed_mono: float | None = None
        self._authority_status = "STARTUP"
        self._local_watchdog = LocalOdomWatchdog(
            ordinary_linear_rate_mps=float(
                self.get_parameter("local_rate_abort_mps").value
            ),
            ordinary_yaw_rate_radps=float(
                self.get_parameter("local_yaw_rate_abort_radps").value
            ),
            ordinary_abort_count=int(
                self.get_parameter("local_rate_abort_count").value
            ),
            catastrophic_linear_rate_mps=float(
                self.get_parameter("local_catastrophic_rate_mps").value
            ),
            catastrophic_yaw_rate_radps=float(
                self.get_parameter("local_catastrophic_yaw_rate_radps").value
            ),
        )
        self._local_watchdog_result = None
        self._global_watchdog = GlobalCorrectionWatchdog(
            max_translation_rate_mps=float(
                self.get_parameter("global_correction_rate_mps").value
            ),
            max_yaw_rate_radps=math.radians(
                float(self.get_parameter("global_correction_yaw_rate_degps").value)
            ),
            max_authority_age_s=self._motion_authority_max_age_s,
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._nav2_state_clients = {
            node_name: self.create_client(GetState, f"/{node_name}/get_state")
            for node_name in self._nav2_lifecycle_nodes
        }

        self._projector = FixedENUProjector(
            self._enu_origin_lat, self._enu_origin_lon, self._enu_origin_alt
        )
        self._route = self._load_route(self._route_file)
        self._stop_override_timer = self.create_timer(
            1.0 / self._stop_override_publish_hz,
            self._publish_stop_override_heartbeat,
        )
        self._publish_stop_override(True)

    def _publish_status(self, text: str) -> None:
        self.get_logger().info(text)
        self._status_pub.publish(String(data=text))

    def _fix_callback(self, msg: NavSatFix) -> None:
        self._latest_fix = msg

    def _motion_allowed_callback(self, msg: Bool) -> None:
        self._motion_allowed = bool(msg.data)
        self._motion_allowed_mono = time.monotonic()

    def _authority_status_callback(self, msg: String) -> None:
        self._authority_status = msg.data

    def _lio_odom_callback(self, msg: Odometry) -> None:
        stamp_s = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        yaw = quaternion_to_yaw(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        self._local_watchdog_result = self._local_watchdog.update(
            stamp_s,
            float(position.x),
            float(position.y),
            yaw,
        )

    def _alignment_callback(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 4 or msg.data[3] < 0.5:
            return
        theta, tx, ty = float(msg.data[0]), float(msg.data[1]), float(msg.data[2])
        current = self._latest_alignment
        if (
            current is not None
            and abs(current.theta - theta) < 1e-6
            and abs(current.tx - tx) < 1e-4
            and abs(current.ty - ty) < 1e-4
        ):
            return
        self._alignment_revision += 1
        self._latest_alignment = Alignment2D(
            theta=theta,
            tx=tx,
            ty=ty,
            source="aligner",
            revision=self._alignment_revision,
        )
        self.get_logger().info(
            "Received valid ENU->map transform: theta=%.2fdeg tx=%.2f ty=%.2f"
            % (math.degrees(theta), tx, ty)
        )

    def _load_route(self, path: FSPath) -> dict:
        if not path.exists():
            raise RuntimeError(f"route file not found: {path}")
        with open(path, "r", encoding="utf-8") as route_file:
            data = yaml.safe_load(route_file) or {}

        for key in ("enu_origin", "start_ref", "waypoints", "launch_yaw_deg"):
            if key not in data:
                raise RuntimeError(f"missing key in route file: {key}")
        if not isinstance(data["waypoints"], list) or not data["waypoints"]:
            raise RuntimeError("route file has no waypoints")

        self._validate_origin_match(data["enu_origin"])

        start_ref = data["start_ref"]
        start_ref["lat"] = float(start_ref["lat"])
        start_ref["lon"] = float(start_ref["lon"])
        start_ref["alt"] = float(start_ref.get("alt", 0.0))
        start_ref["enu_x"], start_ref["enu_y"] = self._projector.forward(
            start_ref["lat"], start_ref["lon"]
        )

        waypoints: list[RouteWaypoint] = []
        for index, raw_waypoint in enumerate(data["waypoints"], start=1):
            lat = float(raw_waypoint["lat"])
            lon = float(raw_waypoint["lon"])
            alt = float(raw_waypoint.get("alt", 0.0))
            enu_x, enu_y = self._projector.forward(lat, lon)
            waypoints.append(
                RouteWaypoint(
                    name=str(raw_waypoint.get("name", f"wp{index}")),
                    lat=lat,
                    lon=lon,
                    alt=alt,
                    enu_x=enu_x,
                    enu_y=enu_y,
                )
            )

        data["launch_yaw_deg"] = float(data["launch_yaw_deg"])
        data["start_ref"] = start_ref
        data["waypoints"] = waypoints
        return data

    def _validate_origin_match(self, enu_origin: dict) -> None:
        route_lat = float(enu_origin["lat"])
        route_lon = float(enu_origin["lon"])
        route_alt = float(enu_origin.get("alt", 0.0))
        if abs(route_lat - self._enu_origin_lat) > 1e-7:
            raise RuntimeError(
                f"route enu_origin.lat {route_lat:.7f} != runtime {self._enu_origin_lat:.7f}"
            )
        if abs(route_lon - self._enu_origin_lon) > 1e-7:
            raise RuntimeError(
                f"route enu_origin.lon {route_lon:.7f} != runtime {self._enu_origin_lon:.7f}"
            )
        if abs(route_alt - self._enu_origin_alt) > 0.1:
            raise RuntimeError(
                f"route enu_origin.alt {route_alt:.2f} != runtime {self._enu_origin_alt:.2f}"
            )

    def _sample_key(self, msg: NavSatFix) -> tuple:
        return (
            msg.header.stamp.sec,
            msg.header.stamp.nanosec,
            round(msg.latitude, 9),
            round(msg.longitude, 9),
            round(float(msg.altitude), 4),
        )

    def _wait_for_stable_fix(self) -> dict:
        sample_count = int(self._route.get("startup_fix_sample_count", 10))
        spread_limit = float(self._route.get("startup_fix_spread_max_m", 2.0))
        route_timeout_s = float(
            self._route.get("startup_fix_timeout_s", self._startup_wait_timeout_s)
        )
        timeout_s = max(route_timeout_s, self._startup_wait_timeout_s)
        deadline = time.time() + timeout_s
        samples: list[tuple[float, float, float]] = []
        self._publish_status("WAITING_FOR_STABLE_FIX")

        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            msg = self._latest_fix
            if not valid_fix(msg):
                continue
            key = self._sample_key(msg)
            if key == self._last_fix_key:
                continue
            self._last_fix_key = key
            samples.append((msg.latitude, msg.longitude, float(msg.altitude)))
            if len(samples) < sample_count:
                continue
            if len(samples) > sample_count:
                samples = samples[-sample_count:]

            max_spread = 0.0
            for i in range(len(samples)):
                for j in range(i + 1, len(samples)):
                    spread = haversine_m(
                        samples[i][0],
                        samples[i][1],
                        samples[j][0],
                        samples[j][1],
                    )
                    max_spread = max(max_spread, spread)
            if max_spread > spread_limit:
                continue

            return {
                "lat": sum(sample[0] for sample in samples) / len(samples),
                "lon": sum(sample[1] for sample in samples) / len(samples),
                "alt": sum(sample[2] for sample in samples) / len(samples),
                "samples": len(samples),
                "spread_m": round(max_spread, 2),
            }

        raise RuntimeError("timed out waiting for stable /fix samples")

    def _validate_startup(self, startup_fix: dict) -> float:
        start_ref = self._route["start_ref"]
        distance_m = haversine_m(
            startup_fix["lat"],
            startup_fix["lon"],
            start_ref["lat"],
            start_ref["lon"],
        )
        tolerance_m = float(self._route.get("startup_gps_tolerance_m", 15.0))
        self.get_logger().info(
            "Startup fix mean lat=%.7f lon=%.7f spread=%.2fm distance_to_start_ref=%.2fm"
            % (
                startup_fix["lat"],
                startup_fix["lon"],
                startup_fix["spread_m"],
                distance_m,
            )
        )
        if distance_m > tolerance_m:
            raise RuntimeError(
                f"startup GPS is {distance_m:.2f}m from start_ref (limit {tolerance_m:.2f}m)"
            )
        return distance_m

    def _wait_for_nav2(self) -> None:
        self._publish_status("WAITING_FOR_NAV2")
        deadline = time.time() + self._startup_wait_timeout_s
        action_ready = False
        while rclpy.ok() and time.time() < deadline:
            if not action_ready:
                action_ready = self._nav_client.wait_for_server(timeout_sec=1.0)
            if action_ready and self._nav2_lifecycle_ready():
                self.get_logger().info("Nav2 action server and lifecycle nodes are active")
                return
            rclpy.spin_once(self, timeout_sec=0.2)
        raise RuntimeError("navigate_to_pose action server or lifecycle nodes not available")

    def _nav2_lifecycle_ready(self) -> bool:
        if not self._nav2_lifecycle_nodes:
            return True

        state_by_node: dict[str, int] = {}
        for node_name, client in self._nav2_state_clients.items():
            if not client.wait_for_service(timeout_sec=0.1):
                continue
            future = client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.3)
            if not future.done() or future.result() is None:
                continue
            state_by_node[node_name] = int(future.result().current_state.id)

        summary = summarize_lifecycle_states(self._nav2_lifecycle_nodes, state_by_node)
        if not summary.ready:
            self.get_logger().debug(
                "Waiting for Nav2 lifecycle active: missing=%s inactive=%s"
                % (list(summary.missing_nodes), list(summary.inactive_nodes))
            )
        return summary.ready

    def _wait_for_alignment(self) -> Alignment2D:
        self._publish_status("WAITING_FOR_ALIGNMENT")
        deadline = time.time() + self._startup_wait_timeout_s
        while rclpy.ok() and time.time() < deadline:
            if self._latest_alignment is not None:
                return self._latest_alignment
            rclpy.spin_once(self, timeout_sec=0.2)
        raise RuntimeError("timed out waiting for valid ENU->map alignment")

    def _lookup_current_pose(self, announce_wait: bool = False) -> tuple[float, float, float]:
        deadline = time.time() + self._startup_wait_timeout_s
        if announce_wait:
            self._publish_status("WAITING_FOR_MAP_TF")
        last_stale_age_s: float | None = None
        while rclpy.ok() and time.time() < deadline:
            try:
                transform = self._tf_buffer.lookup_transform(
                    self._route_frame,
                    self._base_frame,
                    Time(),
                    timeout=Duration(seconds=0.5),
                )
                pose = self._pose_from_transform(transform)
                if pose is not None:
                    return pose
                stamp_s = self._stamp_to_seconds(transform.header.stamp)
                last_stale_age_s = self._now_seconds() - stamp_s
            except TransformException:
                pass
            rclpy.spin_once(self, timeout_sec=0.2)
        if last_stale_age_s is not None:
            raise RuntimeError(
                "timed out waiting for fresh TF %s->%s (last age %.2fs, max %.2fs)"
                % (
                    self._route_frame,
                    self._base_frame,
                    last_stale_age_s,
                    self._tf_pose_max_age_s,
                )
            )
        raise RuntimeError(f"timed out waiting for TF {self._route_frame}->{self._base_frame}")

    def _current_xy(self) -> tuple[float, float]:
        x, y, _ = self._lookup_current_pose(announce_wait=False)
        return x, y

    def _try_lookup_current_pose(
        self, timeout_s: float = 0.05
    ) -> tuple[float, float, float] | None:
        try:
            transform = self._tf_buffer.lookup_transform(
                self._route_frame,
                self._base_frame,
                Time(),
                timeout=Duration(seconds=timeout_s),
            )
        except TransformException:
            return None
        return self._pose_from_transform(transform)

    def _stamp_to_seconds(self, stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _now_seconds(self) -> float:
        return float(self.get_clock().now().nanoseconds) * 1e-9

    def _pose_from_transform(self, transform) -> tuple[float, float, float] | None:
        stamp_s = self._stamp_to_seconds(transform.header.stamp)
        freshness = summarize_tf_freshness(
            now_s=self._now_seconds(),
            stamp_s=stamp_s,
            max_age_s=self._tf_pose_max_age_s,
        )
        if not freshness.ok:
            self.get_logger().debug(
                "Ignoring stale TF %s->%s age=%.2fs max=%.2fs"
                % (
                    self._route_frame,
                    self._base_frame,
                    freshness.age_s,
                    self._tf_pose_max_age_s,
                )
            )
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = quaternion_to_yaw(rotation.x, rotation.y, rotation.z, rotation.w)
        return float(translation.x), float(translation.y), yaw

    def _enu_to_map(self, enu_x: float, enu_y: float, alignment: Alignment2D) -> tuple[float, float]:
        cos_theta = math.cos(alignment.theta)
        sin_theta = math.sin(alignment.theta)
        map_x = cos_theta * enu_x - sin_theta * enu_y + alignment.tx
        map_y = sin_theta * enu_x + cos_theta * enu_y + alignment.ty
        return map_x, map_y

    def _map_to_enu(self, map_x: float, map_y: float, alignment: Alignment2D) -> tuple[float, float]:
        dx = map_x - alignment.tx
        dy = map_y - alignment.ty
        cos_theta = math.cos(alignment.theta)
        sin_theta = math.sin(alignment.theta)
        enu_x = cos_theta * dx + sin_theta * dy
        enu_y = -sin_theta * dx + cos_theta * dy
        return enu_x, enu_y

    def _latest_fix_map_xy(self, alignment: Alignment2D) -> tuple[float, float] | None:
        if not valid_fix(self._latest_fix):
            return None
        enu_x, enu_y = self._projector.forward(
            float(self._latest_fix.latitude), float(self._latest_fix.longitude)
        )
        return self._enu_to_map(enu_x, enu_y, alignment)

    def _map_gps_consistent(
        self,
        current_xy: tuple[float, float],
        alignment: Alignment2D,
        context: str,
    ) -> bool:
        gps_map_xy = self._latest_fix_map_xy(alignment)
        if gps_map_xy is None:
            self.get_logger().warn("Cannot check map/GPS consistency without a valid /fix")
            return True

        summary = summarize_map_gps_consistency(
            current_xy,
            gps_map_xy,
            self._map_gps_divergence_warn_m,
            self._map_gps_divergence_abort_m,
        )
        if not summary.warn:
            return True

        detail = (
            "%s divergence=%.2fm map=(%.2f,%.2f) gps_map=(%.2f,%.2f)"
            % (
                context,
                summary.distance_m,
                current_xy[0],
                current_xy[1],
                gps_map_xy[0],
                gps_map_xy[1],
            )
        )
        if not summary.ok:
            self.get_logger().error("Map/GPS divergence abort: %s" % detail)
            self._publish_status("MAP_GPS_DIVERGENCE_ABORT|%s" % detail)
            self._publish_stop_override(True)
            return False

        self.get_logger().warn("Map/GPS divergence warning: %s" % detail)
        return True

    def _segment_plan(self, waypoint_index: int) -> SegmentPlan:
        waypoint = self._route["waypoints"][waypoint_index]
        if waypoint_index == 0:
            start_enu = (
                float(self._route["start_ref"]["enu_x"]),
                float(self._route["start_ref"]["enu_y"]),
            )
        else:
            prev_waypoint = self._route["waypoints"][waypoint_index - 1]
            start_enu = (prev_waypoint.enu_x, prev_waypoint.enu_y)
        end_enu = (waypoint.enu_x, waypoint.enu_y)
        dx = end_enu[0] - start_enu[0]
        dy = end_enu[1] - start_enu[1]
        total_length_m = math.hypot(dx, dy)
        if total_length_m < 1e-3:
            return SegmentPlan(
                waypoint=waypoint,
                start_enu=start_enu,
                end_enu=end_enu,
                total_length_m=0.0,
                total_subgoals=1,
                dir_x=1.0,
                dir_y=0.0,
            )
        segment_length_m = float(self._route.get("segment_length_m", 5.0))
        total_subgoals = max(1, int(math.ceil(total_length_m / max(0.5, segment_length_m))))
        return SegmentPlan(
            waypoint=waypoint,
            start_enu=start_enu,
            end_enu=end_enu,
            total_length_m=total_length_m,
            total_subgoals=total_subgoals,
            dir_x=dx / total_length_m,
            dir_y=dy / total_length_m,
        )

    def _segment_projection(
        self, segment: SegmentPlan, current_enu: tuple[float, float]
    ) -> tuple[float, float]:
        rel_x = current_enu[0] - segment.start_enu[0]
        rel_y = current_enu[1] - segment.start_enu[1]
        along = rel_x * segment.dir_x + rel_y * segment.dir_y
        cross = rel_x * (-segment.dir_y) + rel_y * segment.dir_x
        return along, cross

    def _progress_on_segment(
        self, segment: SegmentPlan, current_enu: tuple[float, float]
    ) -> tuple[float, float]:
        along, cross = self._segment_projection(segment, current_enu)
        clamped_along = max(0.0, min(segment.total_length_m, along))
        return clamped_along, cross

    def _point_on_segment(self, segment: SegmentPlan, progress_m: float) -> tuple[float, float]:
        clamped_progress = max(0.0, min(segment.total_length_m, progress_m))
        return (
            segment.start_enu[0] + segment.dir_x * clamped_progress,
            segment.start_enu[1] + segment.dir_y * clamped_progress,
        )

    def _segment_pose(
        self, segment: SegmentPlan, progress_m: float, alignment: Alignment2D
    ) -> PoseStamped:
        enu_x, enu_y = self._point_on_segment(segment, progress_m)
        map_x, map_y = self._enu_to_map(enu_x, enu_y, alignment)
        heading = math.atan2(segment.dir_y, segment.dir_x) + alignment.theta
        qx, qy, qz, qw = yaw_to_quaternion(heading)
        pose = PoseStamped()
        pose.header.frame_id = self._route_frame
        pose.pose.position.x = map_x
        pose.pose.position.y = map_y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def _subgoal_index(
        self, segment: SegmentPlan, target_progress_m: float, segment_length_m: float
    ) -> int:
        if segment.total_subgoals <= 1:
            return 1
        safe_segment_length_m = max(0.5, segment_length_m)
        clamped_target_progress_m = min(segment.total_length_m, max(0.0, target_progress_m))
        best_index = 1
        best_error = float("inf")
        for subgoal_index in range(1, segment.total_subgoals + 1):
            nominal_progress_m = min(segment.total_length_m, subgoal_index * safe_segment_length_m)
            error_m = abs(clamped_target_progress_m - nominal_progress_m)
            if error_m < best_error:
                best_error = error_m
                best_index = subgoal_index
        return best_index

    def _append_map_segment(
        self,
        path: NavPath,
        start_map: tuple[float, float],
        end_map: tuple[float, float],
        segment_length_m: float,
    ) -> None:
        x0, y0 = start_map
        x1, y1 = end_map
        dx = x1 - x0
        dy = y1 - y0
        distance_m = math.hypot(dx, dy)
        if distance_m < 1e-3:
            return
        steps = max(1, int(math.ceil(distance_m / max(0.5, segment_length_m))))
        heading = math.atan2(dy, dx)
        qx, qy, qz, qw = yaw_to_quaternion(heading)
        stamp = path.header.stamp
        for step in range(1, steps + 1):
            ratio = step / steps
            pose = PoseStamped()
            pose.header.frame_id = self._route_frame
            pose.header.stamp = stamp
            pose.pose.position.x = x0 + dx * ratio
            pose.pose.position.y = y0 + dy * ratio
            pose.pose.position.z = 0.0
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            path.poses.append(pose)

    def _publish_remaining_path(
        self,
        current_xy: tuple[float, float],
        waypoint_index: int,
        alignment: Alignment2D,
        current_progress_m: float,
    ) -> None:
        path = NavPath()
        path.header.frame_id = self._route_frame
        path.header.stamp = self.get_clock().now().to_msg()
        segment_length_m = float(self._route.get("segment_length_m", 5.0))

        segment = self._segment_plan(waypoint_index)
        current_enu = self._point_on_segment(segment, current_progress_m)
        current_start_map = self._enu_to_map(current_enu[0], current_enu[1], alignment)
        segment_end_map = self._enu_to_map(segment.end_enu[0], segment.end_enu[1], alignment)
        self._append_map_segment(path, current_start_map, segment_end_map, segment_length_m)

        for index in range(waypoint_index + 1, len(self._route["waypoints"])):
            future_segment = self._segment_plan(index)
            future_start_map = self._enu_to_map(
                future_segment.start_enu[0],
                future_segment.start_enu[1],
                alignment,
            )
            future_end_map = self._enu_to_map(
                future_segment.end_enu[0],
                future_segment.end_enu[1],
                alignment,
            )
            self._append_map_segment(path, future_start_map, future_end_map, segment_length_m)

        self._path_pub.publish(path)
        if path.poses:
            self._goal_pub.publish(path.poses[0])

    def _publish_stop_override(self, stop: bool) -> None:
        self._stop_override = bool(stop)
        self._stop_override_pub.publish(Bool(data=self._stop_override))

    def _publish_stop_override_heartbeat(self) -> None:
        self._stop_override_pub.publish(Bool(data=self._stop_override))

    def _publish_terminal_stop_hold(self) -> None:
        period_s = 1.0 / self._terminal_stop_publish_hz
        repeat = max(1, int(math.ceil(self._terminal_stop_hold_s / period_s)))
        self.get_logger().info(
            "Holding stop override for %.2fs before terminal status (%d samples @ %.1fHz)"
            % (self._terminal_stop_hold_s, repeat, self._terminal_stop_publish_hz)
        )
        self._publish_stop_override(True)
        for _ in range(repeat):
            rclpy.spin_once(self, timeout_sec=period_s)

    def _authority_age_s(self) -> float:
        if self._motion_allowed_mono is None:
            return math.inf
        return max(0.0, time.monotonic() - self._motion_allowed_mono)

    def _authority_faulted(self) -> bool:
        return (
            "FAULT_HOLD" in self._authority_status
            or "FAULT_LATCHED" in self._authority_status
        )

    def _global_correction_result(self):
        authority_age_s = self._authority_age_s()
        try:
            transform = self._tf_buffer.lookup_transform(
                self._route_frame,
                "odom",
                Time(),
                timeout=Duration(seconds=0.02),
            )
        except TransformException:
            return self._global_watchdog.update(
                1.0,
                0.0,
                0.0,
                0.0,
                authority_allowed=False,
                authority_age_s=authority_age_s,
            )
        stamp_s = float(transform.header.stamp.sec) + float(
            transform.header.stamp.nanosec
        ) * 1e-9
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = quaternion_to_yaw(rotation.x, rotation.y, rotation.z, rotation.w)
        return self._global_watchdog.update(
            stamp_s,
            float(translation.x),
            float(translation.y),
            yaw,
            authority_allowed=self._motion_allowed,
            authority_age_s=authority_age_s,
        )

    def _cancel_for_global_hold(
        self,
        goal_handle,
        waypoint_name: str,
        subgoal_index: int,
        reason: str,
    ) -> int:
        hold_started_mono = time.monotonic()
        self._publish_stop_override(True)
        self._publish_status(
            "GLOBAL_CORRECTION_HOLD|%s|%d|%s"
            % (waypoint_name, subgoal_index, reason)
        )
        cancel_future = goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(
            self, cancel_future, timeout_sec=self._cancel_ack_timeout_s
        )
        cancel_response = cancel_future.result() if cancel_future.done() else None
        if cancel_response is None or not cancel_response.goals_canceling:
            self._publish_status(
                "GLOBAL_CORRECTION_ABORT|%s|%d|CANCEL_NOT_ACKNOWLEDGED"
                % (waypoint_name, subgoal_index)
            )
            return GoalStatus.STATUS_ABORTED

        readiness = ContinuousReadiness(self._authority_ready_confirmation_s)
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=self._odom_watchdog_monitor_period_s)
            now_mono = time.monotonic()
            if self._authority_faulted():
                self._publish_status(
                    "GLOBAL_CORRECTION_ABORT|%s|%d|FAULT_HOLD"
                    % (waypoint_name, subgoal_index)
                )
                return GoalStatus.STATUS_ABORTED
            if now_mono - hold_started_mono > self._global_hold_timeout_s:
                self._publish_status(
                    "GLOBAL_CORRECTION_ABORT|%s|%d|HOLD_TIMEOUT"
                    % (waypoint_name, subgoal_index)
                )
                return GoalStatus.STATUS_ABORTED

            ready = (
                self._motion_allowed
                and self._authority_age_s() <= self._motion_authority_max_age_s
            )
            if not readiness.update(ready=ready, now_s=now_mono):
                continue

            self._publish_stop_override(False)
            self._publish_status(
                "GLOBAL_CORRECTION_RETRY|%s|%d" % (waypoint_name, subgoal_index)
            )
            return GOAL_STATUS_GLOBAL_CORRECTION_HOLD

    def _abort_goal_with_watchdog(
        self,
        goal_handle,
        waypoint_name: str,
        subgoal_index: int,
        reason: str,
    ) -> int:
        self.get_logger().error(
            "Aborting %s subgoal %d due to odom watchdog: %s"
            % (waypoint_name, subgoal_index, reason)
        )
        self._publish_status(
            "ODOM_DIVERGENCE_ABORT|%s|%d|%s" % (waypoint_name, subgoal_index, reason)
        )
        self._publish_stop_override(True)
        cancel_future = goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=2.0)
        return GoalStatus.STATUS_ABORTED

    def _wait_for_authority_ready(self, timeout_s: float) -> bool:
        self._publish_stop_override(True)
        deadline_mono = time.monotonic() + timeout_s
        readiness = ContinuousReadiness(self._authority_ready_confirmation_s)
        while rclpy.ok() and time.monotonic() < deadline_mono:
            rclpy.spin_once(self, timeout_sec=self._odom_watchdog_monitor_period_s)
            now_mono = time.monotonic()
            if self._authority_faulted():
                return False
            ready = (
                self._motion_allowed
                and self._authority_age_s() <= self._motion_authority_max_age_s
            )
            if readiness.update(ready=ready, now_s=now_mono):
                return True
        return False

    def _send_goal(
        self,
        pose: PoseStamped,
        waypoint_name: str,
        subgoal_index: int,
        alignment_at_send: Alignment2D | None = None,
    ) -> int:
        if not self._wait_for_authority_ready(self._global_hold_timeout_s):
            self._publish_status(
                "GLOBAL_CORRECTION_ABORT|%s|%d|AUTHORITY_NOT_READY"
                % (waypoint_name, subgoal_index)
            )
            return GoalStatus.STATUS_ABORTED
        goal = NavigateToPose.Goal()
        pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose = pose
        send_future = self._nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return GoalStatus.STATUS_ABORTED
        self._publish_stop_override(False)
        result_future = goal_handle.get_result_async()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=self._odom_watchdog_monitor_period_s)
            if result_future.done():
                self._publish_stop_override(True)
                result = result_future.result()
                if result is None:
                    return GoalStatus.STATUS_UNKNOWN
                return int(result.status)

            local_result = self._local_watchdog_result
            if (
                local_result is not None
                and local_result.decision is WatchdogDecision.LOCAL_ABORT
            ):
                return self._abort_goal_with_watchdog(
                    goal_handle,
                    waypoint_name,
                    subgoal_index,
                    local_result.reason or "LOCAL_ODOM_INVALID",
                )

            global_result = self._global_correction_result()
            if global_result.decision is WatchdogDecision.GLOBAL_HOLD:
                return self._cancel_for_global_hold(
                    goal_handle,
                    waypoint_name,
                    subgoal_index,
                    global_result.reason or "GLOBAL_CORRECTION_HOLD",
                )

            if alignment_at_send is not None and self._latest_alignment is not None:
                alignment_check_mono = time.monotonic()
                if (
                    alignment_check_mono - self._last_alignment_shift_mono
                    >= self._alignment_shift_cooldown_s
                ):
                    goal_enu = self._map_to_enu(
                        pose.pose.position.x, pose.pose.position.y, alignment_at_send
                    )
                    new_map = self._enu_to_map(
                        goal_enu[0], goal_enu[1], self._latest_alignment
                    )
                    displacement_m = math.hypot(
                        new_map[0] - pose.pose.position.x,
                        new_map[1] - pose.pose.position.y,
                    )
                    if displacement_m > self._alignment_shift_cancel_threshold_m:
                        self.get_logger().info(
                            "Alignment shifted %.2fm for %s subgoal %d, preempting with new goal"
                            % (displacement_m, waypoint_name, subgoal_index)
                        )
                        self._last_alignment_shift_mono = alignment_check_mono
                        return self._cancel_for_global_hold(
                            goal_handle,
                            waypoint_name,
                            subgoal_index,
                            "ALIGNMENT_SHIFT",
                        )

        return GoalStatus.STATUS_UNKNOWN

    def _choose_waypoint_alignment(
        self,
        waypoint_index: int,
        segment: SegmentPlan,
        current_xy: tuple[float, float],
    ) -> tuple[Alignment2D, float]:
        alignment = self._latest_alignment
        if alignment is None:
            raise RuntimeError("lost ENU->map alignment before starting waypoint")
        current_enu = self._map_to_enu(current_xy[0], current_xy[1], alignment)
        raw_progress_m, _ = self._progress_on_segment(segment, current_enu)
        progress_m = max(0.0, min(segment.total_length_m, raw_progress_m))
        return alignment, progress_m

    def _run_waypoint(self, waypoint_index: int) -> tuple[bool, tuple[float, float]]:
        segment = self._segment_plan(waypoint_index)
        waypoint = segment.waypoint
        segment_length_m = float(self._route.get("segment_length_m", 5.0))
        waypoint_tolerance_m = float(self._route.get("waypoint_xy_tolerance_m", 0.35))
        success_shortfall_tolerance_m = float(
            self._route.get(
                "nav2_success_shortfall_tolerance_m",
                max(0.75, waypoint_tolerance_m),
            )
        )
        current_xy = self._current_xy()
        starting_alignment, current_progress_m = self._choose_waypoint_alignment(
            waypoint_index, segment, current_xy
        )
        self.get_logger().info(
            "Starting waypoint %s with alignment theta=%.2fdeg tx=%.2f ty=%.2f rev=%d"
            % (
                waypoint.name,
                math.degrees(starting_alignment.theta),
                starting_alignment.tx,
                starting_alignment.ty,
                starting_alignment.revision,
            )
        )
        retry_progress_m: float | None = None

        while rclpy.ok():
            current_xy = self._current_xy()
            live_alignment = self._latest_alignment
            if live_alignment is None:
                self.get_logger().error("Lost alignment during waypoint %s" % waypoint.name)
                return False, current_xy
            if not self._map_gps_consistent(
                current_xy, live_alignment, "waypoint_%s" % waypoint.name
            ):
                return False, current_xy
            current_enu = self._map_to_enu(current_xy[0], current_xy[1], live_alignment)
            projected_progress_m, _ = self._progress_on_segment(segment, current_enu)
            current_progress_m = max(current_progress_m, projected_progress_m)
            remaining_m = max(0.0, segment.total_length_m - current_progress_m)
            if remaining_m <= waypoint_tolerance_m:
                return True, current_xy

            self._publish_remaining_path(
                current_xy, waypoint_index, live_alignment, current_progress_m
            )

            next_progress_m = (
                retry_progress_m
                if retry_progress_m is not None
                else min(
                    segment.total_length_m,
                    current_progress_m + segment_length_m,
                )
            )
            next_subgoal = self._segment_pose(segment, next_progress_m, live_alignment)
            subgoal_index = self._subgoal_index(segment, next_progress_m, segment_length_m)
            self._publish_status(
                "NAVIGATING_SUBGOAL|%s|%d|%d|%.2f|%.2f|%s"
                % (
                    waypoint.name,
                    subgoal_index,
                    segment.total_subgoals,
                    next_subgoal.pose.position.x,
                    next_subgoal.pose.position.y,
                    live_alignment.source,
                )
            )
            self._goal_pub.publish(next_subgoal)
            self.get_logger().info(
                "Sending %s subgoal %d/%d x=%.2f y=%.2f progress=%.2f/%.2f source=%s"
                % (
                    waypoint.name,
                    subgoal_index,
                    segment.total_subgoals,
                    next_subgoal.pose.position.x,
                    next_subgoal.pose.position.y,
                    next_progress_m,
                    segment.total_length_m,
                    live_alignment.source,
                )
            )

            status = self._send_goal(
                next_subgoal,
                waypoint.name,
                subgoal_index,
                alignment_at_send=live_alignment,
            )
            if status in (
                GOAL_STATUS_ALIGNMENT_SHIFT,
                GOAL_STATUS_GLOBAL_CORRECTION_HOLD,
            ):
                retry_progress_m = next_progress_m
                self.get_logger().info(
                    "Re-computing the same ENU subgoal for %s after global hold"
                    % waypoint.name
                )
                continue
            retry_progress_m = None
            if status != GoalStatus.STATUS_SUCCEEDED:
                self._publish_terminal_stop_hold()
                self._publish_status(
                    f"FAILED_WAYPOINT_{waypoint.name}_SUBGOAL_{subgoal_index}_STATUS_{status}"
                )
                return False, current_xy

            verified_ok, current_xy, current_progress_m = self._verify_nav2_success_progress(
                segment=segment,
                waypoint_name=waypoint.name,
                subgoal_index=subgoal_index,
                target_progress_m=next_progress_m,
                current_progress_m=current_progress_m,
                waypoint_tolerance_m=waypoint_tolerance_m,
                success_shortfall_tolerance_m=success_shortfall_tolerance_m,
            )
            if not verified_ok:
                return False, current_xy

        return False, self._current_xy()

    def _verify_nav2_success_progress(
        self,
        segment: SegmentPlan,
        waypoint_name: str,
        subgoal_index: int,
        target_progress_m: float,
        current_progress_m: float,
        waypoint_tolerance_m: float,
        success_shortfall_tolerance_m: float,
    ) -> tuple[bool, tuple[float, float], float]:
        current_xy = self._current_xy()
        live_alignment = self._latest_alignment
        if live_alignment is None:
            self.get_logger().error("Lost alignment after Nav2 success for %s" % waypoint_name)
            self._publish_terminal_stop_hold()
            self._publish_status(
                "NAV2_FALSE_SUCCESS_ABORT|%s|%d|LOST_ALIGNMENT"
                % (waypoint_name, subgoal_index)
            )
            return False, current_xy, current_progress_m
        if not self._map_gps_consistent(
            current_xy, live_alignment, "nav2_success_%s" % waypoint_name
        ):
            return False, current_xy, current_progress_m

        current_enu = self._map_to_enu(current_xy[0], current_xy[1], live_alignment)
        observed_progress_m, _ = self._progress_on_segment(segment, current_enu)
        verified_progress_m = max(current_progress_m, observed_progress_m)
        progress_summary = summarize_nav2_success_progress(
            target_progress_m=target_progress_m,
            verified_progress_m=verified_progress_m,
            waypoint_tolerance_m=waypoint_tolerance_m,
            success_shortfall_tolerance_m=success_shortfall_tolerance_m,
        )
        if progress_summary.ok:
            return True, current_xy, verified_progress_m

        detail = (
            "%s|%d|target=%.2f|progress=%.2f|shortfall=%.2f|tolerance=%.2f"
            % (
                waypoint_name,
                subgoal_index,
                target_progress_m,
                verified_progress_m,
                progress_summary.shortfall_m,
                progress_summary.tolerance_m,
            )
        )
        self.get_logger().error(
            "Nav2 reported success without route progress: %s" % detail
        )
        self._publish_terminal_stop_hold()
        self._publish_status("NAV2_FALSE_SUCCESS_ABORT|%s" % detail)
        return False, current_xy, verified_progress_m

    def run(self) -> bool:
        self._publish_status("INITIALIZING")
        self.get_logger().info(f"Loaded route file: {self._route_file}")
        self.get_logger().info(
            "Route %s with %d waypoints"
            % (self._route.get("route_name", "unnamed_route"), len(self._route["waypoints"]))
        )

        startup_fix = self._wait_for_stable_fix()
        self._validate_startup(startup_fix)
        self._wait_for_nav2()
        alignment = self._wait_for_alignment()
        self.get_logger().info(
            "Using stable ENU->map alignment: theta=%.2fdeg tx=%.2f ty=%.2f"
            % (math.degrees(alignment.theta), alignment.tx, alignment.ty)
        )
        self._publish_status("ALIGNMENT_READY")
        self._publish_status("RUNNING_ROUTE")

        x0, y0, _ = self._lookup_current_pose(announce_wait=True)
        current_xy = (x0, y0)
        if not self._map_gps_consistent(current_xy, alignment, "route_start"):
            return False
        for waypoint_index, waypoint in enumerate(self._route["waypoints"]):
            self._publish_status(
                "WAYPOINT_TARGET|%d|%d|%s"
                % (waypoint_index + 1, len(self._route["waypoints"]), waypoint.name)
            )
            self.get_logger().info(
                "Navigating to waypoint %d/%d: %s"
                % (waypoint_index + 1, len(self._route["waypoints"]), waypoint.name)
            )
            ok, current_xy = self._run_waypoint(waypoint_index)
            if not ok:
                return False
            self._publish_status(
                "WAYPOINT_REACHED|%d|%d|%s"
                % (waypoint_index + 1, len(self._route["waypoints"]), waypoint.name)
            )

        self._publish_status("STOPPING_BEFORE_EXIT")
        self._publish_terminal_stop_hold()
        self._publish_status("SUCCEEDED")
        return True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GPSRouteRunner()
    ok = False
    try:
        ok = node.run()
    except KeyboardInterrupt:
        node._publish_status("INTERRUPTED")
    except Exception as exc:
        node.get_logger().error(str(exc))
        node._publish_status(f"ABORTED: {exc}")
    finally:
        if rclpy.ok():
            node._publish_stop_override(True)
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
