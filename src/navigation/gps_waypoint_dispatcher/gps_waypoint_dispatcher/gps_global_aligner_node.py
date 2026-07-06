#!/usr/bin/env python3
from __future__ import annotations

import math
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path as FSPath

import rclpy
import yaml
from geometry_msgs.msg import QuaternionStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64MultiArray, String
from tf2_ros import Buffer, TransformException, TransformListener

from gps_waypoint_dispatcher.alignment_math import (
    choose_bootstrap_yaw,
    compute_bootstrap_alignment,
    heading_quaternion_yaw_to_enu_yaw,
)
from gps_waypoint_dispatcher.route_safety import is_rcl_context_shutdown_error_message
from gps_waypoint_dispatcher.scene_runtime import (
    FixedENUProjector,
    default_route_file,
    haversine_m,
    normalize_angle,
    quaternion_to_yaw,
)


def valid_fix(msg: NavSatFix | None) -> bool:
    if msg is None:
        return False
    if msg.status.status < 0:
        return False
    if not math.isfinite(msg.latitude) or not math.isfinite(msg.longitude):
        return False
    return True


@dataclass
class Alignment2D:
    theta: float
    tx: float
    ty: float
    source: str
    revision: int


@dataclass
class AlignmentPair:
    stamp_mono: float
    enu_x: float
    enu_y: float
    map_x: float
    map_y: float


@dataclass
class CalibrationPair:
    label: str
    enu_x: float
    enu_y: float
    map_x: float
    map_y: float


class GPSGlobalAligner(Node):
    def __init__(self) -> None:
        super().__init__("gps_global_aligner")
        self.declare_parameter("route_file", str(default_route_file()))
        self.declare_parameter("route_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("fix_topic", "/fix")
        self.declare_parameter("heading_topic", "/heading")
        self.declare_parameter("use_rtk_heading_for_bootstrap", True)
        self.declare_parameter("heading_quaternion_yaw_is_compass", True)
        self.declare_parameter("startup_heading_timeout_s", 8.0)
        self.declare_parameter("startup_heading_sample_count", 5)
        self.declare_parameter("startup_heading_spread_max_deg", 3.0)
        self.declare_parameter("startup_heading_route_mismatch_warn_deg", 10.0)
        self.declare_parameter("alignment_topic", "/gps_corridor/enu_to_map")
        self.declare_parameter("status_topic", "/gps_corridor/alignment_status")
        self.declare_parameter("debug_topic", "/gps_corridor/alignment_debug")
        self.declare_parameter("calibration_request_topic", "/gps_corridor/calibration_request")
        self.declare_parameter("calibration_status_topic", "/gps_corridor/calibration_status")
        self.declare_parameter("startup_wait_timeout_s", 90.0)
        self.declare_parameter("enu_origin_lat", 0.0)
        self.declare_parameter("enu_origin_lon", 0.0)
        self.declare_parameter("enu_origin_alt", 0.0)
        self.declare_parameter("pair_window_s", 90.0)
        self.declare_parameter("pair_min_spacing_m", 2.0)
        self.declare_parameter("alignment_min_pairs", 5)
        self.declare_parameter("alignment_min_spread_m", 20.0)
        self.declare_parameter("calibration_min_spread_m", 20.0)
        self.declare_parameter("max_theta_step_deg", 0.5)
        self.declare_parameter("max_translation_step_m", 0.30)
        self.declare_parameter("max_bootstrap_delta_deg", 25.0)
        self.declare_parameter("max_bootstrap_translation_delta_m", 8.0)
        self.declare_parameter("max_calibration_translation_delta_m", 12.0)
        self.declare_parameter("max_alignment_step_warning_deg", 5.0)
        self.declare_parameter("publish_period_s", 0.2)
        self.declare_parameter("status_log_period_s", 5.0)

        self._route_file = FSPath(self.get_parameter("route_file").value).expanduser()
        self._route_frame = str(self.get_parameter("route_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._fix_topic = str(self.get_parameter("fix_topic").value)
        self._heading_topic = str(self.get_parameter("heading_topic").value)
        self._use_rtk_heading_for_bootstrap = bool(
            self.get_parameter("use_rtk_heading_for_bootstrap").value
        )
        self._heading_quaternion_yaw_is_compass = bool(
            self.get_parameter("heading_quaternion_yaw_is_compass").value
        )
        self._startup_heading_timeout_s = float(
            self.get_parameter("startup_heading_timeout_s").value
        )
        self._startup_heading_sample_count = int(
            self.get_parameter("startup_heading_sample_count").value
        )
        self._startup_heading_spread_max_deg = float(
            self.get_parameter("startup_heading_spread_max_deg").value
        )
        self._startup_heading_route_mismatch_warn_deg = float(
            self.get_parameter("startup_heading_route_mismatch_warn_deg").value
        )
        self._alignment_topic = str(self.get_parameter("alignment_topic").value)
        self._status_topic = str(self.get_parameter("status_topic").value)
        self._debug_topic = str(self.get_parameter("debug_topic").value)
        self._calibration_request_topic = str(
            self.get_parameter("calibration_request_topic").value
        )
        self._calibration_status_topic = str(
            self.get_parameter("calibration_status_topic").value
        )
        self._startup_wait_timeout_s = float(self.get_parameter("startup_wait_timeout_s").value)
        self._enu_origin_lat = float(self.get_parameter("enu_origin_lat").value)
        self._enu_origin_lon = float(self.get_parameter("enu_origin_lon").value)
        self._enu_origin_alt = float(self.get_parameter("enu_origin_alt").value)
        self._pair_window_s = float(self.get_parameter("pair_window_s").value)
        self._pair_min_spacing_m = float(self.get_parameter("pair_min_spacing_m").value)
        self._alignment_min_pairs = int(self.get_parameter("alignment_min_pairs").value)
        self._alignment_min_spread_m = float(self.get_parameter("alignment_min_spread_m").value)
        self._calibration_min_spread_m = float(
            self.get_parameter("calibration_min_spread_m").value
        )
        self._max_theta_step_rad = math.radians(
            float(self.get_parameter("max_theta_step_deg").value)
        )
        self._max_translation_step_m = float(self.get_parameter("max_translation_step_m").value)
        self._max_bootstrap_delta_rad = math.radians(
            float(self.get_parameter("max_bootstrap_delta_deg").value)
        )
        self._max_bootstrap_translation_delta_m = float(
            self.get_parameter("max_bootstrap_translation_delta_m").value
        )
        self._max_calibration_translation_delta_m = float(
            self.get_parameter("max_calibration_translation_delta_m").value
        )
        self._max_alignment_step_warning_rad = math.radians(
            float(self.get_parameter("max_alignment_step_warning_deg").value)
        )
        self._publish_period_s = float(self.get_parameter("publish_period_s").value)
        self._status_log_period_s = float(self.get_parameter("status_log_period_s").value)

        self._alignment_pub = self.create_publisher(
            Float64MultiArray, self._alignment_topic, 10
        )
        self._status_pub = self.create_publisher(String, self._status_topic, 10)
        self._debug_pub = self.create_publisher(Float64MultiArray, self._debug_topic, 10)
        self._calibration_status_pub = self.create_publisher(
            String, self._calibration_status_topic, 10
        )
        self._fix_sub = self.create_subscription(NavSatFix, self._fix_topic, self._fix_callback, 10)
        self._heading_sub = self.create_subscription(
            QuaternionStamped, self._heading_topic, self._heading_callback, 10
        )
        self._calibration_request_sub = self.create_subscription(
            String, self._calibration_request_topic, self._calibration_request_callback, 10
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._latest_fix: NavSatFix | None = None
        self._latest_heading_enu_yaw: float | None = None
        self._latest_heading_sample_key: tuple | None = None
        self._heading_sequence = 0
        self._last_fix_key: tuple | None = None
        self._last_heading_key: tuple | None = None
        self._alignment_revision = 0
        self._bootstrap_alignment: Alignment2D | None = None
        self._current_alignment: Alignment2D | None = None
        self._raw_alignment: Alignment2D | None = None
        self._pair_buffer: deque[AlignmentPair] = deque()
        self._calibration_pairs: list[CalibrationPair] = []
        self._pending_calibration_request: tuple[int, str] | None = None
        self._last_pair_enu: tuple[float, float] | None = None
        self._last_pair_map: tuple[float, float] | None = None
        self._last_status_line = ""
        self._last_status_log_mono = 0.0

        self._projector = FixedENUProjector(
            self._enu_origin_lat, self._enu_origin_lon, self._enu_origin_alt
        )
        self._route = self._load_route(self._route_file)

    def _publish_status(self, text: str) -> None:
        now_mono = time.monotonic()
        should_log = (
            text != self._last_status_line
            or now_mono - self._last_status_log_mono >= self._status_log_period_s
        )
        if should_log:
            self.get_logger().info(text)
            self._last_status_line = text
            self._last_status_log_mono = now_mono
        self._status_pub.publish(String(data=text))

    def _publish_calibration_status(self, text: str) -> None:
        self.get_logger().info(text)
        self._calibration_status_pub.publish(String(data=text))

    def _publish_alignment(self, alignment: Alignment2D | None, valid: bool) -> None:
        msg = Float64MultiArray()
        if alignment is None:
            msg.data = [0.0, 0.0, 0.0, 0.0]
        else:
            msg.data = [alignment.theta, alignment.tx, alignment.ty, 1.0 if valid else 0.0]
        self._alignment_pub.publish(msg)

    def _publish_debug(
        self,
        pair_count: int,
        spread_m: float,
        raw_alignment: Alignment2D | None,
        output_alignment: Alignment2D | None,
    ) -> None:
        raw_theta_deg = math.degrees(raw_alignment.theta) if raw_alignment is not None else 0.0
        raw_tx = raw_alignment.tx if raw_alignment is not None else 0.0
        raw_ty = raw_alignment.ty if raw_alignment is not None else 0.0
        out_theta_deg = (
            math.degrees(output_alignment.theta) if output_alignment is not None else 0.0
        )
        out_tx = output_alignment.tx if output_alignment is not None else 0.0
        out_ty = output_alignment.ty if output_alignment is not None else 0.0
        msg = Float64MultiArray()
        msg.data = [
            raw_theta_deg,
            raw_tx,
            raw_ty,
            out_theta_deg,
            out_tx,
            out_ty,
            float(pair_count),
            spread_m,
        ]
        self._debug_pub.publish(msg)

    def _fix_callback(self, msg: NavSatFix) -> None:
        self._latest_fix = msg

    def _heading_callback(self, msg: QuaternionStamped) -> None:
        yaw = quaternion_to_yaw(
            msg.quaternion.x,
            msg.quaternion.y,
            msg.quaternion.z,
            msg.quaternion.w,
        )
        self._latest_heading_enu_yaw = heading_quaternion_yaw_to_enu_yaw(
            yaw,
            quaternion_yaw_is_compass=self._heading_quaternion_yaw_is_compass,
        )
        self._heading_sequence += 1
        self._latest_heading_sample_key = (
            self._heading_sequence,
            msg.header.stamp.sec,
            msg.header.stamp.nanosec,
            round(self._latest_heading_enu_yaw, 6),
        )

    def _calibration_request_callback(self, msg: String) -> None:
        parts = msg.data.split("|")
        if len(parts) < 3 or parts[0] != "CALIBRATE":
            return
        try:
            waypoint_index = int(parts[1]) - 1
        except ValueError:
            self.get_logger().warn(
                "Ignoring malformed calibration request: %s" % msg.data
            )
            return
        if self._pending_calibration_request is not None:
            self.get_logger().warn(
                "Dropping calibration request for %s because one is already pending"
                % parts[2]
            )
            return
        self._pending_calibration_request = (waypoint_index, parts[2])

    def _load_route(self, path: FSPath) -> dict:
        if not path.exists():
            raise RuntimeError(f"route file not found: {path}")
        with open(path, "r", encoding="utf-8") as route_file:
            data = yaml.safe_load(route_file) or {}

        for key in ("enu_origin", "start_ref", "waypoints", "launch_yaw_deg"):
            if key not in data:
                raise RuntimeError(f"missing key in route file: {key}")

        self._validate_origin_match(data["enu_origin"])
        start_ref = data["start_ref"]
        start_ref["lat"] = float(start_ref["lat"])
        start_ref["lon"] = float(start_ref["lon"])
        start_ref["alt"] = float(start_ref.get("alt", 0.0))
        start_ref["enu_x"], start_ref["enu_y"] = self._projector.forward(
            start_ref["lat"], start_ref["lon"]
        )
        data["start_ref"] = start_ref
        data["launch_yaw_deg"] = float(data["launch_yaw_deg"])
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
        samples: deque[tuple[float, float, float]] = deque(maxlen=sample_count)
        self._publish_status("ALIGNER_WAITING_FOR_STABLE_FIX")

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

            max_spread = 0.0
            sample_list = list(samples)
            for i in range(len(sample_list)):
                for j in range(i + 1, len(sample_list)):
                    spread = haversine_m(
                        sample_list[i][0],
                        sample_list[i][1],
                        sample_list[j][0],
                        sample_list[j][1],
                    )
                    max_spread = max(max_spread, spread)
            if max_spread > spread_limit:
                continue

            return {
                "lat": sum(sample[0] for sample in sample_list) / len(sample_list),
                "lon": sum(sample[1] for sample in sample_list) / len(sample_list),
                "alt": sum(sample[2] for sample in sample_list) / len(sample_list),
                "samples": len(sample_list),
                "spread_m": round(max_spread, 2),
            }

        raise RuntimeError("timed out waiting for stable /fix samples")

    def _heading_spread_deg(self, samples: list[float]) -> float:
        max_spread = 0.0
        for i in range(len(samples)):
            for j in range(i + 1, len(samples)):
                spread = abs(math.degrees(normalize_angle(samples[i] - samples[j])))
                max_spread = max(max_spread, spread)
        return max_spread

    def _mean_heading_rad(self, samples: list[float]) -> float:
        sin_sum = sum(math.sin(sample) for sample in samples)
        cos_sum = sum(math.cos(sample) for sample in samples)
        return normalize_angle(math.atan2(sin_sum, cos_sum))

    def _wait_for_stable_heading(self) -> float | None:
        if not self._use_rtk_heading_for_bootstrap:
            return None

        sample_count = max(1, self._startup_heading_sample_count)
        deadline = time.time() + max(0.0, self._startup_heading_timeout_s)
        samples: deque[float] = deque(maxlen=sample_count)
        self._publish_status("ALIGNER_WAITING_FOR_STABLE_HEADING")

        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            heading = self._latest_heading_enu_yaw
            if heading is None or not math.isfinite(heading):
                continue
            key = self._latest_heading_sample_key or (round(heading, 6),)
            if key == self._last_heading_key:
                continue
            self._last_heading_key = key
            samples.append(heading)
            if len(samples) < sample_count:
                continue
            sample_list = list(samples)
            spread_deg = self._heading_spread_deg(sample_list)
            if spread_deg > self._startup_heading_spread_max_deg:
                continue
            mean_heading = self._mean_heading_rad(sample_list)
            self.get_logger().info(
                "Aligner startup RTK heading mean=%.2fdeg spread=%.2fdeg samples=%d"
                % (math.degrees(mean_heading), spread_deg, len(sample_list))
            )
            return mean_heading

        self.get_logger().warn(
            "No stable RTK heading within %.1fs; falling back to route launch_yaw_deg"
            % self._startup_heading_timeout_s
        )
        return None

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
            "Aligner startup fix mean lat=%.7f lon=%.7f spread=%.2fm distance_to_start_ref=%.2fm"
            % (
                startup_fix["lat"],
                startup_fix["lon"],
                startup_fix["spread_m"],
                distance_m,
            )
        )
        if distance_m > tolerance_m:
            raise RuntimeError(
                f"aligner startup GPS is {distance_m:.2f}m from start_ref (limit {tolerance_m:.2f}m)"
            )
        return distance_m

    def _lookup_current_pose(self, status_text: str) -> tuple[float, float, float]:
        deadline = time.time() + self._startup_wait_timeout_s
        self._publish_status(status_text)
        while rclpy.ok() and time.time() < deadline:
            try:
                transform = self._tf_buffer.lookup_transform(
                    self._route_frame,
                    self._base_frame,
                    Time(),
                    timeout=Duration(seconds=0.5),
                )
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                yaw = quaternion_to_yaw(rotation.x, rotation.y, rotation.z, rotation.w)
                return float(translation.x), float(translation.y), yaw
            except TransformException:
                rclpy.spin_once(self, timeout_sec=0.2)
        raise RuntimeError(f"timed out waiting for TF {self._route_frame}->{self._base_frame}")

    def _try_lookup_current_xy(self) -> tuple[float, float] | None:
        try:
            transform = self._tf_buffer.lookup_transform(
                self._route_frame,
                self._base_frame,
                Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException:
            return None
        translation = transform.transform.translation
        return float(translation.x), float(translation.y)

    def _build_bootstrap_alignment(
        self,
        x0: float,
        y0: float,
        yaw0: float,
        rtk_heading_enu_yaw: float | None,
        startup_fix: dict | None = None,
    ) -> Alignment2D:
        yaw_choice = choose_bootstrap_yaw(
            route_launch_heading_deg=float(self._route["launch_yaw_deg"]),
            rtk_heading_yaw_rad=rtk_heading_enu_yaw,
            use_rtk_heading=self._use_rtk_heading_for_bootstrap,
            mismatch_warn_deg=self._startup_heading_route_mismatch_warn_deg,
        )
        if startup_fix is not None:
            anchor_enu_x, anchor_enu_y = self._projector.forward(
                startup_fix["lat"], startup_fix["lon"]
            )
        else:
            start_ref = self._route["start_ref"]
            anchor_enu_x = start_ref["enu_x"]
            anchor_enu_y = start_ref["enu_y"]
        self.get_logger().info(
            "Bootstrap anchor: %s enu=(%.2f, %.2f)"
            % ("startup_fix" if startup_fix else "start_ref", anchor_enu_x, anchor_enu_y)
        )
        if yaw_choice.warn_route_mismatch and yaw_choice.route_mismatch_deg is not None:
            self.get_logger().warn(
                "RTK heading differs from route launch_yaw by %.2fdeg; using RTK heading for bootstrap"
                % yaw_choice.route_mismatch_deg
            )
        alignment = compute_bootstrap_alignment(
            map_x=x0,
            map_y=y0,
            map_yaw_rad=yaw0,
            anchor_enu_x=anchor_enu_x,
            anchor_enu_y=anchor_enu_y,
            selected_enu_yaw_rad=yaw_choice.enu_yaw_rad,
        )
        self._alignment_revision += 1
        return Alignment2D(
            theta=alignment.theta,
            tx=alignment.tx,
            ty=alignment.ty,
            source=f"bootstrap_{yaw_choice.source}",
            revision=self._alignment_revision,
        )

    def _enu_to_map(
        self, enu_x: float, enu_y: float, alignment: Alignment2D
    ) -> tuple[float, float]:
        cos_theta = math.cos(alignment.theta)
        sin_theta = math.sin(alignment.theta)
        map_x = cos_theta * enu_x - sin_theta * enu_y + alignment.tx
        map_y = sin_theta * enu_x + cos_theta * enu_y + alignment.ty
        return map_x, map_y

    def _compute_pair_spread_m(self, pairs: list[AlignmentPair]) -> float:
        max_spread = 0.0
        for i in range(len(pairs)):
            for j in range(i + 1, len(pairs)):
                spread = math.hypot(
                    pairs[i].enu_x - pairs[j].enu_x,
                    pairs[i].enu_y - pairs[j].enu_y,
            )
            max_spread = max(max_spread, spread)
        return max_spread

    def _upsert_calibration_pair(
        self, label: str, enu_x: float, enu_y: float, map_x: float, map_y: float
    ) -> None:
        for index, pair in enumerate(self._calibration_pairs):
            if pair.label == label:
                self._calibration_pairs[index] = CalibrationPair(
                    label=label,
                    enu_x=enu_x,
                    enu_y=enu_y,
                    map_x=map_x,
                    map_y=map_y,
                )
                return
        self._calibration_pairs.append(
            CalibrationPair(
                label=label,
                enu_x=enu_x,
                enu_y=enu_y,
                map_x=map_x,
                map_y=map_y,
            )
        )

    def _solve_alignment(self, pairs: list[AlignmentPair]) -> Alignment2D | None:
        if self._bootstrap_alignment is None:
            return None
        if len(pairs) < self._alignment_min_pairs:
            return None
        spread_m = self._compute_pair_spread_m(pairs)
        if spread_m < self._alignment_min_spread_m:
            return None

        theta = self._bootstrap_alignment.theta
        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)
        tx_sum = 0.0
        ty_sum = 0.0
        for pair in pairs:
            tx_sum += pair.map_x - (cos_theta * pair.enu_x - sin_theta * pair.enu_y)
            ty_sum += pair.map_y - (sin_theta * pair.enu_x + cos_theta * pair.enu_y)
        tx = tx_sum / len(pairs)
        ty = ty_sum / len(pairs)
        if not (math.isfinite(theta) and math.isfinite(tx) and math.isfinite(ty)):
            return None

        self._alignment_revision += 1
        return Alignment2D(theta=theta, tx=tx, ty=ty, source="raw_gps", revision=self._alignment_revision)

    def _solve_calibration_alignment(
        self, pairs: list[CalibrationPair]
    ) -> Alignment2D | None:
        if self._bootstrap_alignment is None:
            return None
        if len(pairs) < 2:
            return None
        spread_m = self._compute_pair_spread_m(pairs)
        if spread_m < self._calibration_min_spread_m:
            return None

        theta = self._bootstrap_alignment.theta
        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)
        tx_sum = 0.0
        ty_sum = 0.0
        for pair in pairs:
            tx_sum += pair.map_x - (cos_theta * pair.enu_x - sin_theta * pair.enu_y)
            ty_sum += pair.map_y - (sin_theta * pair.enu_x + cos_theta * pair.enu_y)
        tx = tx_sum / len(pairs)
        ty = ty_sum / len(pairs)
        if not (math.isfinite(theta) and math.isfinite(tx) and math.isfinite(ty)):
            return None
        return Alignment2D(
            theta=theta,
            tx=tx,
            ty=ty,
            source="waypoint_calibration_raw",
            revision=self._alignment_revision,
        )

    def _apply_calibration_alignment(self, raw_alignment: Alignment2D) -> Alignment2D | None:
        if self._current_alignment is None:
            return None
        theta_delta = abs(normalize_angle(raw_alignment.theta - self._current_alignment.theta))
        if theta_delta > self._max_bootstrap_delta_rad:
            self.get_logger().warn(
                "Rejecting calibration alignment: theta delta %.2fdeg > %.2fdeg"
                % (math.degrees(theta_delta), math.degrees(self._max_bootstrap_delta_rad))
            )
            return None
        translation_delta = math.hypot(
            raw_alignment.tx - self._current_alignment.tx,
            raw_alignment.ty - self._current_alignment.ty,
        )
        if translation_delta > self._max_calibration_translation_delta_m:
            self.get_logger().warn(
                "Rejecting calibration alignment: translation delta %.2fm > %.2fm"
                % (translation_delta, self._max_calibration_translation_delta_m)
            )
            return None

        self._alignment_revision += 1
        accepted = Alignment2D(
            theta=raw_alignment.theta,
            tx=raw_alignment.tx,
            ty=raw_alignment.ty,
            source="waypoint_calibration",
            revision=self._alignment_revision,
        )
        self._current_alignment = accepted
        self._bootstrap_alignment = accepted
        self._raw_alignment = accepted
        return accepted

    def _handle_pending_calibration_request(self) -> None:
        request = self._pending_calibration_request
        if request is None:
            return
        self._pending_calibration_request = None
        waypoint_index, waypoint_name = request
        self._publish_calibration_status(
            "CALIBRATION_STARTED|%d|%s" % (waypoint_index + 1, waypoint_name)
        )
        try:
            calibration_fix = self._wait_for_stable_fix()
            current_pose = self._lookup_current_pose("ALIGNER_WAITING_FOR_MAP_TF")
        except Exception as exc:
            self.get_logger().warn(
                "Calibration failed at %s: %s" % (waypoint_name, exc)
            )
            self._publish_calibration_status(
                "CALIBRATION_TIMEOUT|%d|%s" % (waypoint_index + 1, waypoint_name)
            )
            return

        calibration_enu_x, calibration_enu_y = self._projector.forward(
            calibration_fix["lat"], calibration_fix["lon"]
        )
        self._upsert_calibration_pair(
            "wp%d:%s" % (waypoint_index + 1, waypoint_name),
            calibration_enu_x,
            calibration_enu_y,
            current_pose[0],
            current_pose[1],
        )

        spread_m = (
            self._compute_pair_spread_m(self._calibration_pairs)
            if len(self._calibration_pairs) >= 2
            else 0.0
        )
        raw_alignment = self._solve_calibration_alignment(self._calibration_pairs)
        if raw_alignment is None:
            self._publish_calibration_status(
                "CALIBRATION_FAILED|%d|%s" % (waypoint_index + 1, waypoint_name)
            )
            return

        accepted_alignment = self._apply_calibration_alignment(raw_alignment)
        if accepted_alignment is None:
            self._publish_calibration_status(
                "CALIBRATION_FAILED|%d|%s" % (waypoint_index + 1, waypoint_name)
            )
            return

        self.get_logger().info(
            "Calibration alignment accepted at %s: theta=%.2fdeg tx=%.2f ty=%.2f pairs=%d spread=%.2fm"
            % (
                waypoint_name,
                math.degrees(accepted_alignment.theta),
                accepted_alignment.tx,
                accepted_alignment.ty,
                len(self._calibration_pairs),
                spread_m,
            )
        )
        self._publish_alignment(accepted_alignment, True)
        self._publish_calibration_status(
            "CALIBRATION_COMPLETE|%d|%s|%d|%d|%.2f"
            % (
                waypoint_index + 1,
                waypoint_name,
                accepted_alignment.revision,
                len(self._calibration_pairs),
                spread_m,
            )
        )

    def _trim_pairs(self, now_mono: float) -> None:
        while self._pair_buffer and now_mono - self._pair_buffer[0].stamp_mono > self._pair_window_s:
            self._pair_buffer.popleft()
        if self._pair_buffer:
            tail = self._pair_buffer[-1]
            self._last_pair_enu = (tail.enu_x, tail.enu_y)
            self._last_pair_map = (tail.map_x, tail.map_y)
        else:
            self._last_pair_enu = None
            self._last_pair_map = None

    def _ingest_latest_fix_pair(self) -> bool:
        msg = self._latest_fix
        if not valid_fix(msg) or self._current_alignment is None:
            return False

        key = self._sample_key(msg)
        if key == self._last_fix_key:
            return False
        self._last_fix_key = key

        current_xy = self._try_lookup_current_xy()
        if current_xy is None:
            return False

        enu_x, enu_y = self._projector.forward(msg.latitude, msg.longitude)
        now_mono = time.monotonic()
        self._trim_pairs(now_mono)
        if self._pair_buffer:
            last_pair = self._pair_buffer[-1]
            dt_s = now_mono - last_pair.stamp_mono
            gps_jump_m = math.hypot(enu_x - last_pair.enu_x, enu_y - last_pair.enu_y)
            if dt_s <= 0.5 and gps_jump_m > 10.0:
                self.get_logger().warn(
                    "Dropping GPS jump sample: %.2fm over %.2fs" % (gps_jump_m, dt_s)
                )
                return False
        if self._last_pair_enu is not None and self._last_pair_map is not None:
            enu_step = math.hypot(enu_x - self._last_pair_enu[0], enu_y - self._last_pair_enu[1])
            map_step = math.hypot(
                current_xy[0] - self._last_pair_map[0],
                current_xy[1] - self._last_pair_map[1],
            )
            if enu_step < self._pair_min_spacing_m and map_step < self._pair_min_spacing_m:
                return False

        self._pair_buffer.append(
            AlignmentPair(
                stamp_mono=now_mono,
                enu_x=enu_x,
                enu_y=enu_y,
                map_x=current_xy[0],
                map_y=current_xy[1],
            )
        )
        self._trim_pairs(now_mono)
        self._last_pair_enu = (enu_x, enu_y)
        self._last_pair_map = current_xy
        return True

    def _step_alignment_towards(self, raw_alignment: Alignment2D) -> bool:
        if self._bootstrap_alignment is None or self._current_alignment is None:
            return False

        bootstrap_delta = abs(
            normalize_angle(raw_alignment.theta - self._bootstrap_alignment.theta)
        )
        if bootstrap_delta > self._max_bootstrap_delta_rad:
            self.get_logger().warn(
                "Rejecting raw GPS alignment: bootstrap delta %.2fdeg > %.2fdeg"
                % (math.degrees(bootstrap_delta), math.degrees(self._max_bootstrap_delta_rad))
            )
            return False

        bootstrap_translation_delta = math.hypot(
            raw_alignment.tx - self._bootstrap_alignment.tx,
            raw_alignment.ty - self._bootstrap_alignment.ty,
        )
        if bootstrap_translation_delta > self._max_bootstrap_translation_delta_m:
            self.get_logger().warn(
                "Rejecting raw GPS alignment: bootstrap translation delta %.2fm > %.2fm"
                % (bootstrap_translation_delta, self._max_bootstrap_translation_delta_m)
            )
            return False

        theta_delta = normalize_angle(raw_alignment.theta - self._current_alignment.theta)
        theta_step = max(-self._max_theta_step_rad, min(self._max_theta_step_rad, theta_delta))
        tx_delta = raw_alignment.tx - self._current_alignment.tx
        ty_delta = raw_alignment.ty - self._current_alignment.ty
        translation_norm = math.hypot(tx_delta, ty_delta)
        if translation_norm > self._max_translation_step_m and translation_norm > 1e-6:
            scale = self._max_translation_step_m / translation_norm
            tx_delta *= scale
            ty_delta *= scale

        next_theta = normalize_angle(self._current_alignment.theta + theta_step)
        next_tx = self._current_alignment.tx + tx_delta
        next_ty = self._current_alignment.ty + ty_delta
        changed = (
            abs(theta_step) > 1e-6
            or abs(tx_delta) > 1e-4
            or abs(ty_delta) > 1e-4
        )
        if not changed:
            return False

        if abs(theta_step) > self._max_alignment_step_warning_rad:
            self.get_logger().warn(
                "Large alignment theta step %.2fdeg" % math.degrees(theta_step)
            )

        self._alignment_revision += 1
        self._current_alignment = Alignment2D(
            theta=next_theta,
            tx=next_tx,
            ty=next_ty,
            source="global_aligner",
            revision=self._alignment_revision,
        )
        return True

    def run(self) -> bool:
        self._publish_status("ALIGNER_INITIALIZING")
        self.get_logger().info(f"Loaded route file: {self._route_file}")

        startup_fix = self._wait_for_stable_fix()
        self._validate_startup(startup_fix)
        startup_heading = self._wait_for_stable_heading()
        x0, y0, yaw0 = self._lookup_current_pose("ALIGNER_WAITING_FOR_MAP_TF")
        self._bootstrap_alignment = self._build_bootstrap_alignment(
            x0, y0, yaw0, startup_heading, startup_fix
        )
        self._current_alignment = self._bootstrap_alignment
        self._raw_alignment = self._bootstrap_alignment
        startup_enu_x, startup_enu_y = self._projector.forward(
            startup_fix["lat"], startup_fix["lon"]
        )
        self._upsert_calibration_pair("start_ref", startup_enu_x, startup_enu_y, x0, y0)

        self.get_logger().info(
            "Bootstrap ENU->map ready: yaw0=%.2fdeg launch_yaw=%.2fdeg rtk_heading=%s theta=%.2fdeg tx=%.2f ty=%.2f source=%s"
            % (
                math.degrees(yaw0),
                float(self._route["launch_yaw_deg"]),
                "%.2fdeg" % math.degrees(startup_heading)
                if startup_heading is not None
                else "unavailable",
                math.degrees(self._bootstrap_alignment.theta),
                self._bootstrap_alignment.tx,
                self._bootstrap_alignment.ty,
                self._bootstrap_alignment.source,
            )
        )
        self._publish_status("ALIGNER_BOOTSTRAP_READY")

        last_publish_mono = 0.0
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)
            self._handle_pending_calibration_request()
            ingested = self._ingest_latest_fix_pair()
            pairs = list(self._pair_buffer)
            spread_m = self._compute_pair_spread_m(pairs) if len(pairs) >= 2 else 0.0
            raw_alignment = self._solve_alignment(pairs)
            if raw_alignment is not None:
                self._raw_alignment = raw_alignment
                if self._step_alignment_towards(raw_alignment):
                    self.get_logger().info(
                        "Updated ENU->map: raw theta=%.2fdeg tx=%.2f ty=%.2f -> output theta=%.2fdeg tx=%.2f ty=%.2f pairs=%d spread=%.2fm"
                        % (
                            math.degrees(raw_alignment.theta),
                            raw_alignment.tx,
                            raw_alignment.ty,
                            math.degrees(self._current_alignment.theta),
                            self._current_alignment.tx,
                            self._current_alignment.ty,
                            len(pairs),
                            spread_m,
                        )
                    )
            elif ingested:
                self._publish_status(
                    "ALIGNER_ACCUMULATING|%d|%.2f" % (len(pairs), spread_m)
                )

            now_mono = time.monotonic()
            if now_mono - last_publish_mono >= self._publish_period_s:
                self._publish_alignment(self._current_alignment, self._current_alignment is not None)
                self._publish_debug(len(pairs), spread_m, self._raw_alignment, self._current_alignment)
                if raw_alignment is not None:
                    self._publish_status(
                        "ALIGNER_TRACKING|%d|%.2f|%.2f"
                        % (
                            len(pairs),
                            spread_m,
                            math.degrees(self._current_alignment.theta),
                        )
                    )
                last_publish_mono = now_mono

        return True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GPSGlobalAligner()
    ok = False
    try:
        ok = node.run()
    except KeyboardInterrupt:
        node._publish_status("ALIGNER_INTERRUPTED")
    except Exception as exc:
        if is_rcl_context_shutdown_error_message(str(exc)):
            ok = True
        else:
            node.get_logger().error(str(exc))
            node._publish_status(f"ALIGNER_ABORTED: {exc}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
