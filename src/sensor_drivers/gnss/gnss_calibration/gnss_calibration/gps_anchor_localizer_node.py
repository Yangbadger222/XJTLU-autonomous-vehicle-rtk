#!/usr/bin/env python3

from __future__ import annotations

import math
from collections import deque
from pathlib import Path

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int32, String
from tf2_ros import Buffer, TransformException, TransformListener
import yaml

try:
    from pyproj import Transformer
    from pyproj.enums import TransformDirection

    PYPROJ_AVAILABLE = True
except ImportError:
    Transformer = None
    TransformDirection = None
    PYPROJ_AVAILABLE = False


EARTH_RADIUS_M = 6378137.0


def euclidean_xy(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


class LocalENUProjector:
    def __init__(self, origin_lat: float, origin_lon: float, origin_alt: float = 0.0) -> None:
        self.origin_lat = float(origin_lat)
        self.origin_lon = float(origin_lon)
        self.origin_alt = float(origin_alt)
        self._origin_lat_rad = math.radians(self.origin_lat)

    def forward(self, lat: float, lon: float, alt: float) -> tuple[float, float, float]:
        x = (
            math.radians(float(lon) - self.origin_lon)
            * EARTH_RADIUS_M
            * math.cos(self._origin_lat_rad)
        )
        y = math.radians(float(lat) - self.origin_lat) * EARTH_RADIUS_M
        z = float(alt) - self.origin_alt
        return float(x), float(y), float(z)

    def inverse(self, x: float, y: float, z: float) -> tuple[float, float, float]:
        lon = self.origin_lon + math.degrees(
            float(x) / (EARTH_RADIUS_M * math.cos(self._origin_lat_rad))
        )
        lat = self.origin_lat + math.degrees(float(y) / EARTH_RADIUS_M)
        alt = self.origin_alt + float(z)
        return float(lat), float(lon), float(alt)


class GPSAnchorLocalizer(Node):
    def __init__(self) -> None:
        super().__init__("gps_anchor_localizer")

        self.declare_parameter("scene_points_file", "")
        self.declare_parameter("enu_origin_lat", 0.0)
        self.declare_parameter("enu_origin_lon", 0.0)
        self.declare_parameter("enu_origin_alt", 0.0)
        self.declare_parameter("anchor_match_radius_m", 8.0)
        self.declare_parameter("ambiguity_margin_m", 3.0)
        self.declare_parameter("fix_sample_count", 10)
        self.declare_parameter("fix_spread_max_m", 2.0)
        self.declare_parameter("fix_sigma_xy_max_m", 6.0)
        self.declare_parameter("nav_ready_map_residual_m", 4.0)
        self.declare_parameter("nav_ready_required_consecutive_samples", 3)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")

        self.scene_points_file = Path(
            str(self.get_parameter("scene_points_file").value)
        ).expanduser()
        self.origin_lat = float(self.get_parameter("enu_origin_lat").value)
        self.origin_lon = float(self.get_parameter("enu_origin_lon").value)
        self.origin_alt = float(self.get_parameter("enu_origin_alt").value)
        self.anchor_match_radius_m = float(self.get_parameter("anchor_match_radius_m").value)
        self.ambiguity_margin_m = float(self.get_parameter("ambiguity_margin_m").value)
        self.fix_sample_count = int(self.get_parameter("fix_sample_count").value)
        self.fix_spread_max_m = float(self.get_parameter("fix_spread_max_m").value)
        self.fix_sigma_xy_max_m = float(self.get_parameter("fix_sigma_xy_max_m").value)
        self.nav_ready_map_residual_m = float(self.get_parameter("nav_ready_map_residual_m").value)
        self.nav_ready_required_consecutive_samples = int(
            self.get_parameter("nav_ready_required_consecutive_samples").value
        )
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)

        self.transformer = self._build_transformer()
        self.scene_nodes, self.anchor_nodes = self._load_scene_points()
        self.fix_window: deque[dict] = deque(maxlen=self.fix_sample_count)
        self.nav_ready_hits = 0
        self.current_state = "NO_FIX"
        self.current_anchor_id: int | None = None
        self.session_anchor: dict | None = None
        self.session_offset_xyz: tuple[float, float, float] | None = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.gnss_pub = self.create_publisher(NavSatFix, "/gnss", 10)
        self.status_pub = self.create_publisher(String, "/gps_system/status", 10)
        self.anchor_pub = self.create_publisher(String, "/gps_system/nearest_anchor", 10)
        self.anchor_id_pub = self.create_publisher(Int32, "/gps_system/nearest_anchor_id", 10)
        self.create_subscription(NavSatFix, "/fix", self._fix_callback, 10)

        self.get_logger().info(
            "GPS anchor localizer ready: scene=%s anchors=%d radius=%.1fm"
            % (self.scene_points_file, len(self.anchor_nodes), self.anchor_match_radius_m)
        )
        self._publish_anchor(None)
        self._publish_state("NO_FIX")

    def _build_transformer(self):
        if not PYPROJ_AVAILABLE:
            return LocalENUProjector(self.origin_lat, self.origin_lon, self.origin_alt)

        pipeline = (
            "+proj=pipeline "
            "+step +proj=cart +ellps=WGS84 "
            f"+step +proj=topocentric +ellps=WGS84 +lat_0={self.origin_lat} "
            f"+lon_0={self.origin_lon} +h_0={self.origin_alt}"
        )
        return Transformer.from_pipeline(pipeline)

    def _load_scene_points(self) -> tuple[dict[int, dict], dict[int, dict]]:
        if not self.scene_points_file.exists():
            raise RuntimeError(f"scene_points_file does not exist: {self.scene_points_file}")

        with open(self.scene_points_file, "r", encoding="utf-8") as scene_file:
            raw = yaml.safe_load(scene_file) or {}

        raw_nodes = raw.get("nodes", {})
        if not isinstance(raw_nodes, dict) or not raw_nodes:
            raise RuntimeError(f"scene_points_file has no valid nodes: {self.scene_points_file}")

        nodes: dict[int, dict] = {}
        anchors: dict[int, dict] = {}
        for raw_id, raw_node in raw_nodes.items():
            node_id = int(raw_id)
            node = {
                "id": node_id,
                "name": str(raw_node["name"]),
                "lat": float(raw_node["lat"]),
                "lon": float(raw_node["lon"]),
                "alt": float(raw_node.get("alt", self.origin_alt)),
                "x": float(raw_node["x"]),
                "y": float(raw_node["y"]),
                "z": float(raw_node.get("z", 0.0)),
                "anchor": bool(raw_node.get("anchor", False)),
                "dest": bool(raw_node.get("dest", False)),
            }
            nodes[node_id] = node
            if node["anchor"]:
                anchors[node_id] = node

        if not anchors:
            raise RuntimeError(f"scene_points_file has no anchor nodes: {self.scene_points_file}")

        return nodes, anchors

    def _latlon_to_enu(self, lat: float, lon: float, alt: float) -> tuple[float, float, float]:
        if not PYPROJ_AVAILABLE:
            return self.transformer.forward(lat, lon, alt)

        x, y, z = self.transformer.transform(lon, lat, alt, radians=False)
        return float(x), float(y), float(z)

    def _enu_to_latlon(self, x: float, y: float, z: float) -> tuple[float, float, float]:
        if not PYPROJ_AVAILABLE:
            return self.transformer.inverse(x, y, z)

        lon, lat, alt = self.transformer.transform(
            x,
            y,
            z,
            radians=False,
            direction=TransformDirection.INVERSE,
        )
        return float(lat), float(lon), float(alt)

    def _fix_sigma_xy_m(self, msg: NavSatFix) -> float:
        if msg.position_covariance_type == NavSatFix.COVARIANCE_TYPE_UNKNOWN:
            return float("inf")
        var_x = max(float(msg.position_covariance[0]), 0.0)
        var_y = max(float(msg.position_covariance[4]), 0.0)
        return math.sqrt(max(var_x, var_y))

    def _max_spread_m(self) -> float:
        if len(self.fix_window) < 2:
            return 0.0
        max_spread = 0.0
        samples = list(self.fix_window)
        for i in range(len(samples)):
            for j in range(i + 1, len(samples)):
                max_spread = max(
                    max_spread,
                    euclidean_xy(
                        (samples[i]["x"], samples[i]["y"]),
                        (samples[j]["x"], samples[j]["y"]),
                    ),
                )
        return max_spread

    def _window_average(self) -> dict:
        samples = list(self.fix_window)
        count = len(samples)
        return {
            "lat": sum(sample["lat"] for sample in samples) / count,
            "lon": sum(sample["lon"] for sample in samples) / count,
            "alt": sum(sample["alt"] for sample in samples) / count,
            "x": sum(sample["x"] for sample in samples) / count,
            "y": sum(sample["y"] for sample in samples) / count,
            "z": sum(sample["z"] for sample in samples) / count,
            "sigma_xy_m": sum(sample["sigma_xy_m"] for sample in samples) / count,
            "last_msg": samples[-1]["msg"],
        }

    def _sample_from_msg(self, msg: NavSatFix) -> dict:
        x, y, z = self._latlon_to_enu(msg.latitude, msg.longitude, msg.altitude)
        return {
            "lat": float(msg.latitude),
            "lon": float(msg.longitude),
            "alt": float(msg.altitude),
            "x": x,
            "y": y,
            "z": z,
            "sigma_xy_m": self._fix_sigma_xy_m(msg),
            "msg": msg,
            "last_msg": msg,
        }

    def _lookup_map_pose(self) -> tuple[float, float] | None:
        try:
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
            )
        except TransformException:
            return None

        return (
            float(transform.transform.translation.x),
            float(transform.transform.translation.y),
        )

    def _match_anchor(self, avg_sample: dict) -> tuple[str, dict | None]:
        candidates = []
        for anchor in self.anchor_nodes.values():
            distance = euclidean_xy((avg_sample["x"], avg_sample["y"]), (anchor["x"], anchor["y"]))
            candidates.append((distance, anchor))

        if not candidates:
            return "NO_ANCHOR", None

        candidates.sort(key=lambda item: item[0])
        best_distance, best_anchor = candidates[0]
        if best_distance > self.anchor_match_radius_m:
            return "NO_ANCHOR", None

        if len(candidates) > 1:
            second_distance = candidates[1][0]
            if (second_distance - best_distance) <= self.ambiguity_margin_m:
                return "AMBIGUOUS_ANCHOR", None

        return "GNSS_READY", best_anchor

    def _publish_state(self, state: str) -> None:
        if state != self.current_state:
            self.get_logger().info(f"gps system state -> {state}")
            self.current_state = state
        self.status_pub.publish(String(data=state))

    def _publish_anchor(self, anchor: dict | None) -> None:
        if anchor is None:
            self.current_anchor_id = None
            self.anchor_pub.publish(String(data=""))
            self.anchor_id_pub.publish(Int32(data=-1))
            return

        self.current_anchor_id = int(anchor["id"])
        self.anchor_pub.publish(String(data=anchor["name"]))
        self.anchor_id_pub.publish(Int32(data=int(anchor["id"])))

    def _clear_session(self) -> None:
        self.nav_ready_hits = 0
        self.session_anchor = None
        self.session_offset_xyz = None
        self._publish_anchor(None)

    def _lock_session_anchor(self, avg_sample: dict, anchor: dict) -> None:
        offset_x = float(anchor["x"]) - float(avg_sample["x"])
        offset_y = float(anchor["y"]) - float(avg_sample["y"])
        offset_z = float(anchor["z"]) - float(avg_sample["z"])
        self.session_anchor = anchor
        self.session_offset_xyz = (offset_x, offset_y, offset_z)
        self._publish_anchor(anchor)
        self.get_logger().info(
            "locked anchor=%s offset=(%.3f, %.3f, %.3f)m"
            % (anchor["name"], offset_x, offset_y, offset_z)
        )

    def _corrected_sample(self, avg_sample: dict) -> dict:
        if self.session_offset_xyz is None:
            raise RuntimeError("session offset is not available")

        corrected_x = float(avg_sample["x"]) + self.session_offset_xyz[0]
        corrected_y = float(avg_sample["y"]) + self.session_offset_xyz[1]
        corrected_z = float(avg_sample["z"]) + self.session_offset_xyz[2]
        corrected_lat, corrected_lon, corrected_alt = self._enu_to_latlon(
            corrected_x, corrected_y, corrected_z
        )

        return {
            "lat": corrected_lat,
            "lon": corrected_lon,
            "alt": corrected_alt,
            "x": corrected_x,
            "y": corrected_y,
            "z": corrected_z,
            "sigma_xy_m": float(avg_sample["sigma_xy_m"]),
            "last_msg": avg_sample["last_msg"],
        }

    def _publish_gnss(self, corrected_sample: dict) -> None:
        latest_msg: NavSatFix = corrected_sample["last_msg"]
        corrected = NavSatFix()
        corrected.header = latest_msg.header
        corrected.status = latest_msg.status
        corrected.latitude = float(corrected_sample["lat"])
        corrected.longitude = float(corrected_sample["lon"])
        corrected.altitude = float(corrected_sample["alt"])
        corrected.position_covariance = latest_msg.position_covariance
        corrected.position_covariance_type = latest_msg.position_covariance_type
        self.gnss_pub.publish(corrected)

    def _fix_callback(self, msg: NavSatFix) -> None:
        if msg.status.status < 0:
            self.fix_window.clear()
            self._clear_session()
            self._publish_state("NO_FIX")
            return

        if not all(math.isfinite(value) for value in (msg.latitude, msg.longitude, msg.altitude)):
            self.fix_window.clear()
            self._clear_session()
            self._publish_state("NO_FIX")
            return

        if self.session_anchor is None or self.session_offset_xyz is None:
            self.fix_window.append(self._sample_from_msg(msg))

            if len(self.fix_window) < self.fix_sample_count:
                self.nav_ready_hits = 0
                self._publish_anchor(None)
                self._publish_state("UNSTABLE_FIX")
                return

            spread_m = self._max_spread_m()
            avg_sample = self._window_average()
            if (
                spread_m > self.fix_spread_max_m
                or avg_sample["sigma_xy_m"] > self.fix_sigma_xy_max_m
            ):
                self.fix_window.clear()
                self.nav_ready_hits = 0
                self._publish_anchor(None)
                self._publish_state("UNSTABLE_FIX")
                return

            anchor_state, anchor = self._match_anchor(avg_sample)
            if anchor is None:
                self.fix_window.clear()
                self.nav_ready_hits = 0
                self._publish_anchor(None)
                self._publish_state(anchor_state)
                return

            self._lock_session_anchor(avg_sample, anchor)
            corrected_sample = self._corrected_sample(avg_sample)
        else:
            self._publish_anchor(self.session_anchor)
            current_sample = self._sample_from_msg(msg)
            corrected_sample = self._corrected_sample(current_sample)
            if current_sample["sigma_xy_m"] > self.fix_sigma_xy_max_m:
                self.nav_ready_hits = 0
                self._publish_state("GNSS_READY")
                return

        self._publish_gnss(corrected_sample)

        map_pose = self._lookup_map_pose()
        if map_pose is None:
            self.nav_ready_hits = 0
            self._publish_state("GNSS_READY")
            return

        residual_m = euclidean_xy(map_pose, (corrected_sample["x"], corrected_sample["y"]))
        if residual_m <= self.nav_ready_map_residual_m:
            self.nav_ready_hits += 1
        else:
            self.nav_ready_hits = 0

        if self.nav_ready_hits >= self.nav_ready_required_consecutive_samples:
            self._publish_state("NAV_READY")
        else:
            self._publish_state("GNSS_READY")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GPSAnchorLocalizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
