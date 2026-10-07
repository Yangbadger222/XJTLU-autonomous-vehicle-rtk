"""Fail-closed ROS boundary for persisted active-road evidence.

The node does not invent a road from a missing file.  It publishes a map
version only when an EvidenceStore can be loaded and relays an external
odom-frame reference path only while that version is valid.  Global graph
selection and camera/LiDAR evidence ingestion remain explicit upstream
contracts.
"""
from __future__ import annotations

from pathlib import Path

try:
    import rclpy
    from nav_msgs.msg import Path as PathMessage
    from rclpy.node import Node
    from std_msgs.msg import String
except ImportError:  # permits source tests without ROS 2 on the workstation
    rclpy = None
    PathMessage = String = object
    Node = object

from research_runtime.runtime_paths import research_path

from research_runtime.active_road import EvidenceStore


def _valid_version(value: str) -> bool:
    return bool(value and value.strip() and value.strip().upper() != "UNKNOWN")


class ActiveRoadMapNode(Node if rclpy else object):
    def __init__(self):
        super().__init__("active_road_map")
        self.declare_parameter(
            "evidence_store_path",
            "runtime-data/research/active_road/evidence.json")
        self.declare_parameter("map_version_topic", "/research/map_version")
        self.declare_parameter(
            "road_reference_input_topic", "/research/road_reference_input")
        self.declare_parameter(
            "road_reference_output_topic", "/research/road_reference")
        self.declare_parameter("reload_period_s", 1.0)
        self._path = research_path(str(self.get_parameter("evidence_store_path").value))
        self._store: EvidenceStore | None = None
        self._store_mtime_ns: int | None = None
        self._last_rejection = ""
        self._version_pub = self.create_publisher(
            String, str(self.get_parameter("map_version_topic").value), 10)
        self._reference_pub = self.create_publisher(
            PathMessage,
            str(self.get_parameter("road_reference_output_topic").value), 10)
        self.create_subscription(
            PathMessage,
            str(self.get_parameter("road_reference_input_topic").value),
            self._reference_callback, 10)
        self._reload_timer = self.create_timer(
            max(0.1, float(self.get_parameter("reload_period_s").value)),
            self._reload_and_publish)
        self._reload_and_publish()

    def _reject(self, reason: str) -> None:
        if reason != self._last_rejection:
            self.get_logger().warning(f"active-road reference withheld: {reason}")
            self._last_rejection = reason

    def _reload_and_publish(self) -> None:
        try:
            stat = self._path.stat()
        except OSError:
            self._store = None
            self._store_mtime_ns = None
        else:
            if stat.st_mtime_ns != self._store_mtime_ns:
                try:
                    self._store = EvidenceStore.load(self._path)
                    self._store_mtime_ns = stat.st_mtime_ns
                    self._last_rejection = ""
                except (OSError, KeyError, TypeError, ValueError) as exc:
                    self._store = None
                    self._store_mtime_ns = stat.st_mtime_ns
                    self._reject(f"evidence store invalid: {exc}")
        msg = String()
        msg.data = self._store.map_version if self._store and _valid_version(self._store.map_version) else "UNKNOWN"
        self._version_pub.publish(msg)

    def _reference_callback(self, msg: PathMessage) -> None:
        if self._store is None or not _valid_version(self._store.map_version):
            self._reject("no valid persisted map version")
            return
        if str(msg.header.frame_id) != "odom":
            self._reject("reference path must already be in odom frame")
            return
        if not msg.poses:
            self._reject("reference path is empty")
            return
        self._reference_pub.publish(msg)
        self._last_rejection = ""


def main(args=None):
    if rclpy is None:
        raise RuntimeError("ROS 2 rclpy is required on the Humble target")
    rclpy.init(args=args)
    node = ActiveRoadMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
