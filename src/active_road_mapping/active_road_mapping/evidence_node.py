"""Typed ROS boundary for measured active-road evidence and view candidates.

This node never invents a camera model, a global transform, a road event, or
simulator truth.  It accepts geometry that an upstream sensor/TF pipeline has
already expressed in ``odom`` at acquisition time, persists it idempotently,
and chooses only among externally supplied candidates whose hard safety flags
are true.  A missing or invalid EvidenceStore blocks both operations.
"""
from __future__ import annotations

from pathlib import Path
import copy
import math
import os
import tempfile

try:
    import rclpy
    from geometry_msgs.msg import Point
    from rclpy.node import Node
    from research_interfaces.msg import (ObservationGoal, RoadEvent, RoadEvidence2D)
except ImportError:  # permits ROS-free contract tests on the workstation
    rclpy = None
    Point = ObservationGoal = RoadEvent = RoadEvidence2D = object
    Node = object

from research_runtime.active_road import (EvidenceState, EvidenceStore,
                                          ObservationCandidate, RoadEvidence,
                                          choose_observation)


def _valid_version(value: str) -> bool:
    value = str(value).strip()
    return bool(value and value.upper() != "UNKNOWN")


def _stamp_seconds(stamp) -> float:
    return float(stamp.sec) + 1e-9 * float(stamp.nanosec)


def _stamp_is_set(stamp) -> bool:
    """Evidence must carry acquisition time; zero cannot mean an unknown epoch."""
    try:
        sec = int(stamp.sec)
        nanosec = int(stamp.nanosec)
    except (AttributeError, TypeError, ValueError):
        return False
    return sec >= 0 and 0 <= nanosec < 1_000_000_000 and (sec > 0 or nanosec > 0)


def _finite_point(point) -> bool:
    return all(math.isfinite(float(value)) for value in (point.x, point.y, point.z))


def _atomic_save(store: EvidenceStore, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=str(path.parent))
    try:
        os.close(fd)
        store.save(temporary)
        Path(temporary).replace(path)
    finally:
        try:
            Path(temporary).unlink()
        except FileNotFoundError:
            pass


class ActiveRoadEvidenceNode(Node if rclpy else object):
    def __init__(self):
        super().__init__("active_road_evidence")
        self.declare_parameter("evidence_store_path", "runtime-data/research/active_road/evidence.json")
        self.declare_parameter("evidence_topic", "/research/road_evidence")
        self.declare_parameter("candidate_topic", "/research/observation_candidate")
        self.declare_parameter("event_topic", "/research/road_events")
        self.declare_parameter("goal_topic", "/research/observation_goal")
        self.declare_parameter("reload_period_s", 0.20)
        self._path = Path(str(self.get_parameter("evidence_store_path").value))
        self._store: EvidenceStore | None = None
        self._store_mtime_ns: int | None = None
        self._candidates: dict[str, dict[str, ObservationCandidate]] = {}
        self._candidate_messages: dict[str, dict[str, ObservationGoal]] = {}
        self._events: dict[str, RoadEvent] = {}
        self._last_rejection = ""
        self._event_pub = self.create_publisher(
            RoadEvent, str(self.get_parameter("event_topic").value), 10)
        self._goal_pub = self.create_publisher(
            ObservationGoal, str(self.get_parameter("goal_topic").value), 10)
        self.create_subscription(
            RoadEvidence2D, str(self.get_parameter("evidence_topic").value),
            self._evidence_callback, 10)
        self.create_subscription(
            ObservationGoal, str(self.get_parameter("candidate_topic").value),
            self._candidate_callback, 10)
        self._timer = self.create_timer(
            max(0.1, float(self.get_parameter("reload_period_s").value)), self._reload)
        self._reload()

    def _reject(self, reason: str) -> None:
        if reason != self._last_rejection:
            self.get_logger().warning("active-road evidence withheld: %s", reason)
            self._last_rejection = reason

    def _reload(self) -> None:
        try:
            stat = self._path.stat()
        except OSError:
            self._store = None
            self._store_mtime_ns = None
            return
        if stat.st_mtime_ns == self._store_mtime_ns:
            return
        try:
            loaded = EvidenceStore.load(self._path)
            if not _valid_version(loaded.map_version):
                raise ValueError("persisted map version is UNKNOWN")
        except (OSError, KeyError, TypeError, ValueError) as exc:
            self._store = None
            self._store_mtime_ns = stat.st_mtime_ns
            self._reject(f"evidence store invalid: {exc}")
            return
        self._store = loaded
        self._store_mtime_ns = stat.st_mtime_ns
        self._last_rejection = ""

    def _evidence_callback(self, msg: RoadEvidence2D) -> None:
        if self._store is None:
            self._reject("no valid persisted EvidenceStore")
            return
        if str(msg.header.frame_id) != "odom":
            self._reject("evidence geometry must already be in odom")
            return
        if not _stamp_is_set(msg.header.stamp):
            self._reject("evidence has no acquisition timestamp")
            return
        if not msg.evidence_id or not msg.source or not msg.local_submap_id:
            self._reject("evidence id/source/local_submap_id are required")
            return
        if len(msg.geometry) < 2 or not all(_finite_point(point) for point in msg.geometry):
            self._reject("evidence geometry must contain finite 3-D points")
            return
        try:
            state = EvidenceState(str(msg.state))
            depth = None
            if msg.valid_depth_min_m != 0.0 or msg.valid_depth_max_m != 0.0:
                depth = (float(msg.valid_depth_min_m), float(msg.valid_depth_max_m))
            evidence = RoadEvidence(
                evidence_id=str(msg.evidence_id),
                geometry_xy=[(float(point.x), float(point.y)) for point in msg.geometry],
                state=state,
                stamp=_stamp_seconds(msg.header.stamp),
                source=str(msg.source),
                local_submap_id=str(msg.local_submap_id),
                pose_uncertainty_m=float(msg.pose_uncertainty_m),
                observed_length_m=float(msg.observed_length_m),
                valid_depth_m=depth)
            added = self._store.add(evidence)
            if added:
                _atomic_save(self._store, self._path)
                self._store_mtime_ns = self._path.stat().st_mtime_ns
            self._publish_evidence_event(evidence)
            self._last_rejection = ""
        except (TypeError, ValueError, OSError) as exc:
            self._reject(f"evidence rejected: {exc}")

    def _publish_evidence_event(self, evidence: RoadEvidence) -> None:
        first = evidence.geometry_xy[0]
        last = evidence.geometry_xy[-1]
        event = RoadEvent()
        event.header.frame_id = "odom"
        event.header.stamp.sec = int(evidence.stamp)
        event.header.stamp.nanosec = int((evidence.stamp - int(evidence.stamp)) * 1e9)
        event.event_id = f"evidence:{evidence.evidence_id}"
        event.kind = "observed_geometry"
        event.x = 0.5 * (first[0] + last[0])
        event.y = 0.5 * (first[1] + last[1])
        event.observed_length_m = evidence.observed_length_m
        event.unknown_length_m = 0.0
        event.state = evidence.state.value
        event.impact = 0.0
        event.source = evidence.source
        self._event_pub.publish(event)

    def _candidate_callback(self, msg: ObservationGoal) -> None:
        if str(msg.header.frame_id) != "odom":
            self._reject("observation candidate must already be in odom")
            return
        if not msg.goal_id or not msg.event_id:
            self._reject("observation candidate id and event_id are required")
            return
        if not all(math.isfinite(float(value)) for value in
                   (msg.x, msg.y, msg.yaw, msg.score,
                    msg.cost, msg.observable_fraction)):
            self._reject("observation candidate values must be finite")
            return
        candidate = ObservationCandidate(
            candidate_id=str(msg.goal_id), event_id=str(msg.event_id),
            reachable=bool(msg.reachable), safe=bool(msg.safe),
            pose_trustworthy=bool(msg.pose_trustworthy), sensor_valid=bool(msg.sensor_valid),
            impact=float(msg.score), observable_fraction=float(msg.observable_fraction),
            cost=float(msg.cost), reason=str(msg.reason))
        bucket = self._candidates.setdefault(candidate.event_id, {})
        bucket[candidate.candidate_id] = candidate
        self._candidate_messages.setdefault(candidate.event_id, {})[
            candidate.candidate_id] = copy.deepcopy(msg)
        selected = choose_observation(bucket.values())
        if selected is None:
            return
        selected_message = self._candidate_messages[candidate.event_id][selected.candidate_id]
        output = ObservationGoal()
        output.header = selected_message.header
        output.header.frame_id = "odom"
        output.goal_id = selected.candidate_id
        output.event_id = selected.event_id
        output.x = float(selected_message.x)
        output.y = float(selected_message.y)
        output.yaw = float(selected_message.yaw)
        output.score = float(selected.score)
        output.cost = float(selected.cost)
        output.observable_fraction = float(selected.observable_fraction)
        output.reason = selected.reason or "task_aware_candidate"
        output.reachable = selected.reachable
        output.safe = selected.safe
        output.pose_trustworthy = selected.pose_trustworthy
        output.sensor_valid = selected.sensor_valid
        self._goal_pub.publish(output)


def main(args=None):
    if rclpy is None:
        raise RuntimeError("ROS 2 rclpy and research_interfaces are required on the target")
    rclpy.init(args=args)
    node = ActiveRoadEvidenceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
