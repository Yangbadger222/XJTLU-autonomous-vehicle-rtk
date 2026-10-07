"""Typed ROS boundary for measured active-road evidence and view candidates.

This node persists acquisition-time odom geometry idempotently. Global
anchors require the existing RTK authority, a fresh map/odom transform and
explicitly verified uncertainty. Actual EGO queries own view selection.
"""
from __future__ import annotations

from pathlib import Path
import math
import os
import tempfile

try:
    import rclpy
    from rclpy.node import Node
    from research_interfaces.msg import (RoadEvent, RoadEvidence2D,RoadGraphUpdate2D)
    from std_msgs.msg import Bool, String
    from tf2_ros import Buffer, TransformListener
except ImportError:  # permits ROS-free contract tests on the workstation
    rclpy = None
    RoadEvent = RoadEvidence2D = RoadGraphUpdate2D = object
    Node = object

from research_runtime.runtime_paths import research_path
from research_runtime.safety_bridge import rtk_mode_is_allowed

from research_runtime.active_road import EvidenceState, EvidenceStore, RoadEvidence


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
        self.declare_parameter("event_topic", "/research/road_events")
        self.declare_parameter("reload_period_s", 0.20)
        self.declare_parameter("global_anchor_uncertainty_verified", False)
        self.declare_parameter("global_anchor_uncertainty_m", -1.0)
        self.declare_parameter("localization_session_id", "")
        self._session_id=str(self.get_parameter("localization_session_id").value)
        # Producers namespace submap IDs with this LIO initialization epoch.
        # An unknown epoch can retain local evidence but cannot global-anchor it.
        from rclpy.qos import QoSProfile,DurabilityPolicy
        self._session_pub=self.create_publisher(String,"/research/localization_session_id",
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._session_pub.publish(String(data=self._session_id))
        self._authority = False
        self._authority_received = None
        self._authority_mode,self._mode_received="UNKNOWN",0.
        self._tf_integrity,self._tf_received=False,0.
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.create_subscription(Bool, "/localization_authority/motion_allowed",
                                 self._authority_callback, 10)
        self.create_subscription(Bool,"/research/tf_integrity",self._tf_callback,10)
        self.create_subscription(String,"/localization_authority/mode",self._mode_callback,10)
        self._path = research_path(str(self.get_parameter("evidence_store_path").value))
        self._store: EvidenceStore | None = None
        self._store_mtime_ns: int | None = None
        self._last_rejection = ""
        self._ack_pub=self.create_publisher(String,"/research/road_evidence_ack",1000)
        self._event_pub = self.create_publisher(
            RoadEvent, str(self.get_parameter("event_topic").value), 10)
        self.create_subscription(
            RoadEvidence2D, str(self.get_parameter("evidence_topic").value),
            self._evidence_callback, 1000)
        self.create_subscription(RoadGraphUpdate2D,"/research/road_graph_updates",self._graph_update_callback,100)
        self._timer = self.create_timer(
            max(0.1, float(self.get_parameter("reload_period_s").value)), self._reload)
        self.create_timer(.10, self._refresh_anchors)
        self._reload()

    def _reject(self, reason: str) -> None:
        if reason != self._last_rejection:
            self.get_logger().warning(f"active-road evidence withheld: {reason}")
            self._last_rejection = reason

    def _authority_callback(self, msg):
        self._authority = msg.data is True
        self._authority_received = self.get_clock().now()

    def _mode_callback(self,msg):
        self._authority_mode,self._mode_received=str(msg.data),self.get_clock().now().nanoseconds*1e-9

    def _tf_callback(self,msg):
        self._tf_integrity,self._tf_received=msg.data is True,self.get_clock().now().nanoseconds*1e-9

    def _refresh_anchors(self):
        if self._store is None:
            return
        now = self.get_clock().now()
        fresh = self._authority_received is not None and 0 <= (now-self._authority_received).nanoseconds*1e-9 <= .5
        uncertainty = float(self.get_parameter("global_anchor_uncertainty_m").value)
        epoch=now.nanoseconds*1e-9
        valid = (self._authority and fresh and rtk_mode_is_allowed(self._authority_mode,epoch-self._mode_received) and
                 self._tf_integrity and 0<=epoch-self._tf_received<=.20 and
                 self.get_parameter("global_anchor_uncertainty_verified").value is True and
                 math.isfinite(uncertainty) and uncertainty >= 0)
        transform = None
        if valid:
            try:
                transform = self._tf_buffer.lookup_transform("map", "odom",
                    now-rclpy.duration.Duration(seconds=.10))
                age = (now-rclpy.time.Time.from_msg(transform.header.stamp)).nanoseconds*1e-9
                q = transform.transform.rotation
                valid = (0 <= age <= .30 and abs(q.x)+abs(q.y) < 1e-6 and
                         abs(q.z*q.z+q.w*q.w-1.) < 1e-3)
            except Exception:
                valid = False
        current_submaps={e.local_submap_id for e in self._store.evidence()
            if self._session_id and e.local_submap_id.startswith(self._session_id+"/")}
        changed = False
        if not valid:
            changed = self._store.mark_anchors_stale(current_submaps)
        else:
            p,q = transform.transform.translation,transform.transform.rotation
            xy_yaw = (p.x,p.y,2*math.atan2(q.z,q.w))
            # Anchor a new submap once. Recovery reanchors stale local UUIDs;
            # equal transforms do not rewrite versions; verified RTK corrections
            # update the existing anchor while preserving every local UUID.
            # A current map->odom belongs only to the current LIO epoch.
            # Historical epochs retain their own verified anchors. Reanchoring
            # a historical epoch requires independent registration, never this TF.
            for identity in current_submaps:
                anchor = self._store.submap_anchors.get(identity)
                if (anchor is None or anchor["state"] == "STALE" or
                    math.dist(anchor["map_from_local_xyyaw"][:2],xy_yaw[:2])>1e-6 or
                    abs(math.atan2(math.sin(anchor["map_from_local_xyyaw"][2]-xy_yaw[2]),
                                   math.cos(anchor["map_from_local_xyyaw"][2]-xy_yaw[2])))>1e-6):
                    self._store.anchor_submap(identity,xy_yaw,stamp=_stamp_seconds(transform.header.stamp),
                        uncertainty_m=uncertainty,authority_valid=True)
                    changed = True
        if changed:
            _atomic_save(self._store,self._path)
            self._store_mtime_ns = self._path.stat().st_mtime_ns

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
            if state == EvidenceState.TRAVERSED:
                raise ValueError("TRAVERSED needs a checked vehicle trace, not an observation message")
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
            self._ack_pub.publish(String(data=evidence.evidence_id))
            self._publish_evidence_event(evidence)
            self._last_rejection = ""
        except (TypeError, ValueError, OSError) as exc:
            self._reject(f"evidence rejected: {exc}")

    def _graph_update_callback(self,msg):
        if self._store is None or msg.header.frame_id!="odom" or not _stamp_is_set(msg.header.stamp):return
        if not self._session_id or not msg.local_submap_id.startswith(self._session_id+"/"):
            self._reject("graph update does not belong to current localization session")
            return
        now=self.get_clock().now().nanoseconds*1e-9
        stamp=_stamp_seconds(msg.header.stamp)
        if not 0<=now-stamp<=.50:return
        try:
            update=dict(update_id=msg.update_id,prior_version=msg.prior_version,start_node_id=msg.start_node_id,
                end_node_id=msg.end_node_id,geometry_xy=[(p.x,p.y) for p in msg.geometry],stamp=stamp,
                supported_width_m=msg.supported_width_m,source=msg.source,local_submap_id=msg.local_submap_id,
                pose_uncertainty_m=msg.pose_uncertainty_m,evidence_ids=list(msg.evidence_ids))
            if self._store.add_graph_update(update):
                _atomic_save(self._store,self._path);self._store_mtime_ns=self._path.stat().st_mtime_ns
        except (TypeError,ValueError,KeyError,OSError) as exc:self._reject('graph increment rejected: '+str(exc))

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

    def _candidate_callback(self, msg):
        self._reject("external boolean reachability cannot authorize an observation; use actual EGO query")


def main(args=None):
    if rclpy is None:
        raise RuntimeError("ROS 2 rclpy and research_interfaces are required on the target")
    rclpy.init(args=args)
    node = ActiveRoadEvidenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
