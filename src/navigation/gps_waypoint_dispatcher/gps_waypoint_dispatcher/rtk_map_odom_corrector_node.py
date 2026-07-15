#!/usr/bin/env python3
from __future__ import annotations

import math
import time
from collections import deque
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import QuaternionStamped, TransformStamped
from nav_msgs.msg import Odometry
from nmea_msgs.msg import Sentence
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Float64MultiArray, String
from tf2_ros import TransformBroadcaster

from gps_waypoint_dispatcher.alignment_math import heading_quaternion_yaw_to_enu_yaw
from gps_waypoint_dispatcher.corridor_quality import parse_gga_quality
from gps_waypoint_dispatcher.rtk_authority import (
    CorrectionGate,
    CorrectionGateState,
    CorrectionReleaseMode,
    CorrectionReleaseState,
    Pose2D,
    PrerequisiteFailureKind,
    StampedPoseHistory,
    normalize_angle,
)
from gps_waypoint_dispatcher.scene_runtime import (
    FixedENUProjector,
    load_scene_points,
    quaternion_to_yaw,
    yaw_to_quaternion,
)


@dataclass(frozen=True)
class PendingHeading:
    stamp_s: float
    enu_yaw: float
    received_mono_s: float


@dataclass(frozen=True)
class PendingFix:
    stamp_s: float
    latitude: float
    longitude: float
    altitude: float
    received_mono_s: float


@dataclass(frozen=True)
class GgaQuality:
    stamp_s: float
    quality: int
    received_mono_s: float


def _stamp_s(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _finite_pose_from_odom(msg: Odometry) -> Pose2D | None:
    position = msg.pose.pose.position
    orientation = msg.pose.pose.orientation
    values = (
        position.x,
        position.y,
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w,
    )
    if not all(math.isfinite(float(value)) for value in values):
        return None
    yaw = quaternion_to_yaw(
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w,
    )
    if not math.isfinite(yaw):
        return None
    return Pose2D(float(position.x), float(position.y), yaw)


class RtkMapOdomCorrector(Node):
    def __init__(self) -> None:
        super().__init__("rtk_map_odom_corrector")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("fix_topic", "/fix")
        self.declare_parameter("heading_topic", "/heading")
        self.declare_parameter("nmea_topic", "/rtk/nmea_sentence")
        self.declare_parameter("lio_odom_topic", "/fastlio2/lio_odom")
        self.declare_parameter("alignment_topic", "/gps_corridor/enu_to_map")
        self.declare_parameter("scene_points_file", "")
        self.declare_parameter("use_scene_identity_alignment", False)
        self.declare_parameter("mode_topic", "/localization_authority/mode")
        self.declare_parameter("status_topic", "/localization_authority/status")
        self.declare_parameter(
            "diagnostics_topic", "/localization_authority/diagnostics"
        )
        self.declare_parameter(
            "motion_allowed_topic", "/localization_authority/motion_allowed"
        )
        self.declare_parameter("enu_origin_lat", 0.0)
        self.declare_parameter("enu_origin_lon", 0.0)
        self.declare_parameter("enu_origin_alt", 0.0)
        self.declare_parameter("heading_quaternion_yaw_is_compass", True)
        self.declare_parameter("publish_period_s", 0.10)
        self.declare_parameter("observation_fifo_capacity", 10)
        self.declare_parameter("max_pending_observation_s", 0.30)
        self.declare_parameter("fix_quality_wait_s", 0.25)
        self.declare_parameter("heading_quality_wait_s", 0.30)
        self.declare_parameter("max_future_stamp_s", 0.10)
        self.declare_parameter("max_odom_bracket_s", 0.20)
        self.declare_parameter("max_gga_heading_age_s", 1.50)
        self.declare_parameter("max_fix_age_s", 1.0)
        self.declare_parameter("max_heading_age_s", 1.0)
        self.declare_parameter("max_alignment_age_s", 1.0)
        self.declare_parameter("max_gga_age_s", 1.5)
        self.declare_parameter("max_lio_age_s", 0.20)
        self.declare_parameter("max_heading_for_fix_age_s", 0.30)
        self.declare_parameter("max_translation_rate_mps", 0.20)
        self.declare_parameter("max_yaw_rate_degps", 2.0)
        self.declare_parameter("heading_locked_innovation_deg", 15.0)
        self.declare_parameter("position_locked_innovation_m", 1.0)
        self.declare_parameter("heading_recovery_spread_deg", 5.0)
        self.declare_parameter("position_recovery_diameter_m", 0.30)
        self.declare_parameter("recovery_min_samples", 5)
        self.declare_parameter("recovery_min_span_s", 0.30)
        self.declare_parameter("gate_max_candidates", 20)
        self.declare_parameter("gate_max_failures", 5)
        self.declare_parameter("gate_processable_timeout_s", 1.0)
        self.declare_parameter("backlog_translation_m", 0.50)
        self.declare_parameter("backlog_yaw_deg", 5.0)
        self.declare_parameter("fault_translation_m", 2.0)
        self.declare_parameter("fault_yaw_deg", 20.0)
        self.declare_parameter("stopped_linear_rate_mps", 0.05)
        self.declare_parameter("stopped_yaw_rate_degps", 2.0)
        self.declare_parameter("stopped_confirmation_s", 1.0)
        self.declare_parameter("recovery_translation_m", 0.15)
        self.declare_parameter("recovery_yaw_deg", 2.0)
        self.declare_parameter("recovery_confirmation_s", 1.0)

        self._map_frame = str(self.get_parameter("map_frame").value)
        self._odom_frame = str(self.get_parameter("odom_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._fix_topic = str(self.get_parameter("fix_topic").value)
        self._heading_topic = str(self.get_parameter("heading_topic").value)
        self._nmea_topic = str(self.get_parameter("nmea_topic").value)
        self._lio_odom_topic = str(self.get_parameter("lio_odom_topic").value)
        self._alignment_topic = str(self.get_parameter("alignment_topic").value)
        self._scene_points_file = str(self.get_parameter("scene_points_file").value).strip()
        self._use_scene_identity_alignment = bool(
            self.get_parameter("use_scene_identity_alignment").value
        )
        self._mode_topic = str(self.get_parameter("mode_topic").value)
        self._status_topic = str(self.get_parameter("status_topic").value)
        self._diagnostics_topic = str(self.get_parameter("diagnostics_topic").value)
        self._motion_allowed_topic = str(
            self.get_parameter("motion_allowed_topic").value
        )
        self._heading_quaternion_yaw_is_compass = bool(
            self.get_parameter("heading_quaternion_yaw_is_compass").value
        )
        self._publish_period_s = float(self.get_parameter("publish_period_s").value)
        self._observation_fifo_capacity = int(
            self.get_parameter("observation_fifo_capacity").value
        )
        self._max_pending_observation_s = float(
            self.get_parameter("max_pending_observation_s").value
        )
        self._fix_quality_wait_s = float(
            self.get_parameter("fix_quality_wait_s").value
        )
        self._heading_quality_wait_s = float(
            self.get_parameter("heading_quality_wait_s").value
        )
        self._max_future_stamp_s = float(
            self.get_parameter("max_future_stamp_s").value
        )
        self._max_odom_bracket_s = float(
            self.get_parameter("max_odom_bracket_s").value
        )
        self._max_gga_heading_age_s = float(
            self.get_parameter("max_gga_heading_age_s").value
        )
        self._max_fix_age_s = float(self.get_parameter("max_fix_age_s").value)
        self._max_heading_age_s = float(
            self.get_parameter("max_heading_age_s").value
        )
        self._max_alignment_age_s = float(
            self.get_parameter("max_alignment_age_s").value
        )
        self._max_gga_age_s = float(self.get_parameter("max_gga_age_s").value)
        self._max_lio_age_s = float(self.get_parameter("max_lio_age_s").value)
        self._max_heading_for_fix_age_s = float(
            self.get_parameter("max_heading_for_fix_age_s").value
        )

        origin_lat = float(self.get_parameter("enu_origin_lat").value)
        origin_lon = float(self.get_parameter("enu_origin_lon").value)
        origin_alt = float(self.get_parameter("enu_origin_alt").value)
        if self._scene_points_file:
            scene = load_scene_points(self._scene_points_file)
            fixed_origin = scene.get("fixed_origin", {})
            try:
                origin_lat = float(fixed_origin["lat"])
                origin_lon = float(fixed_origin["lon"])
                origin_alt = float(fixed_origin.get("alt", 0.0))
            except (KeyError, TypeError, ValueError) as exc:
                raise RuntimeError(
                    f"scene_points_file has no valid fixed_origin: {self._scene_points_file}"
                ) from exc

        self._projector = FixedENUProjector(origin_lat, origin_lon, origin_alt)
        self._scene_identity_alignment = (
            (0.0, 0.0, 0.0, True) if self._use_scene_identity_alignment else None
        )
        self._lio_history = StampedPoseHistory(
            max_age_s=2.0,
            max_samples=200,
            frame_id=self._odom_frame,
            child_frame_id=self._base_frame,
        )
        gate_common = {
            "min_candidates": int(self.get_parameter("recovery_min_samples").value),
            "min_span_s": float(self.get_parameter("recovery_min_span_s").value),
            "max_candidates": int(self.get_parameter("gate_max_candidates").value),
            "max_consecutive_failures": int(
                self.get_parameter("gate_max_failures").value
            ),
            "processable_timeout_s": float(
                self.get_parameter("gate_processable_timeout_s").value
            ),
        }
        self._heading_gate = CorrectionGate.yaw(
            locked_threshold=math.radians(
                float(self.get_parameter("heading_locked_innovation_deg").value)
            ),
            recovery_threshold=math.radians(
                float(self.get_parameter("heading_recovery_spread_deg").value)
            ),
            **gate_common,
        )
        self._position_gate = CorrectionGate.translation(
            locked_threshold=float(
                self.get_parameter("position_locked_innovation_m").value
            ),
            recovery_threshold=float(
                self.get_parameter("position_recovery_diameter_m").value
            ),
            **gate_common,
        )
        self._release_state = CorrectionReleaseState(
            max_translation_rate_mps=float(
                self.get_parameter("max_translation_rate_mps").value
            ),
            max_yaw_rate_radps=math.radians(
                float(self.get_parameter("max_yaw_rate_degps").value)
            ),
            max_lio_age_s=self._max_lio_age_s,
            backlog_translation_m=float(
                self.get_parameter("backlog_translation_m").value
            ),
            backlog_yaw_rad=math.radians(
                float(self.get_parameter("backlog_yaw_deg").value)
            ),
            fault_translation_m=float(
                self.get_parameter("fault_translation_m").value
            ),
            fault_yaw_rad=math.radians(
                float(self.get_parameter("fault_yaw_deg").value)
            ),
            stopped_linear_rate_mps=float(
                self.get_parameter("stopped_linear_rate_mps").value
            ),
            stopped_yaw_rate_radps=math.radians(
                float(self.get_parameter("stopped_yaw_rate_degps").value)
            ),
            stopped_confirmation_s=float(
                self.get_parameter("stopped_confirmation_s").value
            ),
            recovery_translation_m=float(
                self.get_parameter("recovery_translation_m").value
            ),
            recovery_yaw_rad=math.radians(
                float(self.get_parameter("recovery_yaw_deg").value)
            ),
            recovery_confirmation_s=float(
                self.get_parameter("recovery_confirmation_s").value
            ),
        )
        self.get_logger().info(
            "RTK authority gates: yaw=%.1fdeg position=%.2fm recovery=%d/%.2fs; "
            "release=%.2fm/s %.1fdeg/s"
            % (
                float(self.get_parameter("heading_locked_innovation_deg").value),
                float(self.get_parameter("position_locked_innovation_m").value),
                gate_common["min_candidates"],
                gate_common["min_span_s"],
                float(self.get_parameter("max_translation_rate_mps").value),
                float(self.get_parameter("max_yaw_rate_degps").value),
            )
        )

        self._heading_queue: deque[PendingHeading] = deque()
        self._fix_queue: deque[PendingFix] = deque()
        self._gga_history: deque[GgaQuality] = deque(maxlen=100)
        self._accepted_heading_corrections: deque[tuple[float, float]] = deque(
            maxlen=100
        )
        self._last_heading_enqueue_stamp_s: float | None = None
        self._last_fix_enqueue_stamp_s: float | None = None
        self._latest_lio_stamp_s: float | None = None
        self._latest_lio_pose: Pose2D | None = None
        self._previous_lio_stamp_s: float | None = None
        self._previous_lio_pose: Pose2D | None = None
        self._local_linear_rate_mps = math.inf
        self._local_yaw_rate_radps = math.inf
        self._latest_lio_mono_s: float | None = None
        self._latest_fix_mono_s: float | None = None
        self._latest_heading_mono_s: float | None = None
        self._latest_gga_mono_s: float | None = None
        self._latest_alignment_mono_s: float | None = None
        self._latest_alignment: tuple[float, float, float, bool] | None = None
        self._bootstrap_alignment: tuple[float, float, float, bool] | None = None
        self._last_output: Pose2D | None = None
        self._last_release = None
        self._last_heading_innovation_rad: float | None = None
        self._last_position_innovation_m: float | None = None
        self._last_mode = ""
        self._last_status = ""

        self._mode_pub = self.create_publisher(String, self._mode_topic, 10)
        self._status_pub = self.create_publisher(String, self._status_topic, 10)
        self._diagnostics_pub = self.create_publisher(
            Float64MultiArray, self._diagnostics_topic, 10
        )
        self._motion_allowed_pub = self.create_publisher(
            Bool, self._motion_allowed_topic, 10
        )
        self._fix_sub = self.create_subscription(
            NavSatFix, self._fix_topic, self._fix_callback, 10
        )
        self._heading_sub = self.create_subscription(
            QuaternionStamped, self._heading_topic, self._heading_callback, 10
        )
        self._nmea_sub = self.create_subscription(
            Sentence, self._nmea_topic, self._nmea_callback, 50
        )
        self._lio_sub = self.create_subscription(
            Odometry, self._lio_odom_topic, self._lio_callback, 50
        )
        self._alignment_sub = self.create_subscription(
            Float64MultiArray, self._alignment_topic, self._alignment_callback, 10
        )
        self._tf_broadcaster = TransformBroadcaster(self)
        self._timer = self.create_timer(self._publish_period_s, self._timer_callback)

    def _ros_now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _valid_observation_stamp(self, stamp_s: float) -> bool:
        return (
            math.isfinite(stamp_s)
            and stamp_s > 0.0
            and stamp_s <= self._ros_now_s() + self._max_future_stamp_s
        )

    def _enqueue(self, queue: deque, observation) -> bool:
        if len(queue) >= self._observation_fifo_capacity:
            return False
        queue.append(observation)
        return True

    def _fix_callback(self, msg: NavSatFix) -> None:
        stamp_s = _stamp_s(msg.header.stamp)
        if (
            not self._valid_observation_stamp(stamp_s)
            or msg.status.status < 0
            or not all(
                math.isfinite(value)
                for value in (msg.latitude, msg.longitude, msg.altitude)
            )
            or (
                self._last_fix_enqueue_stamp_s is not None
                and stamp_s <= self._last_fix_enqueue_stamp_s
            )
        ):
            return
        received = time.monotonic()
        if self._enqueue(
            self._fix_queue,
            PendingFix(
                stamp_s,
                float(msg.latitude),
                float(msg.longitude),
                float(msg.altitude),
                received,
            ),
        ):
            self._last_fix_enqueue_stamp_s = stamp_s
            self._latest_fix_mono_s = received

    def _heading_callback(self, msg: QuaternionStamped) -> None:
        stamp_s = _stamp_s(msg.header.stamp)
        quaternion = msg.quaternion
        if not self._valid_observation_stamp(stamp_s) or not all(
            math.isfinite(value)
            for value in (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        ):
            return
        yaw = quaternion_to_yaw(
            quaternion.x,
            quaternion.y,
            quaternion.z,
            quaternion.w,
        )
        enu_yaw = heading_quaternion_yaw_to_enu_yaw(
            yaw,
            quaternion_yaw_is_compass=self._heading_quaternion_yaw_is_compass,
        )
        if (
            not math.isfinite(enu_yaw)
            or (
                self._last_heading_enqueue_stamp_s is not None
                and stamp_s <= self._last_heading_enqueue_stamp_s
            )
        ):
            return
        received = time.monotonic()
        if self._enqueue(
            self._heading_queue, PendingHeading(stamp_s, enu_yaw, received)
        ):
            self._last_heading_enqueue_stamp_s = stamp_s
            self._latest_heading_mono_s = received

    def _nmea_callback(self, msg: Sentence) -> None:
        quality = parse_gga_quality(msg.sentence)
        stamp_s = _stamp_s(msg.header.stamp)
        if quality is None or not self._valid_observation_stamp(stamp_s):
            return
        received = time.monotonic()
        self._gga_history.append(GgaQuality(stamp_s, quality, received))
        self._latest_gga_mono_s = received
        if quality != 4:
            self._heading_queue.clear()
            self._fix_queue.clear()
            self._heading_gate.prerequisite_failure(
                now_s=received,
                kind=PrerequisiteFailureKind.NON_FIXED_INPUT,
            )
            self._position_gate.prerequisite_failure(
                now_s=received,
                kind=PrerequisiteFailureKind.NON_FIXED_INPUT,
            )

    def _lio_callback(self, msg: Odometry) -> None:
        stamp_s = _stamp_s(msg.header.stamp)
        pose = _finite_pose_from_odom(msg)
        if pose is None or not self._valid_observation_stamp(stamp_s):
            return
        appended = self._lio_history.append(
            stamp_s,
            pose,
            frame_id=msg.header.frame_id,
            child_frame_id=msg.child_frame_id,
        )
        if not appended.accepted:
            return
        self._previous_lio_stamp_s = self._latest_lio_stamp_s
        self._previous_lio_pose = self._latest_lio_pose
        self._latest_lio_stamp_s = stamp_s
        self._latest_lio_pose = pose
        self._latest_lio_mono_s = time.monotonic()
        if self._previous_lio_stamp_s is None or self._previous_lio_pose is None:
            return
        dt_s = stamp_s - self._previous_lio_stamp_s
        if dt_s <= 0.0:
            return
        self._local_linear_rate_mps = math.hypot(
            pose.x - self._previous_lio_pose.x,
            pose.y - self._previous_lio_pose.y,
        ) / dt_s
        self._local_yaw_rate_radps = abs(
            normalize_angle(pose.yaw - self._previous_lio_pose.yaw)
        ) / dt_s

    def _alignment_callback(self, msg: Float64MultiArray) -> None:
        received = time.monotonic()
        self._latest_alignment_mono_s = received
        if len(msg.data) < 4 or not all(math.isfinite(value) for value in msg.data[:3]):
            self._latest_alignment = None
            return
        self._latest_alignment = (
            float(msg.data[0]),
            float(msg.data[1]),
            float(msg.data[2]),
            float(msg.data[3]) >= 0.5,
        )

    @staticmethod
    def _age_s(received_mono_s: float | None, now_mono_s: float) -> float:
        if received_mono_s is None:
            return math.inf
        return max(0.0, now_mono_s - received_mono_s)

    def _external_alignment(self, now_mono_s: float):
        if self._scene_identity_alignment is not None:
            self._latest_alignment_mono_s = now_mono_s
            return self._scene_identity_alignment
        if (
            self._latest_alignment is None
            or not self._latest_alignment[3]
            or self._age_s(self._latest_alignment_mono_s, now_mono_s)
            > self._max_alignment_age_s
        ):
            return None
        return self._latest_alignment

    def _heading_quality(self, stamp_s: float) -> int | None:
        for sample in reversed(self._gga_history):
            if sample.stamp_s <= stamp_s:
                if stamp_s - sample.stamp_s <= self._max_gga_heading_age_s:
                    return sample.quality
                return None
        return None

    def _fix_quality(self, stamp_s: float) -> int | None:
        epsilon_s = max(1e-6, 4.0 * math.ulp(stamp_s))
        for sample in reversed(self._gga_history):
            if abs(sample.stamp_s - stamp_s) <= epsilon_s:
                return sample.quality
        return None

    def _record_heading_target(self, stamp_s: float) -> None:
        target = self._heading_gate.target
        if isinstance(target, (float, int)) and math.isfinite(target):
            self._accepted_heading_corrections.append((stamp_s, float(target)))

    def _heading_target_for_fix(self, stamp_s: float) -> float | None:
        for heading_stamp_s, correction in reversed(
            self._accepted_heading_corrections
        ):
            if heading_stamp_s <= stamp_s:
                if stamp_s - heading_stamp_s <= self._max_heading_for_fix_age_s:
                    return correction
                return None
        return None

    def _process_heading(self, now_mono_s: float, alignment) -> None:
        while self._heading_queue:
            pending = self._heading_queue[0]
            waited_s = now_mono_s - pending.received_mono_s
            quality = self._heading_quality(pending.stamp_s)
            if quality is None:
                if waited_s < self._heading_quality_wait_s:
                    return
                self._heading_gate.prerequisite_failure(
                    now_s=now_mono_s,
                    kind=PrerequisiteFailureKind.HEADING_QUALITY_UNAVAILABLE,
                )
                self._heading_queue.popleft()
                continue
            if quality != 4:
                self._heading_gate.prerequisite_failure(
                    now_s=now_mono_s,
                    kind=PrerequisiteFailureKind.NON_FIXED_INPUT,
                )
                self._heading_queue.popleft()
                continue
            interpolated = self._lio_history.interpolate(
                pending.stamp_s, self._max_odom_bracket_s
            )
            if not interpolated.ok:
                if waited_s < self._max_pending_observation_s:
                    return
                if interpolated.reason in (
                    "ODOM_AT_STAMP_UNAVAILABLE",
                    "ODOM_BRACKET_TOO_WIDE",
                ):
                    self._heading_gate.odom_timeout(
                        pending.stamp_s, now_s=now_mono_s
                    )
                else:
                    self._heading_gate.prerequisite_failure(
                        now_s=now_mono_s,
                        kind=PrerequisiteFailureKind.MALFORMED_INPUT,
                    )
                self._heading_queue.popleft()
                continue
            map_yaw = normalize_angle(alignment[0] + pending.enu_yaw)
            correction = normalize_angle(map_yaw - interpolated.pose.yaw)
            result = self._heading_gate.observe(
                pending.stamp_s, correction, now_s=now_mono_s
            )
            self._last_heading_innovation_rad = result.innovation
            if result.accepted:
                self._record_heading_target(pending.stamp_s)
            self._heading_queue.popleft()

    def _process_fix(self, now_mono_s: float, alignment) -> None:
        while self._fix_queue:
            pending = self._fix_queue[0]
            waited_s = now_mono_s - pending.received_mono_s
            if self._heading_gate.state is not CorrectionGateState.LOCKED:
                self._fix_queue.popleft()
                continue
            quality = self._fix_quality(pending.stamp_s)
            if quality is None:
                if waited_s < self._fix_quality_wait_s:
                    return
                self._position_gate.prerequisite_failure(
                    now_s=now_mono_s,
                    kind=PrerequisiteFailureKind.FIX_QUALITY_UNAVAILABLE,
                )
                self._fix_queue.popleft()
                continue
            if quality != 4:
                self._position_gate.prerequisite_failure(
                    now_s=now_mono_s,
                    kind=PrerequisiteFailureKind.NON_FIXED_INPUT,
                )
                self._fix_queue.popleft()
                continue
            yaw_correction = self._heading_target_for_fix(pending.stamp_s)
            if yaw_correction is None:
                if waited_s < self._max_pending_observation_s:
                    return
                self._position_gate.prerequisite_failure(
                    now_s=now_mono_s,
                    kind=PrerequisiteFailureKind.ODOM_AT_STAMP_UNAVAILABLE,
                )
                self._fix_queue.popleft()
                continue
            interpolated = self._lio_history.interpolate(
                pending.stamp_s, self._max_odom_bracket_s
            )
            if not interpolated.ok:
                if waited_s < self._max_pending_observation_s:
                    return
                if interpolated.reason in (
                    "ODOM_AT_STAMP_UNAVAILABLE",
                    "ODOM_BRACKET_TOO_WIDE",
                ):
                    self._position_gate.odom_timeout(
                        pending.stamp_s, now_s=now_mono_s
                    )
                else:
                    self._position_gate.prerequisite_failure(
                        now_s=now_mono_s,
                        kind=PrerequisiteFailureKind.MALFORMED_INPUT,
                    )
                self._fix_queue.popleft()
                continue
            enu_x, enu_y = self._projector.forward(
                pending.latitude, pending.longitude
            )
            theta, tx, ty, _ = alignment
            cos_theta = math.cos(theta)
            sin_theta = math.sin(theta)
            map_x = cos_theta * enu_x - sin_theta * enu_y + tx
            map_y = sin_theta * enu_x + cos_theta * enu_y + ty
            local = interpolated.pose
            cos_yaw = math.cos(yaw_correction)
            sin_yaw = math.sin(yaw_correction)
            correction_xy = (
                map_x - (cos_yaw * local.x - sin_yaw * local.y),
                map_y - (sin_yaw * local.x + cos_yaw * local.y),
            )
            result = self._position_gate.observe(
                pending.stamp_s, correction_xy, now_s=now_mono_s
            )
            self._last_position_innovation_m = result.innovation
            self._fix_queue.popleft()

    def _try_bootstrap(self, now_mono_s: float) -> bool:
        if self._bootstrap_alignment is not None:
            return True
        if not self._heading_queue or not self._fix_queue:
            return False
        heading = self._heading_queue[-1]
        fix = self._fix_queue[-1]
        heading_odom = self._lio_history.interpolate(
            heading.stamp_s, self._max_odom_bracket_s
        )
        fix_odom = self._lio_history.interpolate(fix.stamp_s, self._max_odom_bracket_s)
        if not heading_odom.ok or not fix_odom.ok:
            return False
        theta = normalize_angle(heading_odom.pose.yaw - heading.enu_yaw)
        enu_x, enu_y = self._projector.forward(fix.latitude, fix.longitude)
        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)
        tx = fix_odom.pose.x - (cos_theta * enu_x - sin_theta * enu_y)
        ty = fix_odom.pose.y - (sin_theta * enu_x + cos_theta * enu_y)
        if not all(math.isfinite(value) for value in (theta, tx, ty)):
            return False
        self._bootstrap_alignment = (theta, tx, ty, True)
        self._last_output = Pose2D(0.0, 0.0, 0.0)
        self._heading_queue.clear()
        self._fix_queue.clear()
        self._publish_tf(self._last_output)
        self._publish_mode_status("RTK_BOOTSTRAP", "PUBLISHING_BOOTSTRAP_MAP_ODOM")
        self._publish_motion_allowed(False)
        return True

    def _authority_fresh(self, now_mono_s: float) -> bool:
        return (
            self._age_s(self._latest_gga_mono_s, now_mono_s) <= self._max_gga_age_s
            and self._age_s(self._latest_fix_mono_s, now_mono_s)
            <= self._max_fix_age_s
            and self._age_s(self._latest_heading_mono_s, now_mono_s)
            <= self._max_heading_age_s
            and self._age_s(self._latest_alignment_mono_s, now_mono_s)
            <= self._max_alignment_age_s
            and self._age_s(self._latest_lio_mono_s, now_mono_s)
            <= self._max_lio_age_s
        )

    def _release(self, now_mono_s: float):
        if (
            self._heading_gate.state is not CorrectionGateState.LOCKED
            or self._position_gate.state is not CorrectionGateState.LOCKED
            or self._latest_lio_pose is None
            or self._latest_lio_stamp_s is None
        ):
            return None
        heading_target = self._heading_gate.target
        position_target = self._position_gate.target
        if not isinstance(heading_target, (float, int)) or not isinstance(
            position_target, tuple
        ):
            return None
        target = Pose2D(position_target[0], position_target[1], float(heading_target))
        if self._last_output is None:
            self._last_output = (
                target
                if self._scene_identity_alignment is not None
                else Pose2D(0.0, 0.0, 0.0)
            )
        lio_age_s = max(0.0, self._ros_now_s() - self._latest_lio_stamp_s)
        release = self._release_state.update(
            previous_output_map_odom=self._last_output,
            target_map_odom=target,
            local_pose=self._latest_lio_pose,
            now_s=now_mono_s,
            lio_stamp_s=self._latest_lio_stamp_s,
            lio_age_s=lio_age_s,
            local_linear_rate_mps=self._local_linear_rate_mps,
            local_yaw_rate_radps=self._local_yaw_rate_radps,
            gates_locked=True,
        )
        self._last_output = release.output_map_odom
        self._last_release = release
        return release

    def _timer_callback(self) -> None:
        if not rclpy.ok():
            return
        now_mono_s = time.monotonic()
        alignment = self._external_alignment(now_mono_s)
        if alignment is None:
            self._try_bootstrap(now_mono_s)
            self._rebroadcast_last_output()
            self._publish_motion_allowed(False)
            self._publish_diagnostics(False, None)
            return

        self._process_heading(now_mono_s, alignment)
        self._process_fix(now_mono_s, alignment)
        self._heading_gate.check_timeout(now_s=now_mono_s)
        self._position_gate.check_timeout(now_s=now_mono_s)
        release = self._release(now_mono_s)
        fresh = self._authority_fresh(now_mono_s)
        motion_allowed = bool(release and release.motion_allowed and fresh)
        if release is not None:
            self._publish_tf(release.output_map_odom)
            mode = (
                "RTK_AUTHORITATIVE"
                if release.mode is CorrectionReleaseMode.NORMAL and motion_allowed
                else release.mode.value
            )
            status = release.reason.value if release.reason is not None else "LOCKED"
        else:
            self._rebroadcast_last_output()
            mode = "RTK_DEGRADED"
            status = "GATES_NOT_LOCKED"
        self._publish_mode_status(mode, status)
        self._publish_motion_allowed(motion_allowed)
        self._publish_diagnostics(motion_allowed, release)

    def _publish_motion_allowed(self, motion_allowed: bool) -> None:
        self._safe_publish(self._motion_allowed_pub, Bool(data=motion_allowed))

    def _publish_mode_status(self, mode: str, status: str) -> None:
        if mode != self._last_mode:
            self.get_logger().info(mode)
            self._last_mode = mode
        if status != self._last_status:
            self.get_logger().info(status)
            self._last_status = status
        self._safe_publish(self._mode_pub, String(data=mode))
        self._safe_publish(self._status_pub, String(data=status))

    def _publish_diagnostics(self, motion_allowed: bool, release) -> None:
        now_mono_s = time.monotonic()
        heading_innovation_deg = (
            math.degrees(self._last_heading_innovation_rad)
            if self._last_heading_innovation_rad is not None
            else math.nan
        )
        position_innovation_m = (
            self._last_position_innovation_m
            if self._last_position_innovation_m is not None
            else math.nan
        )
        output = self._last_output
        data = [
            1.0 if motion_allowed else 0.0,
            self._age_s(self._latest_fix_mono_s, now_mono_s),
            self._age_s(self._latest_heading_mono_s, now_mono_s),
            self._age_s(self._latest_alignment_mono_s, now_mono_s),
            position_innovation_m,
            heading_innovation_deg,
            output.x if output is not None else math.nan,
            output.y if output is not None else math.nan,
            math.degrees(output.yaw) if output is not None else math.nan,
            1.0 if self._heading_quality(self._last_heading_enqueue_stamp_s or 0.0) == 4 else 0.0,
            release.translation_step_m if release is not None else 0.0,
            math.degrees(release.yaw_step_rad) if release is not None else 0.0,
            float(self._heading_gate.state),
            float(self._position_gate.state),
            heading_innovation_deg,
            position_innovation_m,
            release.translation_gap_m if release is not None else math.nan,
            math.degrees(release.yaw_gap_rad) if release is not None else math.nan,
            1.0 if motion_allowed else 0.0,
        ]
        self._safe_publish(self._diagnostics_pub, Float64MultiArray(data=data))

    def _publish_tf(self, pose: Pose2D) -> None:
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._map_frame
        msg.child_frame_id = self._odom_frame
        msg.transform.translation.x = pose.x
        msg.transform.translation.y = pose.y
        qx, qy, qz, qw = yaw_to_quaternion(pose.yaw)
        msg.transform.rotation.x = qx
        msg.transform.rotation.y = qy
        msg.transform.rotation.z = qz
        msg.transform.rotation.w = qw
        self._safe_send_transform(msg)

    def _rebroadcast_last_output(self) -> bool:
        if self._last_output is None:
            return False
        self._publish_tf(self._last_output)
        return True

    @staticmethod
    def _is_shutdown_publish_error(exc: Exception) -> bool:
        text = str(exc)
        return (
            not rclpy.ok()
            or "context is invalid" in text
            or "publisher's context is invalid" in text
        )

    def _safe_publish(self, publisher, msg) -> bool:
        if not rclpy.ok():
            return False
        try:
            publisher.publish(msg)
        except Exception as exc:
            if self._is_shutdown_publish_error(exc):
                self.get_logger().debug(f"Ignoring publish during shutdown: {exc}")
                return False
            raise
        return True

    def _safe_send_transform(self, msg: TransformStamped) -> bool:
        if not rclpy.ok():
            return False
        try:
            self._tf_broadcaster.sendTransform(msg)
        except Exception as exc:
            if self._is_shutdown_publish_error(exc):
                self.get_logger().debug(f"Ignoring publish during shutdown: {exc}")
                return False
            raise
        return True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RtkMapOdomCorrector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
