#!/usr/bin/env python3
from __future__ import annotations

import math
import time
import traceback
from collections import deque
from dataclasses import dataclass

import rclpy
from rclpy.duration import Duration
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import QuaternionStamped, TransformStamped
from nav_msgs.msg import Odometry
from nmea_msgs.msg import Sentence
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Float32, Float32MultiArray, Float64MultiArray, String
from tf2_ros import TransformBroadcaster

from gps_waypoint_dispatcher.alignment_math import heading_quaternion_yaw_to_enu_yaw
from gps_waypoint_dispatcher.corridor_quality import parse_gga_quality
from gps_waypoint_dispatcher.rtk_authority import (
    CorrectionGate,
    CorrectionGateState,
    CorrectionReleaseMode,
    CorrectionReleaseState,
    LocalOdomBridge,
    LocalOdomBridgeResult,
    LocalOdomBridgeState,
    Pose2D,
    PrerequisiteFailureKind,
    StampedPoseHistory,
    compose_pose,
    compute_map_to_odom,
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


@dataclass(frozen=True)
class LioDegeneracy:
    min_eig: float
    condition_number: float
    regularized: bool
    received_mono_s: float


@dataclass(frozen=True)
class RtkHealth:
    fix_quality: int
    satellites: int
    hdop: float
    heading_valid: bool
    heading_control_eligible: bool
    heading_position_type: str
    heading_solution_status: str
    heading_rejects: int
    ntrip_state: str
    rtcm_age_s: float
    received_mono_s: float


@dataclass(frozen=True)
class HeadingControlReadinessSnapshot:
    stable: bool
    samples: int
    elapsed_s: float


class HeadingControlReadiness:
    """Track the uninterrupted NARROW_INT run required to control the vehicle."""

    def __init__(self, duration_s: float, min_samples: int) -> None:
        if not math.isfinite(duration_s) or duration_s <= 0.0:
            raise ValueError("heading control stable duration must be positive")
        if isinstance(min_samples, bool) or min_samples <= 0:
            raise ValueError("heading control minimum samples must be positive")
        self._duration_s = duration_s
        self._min_samples = min_samples
        self._first_sample_s: float | None = None
        self._last_sample_s: float | None = None
        self._sample_count = 0

    def observe(self, eligible: bool, received_mono_s: float) -> None:
        if not eligible:
            self._first_sample_s = None
            self._last_sample_s = None
            self._sample_count = 0
            return
        if self._first_sample_s is None:
            self._first_sample_s = received_mono_s
        self._last_sample_s = received_mono_s
        self._sample_count += 1

    def reset(self) -> None:
        self._first_sample_s = None
        self._last_sample_s = None
        self._sample_count = 0

    def snapshot(self, now_mono_s: float) -> HeadingControlReadinessSnapshot:
        if self._first_sample_s is None or self._last_sample_s is None:
            return HeadingControlReadinessSnapshot(False, 0, 0.0)
        elapsed_s = max(0.0, self._last_sample_s - self._first_sample_s)
        stale = now_mono_s - self._last_sample_s > self._duration_s
        return HeadingControlReadinessSnapshot(
            stable=(
                not stale
                and self._sample_count >= self._min_samples
                and elapsed_s >= self._duration_s
            ),
            samples=self._sample_count,
            elapsed_s=elapsed_s,
        )


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
        self.declare_parameter("rtk_health_topic", "/rtk/health")
        self.declare_parameter("lio_odom_topic", "/fastlio2/lio_odom")
        self.declare_parameter("lio_degeneracy_topic", "/fastlio2/degeneracy")
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
        self.declare_parameter(
            "motion_speed_limit_topic",
            "/localization_authority/max_linear_speed_mps",
        )
        self.declare_parameter("enu_origin_lat", 0.0)
        self.declare_parameter("enu_origin_lon", 0.0)
        self.declare_parameter("enu_origin_alt", 0.0)
        self.declare_parameter("heading_quaternion_yaw_is_compass", True)
        self.declare_parameter("publish_period_s", 0.10)
        self.declare_parameter("tf_future_tolerance_s", 0.10)
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
        self.declare_parameter("max_rtk_health_age_s", 1.5)
        self.declare_parameter("heading_control_stable_duration_s", 2.0)
        self.declare_parameter("heading_control_min_samples", 5)
        self.declare_parameter("low_speed_heading_strict_mps", 0.10)
        self.declare_parameter("heading_lio_crosscheck_enabled", True)
        self.declare_parameter("heading_lio_crosscheck_max_interval_s", 0.30)
        self.declare_parameter("heading_lio_crosscheck_gnss_jump_deg", 12.0)
        self.declare_parameter("heading_lio_crosscheck_lio_turn_deg", 3.0)
        self.declare_parameter("rtk_min_satellites", 10)
        self.declare_parameter("rtk_max_hdop", 2.0)
        self.declare_parameter("rtk_max_rtcm_age_s", 3.0)
        self.declare_parameter("max_lio_age_s", 0.35)
        self.declare_parameter("max_heading_for_fix_age_s", 0.30)
        self.declare_parameter("max_translation_rate_mps", 0.20)
        self.declare_parameter("max_yaw_rate_degps", 2.0)
        self.declare_parameter("heading_locked_innovation_deg", 15.0)
        self.declare_parameter("position_locked_innovation_m", 1.0)
        self.declare_parameter("heading_recovery_spread_deg", 8.0)
        self.declare_parameter("position_recovery_diameter_m", 0.50)
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
        self.declare_parameter("recovery_translation_m", 0.30)
        self.declare_parameter("recovery_yaw_deg", 4.0)
        self.declare_parameter("recovery_confirmation_s", 1.0)
        self.declare_parameter("enable_local_odom_bridge", True)
        self.declare_parameter("local_bridge_max_step_translation_m", 0.50)
        self.declare_parameter("local_bridge_max_step_yaw_deg", 15.0)
        self.declare_parameter("local_bridge_max_duration_s", 15.0)
        self.declare_parameter("local_bridge_max_distance_m", 3.0)
        self.declare_parameter("local_bridge_max_yaw_change_deg", 30.0)
        self.declare_parameter("max_lio_degeneracy_age_s", 0.50)
        self.declare_parameter("lio_min_eig_healthy", 75.0)
        self.declare_parameter("lio_reject_regularized", True)
        self.declare_parameter("rtk_authoritative_max_linear_speed_mps", 2.0)
        self.declare_parameter("local_bridge_max_linear_speed_mps", 0.25)
        self.declare_parameter("rtk_reacquire_max_linear_speed_mps", 0.25)
        self.declare_parameter("allow_moving_backlog_release", True)
        self.declare_parameter("allow_bounded_backlog_motion", True)
        self.declare_parameter("moving_reacquire_translation_rate_mps", 0.05)
        self.declare_parameter("moving_reacquire_yaw_rate_degps", 0.5)

        self._map_frame = str(self.get_parameter("map_frame").value)
        self._odom_frame = str(self.get_parameter("odom_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._fix_topic = str(self.get_parameter("fix_topic").value)
        self._heading_topic = str(self.get_parameter("heading_topic").value)
        self._nmea_topic = str(self.get_parameter("nmea_topic").value)
        self._rtk_health_topic = str(self.get_parameter("rtk_health_topic").value)
        self._lio_odom_topic = str(self.get_parameter("lio_odom_topic").value)
        self._lio_degeneracy_topic = str(
            self.get_parameter("lio_degeneracy_topic").value
        )
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
        self._motion_speed_limit_topic = str(
            self.get_parameter("motion_speed_limit_topic").value
        )
        self._heading_quaternion_yaw_is_compass = bool(
            self.get_parameter("heading_quaternion_yaw_is_compass").value
        )
        self._publish_period_s = float(self.get_parameter("publish_period_s").value)
        self._tf_future_tolerance_s = float(
            self.get_parameter("tf_future_tolerance_s").value
        )
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
        self._max_rtk_health_age_s = float(
            self.get_parameter("max_rtk_health_age_s").value
        )
        self._heading_control_stable_duration_s = float(
            self.get_parameter("heading_control_stable_duration_s").value
        )
        self._heading_control_min_samples = int(
            self.get_parameter("heading_control_min_samples").value
        )
        self._low_speed_heading_strict_mps = float(
            self.get_parameter("low_speed_heading_strict_mps").value
        )
        self._heading_lio_crosscheck_enabled = bool(
            self.get_parameter("heading_lio_crosscheck_enabled").value
        )
        self._heading_lio_crosscheck_max_interval_s = float(
            self.get_parameter("heading_lio_crosscheck_max_interval_s").value
        )
        self._heading_lio_crosscheck_gnss_jump_rad = math.radians(
            float(self.get_parameter("heading_lio_crosscheck_gnss_jump_deg").value)
        )
        self._heading_lio_crosscheck_lio_turn_rad = math.radians(
            float(self.get_parameter("heading_lio_crosscheck_lio_turn_deg").value)
        )
        self._rtk_min_satellites = int(self.get_parameter("rtk_min_satellites").value)
        self._rtk_max_hdop = float(self.get_parameter("rtk_max_hdop").value)
        self._rtk_max_rtcm_age_s = float(
            self.get_parameter("rtk_max_rtcm_age_s").value
        )
        self._max_lio_age_s = float(self.get_parameter("max_lio_age_s").value)
        self._max_heading_for_fix_age_s = float(
            self.get_parameter("max_heading_for_fix_age_s").value
        )
        self._enable_local_odom_bridge = bool(
            self.get_parameter("enable_local_odom_bridge").value
        )
        self._max_lio_degeneracy_age_s = float(
            self.get_parameter("max_lio_degeneracy_age_s").value
        )
        self._lio_min_eig_healthy = float(
            self.get_parameter("lio_min_eig_healthy").value
        )
        self._lio_reject_regularized = bool(
            self.get_parameter("lio_reject_regularized").value
        )
        self._rtk_authoritative_max_linear_speed_mps = float(
            self.get_parameter("rtk_authoritative_max_linear_speed_mps").value
        )
        self._local_bridge_max_linear_speed_mps = float(
            self.get_parameter("local_bridge_max_linear_speed_mps").value
        )
        self._rtk_reacquire_max_linear_speed_mps = float(
            self.get_parameter("rtk_reacquire_max_linear_speed_mps").value
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
        self._heading_control_readiness = HeadingControlReadiness(
            self._heading_control_stable_duration_s,
            self._heading_control_min_samples,
        )
        self._heading_control_established = False
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
            allow_moving_backlog_release=bool(
                self.get_parameter("allow_moving_backlog_release").value
            ),
            allow_bounded_backlog_motion=bool(
                self.get_parameter("allow_bounded_backlog_motion").value
            ),
            moving_reacquire_translation_rate_mps=float(
                self.get_parameter("moving_reacquire_translation_rate_mps").value
            ),
            moving_reacquire_yaw_rate_radps=math.radians(
                float(self.get_parameter("moving_reacquire_yaw_rate_degps").value)
            ),
        )
        self._local_bridge = LocalOdomBridge(
            max_step_translation_m=float(
                self.get_parameter("local_bridge_max_step_translation_m").value
            ),
            max_step_yaw_rad=math.radians(
                float(self.get_parameter("local_bridge_max_step_yaw_deg").value)
            ),
            max_duration_s=float(
                self.get_parameter("local_bridge_max_duration_s").value
            ),
            max_distance_m=float(
                self.get_parameter("local_bridge_max_distance_m").value
            ),
            max_yaw_change_rad=math.radians(
                float(self.get_parameter("local_bridge_max_yaw_change_deg").value)
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
        self._last_trusted_heading: tuple[float, float, float] | None = None
        self._heading_lio_mismatch_active = False
        self._heading_lio_mismatch_rejects = 0
        self._last_heading_lio_gnss_delta_rad = math.nan
        self._last_heading_lio_delta_rad = math.nan
        self._last_heading_enqueue_stamp_s: float | None = None
        self._last_fix_enqueue_stamp_s: float | None = None
        self._latest_lio_stamp_s: float | None = None
        self._latest_lio_pose: Pose2D | None = None
        self._previous_lio_stamp_s: float | None = None
        self._previous_lio_pose: Pose2D | None = None
        self._local_linear_rate_mps = math.inf
        self._local_yaw_rate_radps = math.inf
        self._latest_lio_mono_s: float | None = None
        self._latest_lio_degeneracy: LioDegeneracy | None = None
        self._latest_rtk_health: RtkHealth | None = None
        self._latest_fix_mono_s: float | None = None
        self._latest_heading_mono_s: float | None = None
        self._latest_gga_mono_s: float | None = None
        self._latest_alignment_mono_s: float | None = None
        self._latest_alignment: tuple[float, float, float, bool] | None = None
        self._bootstrap_alignment: tuple[float, float, float, bool] | None = None
        self._last_output: Pose2D | None = None
        self._coherent_target_map_odom: Pose2D | None = None
        self._coherent_target_stamp_s: float | None = None
        self._last_release = None
        self._last_heading_innovation_rad: float | None = None
        self._last_position_innovation_m: float | None = None
        self._last_authoritative_mono_s: float | None = None
        self._bridge_map_odom: Pose2D | None = None
        self._last_bridge_result: LocalOdomBridgeResult | None = None
        self._diagnostic_failure_class = "NONE"
        self._diagnostic_failure_started_mono_s: float | None = None
        self._heading_rejects_delta = 0
        self._last_mode = ""
        self._last_status = ""

        self._mode_pub = self.create_publisher(String, self._mode_topic, 10)
        self._status_pub = self.create_publisher(String, self._status_topic, 10)
        self._diagnostics_pub = self.create_publisher(
            DiagnosticArray, self._diagnostics_topic, 10
        )
        self._motion_allowed_pub = self.create_publisher(
            Bool, self._motion_allowed_topic, 10
        )
        self._motion_speed_limit_pub = self.create_publisher(
            Float32, self._motion_speed_limit_topic, 10
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
        self._rtk_health_sub = self.create_subscription(
            DiagnosticArray, self._rtk_health_topic, self._rtk_health_callback, 10
        )
        self._lio_sub = self.create_subscription(
            Odometry, self._lio_odom_topic, self._lio_callback, 50
        )
        self._lio_degeneracy_sub = self.create_subscription(
            Float32MultiArray,
            self._lio_degeneracy_topic,
            self._lio_degeneracy_callback,
            10,
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

    def _rtk_health_callback(self, msg: DiagnosticArray) -> None:
        for status in msg.status:
            if status.name != "um982_rtk_driver/health":
                continue
            values = {item.key: item.value for item in status.values}
            try:
                health = RtkHealth(
                    fix_quality=int(values["fix_quality"]),
                    satellites=int(values["satellites"]),
                    hdop=float(values["hdop"]),
                    heading_valid=values["heading_valid"].lower() == "true",
                    heading_control_eligible=(
                        values.get("heading_control_eligible", "false").lower()
                        == "true"
                    ),
                    heading_position_type=str(
                        values.get("uniheading_position_type", "UNKNOWN")
                    ),
                    heading_solution_status=str(
                        values.get("uniheading_status", "UNKNOWN")
                    ),
                    heading_rejects=int(values.get("heading_rejects", "0")),
                    ntrip_state=str(values["ntrip_state"]),
                    rtcm_age_s=float(values["rtcm_age_s"]),
                    received_mono_s=time.monotonic(),
                )
            except (KeyError, TypeError, ValueError):
                return
            previous_health = self._latest_rtk_health
            self._heading_rejects_delta = max(
                0,
                health.heading_rejects
                - (previous_health.heading_rejects if previous_health is not None else health.heading_rejects),
            )
            self._latest_rtk_health = health
            self._heading_control_readiness.observe(
                health.heading_control_eligible, health.received_mono_s
            )
            if health.heading_control_eligible and self._heading_control_readiness.snapshot(
                health.received_mono_s
            ).stable:
                self._heading_control_established = True
            return

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

    def _lio_degeneracy_callback(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 3:
            return
        min_eig, condition_number, regularized = (
            float(value) for value in msg.data[:3]
        )
        if not all(math.isfinite(value) for value in (min_eig, condition_number)):
            return
        self._latest_lio_degeneracy = LioDegeneracy(
            min_eig=min_eig,
            condition_number=condition_number,
            regularized=regularized >= 0.5,
            received_mono_s=time.monotonic(),
        )

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

    def _heading_lio_mismatch(self, pending: PendingHeading, lio_yaw: float) -> bool:
        """Reject a dual-antenna jump when timestamp-matched LIO did not turn."""
        if not self._heading_lio_crosscheck_enabled or self._last_trusted_heading is None:
            return False
        previous_stamp_s, previous_gnss_yaw, previous_lio_yaw = (
            self._last_trusted_heading
        )
        interval_s = pending.stamp_s - previous_stamp_s
        if not 0.0 < interval_s <= self._heading_lio_crosscheck_max_interval_s:
            return False
        self._last_heading_lio_gnss_delta_rad = abs(
            normalize_angle(pending.enu_yaw - previous_gnss_yaw)
        )
        self._last_heading_lio_delta_rad = abs(
            normalize_angle(lio_yaw - previous_lio_yaw)
        )
        return (
            self._last_heading_lio_gnss_delta_rad
            >= self._heading_lio_crosscheck_gnss_jump_rad
            and self._last_heading_lio_delta_rad
            <= self._heading_lio_crosscheck_lio_turn_rad
        )

    def _heading_control_eligible_now(self, now_mono_s: float) -> bool:
        health = self._latest_rtk_health
        return bool(
            health is not None
            and self._age_s(health.received_mono_s, now_mono_s)
            <= self._max_rtk_health_age_s
            and health.heading_control_eligible
        )

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
            if self._heading_lio_mismatch_active and not self._heading_control_eligible_now(
                now_mono_s
            ):
                self._heading_gate.prerequisite_failure(
                    now_s=now_mono_s,
                    kind=PrerequisiteFailureKind.NON_FIXED_INPUT,
                )
                self._heading_queue.popleft()
                continue
            if self._heading_lio_mismatch(pending, interpolated.pose.yaw):
                self._heading_lio_mismatch_active = True
                self._heading_lio_mismatch_rejects += 1
                self._heading_control_readiness.reset()
                self._heading_control_established = False
                self._heading_gate.prerequisite_failure(
                    now_s=now_mono_s,
                    kind=PrerequisiteFailureKind.HEADING_LIO_MISMATCH,
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
                self._heading_lio_mismatch_active = False
                self._last_trusted_heading = (
                    pending.stamp_s,
                    pending.enu_yaw,
                    interpolated.pose.yaw,
                )
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
            map_yaw = normalize_angle(local.yaw + yaw_correction)
            coherent_target = compute_map_to_odom(
                Pose2D(map_x, map_y, map_yaw),
                local,
            )
            reference_target = (
                self._coherent_target_map_odom
                or self._last_output
                or Pose2D(0.0, 0.0, 0.0)
            )
            reference_map_base = compose_pose(reference_target, local)
            correction_innovation_xy = (
                map_x - reference_map_base.x,
                map_y - reference_map_base.y,
            )
            result = self._position_gate.observe(
                pending.stamp_s,
                correction_innovation_xy,
                now_s=now_mono_s,
            )
            self._last_position_innovation_m = result.innovation
            if result.accepted:
                self._coherent_target_map_odom = coherent_target
                self._coherent_target_stamp_s = pending.stamp_s
                self._position_gate.rebase_locked_target((0.0, 0.0))
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
        self._coherent_target_map_odom = None
        self._coherent_target_stamp_s = None
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
            and self._rtk_health_reason(now_mono_s) is None
        )

    def _rtk_health_reason(self, now_mono_s: float) -> str | None:
        health = self._latest_rtk_health
        if health is None or (
            now_mono_s - health.received_mono_s > self._max_rtk_health_age_s
        ):
            return "RTK_HEALTH_STALE"
        if health.fix_quality != 4:
            return "GNSS_NOT_RTK_FIXED"
        if health.satellites < self._rtk_min_satellites:
            return "GNSS_LOW_SATELLITES"
        if not math.isfinite(health.hdop) or health.hdop > self._rtk_max_hdop:
            return "GNSS_HDOP_EXCEEDED"
        if not health.heading_valid:
            return "GNSS_HEADING_INVALID"
        if self._heading_lio_mismatch_active:
            return "GNSS_LIO_HEADING_MISMATCH"
        low_speed = (
            math.isfinite(self._local_linear_rate_mps)
            and abs(self._local_linear_rate_mps) < self._low_speed_heading_strict_mps
        )
        if low_speed and not health.heading_control_eligible:
            return "GNSS_HEADING_LOW_SPEED_UNSTABLE"
        readiness = self._heading_control_readiness.snapshot(now_mono_s)
        if health.heading_control_eligible:
            if not readiness.stable:
                return "GNSS_HEADING_STABILIZING"
        elif health.heading_position_type == "NARROW_FLOAT":
            # Float solutions may maintain a locked authority, but never create
            # or recover one after a heading hold.
            if (
                not self._heading_control_established
                or self._heading_gate.state is not CorrectionGateState.LOCKED
                or self._release_state.requires_heading_recovery
            ):
                return "GNSS_HEADING_FLOAT"
        else:
            return f"GNSS_HEADING_{health.heading_position_type}"
        if health.ntrip_state != "RTCM_FRESH":
            return f"NTRIP_{health.ntrip_state}"
        if (
            not math.isfinite(health.rtcm_age_s)
            or health.rtcm_age_s > self._rtk_max_rtcm_age_s
        ):
            return "RTCM_STALE"
        return None

    def _local_odom_fresh(self, now_mono_s: float) -> bool:
        return (
            self._latest_lio_pose is not None
            and self._latest_lio_stamp_s is not None
            and self._age_s(self._latest_lio_mono_s, now_mono_s)
            <= self._max_lio_age_s
        )

    def _local_odom_health_reason(self, now_mono_s: float) -> str | None:
        if not self._local_odom_fresh(now_mono_s):
            return "LOCAL_ODOM_STALE"
        degeneracy = self._latest_lio_degeneracy
        if degeneracy is None or (
            self._age_s(degeneracy.received_mono_s, now_mono_s)
            > self._max_lio_degeneracy_age_s
        ):
            return "LIO_HEALTH_UNAVAILABLE"
        if degeneracy.min_eig < self._lio_min_eig_healthy:
            return "LIO_DEGENERATE_MIN_EIG"
        if self._lio_reject_regularized and degeneracy.regularized:
            return "LIO_DEGENERATE_REGULARIZED"
        return None

    def _evaluate_local_odom_bridge(
        self, now_mono_s: float
    ) -> LocalOdomBridgeResult | None:
        if not self._enable_local_odom_bridge:
            return None
        if self._last_authoritative_mono_s is None or self._last_output is None:
            return None
        if self._local_bridge.state is LocalOdomBridgeState.IDLE:
            # Keep the exact transform that was trusted under RTK. During the
            # bridge only FAST-LIO advances map->base through odom->base.
            self._bridge_map_odom = self._last_output
        health_reason = self._local_odom_health_reason(now_mono_s)
        if health_reason is not None:
            result = (
                self._local_bridge.evaluate(
                    now_s=now_mono_s,
                    local_pose=self._latest_lio_pose,
                    local_fresh=True,
                )
                if self._local_bridge.state is LocalOdomBridgeState.FAULTED
                else self._local_bridge.fail(health_reason)
            )
            self._last_bridge_result = result
            return result
        result = self._local_bridge.evaluate(
            now_s=now_mono_s,
            local_pose=self._latest_lio_pose,
            local_fresh=True,
        )
        self._last_bridge_result = result
        return result

    def _clear_local_odom_bridge(self) -> None:
        self._local_bridge.reset()
        self._bridge_map_odom = None
        self._last_bridge_result = None

    def _publish_frozen_output(self) -> bool:
        output = self._bridge_map_odom or self._last_output
        if output is None:
            return False
        self._publish_tf(output)
        return True

    def _release(self, now_mono_s: float):
        if (
            self._heading_gate.state is not CorrectionGateState.LOCKED
            or self._position_gate.state is not CorrectionGateState.LOCKED
            or self._latest_lio_pose is None
            or self._latest_lio_stamp_s is None
        ):
            return None
        if self._coherent_target_map_odom is None:
            return None
        target = self._coherent_target_map_odom
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
            if self._try_bootstrap(now_mono_s):
                return
            bridge = self._evaluate_local_odom_bridge(now_mono_s)
            bridge_allowed = bool(bridge and bridge.allowed)
            self._publish_frozen_output()
            if bridge_allowed:
                self._publish_mode_status("LIO_BRIDGE", f"NO_ALIGNMENT;{bridge.reason}")
                self._publish_motion_allowed(
                    True,
                    self._local_bridge_max_linear_speed_mps,
                    now_mono_s=now_mono_s,
                )
            else:
                status = bridge.reason if bridge is not None else "NO_TRUSTED_AUTHORITY"
                self._publish_mode_status("RTK_DEGRADED", f"NO_ALIGNMENT;{status}")
                self._publish_motion_allowed(False)
            self._publish_diagnostics(bridge_allowed, None, bridge)
            return

        self._process_heading(now_mono_s, alignment)
        self._process_fix(now_mono_s, alignment)
        self._heading_gate.check_timeout(now_s=now_mono_s)
        self._position_gate.check_timeout(now_s=now_mono_s)
        release = self._release(now_mono_s)
        health_reason = self._rtk_health_reason(now_mono_s)
        fresh = self._authority_fresh(now_mono_s)
        health = self._latest_rtk_health
        rtk_motion_allowed = bool(release and release.motion_allowed and fresh)
        if rtk_motion_allowed:
            self._last_authoritative_mono_s = now_mono_s
            self._clear_local_odom_bridge()
            self._publish_tf(release.output_map_odom)
            reacquiring = release.mode is CorrectionReleaseMode.RTK_REACQUIRING
            self._publish_mode_status(
                "RTK_REACQUIRING" if reacquiring else "RTK_AUTHORITATIVE",
                release.reason.value if reacquiring and release.reason else "LOCKED",
            )
            self._publish_motion_allowed(
                True,
                (
                    self._rtk_reacquire_max_linear_speed_mps
                    if (
                        reacquiring
                        or not health.heading_control_eligible
                    )
                    else self._rtk_authoritative_max_linear_speed_mps
                ),
                now_mono_s=now_mono_s,
            )
            self._publish_diagnostics(True, release, None)
            return

        release_blocks_bridge = release is not None and release.mode in {
            CorrectionReleaseMode.FAULT_HOLD,
            CorrectionReleaseMode.LOCAL_ODOM_STALE,
        }
        bridge = None if release_blocks_bridge else self._evaluate_local_odom_bridge(now_mono_s)
        bridge_allowed = bool(bridge and bridge.allowed)
        self._publish_frozen_output()
        if bridge_allowed:
            reason = release.reason.value if release and release.reason else "GATES_NOT_LOCKED"
            if health_reason is not None:
                reason = f"{health_reason};{reason}"
            self._publish_mode_status("LIO_BRIDGE", f"{reason};{bridge.reason}")
            self._publish_motion_allowed(
                True,
                self._local_bridge_max_linear_speed_mps,
                now_mono_s=now_mono_s,
            )
        else:
            reason = (
                release.reason.value
                if release is not None and release.reason is not None
                else "GATES_NOT_LOCKED"
            )
            if bridge is not None:
                reason = f"{reason};{bridge.reason}"
            self._publish_mode_status("RTK_DEGRADED", reason)
            self._publish_motion_allowed(False)
        self._publish_diagnostics(bridge_allowed, release, bridge)

    def _publish_motion_allowed(
        self,
        motion_allowed: bool,
        max_linear_speed_mps: float = 0.0,
        *,
        now_mono_s: float | None = None,
    ) -> None:
        self._safe_publish(self._motion_allowed_pub, Bool(data=motion_allowed))
        if not motion_allowed:
            speed_limit = 0.0
        else:
            speed_limit = max_linear_speed_mps
        self._safe_publish(
            self._motion_speed_limit_pub, Float32(data=float(speed_limit))
        )

    def _publish_mode_status(self, mode: str, status: str) -> None:
        if mode != self._last_mode:
            self.get_logger().info(mode)
            self._last_mode = mode
        if status != self._last_status:
            self.get_logger().info(status)
            self._last_status = status
        self._safe_publish(self._mode_pub, String(data=mode))
        self._safe_publish(self._status_pub, String(data=status))

    def _publish_diagnostics(
        self,
        motion_allowed: bool,
        release,
        bridge: LocalOdomBridgeResult | None = None,
    ) -> None:
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
        coherent_target = self._coherent_target_map_odom
        coherent_target_age_s = (
            max(0.0, self._ros_now_s() - self._coherent_target_stamp_s)
            if self._coherent_target_stamp_s is not None
            else math.nan
        )
        degeneracy = self._latest_lio_degeneracy
        degeneracy_age_s = (
            self._age_s(degeneracy.received_mono_s, now_mono_s)
            if degeneracy is not None
            else math.nan
        )
        failure_class, failure_since_s = self._diagnostic_failure(
            now_mono_s, motion_allowed, release, bridge
        )
        health = self._latest_rtk_health
        heading_readiness = self._heading_control_readiness.snapshot(now_mono_s)

        def value(key: str, raw) -> KeyValue:
            if isinstance(raw, float):
                rendered = "nan" if math.isnan(raw) else f"{raw:.6f}"
            else:
                rendered = str(raw)
            return KeyValue(key=key, value=rendered)

        # The first eight keys are the stable action-oriented diagnostic contract.
        # Additional keys preserve the former numeric telemetry with explicit names.
        values = [
            value("failure_class", failure_class),
            value("failure_since_s", failure_since_s),
            value("heading_error_deg", heading_innovation_deg),
            value("position_error_m", position_innovation_m),
            value("rtcm_age_s", health.rtcm_age_s if health is not None else math.nan),
            value("fix_quality", health.fix_quality if health is not None else -1),
            value("satellites", health.satellites if health is not None else -1),
            value("hdop", health.hdop if health is not None else math.nan),
            value(
                "heading_position_type",
                health.heading_position_type if health is not None else "UNKNOWN",
            ),
            value(
                "heading_solution_status",
                health.heading_solution_status if health is not None else "UNKNOWN",
            ),
            value("heading_rejects", health.heading_rejects if health is not None else -1),
            value("heading_rejects_delta", self._heading_rejects_delta),
            value(
                "heading_control_eligible",
                health.heading_control_eligible if health is not None else False,
            ),
            value("heading_control_stable", heading_readiness.stable),
            value("heading_control_samples", heading_readiness.samples),
            value("heading_control_elapsed_s", heading_readiness.elapsed_s),
            value("heading_lio_mismatch_active", self._heading_lio_mismatch_active),
            value("heading_lio_mismatch_rejects", self._heading_lio_mismatch_rejects),
            value(
                "heading_lio_gnss_delta_deg",
                math.degrees(self._last_heading_lio_gnss_delta_rad),
            ),
            value(
                "heading_lio_delta_deg",
                math.degrees(self._last_heading_lio_delta_rad),
            ),
            value("motion_allowed", motion_allowed),
            value("fix_age_s", self._age_s(self._latest_fix_mono_s, now_mono_s)),
            value("heading_age_s", self._age_s(self._latest_heading_mono_s, now_mono_s)),
            value("alignment_age_s", self._age_s(self._latest_alignment_mono_s, now_mono_s)),
            value("output_map_odom_x", output.x if output is not None else math.nan),
            value("output_map_odom_y", output.y if output is not None else math.nan),
            value("output_map_odom_yaw_deg", math.degrees(output.yaw) if output is not None else math.nan),
            value("heading_gate_state", self._heading_gate.state),
            value("position_gate_state", self._position_gate.state),
            value("release_translation_gap_m", release.translation_gap_m if release is not None else math.nan),
            value("release_yaw_gap_deg", math.degrees(release.yaw_gap_rad) if release is not None else math.nan),
            value(
                "heading_yaw_before_deg",
                math.degrees(release.output_map_base.yaw) if release is not None else math.nan,
            ),
            value(
                "heading_yaw_after_deg",
                math.degrees(release.target_map_base.yaw) if release is not None else math.nan,
            ),
            value("coherent_target_x", coherent_target.x if coherent_target is not None else math.nan),
            value("coherent_target_y", coherent_target.y if coherent_target is not None else math.nan),
            value("coherent_target_yaw_deg", math.degrees(coherent_target.yaw) if coherent_target is not None else math.nan),
            value("coherent_target_age_s", coherent_target_age_s),
            value("local_bridge_allowed", bridge is not None and bridge.allowed),
            value("local_bridge_elapsed_s", bridge.elapsed_s if bridge is not None else math.nan),
            value("local_bridge_distance_m", bridge.distance_m if bridge is not None else math.nan),
            value("lio_min_eig", degeneracy.min_eig if degeneracy is not None else math.nan),
            value("lio_condition_number", degeneracy.condition_number if degeneracy is not None else math.nan),
            value("lio_regularized", degeneracy.regularized if degeneracy is not None else False),
            value("lio_degeneracy_age_s", degeneracy_age_s),
        ]
        status = DiagnosticStatus()
        status.name = "localization_authority"
        status.hardware_id = "rtk_map_odom_corrector"
        status.level = (
            DiagnosticStatus.OK
            if failure_class == "NONE"
            else DiagnosticStatus.WARN if motion_allowed else DiagnosticStatus.ERROR
        )
        status.message = failure_class
        status.values = values
        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = self.get_clock().now().to_msg()
        diagnostics.status = [status]
        self._safe_publish(self._diagnostics_pub, diagnostics)

    def _diagnostic_failure(
        self, now_mono_s: float, motion_allowed: bool, release, bridge: LocalOdomBridgeResult | None
    ) -> tuple[str, float]:
        """Collapse detailed guards into a stable operator-facing failure class."""

        release_reason = release.reason.value if release is not None and release.reason is not None else ""
        reason = self._rtk_health_reason(now_mono_s)
        if reason == "GNSS_LIO_HEADING_MISMATCH":
            failure_class = "HEADING_LIO_MISMATCH"
        elif "HEADING" in release_reason:
            failure_class = "HEADING_INNOVATION_EXCEEDED"
        elif reason in {"RTCM_STALE", "RTK_HEALTH_STALE"}:
            failure_class = "RTCM_STALE"
        elif reason is not None and reason.startswith("NTRIP_"):
            failure_class = "NTRIP_DISCONNECTED"
        elif reason == "GNSS_HEADING_STABILIZING":
            failure_class = "HEADING_STABILIZING"
        elif reason is not None and reason.startswith("GNSS_HEADING"):
            failure_class = "HEADING_SOURCE_INVALID"
        elif reason is not None and reason.startswith("GNSS_"):
            failure_class = "GNSS_NO_SOLUTION"
        elif release is not None and release.reason is not None and (
            "TRANSLATION" in release.reason.value or "POSITION" in release.reason.value
        ):
            failure_class = "POSITION_INNOVATION_EXCEEDED"
        else:
            local_reason = self._local_odom_health_reason(now_mono_s)
            if local_reason is not None and "DEGENERATE" in local_reason:
                failure_class = "LIO_DEGENERATE"
            elif local_reason is not None or (
                bridge is not None and "LOCAL_ODOM" in bridge.reason
            ):
                failure_class = "LIO_STALE"
            elif not motion_allowed:
                failure_class = "ODOM_TIMESTAMP_INVALID"
            else:
                failure_class = "NONE"

        if failure_class == "NONE":
            self._diagnostic_failure_class = "NONE"
            self._diagnostic_failure_started_mono_s = None
            return failure_class, 0.0
        if failure_class != self._diagnostic_failure_class:
            self._diagnostic_failure_class = failure_class
            self._diagnostic_failure_started_mono_s = now_mono_s
        started = self._diagnostic_failure_started_mono_s or now_mono_s
        return failure_class, max(0.0, now_mono_s - started)

    def _publish_tf(self, pose: Pose2D) -> None:
        msg = TransformStamped()
        # Nav2 asks for the robot pose at its current control timestamp. Give
        # the map->odom transform a small standard TF horizon so scheduling
        # jitter cannot turn a valid chain into a future-extrapolation error.
        stamp = self.get_clock().now() + Duration(
            seconds=self._tf_future_tolerance_s
        )
        msg.header.stamp = stamp.to_msg()
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
        return self._publish_frozen_output()

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
    except (KeyboardInterrupt, ExternalShutdownException):
        # ROS shutdown is an expected termination path, not a node failure.
        pass
    except Exception:
        # This process is launch-critical. Never let a callback error disappear
        # behind a generic "exited early" message in the launcher.
        node.get_logger().fatal(
            f"Unhandled exception in rtk_map_odom_corrector:\n{traceback.format_exc()}"
        )
        raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
