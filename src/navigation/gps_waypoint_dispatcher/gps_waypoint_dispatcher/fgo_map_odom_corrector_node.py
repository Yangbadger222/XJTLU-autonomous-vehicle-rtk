#!/usr/bin/env python3
"""FGO-GIL map->odom authority for the corridor runtime."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float64MultiArray, String
from tf2_ros import TransformBroadcaster

from gps_waypoint_dispatcher.fgo_corridor_math import (
    ecef_imu_pose_to_geodetic_base_pose,
    normalize_angle,
)
from gps_waypoint_dispatcher.rtk_authority import (
    CorrectionGate,
    CorrectionGateState,
    CorrectionReleaseMode,
    CorrectionReleaseState,
    LocalOdomBridge,
    LocalOdomBridgeResult,
    LocalOdomBridgeState,
    Pose2D,
    StampedPoseHistory,
    compose_pose,
    compute_map_to_odom,
)
from gps_waypoint_dispatcher.scene_runtime import (
    FixedENUProjector,
    quaternion_to_yaw,
    yaw_to_quaternion,
)


def _stamp_s(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def _lio_pose(msg: Odometry) -> Pose2D | None:
    position = msg.pose.pose.position
    orientation = msg.pose.pose.orientation
    values = (position.x, position.y, orientation.x, orientation.y, orientation.z, orientation.w)
    if not all(math.isfinite(float(value)) for value in values):
        return None
    yaw = quaternion_to_yaw(orientation.x, orientation.y, orientation.z, orientation.w)
    if not math.isfinite(yaw):
        return None
    return Pose2D(float(position.x), float(position.y), yaw)


@dataclass(frozen=True)
class Alignment:
    theta: float
    tx: float
    ty: float


class FgoMapOdomCorrector(Node):
    """Publish map->odom only for a fresh, receiver-fixed FGO solution."""

    def __init__(self) -> None:
        super().__init__("fgo_map_odom_corrector")
        self._declare_parameters()
        self._read_parameters()
        self._create_interfaces()

    def _declare_parameters(self) -> None:
        for name, value in (
            ("map_frame", "map"),
            ("odom_frame", "odom"),
            ("base_frame", "base_footprint"),
            ("fgo_odometry_topic", "/fgo_gil/odom"),
            ("fgo_ambiguity_topic", "/fgo_gil/ambiguity_status"),
            ("fgo_performance_topic", "/fgo_gil/performance"),
            ("lio_odom_topic", "/fastlio2/lio_odom"),
            ("source_alignment_topic", "/gps_corridor/enu_to_map"),
            ("frozen_alignment_topic", "/fgo_gil/enu_to_map"),
            ("map_base_topic", "/fgo_gil/map_base"),
            ("mode_topic", "/localization_authority/mode"),
            ("status_topic", "/localization_authority/status"),
            ("diagnostics_topic", "/localization_authority/diagnostics"),
            ("motion_allowed_topic", "/localization_authority/motion_allowed"),
            ("motion_speed_limit_topic", "/localization_authority/max_linear_speed_mps"),
            ("expected_fgo_frame", "ecef"),
            ("publish_period_s", 0.10),
            ("max_future_stamp_s", 0.10),
            ("max_lio_age_s", 0.20),
            ("max_fgo_odom_age_s", 1.50),
            ("max_fgo_status_age_s", 1.50),
            ("max_odom_bracket_s", 0.20),
            ("fgo_gate_timeout_s", 2.50),
            ("enu_origin_lat", 31.274927),
            ("enu_origin_lon", 120.737548),
            ("enu_origin_alt", 0.0),
            ("alignment_min_candidates", 5),
            ("alignment_min_span_s", 0.40),
            ("alignment_locked_translation_m", 0.20),
            ("alignment_locked_yaw_deg", 1.0),
            ("fgo_locked_translation_m", 0.80),
            ("fgo_locked_yaw_deg", 10.0),
            ("fgo_recovery_translation_m", 0.25),
            ("fgo_recovery_yaw_deg", 3.0),
            ("recovery_min_candidates", 3),
            ("recovery_min_span_s", 0.80),
            ("max_translation_rate_mps", 0.20),
            ("max_yaw_rate_degps", 2.0),
            ("backlog_translation_m", 0.50),
            ("backlog_yaw_deg", 5.0),
            ("fault_translation_m", 2.0),
            ("fault_yaw_deg", 20.0),
            ("stopped_linear_rate_mps", 0.05),
            ("stopped_yaw_rate_degps", 2.0),
            ("stopped_confirmation_s", 1.0),
            ("recovery_translation_m", 0.15),
            ("recovery_yaw_deg", 2.0),
            ("recovery_confirmation_s", 1.0),
            ("enable_local_odom_bridge", True),
            ("local_bridge_max_duration_s", 10.0),
            ("local_bridge_max_distance_m", 8.0),
            ("local_bridge_max_step_translation_m", 0.30),
            ("local_bridge_max_step_yaw_deg", 8.0),
            ("local_bridge_max_linear_speed_mps", 0.25),
            ("fgo_authoritative_max_linear_speed_mps", 0.50),
        ):
            self.declare_parameter(name, value)
        self.declare_parameter("base_from_imu_m", [0.0, 0.0, -0.02])
        self.declare_parameter("accepted_solution_statuses", ["RECEIVER_FIXED"])

    def _read_parameters(self) -> None:
        value = lambda name: self.get_parameter(name).value
        self._map_frame = str(value("map_frame"))
        self._odom_frame = str(value("odom_frame"))
        self._base_frame = str(value("base_frame"))
        self._fgo_topic = str(value("fgo_odometry_topic"))
        self._lio_topic = str(value("lio_odom_topic"))
        self._source_alignment_topic = str(value("source_alignment_topic"))
        self._frozen_alignment_topic = str(value("frozen_alignment_topic"))
        self._expected_fgo_frame = str(value("expected_fgo_frame"))
        self._max_future_stamp_s = float(value("max_future_stamp_s"))
        self._max_lio_age_s = float(value("max_lio_age_s"))
        self._max_fgo_odom_age_s = float(value("max_fgo_odom_age_s"))
        self._max_fgo_status_age_s = float(value("max_fgo_status_age_s"))
        self._max_odom_bracket_s = float(value("max_odom_bracket_s"))
        self._accepted_solution_statuses = {str(item) for item in value("accepted_solution_statuses")}
        raw_offset = tuple(float(item) for item in value("base_from_imu_m"))
        if len(raw_offset) != 3 or not all(math.isfinite(item) for item in raw_offset):
            raise ValueError("base_from_imu_m must contain three finite values")
        if not self._accepted_solution_statuses:
            raise ValueError("accepted_solution_statuses must not be empty")
        self._base_from_imu_m = raw_offset
        self._projector = FixedENUProjector(
            float(value("enu_origin_lat")), float(value("enu_origin_lon")), float(value("enu_origin_alt"))
        )
        alignment_candidates, alignment_span = int(value("alignment_min_candidates")), float(value("alignment_min_span_s"))
        self._alignment_yaw_gate = CorrectionGate.yaw(
            locked_threshold=math.radians(float(value("alignment_locked_yaw_deg"))),
            recovery_threshold=math.radians(0.5), min_candidates=alignment_candidates,
            min_span_s=alignment_span, processable_timeout_s=5.0,
        )
        self._alignment_position_gate = CorrectionGate.translation(
            locked_threshold=float(value("alignment_locked_translation_m")), recovery_threshold=0.10,
            min_candidates=alignment_candidates, min_span_s=alignment_span, processable_timeout_s=5.0,
        )
        recovery_candidates, recovery_span = int(value("recovery_min_candidates")), float(value("recovery_min_span_s"))
        fgo_timeout = float(value("fgo_gate_timeout_s"))
        self._fgo_yaw_gate = CorrectionGate.yaw(
            locked_threshold=math.radians(float(value("fgo_locked_yaw_deg"))),
            recovery_threshold=math.radians(float(value("fgo_recovery_yaw_deg"))),
            min_candidates=recovery_candidates, min_span_s=recovery_span, processable_timeout_s=fgo_timeout,
        )
        self._fgo_position_gate = CorrectionGate.translation(
            locked_threshold=float(value("fgo_locked_translation_m")),
            recovery_threshold=float(value("fgo_recovery_translation_m")),
            min_candidates=recovery_candidates, min_span_s=recovery_span, processable_timeout_s=fgo_timeout,
        )
        self._release = CorrectionReleaseState(
            max_translation_rate_mps=float(value("max_translation_rate_mps")),
            max_yaw_rate_radps=math.radians(float(value("max_yaw_rate_degps"))),
            max_lio_age_s=self._max_lio_age_s,
            backlog_translation_m=float(value("backlog_translation_m")),
            backlog_yaw_rad=math.radians(float(value("backlog_yaw_deg"))),
            fault_translation_m=float(value("fault_translation_m")),
            fault_yaw_rad=math.radians(float(value("fault_yaw_deg"))),
            stopped_linear_rate_mps=float(value("stopped_linear_rate_mps")),
            stopped_yaw_rate_radps=math.radians(float(value("stopped_yaw_rate_degps"))),
            stopped_confirmation_s=float(value("stopped_confirmation_s")),
            recovery_translation_m=float(value("recovery_translation_m")),
            recovery_yaw_rad=math.radians(float(value("recovery_yaw_deg"))),
            recovery_confirmation_s=float(value("recovery_confirmation_s")),
        )
        self._enable_bridge = bool(value("enable_local_odom_bridge"))
        self._bridge = LocalOdomBridge(
            max_duration_s=float(value("local_bridge_max_duration_s")),
            max_distance_m=float(value("local_bridge_max_distance_m")),
            max_step_translation_m=float(value("local_bridge_max_step_translation_m")),
            max_step_yaw_rad=math.radians(float(value("local_bridge_max_step_yaw_deg"))),
        )
        self._bridge_speed_mps = float(value("local_bridge_max_linear_speed_mps"))
        self._authoritative_speed_mps = float(value("fgo_authoritative_max_linear_speed_mps"))
        self._lio_history = StampedPoseHistory(max_age_s=3.0)
        self._latest_lio_pose: Pose2D | None = None
        self._latest_lio_stamp_s: float | None = None
        self._latest_lio_mono_s: float | None = None
        self._previous_lio_pose: Pose2D | None = None
        self._previous_lio_stamp_s: float | None = None
        self._local_linear_rate_mps = math.inf
        self._local_yaw_rate_radps = math.inf
        self._latest_fgo: Odometry | None = None
        self._latest_fgo_mono_s: float | None = None
        self._last_processed_fgo_stamp_s: float | None = None
        self._solution_status = "WAITING"
        self._solution_mono_s: float | None = None
        self._performance_healthy = False
        self._performance_mono_s: float | None = None
        self._frozen_alignment: Alignment | None = None
        self._last_output: Pose2D | None = None
        self._last_authoritative_mono_s: float | None = None
        self._bridge_map_odom: Pose2D | None = None
        self._last_mode, self._last_status = "", ""

    def _create_interfaces(self) -> None:
        value = lambda name: self.get_parameter(name).value
        self._mode_pub = self.create_publisher(String, str(value("mode_topic")), 10)
        self._status_pub = self.create_publisher(String, str(value("status_topic")), 10)
        self._diagnostics_pub = self.create_publisher(Float64MultiArray, str(value("diagnostics_topic")), 10)
        self._motion_pub = self.create_publisher(Bool, str(value("motion_allowed_topic")), 10)
        self._speed_pub = self.create_publisher(Float32, str(value("motion_speed_limit_topic")), 10)
        self._alignment_pub = self.create_publisher(Float64MultiArray, self._frozen_alignment_topic, 10)
        self._map_base_pub = self.create_publisher(Odometry, str(value("map_base_topic")), 10)
        self.create_subscription(Odometry, self._fgo_topic, self._fgo_callback, 10)
        self.create_subscription(Odometry, self._lio_topic, self._lio_callback, 50)
        self.create_subscription(DiagnosticArray, str(value("fgo_ambiguity_topic")), self._ambiguity_callback, 10)
        self.create_subscription(DiagnosticArray, str(value("fgo_performance_topic")), self._performance_callback, 10)
        self.create_subscription(Float64MultiArray, self._source_alignment_topic, self._alignment_callback, 10)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._timer = self.create_timer(float(value("publish_period_s")), self._timer_callback)

    def _ros_now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _valid_stamp(self, stamp_s: float) -> bool:
        return math.isfinite(stamp_s) and stamp_s > 0.0 and stamp_s <= self._ros_now_s() + self._max_future_stamp_s

    @staticmethod
    def _age(received_mono_s: float | None, now_mono_s: float) -> float:
        return math.inf if received_mono_s is None else max(0.0, now_mono_s - received_mono_s)

    def _lio_callback(self, msg: Odometry) -> None:
        stamp_s, pose = _stamp_s(msg.header.stamp), _lio_pose(msg)
        if pose is None or not self._valid_stamp(stamp_s):
            return
        if not self._lio_history.append(stamp_s, pose, frame_id=msg.header.frame_id, child_frame_id=msg.child_frame_id).accepted:
            return
        self._previous_lio_pose, self._previous_lio_stamp_s = self._latest_lio_pose, self._latest_lio_stamp_s
        self._latest_lio_pose, self._latest_lio_stamp_s, self._latest_lio_mono_s = pose, stamp_s, time.monotonic()
        if self._previous_lio_pose is None or self._previous_lio_stamp_s is None:
            return
        dt_s = stamp_s - self._previous_lio_stamp_s
        if dt_s > 0.0:
            self._local_linear_rate_mps = math.hypot(pose.x - self._previous_lio_pose.x, pose.y - self._previous_lio_pose.y) / dt_s
            self._local_yaw_rate_radps = abs(normalize_angle(pose.yaw - self._previous_lio_pose.yaw)) / dt_s

    def _fgo_callback(self, msg: Odometry) -> None:
        stamp_s, position, orientation = _stamp_s(msg.header.stamp), msg.pose.pose.position, msg.pose.pose.orientation
        values = (position.x, position.y, position.z, orientation.x, orientation.y, orientation.z, orientation.w)
        if not self._valid_stamp(stamp_s) or msg.header.frame_id != self._expected_fgo_frame or not all(math.isfinite(float(item)) for item in values):
            return
        if self._last_processed_fgo_stamp_s is not None and stamp_s <= self._last_processed_fgo_stamp_s:
            return
        self._latest_fgo, self._latest_fgo_mono_s = msg, time.monotonic()

    def _ambiguity_callback(self, msg: DiagnosticArray) -> None:
        for status in msg.status:
            if status.name == "fgo_gil/ambiguity":
                values = {item.key: item.value for item in status.values}
                self._solution_status, self._solution_mono_s = values.get("solution_status", status.message).strip(), time.monotonic()

    def _performance_callback(self, msg: DiagnosticArray) -> None:
        for status in msg.status:
            if status.name == "fgo_gil/performance":
                values = {item.key: item.value for item in status.values}
                stale = values.get("output_stale", "true").strip().lower() == "true"
                self._performance_healthy, self._performance_mono_s = status.level == 0 and not stale, time.monotonic()

    def _alignment_callback(self, msg: Float64MultiArray) -> None:
        if self._frozen_alignment is not None or len(msg.data) < 4 or float(msg.data[3]) < 0.5:
            return
        theta, tx, ty = (float(item) for item in msg.data[:3])
        if not all(math.isfinite(item) for item in (theta, tx, ty)):
            return
        now_mono_s, stamp_s = time.monotonic(), self._ros_now_s()
        yaw = self._alignment_yaw_gate.observe(stamp_s, theta, now_s=now_mono_s)
        position = self._alignment_position_gate.observe(stamp_s, (tx, ty), now_s=now_mono_s)
        if yaw.state is CorrectionGateState.LOCKED and position.state is CorrectionGateState.LOCKED:
            locked_yaw, locked_xy = self._alignment_yaw_gate.target, self._alignment_position_gate.target
            if isinstance(locked_yaw, (float, int)) and isinstance(locked_xy, tuple):
                self._frozen_alignment = Alignment(float(locked_yaw), locked_xy[0], locked_xy[1])
                self.get_logger().info("FGO authority froze stable ENU->map alignment")

    def _source_healthy(self, now_mono_s: float) -> bool:
        return (
            self._latest_fgo is not None
            and self._age(self._latest_fgo_mono_s, now_mono_s) <= self._max_fgo_odom_age_s
            and self._solution_status in self._accepted_solution_statuses
            and self._age(self._solution_mono_s, now_mono_s) <= self._max_fgo_status_age_s
            and self._performance_healthy
            and self._age(self._performance_mono_s, now_mono_s) <= self._max_fgo_status_age_s
        )

    def _local_fresh(self, now_mono_s: float) -> bool:
        return self._latest_lio_pose is not None and self._latest_lio_stamp_s is not None and self._age(self._latest_lio_mono_s, now_mono_s) <= self._max_lio_age_s

    def _process_fgo(self, now_mono_s: float) -> None:
        msg = self._latest_fgo
        if msg is None or self._frozen_alignment is None or not self._source_healthy(now_mono_s):
            return
        stamp_s = _stamp_s(msg.header.stamp)
        if self._last_processed_fgo_stamp_s is not None and stamp_s <= self._last_processed_fgo_stamp_s:
            return
        self._last_processed_fgo_stamp_s = stamp_s
        position, orientation = msg.pose.pose.position, msg.pose.pose.orientation
        geodetic = ecef_imu_pose_to_geodetic_base_pose(
            position_ecef_m=(float(position.x), float(position.y), float(position.z)),
            orientation_ecef_imu_xyzw=(float(orientation.x), float(orientation.y), float(orientation.z), float(orientation.w)),
            base_from_imu_m=self._base_from_imu_m,
        )
        lio = self._lio_history.interpolate(stamp_s, self._max_odom_bracket_s)
        if geodetic is None or not lio.ok:
            return
        enu_x, enu_y = self._projector.forward(geodetic.latitude_deg, geodetic.longitude_deg)
        alignment = self._frozen_alignment
        cosine, sine = math.cos(alignment.theta), math.sin(alignment.theta)
        map_base = Pose2D(
            alignment.tx + cosine * enu_x - sine * enu_y,
            alignment.ty + sine * enu_x + cosine * enu_y,
            normalize_angle(alignment.theta + geodetic.enu_yaw_rad),
        )
        target = compute_map_to_odom(map_base, lio.pose)
        self._fgo_yaw_gate.observe(stamp_s, target.yaw, now_s=now_mono_s)
        self._fgo_position_gate.observe(stamp_s, (target.x, target.y), now_s=now_mono_s)

    def _target(self) -> Pose2D | None:
        if self._fgo_yaw_gate.state is not CorrectionGateState.LOCKED or self._fgo_position_gate.state is not CorrectionGateState.LOCKED:
            return None
        yaw, xy = self._fgo_yaw_gate.target, self._fgo_position_gate.target
        return Pose2D(xy[0], xy[1], float(yaw)) if isinstance(yaw, (float, int)) and isinstance(xy, tuple) else None

    def _release_authority(self, now_mono_s: float):
        target = self._target()
        if target is None or not self._local_fresh(now_mono_s):
            return None
        if self._last_output is None:
            self._last_output = target
        lio_age_s = max(0.0, self._ros_now_s() - self._latest_lio_stamp_s)
        result = self._release.update(
            previous_output_map_odom=self._last_output, target_map_odom=target,
            local_pose=self._latest_lio_pose, now_s=now_mono_s,
            lio_stamp_s=self._latest_lio_stamp_s, lio_age_s=lio_age_s,
            local_linear_rate_mps=self._local_linear_rate_mps, local_yaw_rate_radps=self._local_yaw_rate_radps,
            gates_locked=True,
        )
        self._last_output = result.output_map_odom
        return result

    def _bridge_result(self, now_mono_s: float) -> LocalOdomBridgeResult | None:
        if not self._enable_bridge or self._last_authoritative_mono_s is None or self._last_output is None:
            return None
        if self._bridge.state is LocalOdomBridgeState.IDLE:
            self._bridge_map_odom = self._last_output
        return self._bridge.evaluate(now_s=now_mono_s, local_pose=self._latest_lio_pose, local_fresh=self._local_fresh(now_mono_s))

    def _publish_tf(self, map_odom: Pose2D) -> None:
        msg = TransformStamped()
        msg.header.stamp, msg.header.frame_id, msg.child_frame_id = self.get_clock().now().to_msg(), self._map_frame, self._odom_frame
        msg.transform.translation.x, msg.transform.translation.y = map_odom.x, map_odom.y
        qx, qy, qz, qw = yaw_to_quaternion(map_odom.yaw)
        msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w = qx, qy, qz, qw
        self._tf_broadcaster.sendTransform(msg)

    def _publish_map_base(self, map_odom: Pose2D) -> None:
        if self._latest_lio_pose is None:
            return
        pose = compose_pose(map_odom, self._latest_lio_pose)
        msg = Odometry()
        msg.header.stamp, msg.header.frame_id, msg.child_frame_id = self.get_clock().now().to_msg(), self._map_frame, self._base_frame
        msg.pose.pose.position.x, msg.pose.pose.position.y = pose.x, pose.y
        qx, qy, qz, qw = yaw_to_quaternion(pose.yaw)
        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = qx, qy, qz, qw
        self._map_base_pub.publish(msg)

    def _publish_alignment(self) -> None:
        alignment = self._frozen_alignment
        data = [0.0, 0.0, 0.0, 0.0] if alignment is None else [alignment.theta, alignment.tx, alignment.ty, 1.0]
        self._alignment_pub.publish(Float64MultiArray(data=data))

    def _publish_mode(self, mode: str, status: str) -> None:
        if mode != self._last_mode:
            self.get_logger().info(mode)
            self._last_mode = mode
        if status != self._last_status:
            self.get_logger().info(status)
            self._last_status = status
        self._mode_pub.publish(String(data=mode))
        self._status_pub.publish(String(data=status))

    def _publish_motion(self, allowed: bool, speed_mps: float = 0.0) -> None:
        self._motion_pub.publish(Bool(data=allowed))
        self._speed_pub.publish(Float32(data=float(speed_mps if allowed else 0.0)))

    def _publish_diagnostics(self, allowed: bool, bridge: LocalOdomBridgeResult | None) -> None:
        now_mono_s, target, output = time.monotonic(), self._target(), self._last_output
        self._diagnostics_pub.publish(Float64MultiArray(data=[
            1.0 if allowed else 0.0, self._age(self._latest_fgo_mono_s, now_mono_s),
            self._age(self._solution_mono_s, now_mono_s), self._age(self._performance_mono_s, now_mono_s),
            1.0 if self._frozen_alignment is not None else 0.0, float(self._fgo_position_gate.state), float(self._fgo_yaw_gate.state),
            target.x if target else math.nan, target.y if target else math.nan, math.degrees(target.yaw) if target else math.nan,
            output.x if output else math.nan, output.y if output else math.nan, math.degrees(output.yaw) if output else math.nan,
            1.0 if bridge and bridge.allowed else 0.0, bridge.elapsed_s if bridge else math.nan, bridge.distance_m if bridge else math.nan,
        ]))

    def _timer_callback(self) -> None:
        if not rclpy.ok():
            return
        now_mono_s = time.monotonic()
        # gps_global_aligner needs a complete map->odom->base chain to derive
        # its initial ENU->map relation. Publish only this stationary bootstrap
        # while authority remains false; the command guard therefore stays shut.
        if self._last_output is None and self._local_fresh(now_mono_s):
            self._last_output = Pose2D(0.0, 0.0, 0.0)
        self._publish_alignment()
        self._process_fgo(now_mono_s)
        release = self._release_authority(now_mono_s) if self._source_healthy(now_mono_s) else None
        if release is not None and release.motion_allowed:
            self._last_authoritative_mono_s = now_mono_s
            self._bridge.reset()
            self._bridge_map_odom = None
            self._publish_tf(release.output_map_odom)
            self._publish_map_base(release.output_map_odom)
            self._publish_mode("FGO_AUTHORITATIVE", "RECEIVER_FIXED")
            self._publish_motion(True, self._authoritative_speed_mps)
            self._publish_diagnostics(True, None)
            return
        self._fgo_yaw_gate.check_timeout(now_s=now_mono_s)
        self._fgo_position_gate.check_timeout(now_s=now_mono_s)
        bridge = None if release and release.mode in {CorrectionReleaseMode.FAULT_HOLD, CorrectionReleaseMode.LOCAL_ODOM_STALE} else self._bridge_result(now_mono_s)
        if bridge is not None and bridge.allowed and self._bridge_map_odom is not None:
            self._publish_tf(self._bridge_map_odom)
            self._publish_map_base(self._bridge_map_odom)
            self._publish_mode("LIO_BRIDGE", bridge.reason)
            self._publish_motion(True, self._bridge_speed_mps)
            self._publish_diagnostics(True, bridge)
            return
        if self._last_output is not None:
            self._publish_tf(self._last_output)
            self._publish_map_base(self._last_output)
        status = "WAITING_FOR_STABLE_ALIGNMENT" if self._frozen_alignment is None else (f"FGO_UNHEALTHY:{self._solution_status}" if not self._source_healthy(now_mono_s) else "WAITING_FOR_FGO_RECOVERY")
        self._publish_mode("FGO_DEGRADED", status)
        self._publish_motion(False)
        self._publish_diagnostics(False, bridge)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FgoMapOdomCorrector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
