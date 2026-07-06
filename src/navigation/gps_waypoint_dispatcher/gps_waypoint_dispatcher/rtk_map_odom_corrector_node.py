#!/usr/bin/env python3
from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import QuaternionStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64MultiArray, String
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from gps_waypoint_dispatcher.alignment_math import heading_quaternion_yaw_to_enu_yaw
from gps_waypoint_dispatcher.rtk_authority import (
    Pose2D,
    compute_bootstrap_alignment_from_current_pose,
    compute_map_to_odom,
    compute_rtk_map_base,
    limit_pose_step,
    normalize_angle,
    summarize_authority_inputs,
)
from gps_waypoint_dispatcher.scene_runtime import (
    FixedENUProjector,
    quaternion_to_yaw,
    yaw_to_quaternion,
)


def valid_fix(msg: NavSatFix | None) -> bool:
    if msg is None:
        return False
    if msg.status.status < 0:
        return False
    return math.isfinite(msg.latitude) and math.isfinite(msg.longitude)


class RtkMapOdomCorrector(Node):
    def __init__(self) -> None:
        super().__init__("rtk_map_odom_corrector")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("fix_topic", "/fix")
        self.declare_parameter("heading_topic", "/heading")
        self.declare_parameter("rtk_status_topic", "/rtk/status")
        self.declare_parameter("alignment_topic", "/gps_corridor/enu_to_map")
        self.declare_parameter("mode_topic", "/localization_authority/mode")
        self.declare_parameter("status_topic", "/localization_authority/status")
        self.declare_parameter("diagnostics_topic", "/localization_authority/diagnostics")
        self.declare_parameter("enu_origin_lat", 0.0)
        self.declare_parameter("enu_origin_lon", 0.0)
        self.declare_parameter("enu_origin_alt", 0.0)
        self.declare_parameter("heading_quaternion_yaw_is_compass", True)
        self.declare_parameter("require_rtk_fixed", True)
        self.declare_parameter("max_fix_age_s", 1.0)
        self.declare_parameter("max_heading_age_s", 1.0)
        self.declare_parameter("max_alignment_age_s", 1.0)
        self.declare_parameter("max_rtk_status_age_s", 2.0)
        self.declare_parameter("max_target_jump_m", 2.0)
        self.declare_parameter("max_target_yaw_jump_deg", 20.0)
        self.declare_parameter("max_translation_step_m", 0.20)
        self.declare_parameter("max_yaw_step_deg", 1.0)
        self.declare_parameter("publish_period_s", 0.05)
        self.declare_parameter("tf_lookup_timeout_s", 0.05)

        self._map_frame = str(self.get_parameter("map_frame").value)
        self._odom_frame = str(self.get_parameter("odom_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._fix_topic = str(self.get_parameter("fix_topic").value)
        self._heading_topic = str(self.get_parameter("heading_topic").value)
        self._rtk_status_topic = str(self.get_parameter("rtk_status_topic").value)
        self._alignment_topic = str(self.get_parameter("alignment_topic").value)
        self._mode_topic = str(self.get_parameter("mode_topic").value)
        self._status_topic = str(self.get_parameter("status_topic").value)
        self._diagnostics_topic = str(self.get_parameter("diagnostics_topic").value)
        self._heading_quaternion_yaw_is_compass = bool(
            self.get_parameter("heading_quaternion_yaw_is_compass").value
        )
        self._require_rtk_fixed = bool(self.get_parameter("require_rtk_fixed").value)
        self._max_fix_age_s = float(self.get_parameter("max_fix_age_s").value)
        self._max_heading_age_s = float(self.get_parameter("max_heading_age_s").value)
        self._max_alignment_age_s = float(self.get_parameter("max_alignment_age_s").value)
        self._max_rtk_status_age_s = float(
            self.get_parameter("max_rtk_status_age_s").value
        )
        self._max_target_jump_m = float(self.get_parameter("max_target_jump_m").value)
        self._max_target_yaw_jump_rad = math.radians(
            float(self.get_parameter("max_target_yaw_jump_deg").value)
        )
        self._max_translation_step_m = float(
            self.get_parameter("max_translation_step_m").value
        )
        self._max_yaw_step_rad = math.radians(
            float(self.get_parameter("max_yaw_step_deg").value)
        )
        self._publish_period_s = float(self.get_parameter("publish_period_s").value)
        self._tf_lookup_timeout_s = float(self.get_parameter("tf_lookup_timeout_s").value)

        self._projector = FixedENUProjector(
            float(self.get_parameter("enu_origin_lat").value),
            float(self.get_parameter("enu_origin_lon").value),
            float(self.get_parameter("enu_origin_alt").value),
        )

        self._mode_pub = self.create_publisher(String, self._mode_topic, 10)
        self._status_pub = self.create_publisher(String, self._status_topic, 10)
        self._diagnostics_pub = self.create_publisher(
            Float64MultiArray, self._diagnostics_topic, 10
        )

        self._fix_sub = self.create_subscription(
            NavSatFix, self._fix_topic, self._fix_callback, 10
        )
        self._heading_sub = self.create_subscription(
            QuaternionStamped, self._heading_topic, self._heading_callback, 10
        )
        self._rtk_status_sub = self.create_subscription(
            String, self._rtk_status_topic, self._rtk_status_callback, 10
        )
        self._alignment_sub = self.create_subscription(
            Float64MultiArray, self._alignment_topic, self._alignment_callback, 10
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._timer = self.create_timer(self._publish_period_s, self._timer_callback)

        self._latest_fix: NavSatFix | None = None
        self._latest_fix_mono: float | None = None
        self._latest_heading_enu_yaw: float | None = None
        self._latest_heading_mono: float | None = None
        self._latest_alignment: tuple[float, float, float, bool] | None = None
        self._latest_alignment_mono: float | None = None
        self._bootstrap_alignment: tuple[float, float, float, bool] | None = None
        self._latest_rtk_fixed = False
        self._latest_rtk_status_mono: float | None = None
        self._last_output: Pose2D | None = None
        self._last_mode = ""
        self._last_status = ""

    def _fix_callback(self, msg: NavSatFix) -> None:
        self._latest_fix = msg
        self._latest_fix_mono = time.monotonic()

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
        self._latest_heading_mono = time.monotonic()

    def _rtk_status_callback(self, msg: String) -> None:
        text = msg.data
        self._latest_rtk_fixed = "q=4" in text or "RTK Fixed" in text
        self._latest_rtk_status_mono = time.monotonic()

    def _alignment_callback(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 4:
            self._latest_alignment = None
            self._latest_alignment_mono = time.monotonic()
            return
        self._latest_alignment = (
            float(msg.data[0]),
            float(msg.data[1]),
            float(msg.data[2]),
            float(msg.data[3]) >= 0.5,
        )
        self._latest_alignment_mono = time.monotonic()

    def _age_s(self, stamp_mono: float | None, now_mono: float) -> float:
        if stamp_mono is None:
            return math.inf
        return now_mono - stamp_mono

    def _lookup_odom_base(self) -> Pose2D | None:
        try:
            transform = self._tf_buffer.lookup_transform(
                self._odom_frame,
                self._base_frame,
                Time(),
                timeout=Duration(seconds=self._tf_lookup_timeout_s),
            )
        except TransformException:
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return Pose2D(
            x=float(translation.x),
            y=float(translation.y),
            yaw=quaternion_to_yaw(rotation.x, rotation.y, rotation.z, rotation.w),
        )

    def _publish_mode_status(self, mode: str, status: str) -> None:
        if mode != self._last_mode:
            self.get_logger().info(mode)
            self._last_mode = mode
        if status != self._last_status:
            self.get_logger().info(status)
            self._last_status = status
        self._mode_pub.publish(String(data=mode))
        self._status_pub.publish(String(data=status))

    def _publish_diagnostics(
        self,
        *,
        ok: bool,
        fix_age_s: float,
        heading_age_s: float,
        alignment_age_s: float,
        target_jump_m: float,
        target_yaw_jump_rad: float,
        output: Pose2D | None,
    ) -> None:
        msg = Float64MultiArray()
        msg.data = [
            1.0 if ok else 0.0,
            fix_age_s,
            heading_age_s,
            alignment_age_s,
            target_jump_m,
            math.degrees(target_yaw_jump_rad),
            output.x if output is not None else 0.0,
            output.y if output is not None else 0.0,
            math.degrees(output.yaw) if output is not None else 0.0,
            1.0 if self._latest_rtk_fixed else 0.0,
        ]
        self._diagnostics_pub.publish(msg)

    def _publish_tf(self, pose: Pose2D) -> None:
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._map_frame
        msg.child_frame_id = self._odom_frame
        msg.transform.translation.x = pose.x
        msg.transform.translation.y = pose.y
        msg.transform.translation.z = 0.0
        qx, qy, qz, qw = yaw_to_quaternion(pose.yaw)
        msg.transform.rotation.x = qx
        msg.transform.rotation.y = qy
        msg.transform.rotation.z = qz
        msg.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(msg)

    def _timer_callback(self) -> None:
        now_mono = time.monotonic()
        fix_age_s = self._age_s(self._latest_fix_mono, now_mono)
        heading_age_s = self._age_s(self._latest_heading_mono, now_mono)
        alignment_age_s = self._age_s(self._latest_alignment_mono, now_mono)
        rtk_status_age_s = self._age_s(self._latest_rtk_status_mono, now_mono)

        odom_base = self._lookup_odom_base()
        odom_available = odom_base is not None
        external_alignment_valid = (
            self._latest_alignment is not None
            and self._latest_alignment[3]
            and alignment_age_s <= self._max_alignment_age_s
        )
        alignment = self._latest_alignment if external_alignment_valid else None
        if (
            alignment is None
            and odom_base is not None
            and valid_fix(self._latest_fix)
            and self._latest_heading_enu_yaw is not None
            and fix_age_s <= self._max_fix_age_s
            and heading_age_s <= self._max_heading_age_s
        ):
            enu_x, enu_y = self._projector.forward(
                self._latest_fix.latitude,
                self._latest_fix.longitude,
            )
            bootstrap = compute_bootstrap_alignment_from_current_pose(
                odom_base=odom_base,
                enu_x=enu_x,
                enu_y=enu_y,
                heading_enu_yaw=self._latest_heading_enu_yaw,
            )
            self._bootstrap_alignment = (
                bootstrap.theta,
                bootstrap.tx,
                bootstrap.ty,
                True,
            )
            alignment = self._bootstrap_alignment
            alignment_age_s = 0.0

        alignment_valid = alignment is not None and alignment[3]
        summary = summarize_authority_inputs(
            alignment_valid=alignment_valid,
            odom_available=odom_available,
            fix_age_s=fix_age_s,
            heading_age_s=heading_age_s,
            target_jump_m=0.0,
            target_yaw_jump_rad=0.0,
            max_fix_age_s=self._max_fix_age_s,
            max_heading_age_s=self._max_heading_age_s,
            max_target_jump_m=self._max_target_jump_m,
            max_target_yaw_jump_rad=self._max_target_yaw_jump_rad,
        )
        if not summary.ok:
            self._publish_mode_status(summary.mode, summary.reason or "RTK_DEGRADED")
            self._publish_diagnostics(
                ok=False,
                fix_age_s=fix_age_s,
                heading_age_s=heading_age_s,
                alignment_age_s=alignment_age_s,
                target_jump_m=0.0,
                target_yaw_jump_rad=0.0,
                output=self._last_output,
            )
            return

        rtk_fixed_ok = (
            not self._require_rtk_fixed
            or (self._latest_rtk_fixed and rtk_status_age_s <= self._max_rtk_status_age_s)
        )
        if not rtk_fixed_ok:
            self._publish_mode_status("RTK_DEGRADED", "NOT_RTK_FIXED")
            self._publish_diagnostics(
                ok=False,
                fix_age_s=fix_age_s,
                heading_age_s=heading_age_s,
                alignment_age_s=alignment_age_s,
                target_jump_m=0.0,
                target_yaw_jump_rad=0.0,
                output=self._last_output,
            )
            return

        if not valid_fix(self._latest_fix) or self._latest_heading_enu_yaw is None:
            self._publish_mode_status("RTK_DEGRADED", "INVALID_RTK_INPUT")
            return

        alignment_theta, alignment_tx, alignment_ty, _ = alignment
        enu_x, enu_y = self._projector.forward(
            self._latest_fix.latitude,
            self._latest_fix.longitude,
        )
        rtk_map_base = compute_rtk_map_base(
            enu_x=enu_x,
            enu_y=enu_y,
            heading_enu_yaw=self._latest_heading_enu_yaw,
            alignment_theta=alignment_theta,
            alignment_tx=alignment_tx,
            alignment_ty=alignment_ty,
        )
        target = compute_map_to_odom(rtk_map_base, odom_base)

        if self._last_output is None:
            target_jump_m = 0.0
            target_yaw_jump_rad = 0.0
            output = target
        else:
            target_jump_m = math.hypot(
                target.x - self._last_output.x,
                target.y - self._last_output.y,
            )
            target_yaw_jump_rad = normalize_angle(target.yaw - self._last_output.yaw)
            jump_summary = summarize_authority_inputs(
                alignment_valid=True,
                odom_available=True,
                fix_age_s=fix_age_s,
                heading_age_s=heading_age_s,
                target_jump_m=target_jump_m,
                target_yaw_jump_rad=target_yaw_jump_rad,
                max_fix_age_s=self._max_fix_age_s,
                max_heading_age_s=self._max_heading_age_s,
                max_target_jump_m=self._max_target_jump_m,
                max_target_yaw_jump_rad=self._max_target_yaw_jump_rad,
            )
            if not jump_summary.ok:
                self._publish_mode_status(
                    jump_summary.mode,
                    jump_summary.reason or "RTK_DEGRADED",
                )
                self._publish_diagnostics(
                    ok=False,
                    fix_age_s=fix_age_s,
                    heading_age_s=heading_age_s,
                    alignment_age_s=alignment_age_s,
                    target_jump_m=target_jump_m,
                    target_yaw_jump_rad=target_yaw_jump_rad,
                    output=self._last_output,
                )
                return

            output = limit_pose_step(
                self._last_output,
                target,
                max_translation_step_m=self._max_translation_step_m,
                max_yaw_step_rad=self._max_yaw_step_rad,
            ).pose

        self._last_output = output
        self._publish_tf(output)
        if external_alignment_valid:
            self._publish_mode_status("RTK_AUTHORITATIVE", "PUBLISHING_MAP_ODOM")
        else:
            self._publish_mode_status("RTK_BOOTSTRAP", "PUBLISHING_BOOTSTRAP_MAP_ODOM")
        self._publish_diagnostics(
            ok=True,
            fix_age_s=fix_age_s,
            heading_age_s=heading_age_s,
            alignment_age_s=alignment_age_s,
            target_jump_m=target_jump_m,
            target_yaw_jump_rad=target_yaw_jump_rad,
            output=output,
        )


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
