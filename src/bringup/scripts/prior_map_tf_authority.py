#!/usr/bin/env python3

import json
import math
from collections import deque

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from interface.msg import LocalizationStatus
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from prior_map_tf_math import (
    bounded_map_to_odom_update,
    compose_se2,
    map_to_odom_from_poses,
    pose_residual,
    should_activate_localizer_seed,
    stable_se2_window,
)


def yaw_from_quaternion(quaternion):
    siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy_cosp = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw):
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


class PriorMapTfAuthority(Node):
    def __init__(self):
        super().__init__("prior_map_tf_authority")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("localizer_transform_topic", "/localizer/map_to_odom")
        self.declare_parameter("localizer_status_topic", "/localizer/status")
        self.declare_parameter("odom_topic", "/fastlio2/lio_odom")
        self.declare_parameter("amcl_pose_topic", "/amcl_pose")
        self.declare_parameter("amcl_initialpose_topic", "/amcl/initialpose")
        self.declare_parameter("initialpose_topic", "/initialpose")
        self.declare_parameter("status_topic", "/travel/prior_map_tf/status")
        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("status_hz", 5.0)
        self.declare_parameter("tf_future_tolerance_s", 0.10)
        self.declare_parameter("odom_history_s", 5.0)
        self.declare_parameter("max_odom_pose_skew_s", 0.15)
        self.declare_parameter("amcl_settle_s", 0.75)
        self.declare_parameter("amcl_stale_s", 3.0)
        self.declare_parameter("amcl_candidate_window_size", 5)
        self.declare_parameter("amcl_min_consistent_samples", 3)
        self.declare_parameter("amcl_window_reset_s", 1.5)
        self.declare_parameter("max_amcl_window_translation_spread_m", 0.05)
        self.declare_parameter("max_amcl_window_yaw_spread_rad", 0.04)
        self.declare_parameter("min_correction_interval_s", 1.0)
        self.declare_parameter("amcl_seed_retry_s", 1.0)
        self.declare_parameter("amcl_seed_max_attempts", 5)
        self.declare_parameter("max_position_variance", 0.25)
        self.declare_parameter("max_yaw_variance", 0.12)
        self.declare_parameter("max_target_base_jump_m", 0.75)
        self.declare_parameter("max_target_yaw_jump_rad", 0.45)
        self.declare_parameter("correction_alpha", 0.15)
        self.declare_parameter("translation_deadband_m", 0.05)
        self.declare_parameter("yaw_deadband_rad", 0.035)
        self.declare_parameter("max_translation_step_m", 0.03)
        self.declare_parameter("max_yaw_step_rad", 0.01)
        self.declare_parameter("max_base_step_m", 0.04)
        self.declare_parameter("max_translation_correction_speed_mps", 0.015)
        self.declare_parameter("max_yaw_correction_speed_rps", 0.006)
        self.declare_parameter("max_base_correction_speed_mps", 0.02)

        def param(name):
            return self.get_parameter(name).value

        self.map_frame = str(param("map_frame"))
        self.odom_frame = str(param("odom_frame"))
        self.publish_hz = max(1.0, float(param("publish_hz")))
        self.tf_future_tolerance_s = max(0.0, float(param("tf_future_tolerance_s")))
        self.odom_history_s = max(1.0, float(param("odom_history_s")))
        self.max_odom_pose_skew_s = max(0.0, float(param("max_odom_pose_skew_s")))
        self.amcl_settle_s = max(0.0, float(param("amcl_settle_s")))
        self.amcl_stale_s = max(0.1, float(param("amcl_stale_s")))
        self.amcl_candidate_window_size = max(
            1, int(param("amcl_candidate_window_size"))
        )
        self.amcl_min_consistent_samples = min(
            self.amcl_candidate_window_size,
            max(1, int(param("amcl_min_consistent_samples"))),
        )
        self.amcl_window_reset_s = max(0.1, float(param("amcl_window_reset_s")))
        self.max_amcl_window_translation_spread_m = max(
            0.0, float(param("max_amcl_window_translation_spread_m"))
        )
        self.max_amcl_window_yaw_spread_rad = max(
            0.0, float(param("max_amcl_window_yaw_spread_rad"))
        )
        self.min_correction_interval_s = max(
            0.0, float(param("min_correction_interval_s"))
        )
        self.amcl_seed_retry_s = max(0.1, float(param("amcl_seed_retry_s")))
        self.amcl_seed_max_attempts = max(1, int(param("amcl_seed_max_attempts")))
        self.max_position_variance = max(0.0, float(param("max_position_variance")))
        self.max_yaw_variance = max(0.0, float(param("max_yaw_variance")))
        self.max_target_base_jump_m = max(0.0, float(param("max_target_base_jump_m")))
        self.max_target_yaw_jump_rad = max(0.0, float(param("max_target_yaw_jump_rad")))
        self.correction_alpha = float(param("correction_alpha"))
        self.translation_deadband_m = max(0.0, float(param("translation_deadband_m")))
        self.yaw_deadband_rad = max(0.0, float(param("yaw_deadband_rad")))
        self.max_translation_step_m = max(0.0, float(param("max_translation_step_m")))
        self.max_yaw_step_rad = max(0.0, float(param("max_yaw_step_rad")))
        self.max_base_step_m = max(0.0, float(param("max_base_step_m")))
        self.max_translation_correction_speed_mps = max(
            0.0, float(param("max_translation_correction_speed_mps"))
        )
        self.max_yaw_correction_speed_rps = max(
            0.0, float(param("max_yaw_correction_speed_rps"))
        )
        self.max_base_correction_speed_mps = max(
            0.0, float(param("max_base_correction_speed_mps"))
        )

        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.tf_broadcaster = TransformBroadcaster(self)
        self.amcl_initialpose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, str(param("amcl_initialpose_topic")), 10
        )
        self.status_publisher = self.create_publisher(String, str(param("status_topic")), 10)
        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray, "/diagnostics", 10
        )
        self.create_subscription(
            TransformStamped,
            str(param("localizer_transform_topic")),
            self.on_localizer_transform,
            latched_qos,
        )
        self.create_subscription(
            LocalizationStatus,
            str(param("localizer_status_topic")),
            self.on_localizer_status,
            10,
        )
        self.create_subscription(Odometry, str(param("odom_topic")), self.on_odom, 50)
        self.create_subscription(
            PoseWithCovarianceStamped,
            str(param("amcl_pose_topic")),
            self.on_amcl_pose,
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            str(param("initialpose_topic")),
            self.on_initial_pose,
            10,
        )

        self.current_map_to_odom = None
        self.localizer_map_to_odom = None
        self.localizer_allowed = False
        self.localizer_state = "UNINITIALIZED"
        self.localizer_reason = "no_status"
        self.pending_localizer_seed = False
        self.odom_history = deque()
        self.pending_amcl_seed = None
        self.waiting_odom_for_seed = False
        self.amcl_seed_attempts = 0
        self.last_amcl_seed_time = None
        self.ignore_amcl_until = None
        self.last_amcl_pose_time = None
        self.last_amcl_valid_time = None
        self.last_amcl_correction_time = None
        self.last_amcl_residual_m = None
        self.last_amcl_residual_yaw = None
        self.last_amcl_window_translation_spread_m = None
        self.last_amcl_window_yaw_spread_rad = None
        self.amcl_candidates = deque(maxlen=self.amcl_candidate_window_size)
        self.correction_target_map_to_odom = None
        self.manual_relocalization_pending = False
        self.state = "WAITING_LOCALIZER"
        self.last_logged_state = None

        self.create_timer(1.0 / self.publish_hz, self.on_publish_timer)
        self.create_timer(
            1.0 / max(1.0, float(param("status_hz"))), self.publish_status
        )

    def on_localizer_status(self, msg):
        was_allowed = self.localizer_allowed
        self.localizer_allowed = bool(
            msg.localized
            and msg.sensors_ready
            and msg.state in (LocalizationStatus.LOCALIZED, LocalizationStatus.DEGRADED)
        )
        self.localizer_state = msg.state_label or str(msg.state)
        self.localizer_reason = msg.reason or "unspecified"
        if should_activate_localizer_seed(
            self.current_map_to_odom is not None,
            self.manual_relocalization_pending,
            self.localizer_allowed,
            self.localizer_reason,
        ):
            self.pending_localizer_seed = True
            self.try_activate_localizer_seed()
        elif self.localizer_allowed and not was_allowed:
            self.state = "LOCALIZER_RECOVERED_HOLD"
        elif not self.localizer_allowed:
            self.state = (
                "MANUAL_RELOCALIZING"
                if self.manual_relocalization_pending
                else (
                    "LOCALIZER_DEGRADED_HOLD"
                    if self.current_map_to_odom
                    else "WAITING_LOCALIZER"
                )
            )

    def on_localizer_transform(self, msg):
        if msg.header.frame_id != self.map_frame or msg.child_frame_id != self.odom_frame:
            self.get_logger().error(
                "Rejected localizer transform with unexpected frames "
                f"{msg.header.frame_id}->{msg.child_frame_id}"
            )
            return
        transform = msg.transform
        self.localizer_map_to_odom = (
            float(transform.translation.x),
            float(transform.translation.y),
            yaw_from_quaternion(transform.rotation),
        )
        self.try_activate_localizer_seed()

    def on_initial_pose(self, _msg):
        self.manual_relocalization_pending = True
        self.pending_localizer_seed = False
        self.pending_amcl_seed = None
        self.correction_target_map_to_odom = None
        self.amcl_candidates.clear()
        self.state = "MANUAL_RELOCALIZING"

    def on_odom(self, msg):
        pose = msg.pose.pose
        odom_pose = (
            float(pose.position.x),
            float(pose.position.y),
            yaw_from_quaternion(pose.orientation),
        )
        sample_time = stamp_seconds(msg.header.stamp)
        if sample_time <= 0.0:
            sample_time = self.get_clock().now().nanoseconds * 1.0e-9
        self.odom_history.append((sample_time, odom_pose))
        cutoff = sample_time - self.odom_history_s
        while self.odom_history and self.odom_history[0][0] < cutoff:
            self.odom_history.popleft()
        if self.current_map_to_odom is not None and self.waiting_odom_for_seed:
            self.prepare_amcl_seed()

    def try_activate_localizer_seed(self):
        if not (
            self.pending_localizer_seed
            and self.localizer_allowed
            and self.localizer_map_to_odom is not None
        ):
            return
        self.current_map_to_odom = self.localizer_map_to_odom
        self.pending_localizer_seed = False
        self.manual_relocalization_pending = False
        self.pending_amcl_seed = None
        self.waiting_odom_for_seed = True
        self.amcl_seed_attempts = 0
        self.ignore_amcl_until = self.get_clock().now() + Duration(
            seconds=self.amcl_settle_s
        )
        self.last_amcl_pose_time = None
        self.last_amcl_valid_time = None
        self.last_amcl_correction_time = None
        self.last_amcl_residual_m = None
        self.last_amcl_residual_yaw = None
        self.last_amcl_window_translation_spread_m = None
        self.last_amcl_window_yaw_spread_rad = None
        self.amcl_candidates.clear()
        self.correction_target_map_to_odom = None
        self.state = "LOCALIZER_SEED"
        self.prepare_amcl_seed()

    def prepare_amcl_seed(self):
        if self.current_map_to_odom is None or not self.odom_history:
            return
        map_from_base = compose_se2(
            self.current_map_to_odom, self.odom_history[-1][1]
        )
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.map_frame
        msg.pose.pose.position.x = map_from_base[0]
        msg.pose.pose.position.y = map_from_base[1]
        qx, qy, qz, qw = quaternion_from_yaw(map_from_base[2])
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance[0] = 0.04
        msg.pose.covariance[7] = 0.04
        msg.pose.covariance[35] = 0.03
        self.pending_amcl_seed = msg
        self.waiting_odom_for_seed = False
        self.state = "WAITING_AMCL"

    def nearest_odom_pose(self, target_time):
        if not self.odom_history:
            return None
        sample_time, pose = min(
            self.odom_history, key=lambda sample: abs(sample[0] - target_time)
        )
        if abs(sample_time - target_time) > self.max_odom_pose_skew_s:
            return None
        return pose

    def on_amcl_pose(self, msg):
        now = self.get_clock().now()
        self.last_amcl_pose_time = now
        if self.current_map_to_odom is None or self.manual_relocalization_pending:
            return
        if self.ignore_amcl_until is not None and now < self.ignore_amcl_until:
            return

        odom_pose = self.nearest_odom_pose(stamp_seconds(msg.header.stamp))
        if odom_pose is None:
            self.state = "AMCL_REJECTED_ODOM_SKEW"
            return
        covariance = msg.pose.covariance
        position_variance = max(float(covariance[0]), float(covariance[7]))
        yaw_variance = float(covariance[35])
        if (
            not math.isfinite(position_variance)
            or not math.isfinite(yaw_variance)
            or position_variance > self.max_position_variance
            or yaw_variance > self.max_yaw_variance
        ):
            self.state = "AMCL_REJECTED_COVARIANCE"
            return

        pose = msg.pose.pose
        map_pose = (
            float(pose.position.x),
            float(pose.position.y),
            yaw_from_quaternion(pose.orientation),
        )
        target_map_to_odom = map_to_odom_from_poses(map_pose, odom_pose)
        current_base = compose_se2(self.current_map_to_odom, odom_pose)
        target_base = compose_se2(target_map_to_odom, odom_pose)
        residual_m, residual_yaw = pose_residual(current_base, target_base)
        self.last_amcl_residual_m = residual_m
        self.last_amcl_residual_yaw = residual_yaw
        if (
            residual_m > self.max_target_base_jump_m
            or residual_yaw > self.max_target_yaw_jump_rad
        ):
            self.state = "AMCL_REJECTED_TARGET_JUMP"
            return

        # A valid AMCL result confirms that the seed was consumed. Stop retrying
        # while the temporal window decides whether the correction is stable.
        self.pending_amcl_seed = None
        if (
            self.last_amcl_valid_time is not None
            and (now - self.last_amcl_valid_time).nanoseconds * 1.0e-9
            > self.amcl_window_reset_s
        ):
            self.amcl_candidates.clear()
        self.last_amcl_valid_time = now
        self.amcl_candidates.append(target_map_to_odom)
        stable_target, translation_spread, yaw_spread = stable_se2_window(
            list(self.amcl_candidates),
            self.amcl_min_consistent_samples,
            self.max_amcl_window_translation_spread_m,
            self.max_amcl_window_yaw_spread_rad,
        )
        self.last_amcl_window_translation_spread_m = translation_spread
        self.last_amcl_window_yaw_spread_rad = yaw_spread
        if stable_target is None:
            if len(self.amcl_candidates) >= self.amcl_min_consistent_samples:
                self.correction_target_map_to_odom = None
            self.state = (
                "AMCL_ACCUMULATING"
                if len(self.amcl_candidates) < self.amcl_min_consistent_samples
                else "AMCL_REJECTED_UNSTABLE"
            )
            return

        stable_base = compose_se2(stable_target, odom_pose)
        residual_m, residual_yaw = pose_residual(current_base, stable_base)
        self.last_amcl_residual_m = residual_m
        self.last_amcl_residual_yaw = residual_yaw
        if (
            residual_m <= self.translation_deadband_m
            and residual_yaw <= self.yaw_deadband_rad
        ):
            self.correction_target_map_to_odom = None
            self.state = "AMCL_STABLE_HOLD"
            return

        if self.last_amcl_correction_time is not None:
            correction_age_s = (
                now - self.last_amcl_correction_time
            ).nanoseconds * 1.0e-9
            if correction_age_s < self.min_correction_interval_s:
                self.state = "AMCL_RATE_LIMITED"
                return

        # Accept a stable target here, but never change the published TF in the
        # AMCL callback. The 20 Hz publisher approaches it with correction-rate
        # limits so Foxglove, costmaps, and planners do not see centimetre steps.
        self.correction_target_map_to_odom = stable_target
        self.last_amcl_correction_time = now
        self.state = "AMCL_TARGET_ACCEPTED"

    def apply_correction_target(self):
        if (
            self.current_map_to_odom is None
            or self.correction_target_map_to_odom is None
            or not self.odom_history
        ):
            return
        if (
            self.last_amcl_valid_time is None
            or self.age_s(self.last_amcl_valid_time) > self.amcl_stale_s
        ):
            self.correction_target_map_to_odom = None
            self.state = "AMCL_STALE_HOLD"
            return

        cycle_s = 1.0 / self.publish_hz
        translation_step = min(
            self.max_translation_step_m,
            self.max_translation_correction_speed_mps * cycle_s,
        )
        yaw_step = min(
            self.max_yaw_step_rad,
            self.max_yaw_correction_speed_rps * cycle_s,
        )
        base_step = min(
            self.max_base_step_m,
            self.max_base_correction_speed_mps * cycle_s,
        )
        updated = bounded_map_to_odom_update(
            self.current_map_to_odom,
            self.correction_target_map_to_odom,
            self.odom_history[-1][1],
            self.correction_alpha,
            self.translation_deadband_m,
            self.yaw_deadband_rad,
            translation_step,
            yaw_step,
            base_step,
        )
        if updated == self.current_map_to_odom:
            self.correction_target_map_to_odom = None
            self.state = "AMCL_STABLE_HOLD"
            return
        self.current_map_to_odom = updated
        self.state = "AMCL_SMOOTHING"

    def publish_amcl_seed_if_needed(self):
        if self.pending_amcl_seed is None:
            return
        if self.amcl_seed_attempts >= self.amcl_seed_max_attempts:
            self.state = "AMCL_SEED_TIMEOUT"
            return
        now = self.get_clock().now()
        if self.last_amcl_seed_time is not None:
            elapsed = (now - self.last_amcl_seed_time).nanoseconds * 1.0e-9
            if elapsed < self.amcl_seed_retry_s:
                return
        if self.amcl_initialpose_publisher.get_subscription_count() < 1:
            return
        self.pending_amcl_seed.header.stamp = now.to_msg()
        self.amcl_initialpose_publisher.publish(self.pending_amcl_seed)
        self.amcl_seed_attempts += 1
        self.last_amcl_seed_time = now
        self.ignore_amcl_until = now + Duration(seconds=self.amcl_settle_s)

    def on_publish_timer(self):
        self.publish_amcl_seed_if_needed()
        if self.current_map_to_odom is None:
            return
        self.apply_correction_target()
        now = self.get_clock().now()
        msg = TransformStamped()
        msg.header.frame_id = self.map_frame
        msg.child_frame_id = self.odom_frame
        msg.header.stamp = (now + Duration(seconds=self.tf_future_tolerance_s)).to_msg()
        msg.transform.translation.x = self.current_map_to_odom[0]
        msg.transform.translation.y = self.current_map_to_odom[1]
        qx, qy, qz, qw = quaternion_from_yaw(self.current_map_to_odom[2])
        msg.transform.rotation.x = qx
        msg.transform.rotation.y = qy
        msg.transform.rotation.z = qz
        msg.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(msg)

    def age_s(self, stamp):
        if stamp is None:
            return None
        return max(0.0, (self.get_clock().now() - stamp).nanoseconds * 1.0e-9)

    def publish_status(self):
        amcl_valid_age = self.age_s(self.last_amcl_valid_time)
        correction_age = self.age_s(self.last_amcl_correction_time)
        reported_state = self.state
        if (
            self.current_map_to_odom is not None
            and not self.manual_relocalization_pending
            and amcl_valid_age is not None
            and amcl_valid_age > self.amcl_stale_s
        ):
            reported_state = "AMCL_STALE_HOLD"
        tf_active = bool(
            self.current_map_to_odom is not None
            and not self.manual_relocalization_pending
        )
        payload = {
            "state": reported_state,
            "tf_active": tf_active,
            "manual_relocalization_pending": self.manual_relocalization_pending,
            "localizer_state": self.localizer_state,
            "localizer_reason": self.localizer_reason,
            "amcl_seed_attempts": self.amcl_seed_attempts,
            "amcl_pose_age_s": self.age_s(self.last_amcl_pose_time),
            "amcl_valid_age_s": amcl_valid_age,
            "amcl_accepted_age_s": amcl_valid_age,
            "amcl_correction_age_s": correction_age,
            "amcl_residual_m": self.last_amcl_residual_m,
            "amcl_residual_yaw_rad": self.last_amcl_residual_yaw,
            "amcl_window_samples": len(self.amcl_candidates),
            "amcl_window_translation_spread_m": (
                self.last_amcl_window_translation_spread_m
            ),
            "amcl_window_yaw_spread_rad": self.last_amcl_window_yaw_spread_rad,
            "correction_pending": self.correction_target_map_to_odom is not None,
            "max_translation_correction_speed_mps": (
                self.max_translation_correction_speed_mps
            ),
            "max_yaw_correction_speed_rps": self.max_yaw_correction_speed_rps,
            "max_base_correction_speed_mps": self.max_base_correction_speed_mps,
        }
        self.status_publisher.publish(String(data=json.dumps(payload, sort_keys=True)))

        diagnostic = DiagnosticStatus()
        diagnostic.name = "travel/prior_map_tf_authority"
        diagnostic.hardware_id = "prior_map_localization"
        diagnostic.message = reported_state
        if reported_state == "WAITING_LOCALIZER":
            diagnostic.level = DiagnosticStatus.ERROR
        elif reported_state.startswith("AMCL_REJECTED") or reported_state in {
            "AMCL_ACCUMULATING",
            "AMCL_SEED_TIMEOUT",
            "AMCL_STALE_HOLD",
            "WAITING_AMCL",
            "LOCALIZER_DEGRADED_HOLD",
            "MANUAL_RELOCALIZING",
        }:
            diagnostic.level = DiagnosticStatus.WARN
        else:
            diagnostic.level = DiagnosticStatus.OK
        diagnostic.values = [
            KeyValue(key=key, value="null" if value is None else str(value))
            for key, value in payload.items()
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [diagnostic]
        self.diagnostics_publisher.publish(array)
        if reported_state != self.last_logged_state:
            self.get_logger().info(f"Prior-map TF authority state: {reported_state}")
            self.last_logged_state = reported_state


def main(args=None):
    rclpy.init(args=args)
    node = PriorMapTfAuthority()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
