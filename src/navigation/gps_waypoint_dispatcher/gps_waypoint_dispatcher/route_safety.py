from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum


class WatchdogDecision(Enum):
    OK = "OK"
    IGNORE = "IGNORE"
    LOCAL_ABORT = "ODOM_DIVERGENCE_ABORT"
    GLOBAL_HOLD = "GLOBAL_CORRECTION_HOLD"


@dataclass(frozen=True)
class WatchdogResult:
    decision: WatchdogDecision
    reason: str | None
    linear_rate_mps: float = 0.0
    yaw_rate_radps: float = 0.0
    consecutive_violations: int = 0


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class LocalOdomWatchdog:
    def __init__(
        self,
        *,
        ordinary_linear_rate_mps: float = 3.0,
        ordinary_yaw_rate_radps: float = 3.0,
        ordinary_abort_count: int = 3,
        catastrophic_linear_rate_mps: float = 10.0,
        catastrophic_yaw_rate_radps: float = 10.0,
    ) -> None:
        self.ordinary_linear_rate_mps = ordinary_linear_rate_mps
        self.ordinary_yaw_rate_radps = ordinary_yaw_rate_radps
        self.ordinary_abort_count = ordinary_abort_count
        self.catastrophic_linear_rate_mps = catastrophic_linear_rate_mps
        self.catastrophic_yaw_rate_radps = catastrophic_yaw_rate_radps
        self._last_stamp_s: float | None = None
        self._last_pose: tuple[float, float, float] | None = None
        self._ordinary_count = 0
        self._aborted_result: WatchdogResult | None = None

    def update(self, stamp_s: float, x: float, y: float, yaw: float) -> WatchdogResult:
        if self._aborted_result is not None:
            return self._aborted_result
        if not all(math.isfinite(value) for value in (stamp_s, x, y, yaw)):
            self._aborted_result = WatchdogResult(
                WatchdogDecision.LOCAL_ABORT, "NONFINITE_LOCAL_ODOM"
            )
            return self._aborted_result
        if stamp_s <= 0.0:
            self._aborted_result = WatchdogResult(
                WatchdogDecision.LOCAL_ABORT, "INVALID_STAMP"
            )
            return self._aborted_result
        if self._last_stamp_s is not None:
            if stamp_s == self._last_stamp_s:
                return WatchdogResult(WatchdogDecision.IGNORE, "DUPLICATE_STAMP")
            if stamp_s < self._last_stamp_s:
                self._aborted_result = WatchdogResult(
                    WatchdogDecision.LOCAL_ABORT, "REGRESSING_STAMP"
                )
                return self._aborted_result

        if self._last_stamp_s is None or self._last_pose is None:
            self._last_stamp_s = stamp_s
            self._last_pose = (x, y, yaw)
            return WatchdogResult(WatchdogDecision.OK, None)

        dt_s = stamp_s - self._last_stamp_s
        previous_x, previous_y, previous_yaw = self._last_pose
        linear_rate_mps = math.hypot(x - previous_x, y - previous_y) / dt_s
        yaw_rate_radps = abs(_normalize_angle(yaw - previous_yaw)) / dt_s
        self._last_stamp_s = stamp_s
        self._last_pose = (x, y, yaw)

        if linear_rate_mps > self.catastrophic_linear_rate_mps:
            self._aborted_result = WatchdogResult(
                WatchdogDecision.LOCAL_ABORT,
                "CATASTROPHIC_LINEAR_RATE",
                linear_rate_mps,
                yaw_rate_radps,
                self._ordinary_count,
            )
            return self._aborted_result
        if yaw_rate_radps > self.catastrophic_yaw_rate_radps:
            self._aborted_result = WatchdogResult(
                WatchdogDecision.LOCAL_ABORT,
                "CATASTROPHIC_YAW_RATE",
                linear_rate_mps,
                yaw_rate_radps,
                self._ordinary_count,
            )
            return self._aborted_result

        violation = (
            linear_rate_mps > self.ordinary_linear_rate_mps
            or yaw_rate_radps > self.ordinary_yaw_rate_radps
        )
        self._ordinary_count = self._ordinary_count + 1 if violation else 0
        decision = (
            WatchdogDecision.LOCAL_ABORT
            if self._ordinary_count >= self.ordinary_abort_count
            else WatchdogDecision.OK
        )
        result = WatchdogResult(
            decision,
            "REPEATED_LOCAL_RATE" if decision is WatchdogDecision.LOCAL_ABORT else None,
            linear_rate_mps,
            yaw_rate_radps,
            self._ordinary_count,
        )
        if decision is WatchdogDecision.LOCAL_ABORT:
            self._aborted_result = result
        return result


class GlobalCorrectionWatchdog:
    def __init__(
        self,
        *,
        max_translation_rate_mps: float = 0.50,
        max_yaw_rate_radps: float = math.radians(5.0),
        max_authority_age_s: float = 0.50,
    ) -> None:
        self.max_translation_rate_mps = max_translation_rate_mps
        self.max_yaw_rate_radps = max_yaw_rate_radps
        self.max_authority_age_s = max_authority_age_s
        self._last_stamp_s: float | None = None
        self._last_pose: tuple[float, float, float] | None = None

    def update(
        self,
        stamp_s: float,
        x: float,
        y: float,
        yaw: float,
        *,
        authority_allowed: bool,
        authority_age_s: float,
    ) -> WatchdogResult:
        if not authority_allowed:
            return WatchdogResult(
                WatchdogDecision.GLOBAL_HOLD, "MOTION_AUTHORITY_FALSE"
            )
        if (
            not math.isfinite(authority_age_s)
            or authority_age_s > self.max_authority_age_s
        ):
            return WatchdogResult(
                WatchdogDecision.GLOBAL_HOLD, "MOTION_AUTHORITY_STALE"
            )
        if not all(math.isfinite(value) for value in (stamp_s, x, y, yaw)):
            return WatchdogResult(
                WatchdogDecision.GLOBAL_HOLD, "NONFINITE_GLOBAL_CORRECTION"
            )
        if stamp_s <= 0.0:
            return WatchdogResult(WatchdogDecision.GLOBAL_HOLD, "INVALID_STAMP")
        if self._last_stamp_s is not None:
            if stamp_s == self._last_stamp_s:
                return WatchdogResult(WatchdogDecision.IGNORE, "DUPLICATE_STAMP")
            if stamp_s < self._last_stamp_s:
                return WatchdogResult(
                    WatchdogDecision.GLOBAL_HOLD, "REGRESSING_GLOBAL_STAMP"
                )
        if self._last_stamp_s is None or self._last_pose is None:
            self._last_stamp_s = stamp_s
            self._last_pose = (x, y, yaw)
            return WatchdogResult(WatchdogDecision.OK, None)

        dt_s = stamp_s - self._last_stamp_s
        previous_x, previous_y, previous_yaw = self._last_pose
        linear_rate_mps = math.hypot(x - previous_x, y - previous_y) / dt_s
        yaw_rate_radps = abs(_normalize_angle(yaw - previous_yaw)) / dt_s
        self._last_stamp_s = stamp_s
        self._last_pose = (x, y, yaw)
        decision = (
            WatchdogDecision.GLOBAL_HOLD
            if linear_rate_mps > self.max_translation_rate_mps
            or yaw_rate_radps > self.max_yaw_rate_radps
            else WatchdogDecision.OK
        )
        return WatchdogResult(
            decision,
            "GLOBAL_CORRECTION_RATE"
            if decision is WatchdogDecision.GLOBAL_HOLD
            else None,
            linear_rate_mps,
            yaw_rate_radps,
        )

    def update_step(
        self,
        stamp_s: float,
        translation_step_m: float,
        yaw_step_rad: float,
        *,
        authority_allowed: bool,
        authority_age_s: float,
    ) -> WatchdogResult:
        """Evaluate one vehicle-space global-correction step.

        ``map -> odom`` translation is origin-dependent: a small yaw release
        far from the odom origin can create a large raw TF translation. The
        caller supplies the correction after both transforms are applied to
        the same current odom->base pose, so the rate is meaningful to Nav2.
        """
        if not authority_allowed:
            return WatchdogResult(
                WatchdogDecision.GLOBAL_HOLD, "MOTION_AUTHORITY_FALSE"
            )
        if (
            not math.isfinite(authority_age_s)
            or authority_age_s > self.max_authority_age_s
        ):
            return WatchdogResult(
                WatchdogDecision.GLOBAL_HOLD, "MOTION_AUTHORITY_STALE"
            )
        if not all(
            math.isfinite(value)
            for value in (stamp_s, translation_step_m, yaw_step_rad)
        ):
            return WatchdogResult(
                WatchdogDecision.GLOBAL_HOLD, "NONFINITE_GLOBAL_CORRECTION"
            )
        if stamp_s <= 0.0:
            return WatchdogResult(WatchdogDecision.GLOBAL_HOLD, "INVALID_STAMP")
        if translation_step_m < 0.0 or yaw_step_rad < 0.0:
            return WatchdogResult(
                WatchdogDecision.GLOBAL_HOLD, "INVALID_GLOBAL_CORRECTION_STEP"
            )
        if self._last_stamp_s is not None:
            if stamp_s == self._last_stamp_s:
                return WatchdogResult(WatchdogDecision.IGNORE, "DUPLICATE_STAMP")
            if stamp_s < self._last_stamp_s:
                return WatchdogResult(
                    WatchdogDecision.GLOBAL_HOLD, "REGRESSING_GLOBAL_STAMP"
                )
        if self._last_stamp_s is None:
            self._last_stamp_s = stamp_s
            return WatchdogResult(WatchdogDecision.OK, None)

        dt_s = stamp_s - self._last_stamp_s
        self._last_stamp_s = stamp_s
        linear_rate_mps = translation_step_m / dt_s
        yaw_rate_radps = yaw_step_rad / dt_s
        decision = (
            WatchdogDecision.GLOBAL_HOLD
            if linear_rate_mps > self.max_translation_rate_mps
            or yaw_rate_radps > self.max_yaw_rate_radps
            else WatchdogDecision.OK
        )
        return WatchdogResult(
            decision,
            "GLOBAL_CORRECTION_RATE"
            if decision is WatchdogDecision.GLOBAL_HOLD
            else None,
            linear_rate_mps,
            yaw_rate_radps,
        )


class ContinuousReadiness:
    def __init__(self, confirmation_s: float = 1.0) -> None:
        if not math.isfinite(confirmation_s) or confirmation_s < 0.0:
            raise ValueError("confirmation_s must be finite and nonnegative")
        self.confirmation_s = confirmation_s
        self._ready_since_s: float | None = None

    def update(self, *, ready: bool, now_s: float) -> bool:
        if not math.isfinite(now_s):
            self._ready_since_s = None
            return False
        if not ready:
            self._ready_since_s = None
            return False
        if self._ready_since_s is None:
            self._ready_since_s = now_s
        return now_s - self._ready_since_s >= self.confirmation_s


class BlockedRetryDecision(Enum):
    INACTIVE = "INACTIVE"
    WAIT = "WAIT"
    RETRY = "RETRY"
    TIMEOUT = "TIMEOUT"


@dataclass(frozen=True)
class BlockedRetryResult:
    decision: BlockedRetryDecision
    elapsed_s: float
    retry_count: int


class BlockedRetryState:
    def __init__(
        self,
        *,
        retry_delay_s: float = 2.0,
        timeout_s: float = 60.0,
        recovery_confirmation_s: float = 3.0,
    ) -> None:
        values = (retry_delay_s, timeout_s, recovery_confirmation_s)
        if not all(math.isfinite(value) and value >= 0.0 for value in values):
            raise ValueError("blocked retry durations must be finite and nonnegative")
        if timeout_s <= 0.0:
            raise ValueError("blocked retry timeout must be positive")
        self.retry_delay_s = retry_delay_s
        self.timeout_s = timeout_s
        self.recovery_confirmation_s = recovery_confirmation_s
        self._started_s: float | None = None
        self._retry_due_s: float | None = None
        self._retry_started_s: float | None = None
        self._recovery_started_s: float | None = None
        self._retry_count = 0

    @property
    def active(self) -> bool:
        return self._started_s is not None

    @property
    def retry_count(self) -> int:
        return self._retry_count

    def enter(self, now_s: float) -> BlockedRetryResult:
        self._validate_now(now_s)
        if self._started_s is None:
            self._started_s = now_s
            self._retry_count = 0
        self._retry_due_s = now_s + self.retry_delay_s
        self._retry_started_s = None
        self._recovery_started_s = None
        return self._result(BlockedRetryDecision.WAIT, now_s)

    def poll(
        self,
        now_s: float,
        *,
        retry_allowed: bool,
    ) -> BlockedRetryResult:
        self._validate_now(now_s)
        if self._started_s is None:
            return BlockedRetryResult(BlockedRetryDecision.INACTIVE, 0.0, 0)
        if now_s - self._started_s >= self.timeout_s:
            return self._result(BlockedRetryDecision.TIMEOUT, now_s)
        if (
            retry_allowed
            and self._retry_due_s is not None
            and now_s >= self._retry_due_s
        ):
            self._retry_due_s = None
            self._retry_started_s = now_s
            self._recovery_started_s = None
            self._retry_count += 1
            return self._result(BlockedRetryDecision.RETRY, now_s)
        return self._result(BlockedRetryDecision.WAIT, now_s)

    def confirm_action_running(self, now_s: float, *, moving: bool) -> bool:
        self._validate_now(now_s)
        if self._retry_started_s is None:
            return False
        if not moving:
            self._recovery_started_s = None
            return False
        if self._recovery_started_s is None:
            self._recovery_started_s = now_s
        if now_s - self._recovery_started_s < self.recovery_confirmation_s:
            return False
        self.clear()
        return True

    def clear(self) -> None:
        self._started_s = None
        self._retry_due_s = None
        self._retry_started_s = None
        self._recovery_started_s = None
        self._retry_count = 0

    @staticmethod
    def _validate_now(now_s: float) -> None:
        if not math.isfinite(now_s):
            raise ValueError("blocked retry time must be finite")

    def _result(
        self,
        decision: BlockedRetryDecision,
        now_s: float,
    ) -> BlockedRetryResult:
        elapsed_s = (
            max(0.0, now_s - self._started_s)
            if self._started_s is not None
            else 0.0
        )
        return BlockedRetryResult(decision, elapsed_s, self._retry_count)


@dataclass(frozen=True)
class MapGpsConsistencySummary:
    ok: bool
    warn: bool
    distance_m: float


@dataclass(frozen=True)
class TfFreshnessSummary:
    ok: bool
    age_s: float


@dataclass(frozen=True)
class TfWatchdogGapSummary:
    abort: bool
    reason: str | None


@dataclass(frozen=True)
class Nav2SuccessProgressSummary:
    ok: bool
    shortfall_m: float
    tolerance_m: float


def summarize_map_gps_consistency(
    map_xy: tuple[float, float],
    gps_map_xy: tuple[float, float],
    warn_m: float,
    abort_m: float,
) -> MapGpsConsistencySummary:
    distance_m = math.hypot(map_xy[0] - gps_map_xy[0], map_xy[1] - gps_map_xy[1])
    return MapGpsConsistencySummary(
        ok=distance_m <= abort_m,
        warn=distance_m > warn_m,
        distance_m=distance_m,
    )


def summarize_tf_freshness(now_s: float, stamp_s: float, max_age_s: float) -> TfFreshnessSummary:
    age_s = now_s - stamp_s
    return TfFreshnessSummary(
        ok=math.isfinite(age_s) and age_s >= -0.1 and age_s <= max_age_s,
        age_s=age_s,
    )


def summarize_tf_watchdog_gap(
    stale_count: int,
    stale_elapsed_s: float,
    abort_count: int,
    abort_after_s: float,
) -> TfWatchdogGapSummary:
    should_abort = (
        stale_count >= abort_count
        and math.isfinite(stale_elapsed_s)
        and stale_elapsed_s >= abort_after_s
    )
    return TfWatchdogGapSummary(
        abort=should_abort,
        reason=("TF_STALE_%.2fs" % stale_elapsed_s) if should_abort else None,
    )


def summarize_nav2_success_progress(
    target_progress_m: float,
    verified_progress_m: float,
    waypoint_tolerance_m: float,
    success_shortfall_tolerance_m: float,
) -> Nav2SuccessProgressSummary:
    shortfall_m = target_progress_m - verified_progress_m
    tolerance_m = max(waypoint_tolerance_m, success_shortfall_tolerance_m)
    ok = (
        math.isfinite(shortfall_m)
        and math.isfinite(tolerance_m)
        and shortfall_m <= tolerance_m
    )
    return Nav2SuccessProgressSummary(
        ok=ok,
        shortfall_m=shortfall_m,
        tolerance_m=tolerance_m,
    )


def is_rcl_context_shutdown_error_message(message: str) -> bool:
    normalized = message.lower()
    return (
        "context is not valid" in normalized
        and (
            "rcl_shutdown" in normalized
            or "rcl_init" in normalized
            or "wait set" in normalized
        )
    )
