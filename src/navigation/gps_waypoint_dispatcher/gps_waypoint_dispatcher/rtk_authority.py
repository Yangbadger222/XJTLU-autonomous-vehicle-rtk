from __future__ import annotations

import math
from bisect import bisect_left
from dataclasses import dataclass
from enum import Enum, IntEnum
from typing import Union


def _time_comparison_epsilon_s(*values: float) -> float:
    finite_ulps = [math.ulp(value) for value in values if math.isfinite(value)]
    return max(1e-9, 4.0 * max(finite_ulps, default=0.0))


def _finite_lerp(start: float, end: float, fraction: float) -> float | None:
    try:
        result = math.fsum(((1.0 - fraction) * start, fraction * end))
    except (OverflowError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _finite_mean(values: list[float]) -> float | None:
    count = len(values)
    try:
        result = math.fsum(value / count for value in values)
    except OverflowError:
        return None
    return result if math.isfinite(result) else None


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class StampedPose:
    stamp_s: float
    pose: Pose2D


@dataclass(frozen=True)
class HistoryAppendResult:
    accepted: bool
    reason: str | None
    sample_count: int


@dataclass(frozen=True)
class PoseInterpolationResult:
    ok: bool
    pose: Pose2D | None
    reason: str | None
    lower_stamp_s: float | None = None
    upper_stamp_s: float | None = None


class StampedPoseHistory:
    def __init__(
        self,
        max_age_s: float = 2.0,
        max_samples: int = 200,
        *,
        frame_id: str = "odom",
        child_frame_id: str = "base_footprint",
    ) -> None:
        if not math.isfinite(max_age_s) or max_age_s <= 0.0:
            raise ValueError("max_age_s must be finite and positive")
        if not isinstance(max_samples, int) or isinstance(max_samples, bool) or max_samples <= 0:
            raise ValueError("max_samples must be a positive integer")
        if not frame_id or not child_frame_id:
            raise ValueError("expected frame IDs must not be empty")

        self.max_age_s = max_age_s
        self.max_samples = max_samples
        self.frame_id = frame_id
        self.child_frame_id = child_frame_id
        self._samples: list[StampedPose] = []

    def __len__(self) -> int:
        return len(self._samples)

    @property
    def samples(self) -> tuple[StampedPose, ...]:
        return tuple(self._samples)

    def append(
        self,
        stamp_s: float,
        pose: Pose2D,
        *,
        frame_id: str,
        child_frame_id: str,
    ) -> HistoryAppendResult:
        if frame_id != self.frame_id:
            return self._append_result(False, "INVALID_FRAME")
        if child_frame_id != self.child_frame_id:
            return self._append_result(False, "INVALID_CHILD_FRAME")
        if not math.isfinite(stamp_s) or stamp_s <= 0.0:
            return self._append_result(False, "INVALID_STAMP")
        if not isinstance(pose, Pose2D) or not all(
            math.isfinite(value) for value in (pose.x, pose.y, pose.yaw)
        ):
            return self._append_result(False, "NONFINITE_POSE")

        if self._samples:
            latest_stamp_s = self._samples[-1].stamp_s
            if stamp_s == latest_stamp_s:
                return self._append_result(False, "DUPLICATE_STAMP")
            if stamp_s < latest_stamp_s:
                return self._append_result(False, "REGRESSING_STAMP")

        self._samples.append(StampedPose(stamp_s=stamp_s, pose=pose))
        oldest_allowed_s = stamp_s - self.max_age_s
        while self._samples and self._samples[0].stamp_s < oldest_allowed_s:
            del self._samples[0]
        if len(self._samples) > self.max_samples:
            del self._samples[: len(self._samples) - self.max_samples]
        return self._append_result(True, None)

    def interpolate(
        self,
        stamp_s: float,
        max_bracket_s: float = 0.20,
    ) -> PoseInterpolationResult:
        if not math.isfinite(stamp_s) or stamp_s <= 0.0:
            return PoseInterpolationResult(False, None, "INVALID_STAMP")
        if not math.isfinite(max_bracket_s) or max_bracket_s < 0.0:
            return PoseInterpolationResult(False, None, "INVALID_BRACKET_LIMIT")
        if not self._samples:
            return PoseInterpolationResult(False, None, "ODOM_AT_STAMP_UNAVAILABLE")
        if stamp_s < self._samples[0].stamp_s or stamp_s > self._samples[-1].stamp_s:
            return PoseInterpolationResult(False, None, "ODOM_AT_STAMP_UNAVAILABLE")

        stamps = [sample.stamp_s for sample in self._samples]
        upper_index = bisect_left(stamps, stamp_s)
        if upper_index < len(self._samples) and self._samples[upper_index].stamp_s == stamp_s:
            sample = self._samples[upper_index]
            return PoseInterpolationResult(
                True,
                sample.pose,
                None,
                lower_stamp_s=sample.stamp_s,
                upper_stamp_s=sample.stamp_s,
            )

        lower = self._samples[upper_index - 1]
        upper = self._samples[upper_index]
        bracket_s = upper.stamp_s - lower.stamp_s
        bracket_epsilon_s = _time_comparison_epsilon_s(
            lower.stamp_s,
            upper.stamp_s,
            lower.stamp_s + max_bracket_s,
        )
        if bracket_s > max_bracket_s + bracket_epsilon_s:
            return PoseInterpolationResult(
                False,
                None,
                "ODOM_BRACKET_TOO_WIDE",
                lower_stamp_s=lower.stamp_s,
                upper_stamp_s=upper.stamp_s,
            )

        fraction = (stamp_s - lower.stamp_s) / bracket_s
        if not math.isfinite(fraction):
            return PoseInterpolationResult(
                False,
                None,
                "NONFINITE_INTERPOLATION",
                lower_stamp_s=lower.stamp_s,
                upper_stamp_s=upper.stamp_s,
            )
        x = _finite_lerp(lower.pose.x, upper.pose.x, fraction)
        y = _finite_lerp(lower.pose.y, upper.pose.y, fraction)
        lower_yaw = normalize_angle(lower.pose.yaw)
        upper_yaw = normalize_angle(upper.pose.yaw)
        yaw_delta = normalize_angle(upper_yaw - lower_yaw)
        yaw = normalize_angle(lower_yaw + fraction * yaw_delta)
        if x is None or y is None or not math.isfinite(yaw):
            return PoseInterpolationResult(
                False,
                None,
                "NONFINITE_INTERPOLATION",
                lower_stamp_s=lower.stamp_s,
                upper_stamp_s=upper.stamp_s,
            )
        return PoseInterpolationResult(
            True,
            Pose2D(x=x, y=y, yaw=yaw),
            None,
            lower_stamp_s=lower.stamp_s,
            upper_stamp_s=upper.stamp_s,
        )

    def _append_result(self, accepted: bool, reason: str | None) -> HistoryAppendResult:
        return HistoryAppendResult(
            accepted=accepted,
            reason=reason,
            sample_count=len(self._samples),
        )


class CorrectionGateState(IntEnum):
    UNINITIALIZED = 0
    LOCKED = 1
    SUSPECT = 2
    REACQUIRING = 3
    DEGRADED = 4


class CorrectionGateKind(Enum):
    YAW = "yaw"
    TRANSLATION = "translation"


class PrerequisiteFailureKind(Enum):
    HEADING_QUALITY_UNAVAILABLE = "HEADING_QUALITY_UNAVAILABLE"
    FIX_QUALITY_UNAVAILABLE = "FIX_QUALITY_UNAVAILABLE"
    ODOM_AT_STAMP_UNAVAILABLE = "ODOM_AT_STAMP_UNAVAILABLE"
    STALE_INPUT = "STALE_INPUT"
    NON_FIXED_INPUT = "NON_FIXED_INPUT"
    MALFORMED_INPUT = "MALFORMED_INPUT"


class CorrectionReleaseMode(Enum):
    NORMAL = "NORMAL"
    CORRECTION_BACKLOG = "CORRECTION_BACKLOG"
    FAULT_HOLD = "FAULT_HOLD"
    LOCAL_ODOM_STALE = "LOCAL_ODOM_STALE"


class CorrectionReleaseReason(Enum):
    BOOTSTRAP = "BOOTSTRAP"
    NONPOSITIVE_DT = "NONPOSITIVE_DT"
    DUPLICATE_LOCAL_ODOM = "DUPLICATE_LOCAL_ODOM"
    LOCAL_ODOM_STALE = "LOCAL_ODOM_STALE"
    INVALID_PROCESS_TIME = "INVALID_PROCESS_TIME"
    MOVING_BACKLOG_HOLD = "MOVING_BACKLOG_HOLD"
    STOP_CONFIRMATION_PENDING = "STOP_CONFIRMATION_PENDING"
    RELEASING_BACKLOG = "RELEASING_BACKLOG"
    FAULT_LATCHED = "FAULT_LATCHED"


_QUALITY_UNAVAILABLE_FAILURES = frozenset(
    {
        PrerequisiteFailureKind.HEADING_QUALITY_UNAVAILABLE,
        PrerequisiteFailureKind.FIX_QUALITY_UNAVAILABLE,
    }
)
_IMMEDIATE_DEGRADE_FAILURES = frozenset(
    {
        PrerequisiteFailureKind.STALE_INPUT,
        PrerequisiteFailureKind.NON_FIXED_INPUT,
        PrerequisiteFailureKind.MALFORMED_INPUT,
    }
)


GateValue = Union[float, tuple[float, float]]


@dataclass(frozen=True)
class CorrectionGateResult:
    processed: bool
    accepted: bool
    state: CorrectionGateState
    reason: str | None
    target: GateValue | None
    innovation: float | None
    candidate_count: int
    candidate_span_s: float
    consecutive_failures: int


@dataclass(frozen=True)
class CorrectionGateSnapshot:
    state: CorrectionGateState
    target: GateValue | None
    candidate_stamps: tuple[float, ...]
    candidate_values: tuple[GateValue, ...]
    last_observation_stamp_s: float | None
    last_processable_time_s: float | None
    consecutive_failures: int


@dataclass(frozen=True)
class _GateCandidate:
    stamp_s: float
    value: GateValue


class CorrectionGate:
    def __init__(
        self,
        kind: CorrectionGateKind | str,
        *,
        locked_threshold: float | None = None,
        recovery_threshold: float | None = None,
        min_candidates: int = 5,
        min_span_s: float = 0.30,
        max_candidates: int = 20,
        max_consecutive_failures: int = 5,
        processable_timeout_s: float = 1.0,
    ) -> None:
        self.kind = CorrectionGateKind(kind)
        default_locked = math.radians(15.0) if self.kind is CorrectionGateKind.YAW else 1.0
        default_recovery = math.radians(5.0) if self.kind is CorrectionGateKind.YAW else 0.30
        self.locked_threshold = default_locked if locked_threshold is None else locked_threshold
        self.recovery_threshold = (
            default_recovery if recovery_threshold is None else recovery_threshold
        )
        self.min_candidates = min_candidates
        self.min_span_s = min_span_s
        self.max_candidates = max_candidates
        self.max_consecutive_failures = max_consecutive_failures
        self.processable_timeout_s = processable_timeout_s
        self._validate_configuration()

        self.state = CorrectionGateState.UNINITIALIZED
        self._target: GateValue | None = None
        self._candidates: list[_GateCandidate] = []
        self._last_observation_stamp_s: float | None = None
        self._last_processable_time_s: float | None = None
        self._unavailable_since_s: float | None = None
        self._consecutive_failures = 0

    @classmethod
    def yaw(cls, **kwargs) -> CorrectionGate:
        return cls(CorrectionGateKind.YAW, **kwargs)

    @classmethod
    def translation(cls, **kwargs) -> CorrectionGate:
        return cls(CorrectionGateKind.TRANSLATION, **kwargs)

    @property
    def target(self) -> GateValue | None:
        return self._target

    @property
    def last_trusted_target(self) -> GateValue | None:
        return self._target

    @property
    def candidate_values(self) -> tuple[GateValue, ...]:
        return tuple(candidate.value for candidate in self._candidates)

    @property
    def candidate_stamps(self) -> tuple[float, ...]:
        return tuple(candidate.stamp_s for candidate in self._candidates)

    def snapshot(self) -> CorrectionGateSnapshot:
        return CorrectionGateSnapshot(
            state=self.state,
            target=self._target,
            candidate_stamps=self.candidate_stamps,
            candidate_values=self.candidate_values,
            last_observation_stamp_s=self._last_observation_stamp_s,
            last_processable_time_s=self._last_processable_time_s,
            consecutive_failures=self._consecutive_failures,
        )

    def observe(
        self,
        stamp_s: float,
        value: GateValue,
        *,
        now_s: float,
    ) -> CorrectionGateResult:
        if not math.isfinite(stamp_s) or stamp_s <= 0.0:
            return self._result(False, False, "INVALID_STAMP")
        stamp_reason = self._stamp_order_reason(stamp_s)
        if stamp_reason is not None:
            return self._result(False, False, stamp_reason)
        normalized_value = self._validated_value(value)
        if normalized_value is None:
            return self._result(False, False, "NONFINITE_CANDIDATE")

        if not math.isfinite(now_s):
            return self._result(False, False, "INVALID_PROCESS_TIME")

        self._last_observation_stamp_s = stamp_s
        self._last_processable_time_s = now_s
        self._unavailable_since_s = None
        self._consecutive_failures = 0

        if self.state is CorrectionGateState.LOCKED:
            innovation = self._distance(self._target, normalized_value)
            if innovation <= self.locked_threshold:
                self._target = normalized_value
                return self._result(True, True, None, innovation=innovation)

            self.state = CorrectionGateState.SUSPECT
            self._candidates.clear()
            return self._result(
                True,
                False,
                "INNOVATION_REJECTED",
                innovation=innovation,
            )

        candidate = _GateCandidate(stamp_s=stamp_s, value=normalized_value)
        replaced = False
        if self.state is not CorrectionGateState.REACQUIRING or not self._candidates:
            self._candidates = [candidate]
        elif self._candidates_consistent([*self._candidates, candidate]):
            self._candidates.append(candidate)
            if len(self._candidates) > self.max_candidates:
                del self._candidates[: len(self._candidates) - self.max_candidates]
        else:
            self._candidates = [candidate]
            replaced = True

        self.state = CorrectionGateState.REACQUIRING
        span_epsilon_s = _time_comparison_epsilon_s(
            self._candidates[0].stamp_s,
            self._candidates[-1].stamp_s,
            self._candidates[0].stamp_s + self.min_span_s,
        )
        if (
            len(self._candidates) >= self.min_candidates
            and self._candidate_span_s() + span_epsilon_s >= self.min_span_s
        ):
            target = self._candidate_mean()
            if target is None:
                self._candidates.clear()
                return self._result(True, False, "NONFINITE_TARGET")
            self._target = target
            self.state = CorrectionGateState.LOCKED
            return self._result(True, True, "RECOVERY_LOCKED")

        reason = "RECOVERY_WINDOW_REPLACED" if replaced else "RECOVERY_PENDING"
        return self._result(True, False, reason)

    def odom_timeout(self, stamp_s: float, *, now_s: float) -> CorrectionGateResult:
        if not math.isfinite(stamp_s) or stamp_s <= 0.0:
            return self._result(False, False, "INVALID_STAMP")
        if not math.isfinite(now_s):
            return self._result(False, False, "INVALID_PROCESS_TIME")
        stamp_reason = self._stamp_order_reason(stamp_s)
        if stamp_reason is not None:
            return self._result(False, False, stamp_reason)
        self._last_observation_stamp_s = stamp_s
        return self.prerequisite_failure(
            now_s=now_s,
            kind=PrerequisiteFailureKind.ODOM_AT_STAMP_UNAVAILABLE,
        )

    def prerequisite_failure(
        self,
        *,
        now_s: float,
        kind: PrerequisiteFailureKind,
    ) -> CorrectionGateResult:
        if not math.isfinite(now_s):
            return self._result(False, False, "INVALID_PROCESS_TIME")
        if not isinstance(kind, PrerequisiteFailureKind):
            raise TypeError("kind must be a PrerequisiteFailureKind")

        self._consecutive_failures += 1
        if self._unavailable_since_s is None:
            self._unavailable_since_s = (
                self._last_processable_time_s
                if self._last_processable_time_s is not None
                else now_s
            )
        if kind in _IMMEDIATE_DEGRADE_FAILURES:
            self.state = CorrectionGateState.DEGRADED
            self._candidates.clear()
            return self._result(True, False, kind.value)

        if kind is PrerequisiteFailureKind.ODOM_AT_STAMP_UNAVAILABLE:
            if self.state is CorrectionGateState.LOCKED:
                self.state = CorrectionGateState.SUSPECT
                self._candidates.clear()
        elif kind not in _QUALITY_UNAVAILABLE_FAILURES:
            raise AssertionError(f"unhandled prerequisite failure kind: {kind.value}")

        timeout_epsilon_s = _time_comparison_epsilon_s(
            self._unavailable_since_s,
            now_s,
            self._unavailable_since_s + self.processable_timeout_s,
        )
        timed_out = (
            now_s - self._unavailable_since_s + timeout_epsilon_s
            >= self.processable_timeout_s
        )
        if self._consecutive_failures >= self.max_consecutive_failures or timed_out:
            self.state = CorrectionGateState.DEGRADED
            self._candidates.clear()
        return self._result(True, False, kind.value)

    def check_timeout(self, *, now_s: float) -> CorrectionGateResult:
        if not math.isfinite(now_s):
            return self._result(False, False, "INVALID_PROCESS_TIME")
        reference_s = self._last_processable_time_s
        if reference_s is None:
            reference_s = self._unavailable_since_s
        timeout_epsilon_s = (
            0.0
            if reference_s is None
            else _time_comparison_epsilon_s(
                reference_s,
                now_s,
                reference_s + self.processable_timeout_s,
            )
        )
        if (
            reference_s is None
            or now_s - reference_s + timeout_epsilon_s < self.processable_timeout_s
        ):
            return self._result(False, False, None)

        self.state = CorrectionGateState.DEGRADED
        self._candidates.clear()
        return self._result(True, False, "NO_PROCESSABLE_OBSERVATION")

    def _validate_configuration(self) -> None:
        for name, value in (
            ("locked_threshold", self.locked_threshold),
            ("recovery_threshold", self.recovery_threshold),
            ("min_span_s", self.min_span_s),
            ("processable_timeout_s", self.processable_timeout_s),
        ):
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and nonnegative")
        for name, value in (
            ("min_candidates", self.min_candidates),
            ("max_candidates", self.max_candidates),
            ("max_consecutive_failures", self.max_consecutive_failures),
        ):
            if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
                raise ValueError(f"{name} must be a positive integer")
        if self.max_candidates < self.min_candidates:
            raise ValueError("max_candidates must be at least min_candidates")

    def _validated_value(self, value: GateValue) -> GateValue | None:
        if self.kind is CorrectionGateKind.YAW:
            if not isinstance(value, (int, float)) or not math.isfinite(value):
                return None
            return normalize_angle(float(value))

        if not isinstance(value, (tuple, list)) or len(value) != 2:
            return None
        x, y = value
        if not isinstance(x, (int, float)) or not isinstance(y, (int, float)):
            return None
        if not math.isfinite(x) or not math.isfinite(y):
            return None
        return (float(x), float(y))

    def _stamp_order_reason(self, stamp_s: float) -> str | None:
        if self._last_observation_stamp_s is None:
            return None
        if stamp_s == self._last_observation_stamp_s:
            return "DUPLICATE_STAMP"
        if stamp_s < self._last_observation_stamp_s:
            return "REGRESSING_STAMP"
        return None

    def _distance(self, first: GateValue | None, second: GateValue) -> float:
        if first is None:
            return math.inf
        if self.kind is CorrectionGateKind.YAW:
            return abs(normalize_angle(float(second) - float(first)))
        first_xy = first
        second_xy = second
        return math.hypot(second_xy[0] - first_xy[0], second_xy[1] - first_xy[1])

    def _candidates_consistent(self, candidates: list[_GateCandidate]) -> bool:
        if self.kind is CorrectionGateKind.YAW:
            spread = self._circular_spread(
                [float(candidate.value) for candidate in candidates]
            )
            return spread <= self.recovery_threshold

        values = [candidate.value for candidate in candidates]
        diameter = max(
            (
                math.hypot(second[0] - first[0], second[1] - first[1])
                for index, first in enumerate(values)
                for second in values[index + 1 :]
            ),
            default=0.0,
        )
        return diameter <= self.recovery_threshold

    @staticmethod
    def _circular_spread(values: list[float]) -> float:
        if len(values) < 2:
            return 0.0
        wrapped = sorted(value % math.tau for value in values)
        gaps = [
            wrapped[index + 1] - wrapped[index]
            for index in range(len(wrapped) - 1)
        ]
        gaps.append(wrapped[0] + math.tau - wrapped[-1])
        return math.tau - max(gaps)

    def _candidate_mean(self) -> GateValue | None:
        if self.kind is CorrectionGateKind.YAW:
            sin_sum = sum(math.sin(float(candidate.value)) for candidate in self._candidates)
            cos_sum = sum(math.cos(float(candidate.value)) for candidate in self._candidates)
            return math.atan2(sin_sum, cos_sum)

        x = _finite_mean([candidate.value[0] for candidate in self._candidates])
        y = _finite_mean([candidate.value[1] for candidate in self._candidates])
        return None if x is None or y is None else (x, y)

    def _candidate_span_s(self) -> float:
        if len(self._candidates) < 2:
            return 0.0
        return self._candidates[-1].stamp_s - self._candidates[0].stamp_s

    def _result(
        self,
        processed: bool,
        accepted: bool,
        reason: str | None,
        *,
        innovation: float | None = None,
    ) -> CorrectionGateResult:
        return CorrectionGateResult(
            processed=processed,
            accepted=accepted,
            state=self.state,
            reason=reason,
            target=self._target,
            innovation=innovation,
            candidate_count=len(self._candidates),
            candidate_span_s=self._candidate_span_s(),
            consecutive_failures=self._consecutive_failures,
        )


@dataclass(frozen=True)
class Alignment2D:
    theta: float
    tx: float
    ty: float


@dataclass(frozen=True)
class LimitedPoseStep:
    pose: Pose2D
    translation_step_m: float
    yaw_step_rad: float
    limited: bool


@dataclass(frozen=True)
class AuthorityInputSummary:
    ok: bool
    mode: str
    reason: str | None


@dataclass(frozen=True)
class CorrectionReleaseResult:
    output_map_odom: Pose2D
    output_map_base: Pose2D
    target_map_base: Pose2D
    mode: CorrectionReleaseMode
    reason: CorrectionReleaseReason | None
    motion_allowed: bool
    dt_s: float
    translation_gap_m: float
    yaw_gap_rad: float
    translation_step_m: float
    yaw_step_rad: float
    stopped_duration_s: float = 0.0
    recovery_duration_s: float = 0.0


AlignmentTuple = tuple[float, float, float, bool]


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def invert_pose(pose: Pose2D) -> Pose2D:
    cos_yaw = math.cos(pose.yaw)
    sin_yaw = math.sin(pose.yaw)
    return Pose2D(
        x=-(cos_yaw * pose.x + sin_yaw * pose.y),
        y=sin_yaw * pose.x - cos_yaw * pose.y,
        yaw=normalize_angle(-pose.yaw),
    )


def compose_pose(parent_child: Pose2D, child_grandchild: Pose2D) -> Pose2D:
    cos_yaw = math.cos(parent_child.yaw)
    sin_yaw = math.sin(parent_child.yaw)
    return Pose2D(
        x=parent_child.x + cos_yaw * child_grandchild.x - sin_yaw * child_grandchild.y,
        y=parent_child.y + sin_yaw * child_grandchild.x + cos_yaw * child_grandchild.y,
        yaw=normalize_angle(parent_child.yaw + child_grandchild.yaw),
    )


def compute_map_to_odom(map_base: Pose2D, odom_base: Pose2D) -> Pose2D:
    return compose_pose(map_base, invert_pose(odom_base))


class CorrectionReleaseState:
    def __init__(
        self,
        *,
        max_translation_rate_mps: float = 0.20,
        max_yaw_rate_radps: float = math.radians(2.0),
        max_dt_s: float = 0.10,
        max_lio_age_s: float = 0.20,
        backlog_translation_m: float = 0.50,
        backlog_yaw_rad: float = math.radians(5.0),
        fault_translation_m: float = 2.0,
        fault_yaw_rad: float = math.radians(20.0),
        stopped_linear_rate_mps: float = 0.05,
        stopped_yaw_rate_radps: float = math.radians(2.0),
        stopped_confirmation_s: float = 1.0,
        recovery_translation_m: float = 0.15,
        recovery_yaw_rad: float = math.radians(2.0),
        recovery_confirmation_s: float = 1.0,
    ) -> None:
        self.max_translation_rate_mps = max_translation_rate_mps
        self.max_yaw_rate_radps = max_yaw_rate_radps
        self.max_dt_s = max_dt_s
        self.max_lio_age_s = max_lio_age_s
        self.backlog_translation_m = backlog_translation_m
        self.backlog_yaw_rad = backlog_yaw_rad
        self.fault_translation_m = fault_translation_m
        self.fault_yaw_rad = fault_yaw_rad
        self.stopped_linear_rate_mps = stopped_linear_rate_mps
        self.stopped_yaw_rate_radps = stopped_yaw_rate_radps
        self.stopped_confirmation_s = stopped_confirmation_s
        self.recovery_translation_m = recovery_translation_m
        self.recovery_yaw_rad = recovery_yaw_rad
        self.recovery_confirmation_s = recovery_confirmation_s
        self._validate_configuration()
        self._last_now_s: float | None = None
        self._last_lio_stamp_s: float | None = None
        self._backlog_active = False
        self._fault_latched = False
        self._stopped_since_s: float | None = None
        self._recovery_since_s: float | None = None

    def update(
        self,
        *,
        previous_output_map_odom: Pose2D,
        target_map_odom: Pose2D,
        local_pose: Pose2D,
        now_s: float,
        lio_stamp_s: float,
        lio_age_s: float,
        local_linear_rate_mps: float,
        local_yaw_rate_radps: float,
        gates_locked: bool,
    ) -> CorrectionReleaseResult:
        previous_map_base = compose_pose(previous_output_map_odom, local_pose)
        target_map_base = compose_pose(target_map_odom, local_pose)
        gap_m = math.hypot(
            target_map_base.x - previous_map_base.x,
            target_map_base.y - previous_map_base.y,
        )
        gap_yaw_rad = abs(normalize_angle(target_map_base.yaw - previous_map_base.yaw))

        if gap_m > self.fault_translation_m or gap_yaw_rad > self.fault_yaw_rad:
            self._fault_latched = True
        elif gap_m >= self.backlog_translation_m or gap_yaw_rad >= self.backlog_yaw_rad:
            self._backlog_active = True

        if self._fault_latched:
            self._stopped_since_s = None
            self._recovery_since_s = None
            return self._frozen_result(
                previous_output_map_odom,
                previous_map_base,
                target_map_base,
                CorrectionReleaseReason.FAULT_LATCHED,
                gap_m,
                gap_yaw_rad,
                mode=CorrectionReleaseMode.FAULT_HOLD,
            )

        if not math.isfinite(now_s):
            self._stopped_since_s = None
            self._recovery_since_s = None
            return self._frozen_result(
                previous_output_map_odom,
                previous_map_base,
                target_map_base,
                CorrectionReleaseReason.INVALID_PROCESS_TIME,
                gap_m,
                gap_yaw_rad,
                mode=CorrectionReleaseMode.LOCAL_ODOM_STALE,
            )

        lio_invalid = not all(
            math.isfinite(value)
            for value in (
                lio_stamp_s,
                lio_age_s,
                local_linear_rate_mps,
                local_yaw_rate_radps,
            )
        ) or lio_stamp_s <= 0.0 or lio_age_s < 0.0
        age_epsilon_s = _time_comparison_epsilon_s(
            lio_stamp_s,
            lio_stamp_s + self.max_lio_age_s,
            lio_age_s,
        )
        if (
            lio_invalid
            or (
                self._last_lio_stamp_s is not None
                and lio_stamp_s < self._last_lio_stamp_s
            )
            or lio_age_s > self.max_lio_age_s + age_epsilon_s
        ):
            self._stopped_since_s = None
            self._recovery_since_s = None
            return self._frozen_result(
                previous_output_map_odom,
                previous_map_base,
                target_map_base,
                CorrectionReleaseReason.LOCAL_ODOM_STALE,
                gap_m,
                gap_yaw_rad,
                mode=CorrectionReleaseMode.LOCAL_ODOM_STALE,
            )

        if self._last_lio_stamp_s is not None and lio_stamp_s == self._last_lio_stamp_s:
            return self._frozen_result(
                previous_output_map_odom,
                previous_map_base,
                target_map_base,
                CorrectionReleaseReason.DUPLICATE_LOCAL_ODOM,
                gap_m,
                gap_yaw_rad,
                mode=self._active_mode(),
            )

        self._last_lio_stamp_s = lio_stamp_s

        if self._last_now_s is None:
            self._last_now_s = now_s
            return self._frozen_result(
                previous_output_map_odom,
                previous_map_base,
                target_map_base,
                CorrectionReleaseReason.BOOTSTRAP,
                gap_m,
                gap_yaw_rad,
                mode=self._active_mode(),
            )

        elapsed_s = now_s - self._last_now_s
        if elapsed_s <= 0.0:
            return self._frozen_result(
                previous_output_map_odom,
                previous_map_base,
                target_map_base,
                CorrectionReleaseReason.NONPOSITIVE_DT,
                gap_m,
                gap_yaw_rad,
                mode=(
                    CorrectionReleaseMode.CORRECTION_BACKLOG
                    if self._backlog_active
                    else CorrectionReleaseMode.NORMAL
                ),
            )

        self._last_now_s = now_s
        dt_s = min(elapsed_s, self.max_dt_s)
        if self._backlog_active:
            stopped = (
                abs(local_linear_rate_mps) < self.stopped_linear_rate_mps
                and abs(local_yaw_rate_radps) < self.stopped_yaw_rate_radps
            )
            if not stopped:
                self._stopped_since_s = None
                self._recovery_since_s = None
                return self._frozen_result(
                    previous_output_map_odom,
                    previous_map_base,
                    target_map_base,
                    CorrectionReleaseReason.MOVING_BACKLOG_HOLD,
                    gap_m,
                    gap_yaw_rad,
                    mode=CorrectionReleaseMode.CORRECTION_BACKLOG,
                )

            if self._stopped_since_s is None:
                self._stopped_since_s = now_s
            stopped_duration_s = max(0.0, now_s - self._stopped_since_s)
            stopped_epsilon_s = _time_comparison_epsilon_s(
                self._stopped_since_s,
                now_s,
                self._stopped_since_s + self.stopped_confirmation_s,
            )
            if (
                stopped_duration_s + stopped_epsilon_s
                < self.stopped_confirmation_s
            ):
                self._recovery_since_s = None
                return self._frozen_result(
                    previous_output_map_odom,
                    previous_map_base,
                    target_map_base,
                    CorrectionReleaseReason.STOP_CONFIRMATION_PENDING,
                    gap_m,
                    gap_yaw_rad,
                    mode=CorrectionReleaseMode.CORRECTION_BACKLOG,
                    stopped_duration_s=stopped_duration_s,
                )
        else:
            stopped_duration_s = 0.0

        limited = limit_pose_step(
            previous_map_base,
            target_map_base,
            max_translation_step_m=self.max_translation_rate_mps * dt_s,
            max_yaw_step_rad=self.max_yaw_rate_radps * dt_s,
        )
        output_map_odom = compute_map_to_odom(limited.pose, local_pose)
        output_gap_m = math.hypot(
            target_map_base.x - limited.pose.x,
            target_map_base.y - limited.pose.y,
        )
        output_gap_yaw_rad = abs(
            normalize_angle(target_map_base.yaw - limited.pose.yaw)
        )
        recovery_duration_s = 0.0
        if self._backlog_active:
            recovery_ready = (
                output_gap_m < self.recovery_translation_m
                and output_gap_yaw_rad < self.recovery_yaw_rad
                and gates_locked
            )
            if not recovery_ready:
                self._recovery_since_s = None
            else:
                if self._recovery_since_s is None:
                    self._recovery_since_s = now_s
                recovery_duration_s = max(0.0, now_s - self._recovery_since_s)
                recovery_epsilon_s = _time_comparison_epsilon_s(
                    self._recovery_since_s,
                    now_s,
                    self._recovery_since_s + self.recovery_confirmation_s,
                )
                if (
                    recovery_duration_s + recovery_epsilon_s
                    >= self.recovery_confirmation_s
                ):
                    self._backlog_active = False
                    self._stopped_since_s = None
                    self._recovery_since_s = None

        return CorrectionReleaseResult(
            output_map_odom=output_map_odom,
            output_map_base=limited.pose,
            target_map_base=target_map_base,
            mode=(
                CorrectionReleaseMode.CORRECTION_BACKLOG
                if self._backlog_active
                else CorrectionReleaseMode.NORMAL
            ),
            reason=(
                CorrectionReleaseReason.RELEASING_BACKLOG
                if self._backlog_active
                else None
            ),
            motion_allowed=gates_locked and not self._backlog_active,
            dt_s=dt_s,
            translation_gap_m=output_gap_m,
            yaw_gap_rad=output_gap_yaw_rad,
            translation_step_m=limited.translation_step_m,
            yaw_step_rad=limited.yaw_step_rad,
            stopped_duration_s=stopped_duration_s,
            recovery_duration_s=recovery_duration_s,
        )

    def _active_mode(self) -> CorrectionReleaseMode:
        if self._fault_latched:
            return CorrectionReleaseMode.FAULT_HOLD
        if self._backlog_active:
            return CorrectionReleaseMode.CORRECTION_BACKLOG
        return CorrectionReleaseMode.NORMAL

    def _validate_configuration(self) -> None:
        positive_values = (
            ("max_translation_rate_mps", self.max_translation_rate_mps),
            ("max_yaw_rate_radps", self.max_yaw_rate_radps),
            ("max_dt_s", self.max_dt_s),
            ("max_lio_age_s", self.max_lio_age_s),
            ("backlog_translation_m", self.backlog_translation_m),
            ("backlog_yaw_rad", self.backlog_yaw_rad),
            ("fault_translation_m", self.fault_translation_m),
            ("fault_yaw_rad", self.fault_yaw_rad),
            ("stopped_linear_rate_mps", self.stopped_linear_rate_mps),
            ("stopped_yaw_rate_radps", self.stopped_yaw_rate_radps),
            ("stopped_confirmation_s", self.stopped_confirmation_s),
            ("recovery_translation_m", self.recovery_translation_m),
            ("recovery_yaw_rad", self.recovery_yaw_rad),
            ("recovery_confirmation_s", self.recovery_confirmation_s),
        )
        for name, value in positive_values:
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if not (
            self.recovery_translation_m
            < self.backlog_translation_m
            <= self.fault_translation_m
        ):
            raise ValueError("translation thresholds must increase through fault")
        if not self.recovery_yaw_rad < self.backlog_yaw_rad <= self.fault_yaw_rad:
            raise ValueError("yaw thresholds must increase through fault")

    @staticmethod
    def _frozen_result(
        output_map_odom: Pose2D,
        output_map_base: Pose2D,
        target_map_base: Pose2D,
        reason: CorrectionReleaseReason,
        gap_m: float,
        gap_yaw_rad: float,
        *,
        mode: CorrectionReleaseMode = CorrectionReleaseMode.NORMAL,
        stopped_duration_s: float = 0.0,
    ) -> CorrectionReleaseResult:
        return CorrectionReleaseResult(
            output_map_odom=output_map_odom,
            output_map_base=output_map_base,
            target_map_base=target_map_base,
            mode=mode,
            reason=reason,
            motion_allowed=False,
            dt_s=0.0,
            translation_gap_m=gap_m,
            yaw_gap_rad=gap_yaw_rad,
            translation_step_m=0.0,
            yaw_step_rad=0.0,
            stopped_duration_s=stopped_duration_s,
        )


def compute_pose_delta(previous: Pose2D | None, current: Pose2D) -> tuple[float, float]:
    if previous is None:
        return 0.0, 0.0
    return (
        math.hypot(current.x - previous.x, current.y - previous.y),
        normalize_angle(current.yaw - previous.yaw),
    )


def compute_authority_target_delta(
    *,
    previous_map_base: Pose2D | None,
    current_map_base: Pose2D,
    previous_map_odom: Pose2D | None,
    current_map_odom: Pose2D,
) -> tuple[float, float]:
    del previous_map_odom, current_map_odom
    return compute_pose_delta(previous_map_base, current_map_base)


def compute_rtk_map_base(
    *,
    enu_x: float,
    enu_y: float,
    heading_enu_yaw: float,
    alignment_theta: float,
    alignment_tx: float,
    alignment_ty: float,
) -> Pose2D:
    cos_theta = math.cos(alignment_theta)
    sin_theta = math.sin(alignment_theta)
    return Pose2D(
        x=alignment_tx + cos_theta * enu_x - sin_theta * enu_y,
        y=alignment_ty + sin_theta * enu_x + cos_theta * enu_y,
        yaw=normalize_angle(alignment_theta + heading_enu_yaw),
    )


def compute_bootstrap_alignment_from_current_pose(
    *,
    odom_base: Pose2D,
    enu_x: float,
    enu_y: float,
    heading_enu_yaw: float,
) -> Alignment2D:
    theta = normalize_angle(odom_base.yaw - heading_enu_yaw)
    cos_theta = math.cos(theta)
    sin_theta = math.sin(theta)
    return Alignment2D(
        theta=theta,
        tx=odom_base.x - (cos_theta * enu_x - sin_theta * enu_y),
        ty=odom_base.y - (sin_theta * enu_x + cos_theta * enu_y),
    )


def select_authority_alignment(
    *,
    latest_alignment: AlignmentTuple | None,
    external_alignment_valid: bool,
    bootstrap_alignment: AlignmentTuple | None,
) -> tuple[AlignmentTuple | None, bool]:
    if external_alignment_valid and latest_alignment is not None and latest_alignment[3]:
        return latest_alignment, True
    if bootstrap_alignment is not None and bootstrap_alignment[3]:
        return bootstrap_alignment, False
    return None, False


def should_publish_bootstrap_without_fixed(
    *,
    rtk_fixed_ok: bool,
    using_external_alignment: bool,
    bootstrap_alignment_valid: bool,
) -> bool:
    return (
        not rtk_fixed_ok
        and not using_external_alignment
        and bootstrap_alignment_valid
    )


def limit_pose_step(
    previous: Pose2D,
    target: Pose2D,
    *,
    max_translation_step_m: float,
    max_yaw_step_rad: float,
) -> LimitedPoseStep:
    dx = target.x - previous.x
    dy = target.y - previous.y
    distance = math.hypot(dx, dy)
    limited = False
    if distance > max_translation_step_m and distance > 1e-9:
        scale = max_translation_step_m / distance
        dx *= scale
        dy *= scale
        distance = max_translation_step_m
        limited = True

    yaw_delta = normalize_angle(target.yaw - previous.yaw)
    if abs(yaw_delta) > max_yaw_step_rad:
        yaw_delta = math.copysign(max_yaw_step_rad, yaw_delta)
        limited = True

    return LimitedPoseStep(
        pose=Pose2D(
            x=previous.x + dx,
            y=previous.y + dy,
            yaw=normalize_angle(previous.yaw + yaw_delta),
        ),
        translation_step_m=distance,
        yaw_step_rad=yaw_delta,
        limited=limited,
    )


def limit_map_to_odom_step_for_base(
    previous: Pose2D,
    target: Pose2D,
    *,
    odom_base: Pose2D,
    max_translation_step_m: float,
    max_yaw_step_rad: float,
    max_base_yaw_step_m: float,
) -> LimitedPoseStep:
    effective_yaw_step_rad = max_yaw_step_rad
    odom_radius_m = math.hypot(odom_base.x, odom_base.y)
    if (
        math.isfinite(max_base_yaw_step_m)
        and max_base_yaw_step_m > 0.0
        and math.isfinite(odom_radius_m)
        and odom_radius_m > 1e-9
    ):
        ratio = min(1.0, max_base_yaw_step_m / (2.0 * odom_radius_m))
        base_limited_yaw_step_rad = 2.0 * math.asin(ratio)
        effective_yaw_step_rad = min(max_yaw_step_rad, base_limited_yaw_step_rad)

    return limit_pose_step(
        previous,
        target,
        max_translation_step_m=max_translation_step_m,
        max_yaw_step_rad=effective_yaw_step_rad,
    )


def blend_pose_target(
    previous: Pose2D | None,
    target: Pose2D,
    *,
    alpha: float,
    translation_deadband_m: float,
    yaw_deadband_rad: float,
) -> Pose2D:
    if previous is None:
        return target

    dx = target.x - previous.x
    dy = target.y - previous.y
    distance = math.hypot(dx, dy)
    yaw_delta = normalize_angle(target.yaw - previous.yaw)
    if (
        distance <= max(0.0, translation_deadband_m)
        and abs(yaw_delta) <= max(0.0, yaw_deadband_rad)
    ):
        return previous

    bounded_alpha = min(1.0, max(0.0, alpha))
    return Pose2D(
        x=previous.x + dx * bounded_alpha,
        y=previous.y + dy * bounded_alpha,
        yaw=normalize_angle(previous.yaw + yaw_delta * bounded_alpha),
    )


def summarize_authority_inputs(
    *,
    alignment_valid: bool,
    odom_available: bool,
    fix_age_s: float,
    heading_age_s: float,
    target_jump_m: float,
    target_yaw_jump_rad: float,
    max_fix_age_s: float,
    max_heading_age_s: float,
    max_target_jump_m: float,
    max_target_yaw_jump_rad: float,
    allow_yaw_reacquire: bool = False,
    max_yaw_reacquire_jump_rad: float | None = None,
) -> AuthorityInputSummary:
    reason = None
    if not alignment_valid:
        reason = "NO_ALIGNMENT"
    elif not odom_available:
        reason = "NO_ODOM_TF"
    elif not math.isfinite(fix_age_s) or fix_age_s > max_fix_age_s:
        reason = "STALE_FIX"
    elif not math.isfinite(heading_age_s) or heading_age_s > max_heading_age_s:
        reason = "STALE_HEADING"
    elif not math.isfinite(target_jump_m) or target_jump_m > max_target_jump_m:
        reason = "TARGET_JUMP"
    elif (
        not math.isfinite(target_yaw_jump_rad)
        or abs(target_yaw_jump_rad) > max_target_yaw_jump_rad
    ):
        if (
            allow_yaw_reacquire
            and max_yaw_reacquire_jump_rad is not None
            and math.isfinite(target_yaw_jump_rad)
            and abs(target_yaw_jump_rad) <= max_yaw_reacquire_jump_rad
        ):
            return AuthorityInputSummary(
                ok=True,
                mode="RTK_AUTHORITATIVE",
                reason="YAW_REACQUIRE",
            )
        reason = "TARGET_YAW_JUMP"

    if reason is not None:
        return AuthorityInputSummary(ok=False, mode="RTK_DEGRADED", reason=reason)
    return AuthorityInputSummary(ok=True, mode="RTK_AUTHORITATIVE", reason=None)
