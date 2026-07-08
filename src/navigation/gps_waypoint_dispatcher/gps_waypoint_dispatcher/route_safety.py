from __future__ import annotations

import math
from dataclasses import dataclass


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
