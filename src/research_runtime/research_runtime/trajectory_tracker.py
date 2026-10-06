"""Small timed-trajectory tracker used before the original command guard.

The tracker consumes a validated odom-frame timed trajectory and measured vehicle
pose. It never owns authority or serial output; a missing state, expired plan or
failed contract returns a stop decision for the safety gate.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Callable, Sequence

from .trajectory import TimedPoint, TimedTrajectory, VehicleLimits, validate_trajectory


@dataclass(frozen=True)
class TrackerState:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class TrackerCommand:
    linear_x: float
    angular_z: float
    target: TimedPoint
    reason: str = "tracked"


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _interpolate(points: tuple[TimedPoint, ...], elapsed: float) -> TimedPoint:
    if elapsed <= points[0].t:
        return points[0]
    for first, second in zip(points, points[1:]):
        if elapsed <= second.t:
            span = second.t - first.t
            if span <= 0.0:
                return second
            ratio = (elapsed - first.t) / span
            yaw_delta = _wrap(second.yaw - first.yaw)
            return TimedPoint(
                t=elapsed,
                x=first.x + ratio * (second.x - first.x),
                y=first.y + ratio * (second.y - first.y),
                yaw=first.yaw + ratio * yaw_delta,
                v=first.v + ratio * (second.v - first.v),
                w=first.w + ratio * (second.w - first.w),
                a=first.a + ratio * (second.a - first.a),
                alpha=first.alpha + ratio * (second.alpha - first.alpha),
                curvature=(second.curvature if ratio >= 1.0 else first.curvature),
            )
    return points[-1]


class TimedTrajectoryTracker:
    """Feedback tracker with explicit speed/yaw-rate limits and no side slip."""

    def __init__(self, limits: VehicleLimits, *, longitudinal_gain: float = 0.8,
                 lateral_gain: float = 1.5, heading_gain: float = 1.0):
        gains = (longitudinal_gain, lateral_gain, heading_gain)
        if not all(math.isfinite(float(gain)) and gain >= 0.0 for gain in gains):
            raise ValueError("tracker gains must be finite and non-negative")
        self.limits = limits
        self.longitudinal_gain = float(longitudinal_gain)
        self.lateral_gain = float(lateral_gain)
        self.heading_gain = float(heading_gain)

    def command(self, trajectory: TimedTrajectory, state: TrackerState, *, now: float,
                expected_map_version: str | None = None,
                footprint: Sequence[tuple[float, float]] | None = None,
                occupied: Callable[[float, float], bool] | None = None,
                resolution: float | None = None) -> TrackerCommand | None:
        if not all(math.isfinite(float(value)) for value in
                   (state.x, state.y, state.yaw, now)):
            return None
        checked = validate_trajectory(trajectory, self.limits, now=now,
                                      expected_map_version=expected_map_version,
                                      footprint=footprint, occupied=occupied,
                                      resolution=resolution)
        if not checked.valid:
            return None
        elapsed = now - trajectory.generated_at
        target = _interpolate(trajectory.points, elapsed)
        dx = target.x - state.x
        dy = target.y - state.y
        longitudinal_error = math.cos(state.yaw) * dx + math.sin(state.yaw) * dy
        lateral_error = -math.sin(state.yaw) * dx + math.cos(state.yaw) * dy
        heading_error = _wrap(target.yaw - state.yaw)
        requested_v = target.v + self.longitudinal_gain * longitudinal_error
        requested_w = (target.w + self.lateral_gain * lateral_error +
                       self.heading_gain * heading_error)
        if not math.isfinite(requested_v) or not math.isfinite(requested_w):
            return None
        requested_v = max(self.limits.min_speed_mps,
                          min(self.limits.max_speed_mps, requested_v))
        requested_w = max(-self.limits.max_yaw_rate_rps,
                          min(self.limits.max_yaw_rate_rps, requested_w))
        return TrackerCommand(requested_v, requested_w, target)
