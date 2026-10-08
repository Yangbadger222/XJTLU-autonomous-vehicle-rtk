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
from .firmware_command import within_firmware_command_envelope


@dataclass(frozen=True)
class TrackerState:
    x: float
    y: float
    yaw: float
    translation_speed: float = 0.
    yaw_rate: float = 0.


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
                motion_mode=first.motion_mode,
            )
    return points[-1]


class TimedTrajectoryTracker:
    """Feedback tracker with explicit speed/yaw-rate limits and no side slip."""

    def __init__(self, limits: VehicleLimits, *, longitudinal_gain: float = 0.8,
                 lateral_gain: float = 1.5, heading_gain: float = 1.0,
                 preview_s: float = 0.0):
        gains = (longitudinal_gain, lateral_gain, heading_gain)
        if not all(math.isfinite(float(gain)) and gain >= 0.0 for gain in gains):
            raise ValueError("tracker gains must be finite and non-negative")
        self.limits = limits
        self.longitudinal_gain = float(longitudinal_gain)
        self.lateral_gain = float(lateral_gain)
        self.heading_gain = float(heading_gain)
        if not math.isfinite(preview_s) or not 0.0 <= preview_s <= 0.25:
            raise ValueError("tracker preview must be finite and within the trajectory validity window")
        self.preview_s = float(preview_s)
        self.last_rejection = ""

    def command(self, trajectory: TimedTrajectory, state: TrackerState, *, now: float,
                expected_map_version: str | None = None,
                footprint: Sequence[tuple[float, float]] | None = None,
                occupied: Callable[[float, float], bool] | None = None,
                resolution: float | None = None,
                occupied_polygon: Callable | None = None) -> TrackerCommand | None:
        if not all(math.isfinite(float(value)) for value in
                   (state.x, state.y, state.yaw,state.translation_speed,state.yaw_rate,now)):
            self.last_rejection = "nonfinite_state"
            return None
        checked = validate_trajectory(trajectory, self.limits, now=now,
                                      expected_map_version=expected_map_version,
                                      footprint=footprint, occupied=occupied,
                                      resolution=resolution, occupied_polygon=occupied_polygon)
        if not checked.valid:
            self.last_rejection = ";".join(dict.fromkeys(checked.reasons))[:250]
            return None
        if footprint and occupied_polygon:
            c,s=math.cos(state.yaw),math.sin(state.yaw)
            actual=[(state.x+c*x-s*y,state.y+s*x+c*y) for x,y in footprint]
            radius=max(math.hypot(x,y) for x,y in footprint)
            # Over the original 0.25 s command timeout any bounded feedback
            # command stays inside this conservative measured-pose envelope.
            # A collision-free nominal path alone does not certify feedback.
            margin=(.25*(self.limits.max_speed_mps+radius*self.limits.max_yaw_rate_rps)+
                    self.limits.max_speed_mps**2/(2*self.limits.max_decel_mps2)+
                    radius*self.limits.max_yaw_rate_rps**2/(2*self.limits.max_yaw_decel_rps2))
            if occupied_polygon(actual,margin):
                self.last_rejection="measured_footprint_or_feedback_sweep_blocked"
                return None
        elapsed = now - trajectory.generated_at
        target = _interpolate(trajectory.points, elapsed + self.preview_s)
        dx = target.x - state.x
        dy = target.y - state.y
        longitudinal_error = math.cos(state.yaw) * dx + math.sin(state.yaw) * dy
        lateral_error = -math.sin(state.yaw) * dx + math.cos(state.yaw) * dy
        heading_error = _wrap(target.yaw - state.yaw)
        requested_v = target.v + self.longitudinal_gain * longitudinal_error
        requested_w = (target.w + self.lateral_gain * lateral_error +
                       self.heading_gain * heading_error)
        rotating=target.motion_mode==1
        if rotating:
            if abs(state.translation_speed)>.001 or abs(state.yaw_rate)>self.limits.max_yaw_rate_rps:
                self.last_rejection="rotation_measured_translation_or_yaw_limit"
                return None
            if math.hypot(dx,dy)>.02:
                self.last_rejection="rotation_pivot_tracking_error"
                return None
            requested_v=0.
            requested_w=target.w+self.heading_gain*heading_error
        if not math.isfinite(requested_v) or not math.isfinite(requested_w):
            self.last_rejection = "nonfinite_feedback"
            return None
        # Saturation hides tracking failure. The caller must stop/replan when
        # feedback requires a command beyond the locked motion envelope.
        if (requested_v < self.limits.min_speed_mps or requested_v > self.limits.max_speed_mps or
                abs(requested_w) > self.limits.max_yaw_rate_rps):
            self.last_rejection = f"feedback_limit:v={requested_v:.6g},w={requested_w:.6g}"
            return None
        if abs(requested_v*requested_w)>self.limits.max_lateral_accel_mps2+1e-9:
            self.last_rejection="feedback_original_guard_turn_product_limit"
            return None
        if not within_firmware_command_envelope(requested_v,requested_w):
            self.last_rejection="feedback_firmware_motor_target_limit"
            return None
        if (not rotating and self.limits.max_curvature_1pm is not None and
                abs(requested_w)>self.limits.max_curvature_1pm*abs(requested_v)+1e-9):
            self.last_rejection="feedback_curvature_limit"
            return None
        self.last_rejection = ""
        return TrackerCommand(requested_v, requested_w, target)
