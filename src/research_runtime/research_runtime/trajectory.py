"""Timed 2-D trajectory contract and UGV feasibility checks.

The checker is deliberately transport independent so the same code is used by
replay, shadow and the ROS adapter. It never clips a bad trajectory: callers
must reject it and request a stop/replan.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Iterable, Optional, Sequence
from .physical_parameter_lock import PHYSICAL_LIMITS, CONTROL_REFERENCE_CONTRACT
from .firmware_command import within_firmware_command_envelope


@dataclass(frozen=True)
class VehicleLimits:
    """The corridor launch effective limits, in SI units.

    The defaults are generated from the locked corridor launch override. Chassis
    footprint and wheel geometry remain explicit inputs and are not guessed.
    """

    max_speed_mps: float = PHYSICAL_LIMITS["max_speed_mps"]
    min_speed_mps: float = PHYSICAL_LIMITS["min_speed_mps"]
    max_yaw_rate_rps: float = PHYSICAL_LIMITS["max_yaw_rate_rps"]
    max_accel_mps2: float = PHYSICAL_LIMITS["max_accel_mps2"]
    max_decel_mps2: float = PHYSICAL_LIMITS["max_decel_mps2"]
    max_yaw_accel_rps2: float = PHYSICAL_LIMITS["max_yaw_accel_rps2"]
    max_yaw_decel_rps2: float = PHYSICAL_LIMITS["max_yaw_decel_rps2"]
    max_lateral_accel_mps2: float = PHYSICAL_LIMITS["max_lateral_accel_mps2"]
    max_curvature_1pm: Optional[float] = None
    max_lateral_speed_mps: float = 0.05
    derivative_tolerance: float = 0.05


@dataclass(frozen=True)
class TimedPoint:
    t: float
    x: float
    y: float
    yaw: float
    v: float
    w: float
    a: float = 0.0
    alpha: float = 0.0
    curvature: Optional[float] = None
    motion_mode: int = 0


@dataclass(frozen=True)
class TimedTrajectory:
    trajectory_id: str
    map_version: str
    frame_id: str
    generated_at: float
    valid_until: float
    points: tuple[TimedPoint, ...]
    source: str = "ego_planner_2d_vehicle_adaptation"
    control_reference_contract: str = CONTROL_REFERENCE_CONTRACT

    @classmethod
    def from_points(cls, trajectory_id: str, map_version: str, frame_id: str,
                    generated_at: float, valid_until: float,
                    points: Iterable[TimedPoint], source: str = "ego_planner_2d_vehicle_adaptation",
                    *, control_reference_contract: str = CONTROL_REFERENCE_CONTRACT):
        return cls(trajectory_id, map_version, frame_id, generated_at, valid_until, tuple(points), source,
                   control_reference_contract)


@dataclass
class ValidationResult:
    valid: bool
    reasons: list[str] = field(default_factory=list)
    checked_points: int = 0
    max_speed: float = 0.0
    max_yaw_rate: float = 0.0
    max_abs_curvature: float = 0.0

    def fail(self, reason: str) -> None:
        self.valid = False
        self.reasons.append(reason)


def _finite(values: Sequence[float]) -> bool:
    return all(math.isfinite(v) for v in values)


def _angle_delta(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


def validate_trajectory(
    trajectory: TimedTrajectory,
    limits: VehicleLimits,
    *,
    now: Optional[float] = None,
    expected_map_version: Optional[str] = None,
    footprint: Optional[Sequence[tuple[float, float]]] = None,
    occupied: Optional[callable] = None,
    resolution: Optional[float] = None,
    occupied_polygon: Optional[callable] = None,
) -> ValidationResult:
    """Validate time, dynamics, non-holonomic motion and footprint sweep.

    ``occupied(x, y)`` must return ``True`` for collision or unknown space;
    using unknown as free is intentionally left to the map adapter and is not
    the default. A footprint without a collision oracle is rejected so a
    point-only path cannot silently become a vehicle command.
    """
    result = ValidationResult(valid=True, checked_points=len(trajectory.points))
    if trajectory.control_reference_contract != CONTROL_REFERENCE_CONTRACT:
        result.fail('control_reference_contract_mismatch')
    if not str(trajectory.trajectory_id):
        result.fail("trajectory_id_missing")
    if not str(trajectory.map_version).strip() or str(trajectory.map_version).strip().upper() == "UNKNOWN":
        result.fail("map_version_unknown")
    if not math.isfinite(float(trajectory.generated_at)):
        result.fail("generated_at_non_finite")
    if not math.isfinite(float(trajectory.valid_until)):
        result.fail("valid_until_non_finite")
    if now is not None and not math.isfinite(float(now)):
        result.fail("now_non_finite")
    if resolution is not None and (not math.isfinite(float(resolution)) or resolution <= 0.0):
        result.fail("invalid_sweep_resolution")
        resolution = None
    if not trajectory.points:
        result.fail("empty_trajectory")
        return result
    if trajectory.frame_id != "odom":
        result.fail(f"unsupported_frame:{trajectory.frame_id}")
    if expected_map_version is not None and trajectory.map_version != expected_map_version:
        result.fail("map_version_mismatch")
    if now is not None and trajectory.valid_until < now:
        result.fail("trajectory_expired")
    if now is not None and trajectory.generated_at > now + 1e-6:
        result.fail("trajectory_from_future")
    if trajectory.valid_until < trajectory.generated_at:
        result.fail("invalid_validity_interval")

    previous: Optional[TimedPoint] = None
    for point in trajectory.points:
        values = (point.t, point.x, point.y, point.yaw, point.v, point.w, point.a, point.alpha)
        if point.curvature is not None:
            values += (point.curvature,)
        if not _finite(values):
            result.fail("non_finite_point")
            continue
        rotating=point.motion_mode==1
        if type(point.motion_mode) is not int or point.motion_mode not in (0,1):
            result.fail("unknown_motion_mode")
        if rotating:
            if abs(point.v)>1e-12 or abs(point.a)>1e-12 or point.curvature not in (None,0.):
                result.fail("rotation_requires_zero_translation_and_undefined_curvature")
        elif limits.max_curvature_1pm is not None and abs(point.w)>limits.max_curvature_1pm*abs(point.v)+1e-9:
            result.fail("moving_curvature_yaw_rate_limit")
        if point.t < 0.0:
            result.fail("negative_point_time")
        result.max_speed = max(result.max_speed, abs(point.v))
        result.max_yaw_rate = max(result.max_yaw_rate, abs(point.w))
        if point.curvature is not None:
            result.max_abs_curvature = max(result.max_abs_curvature, abs(point.curvature))
        if point.v < limits.min_speed_mps - 1e-9 or point.v > limits.max_speed_mps + 1e-9:
            result.fail(f"speed_limit:{point.v:.6g}")
        if abs(point.v*point.w)>limits.max_lateral_accel_mps2+1e-9:
            result.fail("original_guard_turn_product_limit")
        if not within_firmware_command_envelope(point.v,point.w):
            result.fail("firmware_motor_target_limit")
        if abs(point.w) > limits.max_yaw_rate_rps + 1e-9:
            result.fail(f"yaw_rate_limit:{point.w:.6g}")
        if abs(point.a) > limits.max_accel_mps2 + 1e-9 and point.a >= 0:
            result.fail(f"accel_limit:{point.a:.6g}")
        if point.a < -limits.max_decel_mps2 - 1e-9:
            result.fail(f"decel_limit:{point.a:.6g}")
        if point.alpha >= 0 and point.alpha > limits.max_yaw_accel_rps2 + 1e-9:
            result.fail(f"yaw_accel_limit:{point.alpha:.6g}")
        if point.alpha < -limits.max_yaw_decel_rps2 - 1e-9:
            result.fail(f"yaw_decel_limit:{point.alpha:.6g}")
        if limits.max_curvature_1pm is not None and point.curvature is not None and abs(point.curvature) > limits.max_curvature_1pm + 1e-9:
            result.fail(f"curvature_limit:{point.curvature:.6g}")
        if previous is not None:
            dt = point.t - previous.t
            if dt <= 0:
                result.fail("non_monotonic_time")
            else:
                dx, dy = point.x - previous.x, point.y - previous.y
                if rotating and math.hypot(dx,dy)>1e-9:
                    result.fail("rotation_pivot_moved")
                if point.motion_mode!=previous.motion_mode and any(abs(v)>1e-12 for v in
                    (point.v,point.w,previous.v,previous.w)):
                    result.fail("motion_mode_transition_requires_stationary_boundary")
                distance = math.hypot(dx, dy)
                tangent = math.atan2(dy, dx) if distance > 1e-9 else previous.yaw
                # Forward-only body motion: tangent and yaw must agree and
                # body lateral velocity must be near zero.
                if abs(_angle_delta(tangent, point.yaw)) > 0.35 and distance > 1e-4:
                    result.fail("yaw_not_aligned_with_path_tangent")
                if distance > 1e-9:
                    vx, vy = dx / dt, dy / dt
                    forward_speed = math.cos(point.yaw) * vx + math.sin(point.yaw) * vy
                    lateral_speed = -math.sin(point.yaw) * vx + math.cos(point.yaw) * vy
                    if abs(lateral_speed) > limits.max_lateral_speed_mps + limits.derivative_tolerance:
                        result.fail(f"lateral_speed_limit:{lateral_speed:.6g}")
                    speed_tolerance = max(limits.derivative_tolerance, 0.25 * max(abs(point.v), 0.1))
                    if abs(forward_speed - point.v) > speed_tolerance:
                        result.fail("path_speed_mismatch")
                dv_dt = (point.v - previous.v) / dt
                dw_dt = (point.w - previous.w) / dt
                if dv_dt > limits.max_accel_mps2 + 1e-6:
                    result.fail(f"speed_derivative_accel_limit:{dv_dt:.6g}")
                if dv_dt < -limits.max_decel_mps2 - 1e-6:
                    result.fail(f"speed_derivative_decel_limit:{dv_dt:.6g}")
                if dw_dt > limits.max_yaw_accel_rps2 + 1e-6:
                    result.fail(f"yaw_derivative_accel_limit:{dw_dt:.6g}")
                if dw_dt < -limits.max_yaw_decel_rps2 - 1e-6:
                    result.fail(f"yaw_derivative_decel_limit:{dw_dt:.6g}")
                yaw_rate_from_path = _angle_delta(point.yaw, previous.yaw) / dt
                if abs(yaw_rate_from_path - point.w) > max(limits.derivative_tolerance, 0.25 * max(abs(point.w), 0.1)):
                    result.fail("yaw_rate_path_mismatch")
                if not rotating and point.curvature is not None and abs(point.w - point.v * point.curvature) > 0.08:
                    result.fail("yaw_rate_curvature_inconsistent")
        previous = point

    if footprint is not None and occupied is not None and resolution is None:
        result.fail("footprint_sweep_resolution_missing")

    sweep_points: list[TimedPoint] = list(trajectory.points)
    if resolution is not None and len(trajectory.points) > 1:
        sweep_points = [trajectory.points[0]]
        for previous, point in zip(trajectory.points, trajectory.points[1:]):
            distance = math.hypot(point.x - previous.x, point.y - previous.y)
            radius = max((math.hypot(x, y) for x, y in footprint), default=0.0) if footprint else 0.0
            corner_motion_bound = distance + radius * abs(_angle_delta(point.yaw, previous.yaw))
            steps = max(1, int(math.ceil(corner_motion_bound / (0.25 * resolution))))
            for step in range(1, steps + 1):
                fraction = step / steps
                yaw = previous.yaw + _angle_delta(point.yaw, previous.yaw) * fraction
                sweep_points.append(TimedPoint(
                    previous.t + (point.t - previous.t) * fraction,
                    previous.x + (point.x - previous.x) * fraction,
                    previous.y + (point.y - previous.y) * fraction,
                    yaw,
                    previous.v + (point.v - previous.v) * fraction,
                    previous.w + (point.w - previous.w) * fraction,
                    previous.a + (point.a - previous.a) * fraction,
                    previous.alpha + (point.alpha - previous.alpha) * fraction,
                    point.curvature if fraction == 1.0 else previous.curvature,
                    point.motion_mode,
                ))

    if footprint is not None:
        if occupied is None:
            result.fail("footprint_oracle_missing")
        else:
            for point in sweep_points:
                c, s = math.cos(point.yaw), math.sin(point.yaw)
                polygon = tuple((point.x + c * fx - s * fy,
                                 point.y + s * fx + c * fy) for fx, fy in footprint)
                if occupied_polygon is not None:
                    # Any material point moves at most resolution/4 between
                    # samples. Enlarging cells by that amount closes all gaps,
                    # including rotation with no centre translation.
                    if occupied_polygon(polygon, 0.25 * resolution):
                        result.fail("footprint_collision_or_unknown")
                        break
                    continue
                for fx, fy in footprint:
                    wx = point.x + c * fx - s * fy
                    wy = point.y + s * fx + c * fy
                    if occupied(wx, wy):
                        result.fail("footprint_collision_or_unknown")
                        break
    elif occupied is not None:
        for point in sweep_points:
            if occupied(point.x, point.y):
                result.fail("center_collision_or_unknown")
                break
    return result
