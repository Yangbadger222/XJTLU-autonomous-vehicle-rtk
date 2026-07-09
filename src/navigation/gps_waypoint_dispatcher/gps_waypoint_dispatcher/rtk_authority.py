from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


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


def compute_pose_delta(previous: Pose2D | None, current: Pose2D) -> tuple[float, float]:
    if previous is None:
        return 0.0, 0.0
    return (
        math.hypot(current.x - previous.x, current.y - previous.y),
        normalize_angle(current.yaw - previous.yaw),
    )


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
