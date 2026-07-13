import math


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def compose_se2(parent_from_middle, middle_from_child):
    px, py, pyaw = parent_from_middle
    mx, my, myaw = middle_from_child
    cosine = math.cos(pyaw)
    sine = math.sin(pyaw)
    return (
        px + cosine * mx - sine * my,
        py + sine * mx + cosine * my,
        normalize_angle(pyaw + myaw),
    )


def inverse_se2(parent_from_child):
    x, y, yaw = parent_from_child
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return (
        -cosine * x - sine * y,
        sine * x - cosine * y,
        normalize_angle(-yaw),
    )


def map_to_odom_from_poses(map_from_base, odom_from_base):
    return compose_se2(map_from_base, inverse_se2(odom_from_base))


def pose_residual(reference, candidate):
    return (
        math.hypot(candidate[0] - reference[0], candidate[1] - reference[1]),
        abs(normalize_angle(candidate[2] - reference[2])),
    )


def _interpolate_se2(current, target, fraction):
    return (
        current[0] + fraction * (target[0] - current[0]),
        current[1] + fraction * (target[1] - current[1]),
        normalize_angle(
            current[2] + fraction * normalize_angle(target[2] - current[2])
        ),
    )


def bounded_map_to_odom_update(
    current,
    target,
    odom_from_base,
    alpha,
    translation_deadband_m,
    yaw_deadband_rad,
    max_translation_step_m,
    max_yaw_step_rad,
    max_base_step_m,
):
    current_base = compose_se2(current, odom_from_base)
    target_base = compose_se2(target, odom_from_base)
    base_translation_error, base_yaw_error = pose_residual(current_base, target_base)
    if (
        base_translation_error <= translation_deadband_m
        and base_yaw_error <= yaw_deadband_rad
    ):
        return current

    translation_error, yaw_error = pose_residual(current, target)
    fraction = min(1.0, max(0.0, alpha))
    if translation_error > 0.0 and max_translation_step_m > 0.0:
        fraction = min(fraction, max_translation_step_m / translation_error)
    if yaw_error > 0.0 and max_yaw_step_rad > 0.0:
        fraction = min(fraction, max_yaw_step_rad / yaw_error)

    proposed = _interpolate_se2(current, target, fraction)
    for _ in range(4):
        proposed_base = compose_se2(proposed, odom_from_base)
        base_step, _ = pose_residual(current_base, proposed_base)
        if not (base_step > max_base_step_m > 0.0):
            break
        fraction *= max_base_step_m / base_step
        proposed = _interpolate_se2(current, target, fraction)
    return proposed
