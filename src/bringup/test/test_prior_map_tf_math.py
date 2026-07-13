import math
import sys
from pathlib import Path


sys.path.insert(0, str(Path("src/bringup/scripts").resolve()))

from prior_map_tf_math import (  # noqa: E402
    bounded_map_to_odom_update,
    compose_se2,
    map_to_odom_from_poses,
    pose_residual,
)


def test_map_to_odom_reconstructs_map_pose():
    odom_from_base = (12.0, -3.0, 0.4)
    map_from_base = (4.0, 8.0, -0.2)

    map_from_odom = map_to_odom_from_poses(map_from_base, odom_from_base)
    reconstructed = compose_se2(map_from_odom, odom_from_base)

    translation_error, yaw_error = pose_residual(map_from_base, reconstructed)
    assert translation_error < 1.0e-9
    assert yaw_error < 1.0e-9


def test_bounded_update_limits_equivalent_base_jump_far_from_odom_origin():
    current = (0.0, 0.0, 0.0)
    target = (0.0, 0.0, math.radians(10.0))
    odom_from_base = (30.0, 0.0, 0.0)

    updated = bounded_map_to_odom_update(
        current,
        target,
        odom_from_base,
        alpha=1.0,
        translation_deadband_m=0.0,
        yaw_deadband_rad=0.0,
        max_translation_step_m=0.03,
        max_yaw_step_rad=0.05,
        max_base_step_m=0.04,
    )

    old_base = compose_se2(current, odom_from_base)
    new_base = compose_se2(updated, odom_from_base)
    base_step, _ = pose_residual(old_base, new_base)
    assert base_step <= 0.040001
    assert 0.0 < updated[2] < target[2]


def test_bounded_update_ignores_small_amcl_noise_inside_deadband():
    current = (1.0, 2.0, 0.1)
    target = (1.005, 2.004, 0.105)

    updated = bounded_map_to_odom_update(
        current,
        target,
        odom_from_base=(2.0, 3.0, 0.2),
        alpha=0.15,
        translation_deadband_m=0.02,
        yaw_deadband_rad=0.015,
        max_translation_step_m=0.03,
        max_yaw_step_rad=0.01,
        max_base_step_m=0.04,
    )

    assert updated == current
