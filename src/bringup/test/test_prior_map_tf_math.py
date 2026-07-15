import math
import sys
from pathlib import Path


sys.path.insert(0, str(Path("src/bringup/scripts").resolve()))

from prior_map_tf_math import (  # noqa: E402
    bounded_map_to_odom_update,
    compose_se2,
    map_to_odom_from_poses,
    median_se2,
    pose_residual,
    should_activate_localizer_seed,
    stable_se2_window,
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


def test_stable_window_rejects_outlier_then_returns_median_candidate():
    noisy = [
        (1.00, 2.00, 0.10),
        (1.01, 2.01, 0.11),
        (1.30, 1.70, 0.40),
    ]
    candidate, translation_spread, yaw_spread = stable_se2_window(
        noisy,
        min_samples=3,
        max_translation_spread_m=0.05,
        max_yaw_spread_rad=0.04,
    )
    assert candidate is None
    assert translation_spread > 0.05
    assert yaw_spread > 0.04

    stable = noisy[:2] + [(0.99, 2.005, 0.095)]
    candidate, translation_spread, yaw_spread = stable_se2_window(
        stable,
        min_samples=3,
        max_translation_spread_m=0.05,
        max_yaw_spread_rad=0.04,
    )
    assert candidate == median_se2(stable)
    assert translation_spread < 0.05
    assert yaw_spread < 0.04


def test_median_se2_handles_yaw_wraparound():
    candidate = median_se2(
        [
            (0.0, 0.0, math.radians(179.0)),
            (0.0, 0.0, math.radians(-179.0)),
            (0.0, 0.0, math.radians(178.0)),
        ]
    )
    assert abs(abs(math.degrees(candidate[2])) - 179.0) < 1.0e-6


def test_bounded_update_applies_translation_and_yaw_deadbands_independently():
    current = (0.0, 0.0, 0.0)
    odom_from_base = (0.0, 0.0, 0.0)

    translation_only = bounded_map_to_odom_update(
        current,
        target=(0.20, 0.0, 0.01),
        odom_from_base=odom_from_base,
        alpha=1.0,
        translation_deadband_m=0.05,
        yaw_deadband_rad=0.035,
        max_translation_step_m=1.0,
        max_yaw_step_rad=1.0,
        max_base_step_m=1.0,
    )
    assert translation_only == (0.20, 0.0, 0.0)

    yaw_only = bounded_map_to_odom_update(
        current,
        target=(0.01, 0.0, 0.20),
        odom_from_base=odom_from_base,
        alpha=1.0,
        translation_deadband_m=0.05,
        yaw_deadband_rad=0.035,
        max_translation_step_m=1.0,
        max_yaw_step_rad=1.0,
        max_base_step_m=1.0,
    )
    assert yaw_only == (0.0, 0.0, 0.20)


def test_localizer_seed_decision_accepts_manual_pose_while_already_localized():
    assert should_activate_localizer_seed(
        current_exists=True,
        manual_relocalization_pending=True,
        localizer_allowed=True,
        localizer_reason="relocalization_accepted",
    )


def test_localizer_seed_decision_rejects_background_recovery_and_unready_pose():
    assert not should_activate_localizer_seed(
        current_exists=True,
        manual_relocalization_pending=False,
        localizer_allowed=True,
        localizer_reason="relocalization_accepted",
    )
    assert not should_activate_localizer_seed(
        current_exists=True,
        manual_relocalization_pending=True,
        localizer_allowed=False,
        localizer_reason="manual_initial_pose_received",
    )
