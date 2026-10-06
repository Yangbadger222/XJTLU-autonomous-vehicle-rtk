import math

from research_runtime.trajectory import TimedPoint, TimedTrajectory, VehicleLimits, validate_trajectory


def traj(points):
    return TimedTrajectory.from_points("t", "m1", "odom", 0.0, 10.0, points)


def test_real_start_velocity_is_preserved_and_validated():
    result = validate_trajectory(traj([
        TimedPoint(0.0, 0.0, 0.0, 0.0, 0.12, 0.0),
        TimedPoint(1.0, 0.12, 0.0, 0.0, 0.12, 0.0),
    ]), VehicleLimits(), now=1.0, expected_map_version="m1")
    assert result.valid


def test_world_component_speed_is_not_enough_for_curvature():
    result = validate_trajectory(traj([
        TimedPoint(0.0, 0.0, 0.0, 0.0, 0.5, 0.0, curvature=2.0),
        TimedPoint(1.0, 0.5, 0.0, 0.0, 0.5, 0.0, curvature=2.0),
    ]), VehicleLimits(max_curvature_1pm=1.0), now=0.0, expected_map_version="m1")
    assert not result.valid
    assert any("curvature_limit" in reason for reason in result.reasons)


def test_unknown_or_obstacle_footprint_rejects_path():
    result = validate_trajectory(traj([
        TimedPoint(0.0, 0.0, 0.0, 0.0, 0.2, 0.0),
        TimedPoint(1.0, 0.2, 0.0, 0.0, 0.2, 0.0),
    ]), VehicleLimits(), now=0.0, expected_map_version="m1",
    footprint=[(-0.5, -0.3), (-0.5, 0.3), (0.5, -0.3), (0.5, 0.3)],
    occupied=lambda x, y: x > 0.4, resolution=0.1)
    assert not result.valid
    assert "footprint_collision_or_unknown" in result.reasons


def test_dynamic_limits_reject_without_clipping():
    result = validate_trajectory(traj([
        TimedPoint(0.0, 0.0, 0.0, 0.0, 0.2, 0.0, a=0.0),
        TimedPoint(1.0, 0.2, 0.0, 0.0, 0.2, 0.0, a=2.0),
    ]), VehicleLimits(), now=0.0, expected_map_version="m1")
    assert not result.valid
    assert any("accel_limit" in reason for reason in result.reasons)


def test_discrete_body_lateral_speed_is_rejected():
    result = validate_trajectory(traj([
        TimedPoint(0.0, 0.0, 0.0, 0.0, 0.2, 0.0),
        TimedPoint(1.0, 0.2, 0.2, 0.0, 0.2, 0.0),
    ]), VehicleLimits(), now=0.0, expected_map_version="m1")
    assert not result.valid
    assert "lateral_speed_limit:0.2" in result.reasons


def test_discrete_speed_and_yaw_rate_derivatives_are_rejected():
    result = validate_trajectory(traj([
        TimedPoint(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        TimedPoint(1.0, 0.2, 0.0, 0.5, 2.0, 2.0),
    ]), VehicleLimits(), now=0.0, expected_map_version="m1")
    assert not result.valid
    assert any(reason.startswith("speed_derivative_accel_limit") for reason in result.reasons)
    assert any(reason.startswith("yaw_derivative_accel_limit") for reason in result.reasons)


def test_expiry_map_version_and_frame_are_rejected():
    expired = validate_trajectory(
        TimedTrajectory.from_points("t", "old", "map", 0.0, 0.5,
                                    [TimedPoint(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)]),
        VehicleLimits(), now=1.0, expected_map_version="m1")
    assert not expired.valid
    assert {"trajectory_expired", "map_version_mismatch", "unsupported_frame:map"} <= set(expired.reasons)


def test_nonfinite_trajectory_metadata_is_rejected():
    result = validate_trajectory(
        TimedTrajectory.from_points("t", "m1", "odom", 0.0, float("nan"),
                                    [TimedPoint(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)]),
        VehicleLimits(), now=0.0, expected_map_version="m1")
    assert not result.valid
    assert "valid_until_non_finite" in result.reasons


def test_unknown_map_metadata_is_rejected_even_without_expected_version():
    result = validate_trajectory(
        TimedTrajectory.from_points("t", "UNKNOWN", "odom", 0.0, 1.0,
                                    [TimedPoint(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)]),
        VehicleLimits(), now=0.0)
    assert not result.valid
    assert "map_version_unknown" in result.reasons


def test_time_rollback_is_rejected():
    result = validate_trajectory(traj([
        TimedPoint(0.0, 0.0, 0.0, 0.0, 0.2, 0.0),
        TimedPoint(-0.1, 0.02, 0.0, 0.0, 0.2, 0.0),
    ]), VehicleLimits(), now=0.0, expected_map_version="m1")
    assert not result.valid
    assert "non_monotonic_time" in result.reasons


def test_curvature_and_yaw_rate_must_agree_in_timed_contract():
    result = validate_trajectory(traj([
        TimedPoint(0.0, 0.0, 0.0, 0.0, 0.5, 0.0, curvature=1.0),
        TimedPoint(1.0, 0.5, 0.0, 0.0, 0.5, 0.0, curvature=1.0),
    ]), VehicleLimits(), now=0.0, expected_map_version="m1")
    assert not result.valid
    assert "yaw_rate_curvature_inconsistent" in result.reasons


def test_pose_jump_is_rejected_by_discrete_speed_check():
    result = validate_trajectory(traj([
        TimedPoint(0.0, 0.0, 0.0, 0.0, 0.2, 0.0),
        TimedPoint(0.1, 1.0, 0.0, 0.0, 0.2, 0.0),
    ]), VehicleLimits(), now=0.0, expected_map_version="m1")
    assert not result.valid
    assert "path_speed_mismatch" in result.reasons


def test_footprint_sweep_catches_collision_between_trajectory_points():
    result = validate_trajectory(traj([
        TimedPoint(0.0, 0.0, 0.0, 0.0, 1.0, 0.0),
        TimedPoint(1.0, 1.0, 0.0, 0.0, 1.0, 0.0),
    ]), VehicleLimits(), now=0.0, expected_map_version="m1",
    footprint=[(0.0, 0.0)], occupied=lambda x, y: 0.45 < x < 0.55,
    resolution=0.1)
    assert not result.valid
    assert "footprint_collision_or_unknown" in result.reasons


def test_footprint_collision_without_sweep_resolution_fails_closed():
    result = validate_trajectory(traj([
        TimedPoint(0.0, 0.0, 0.0, 0.0, 0.2, 0.0),
        TimedPoint(1.0, 0.2, 0.0, 0.0, 0.2, 0.0),
    ]), VehicleLimits(), now=0.0, expected_map_version="m1",
    footprint=[(0.0, 0.0)], occupied=lambda x, y: False)
    assert not result.valid
    assert "footprint_sweep_resolution_missing" in result.reasons
