import math

from research_runtime.trajectory import TimedPoint, TimedTrajectory, VehicleLimits
from research_runtime.trajectory_tracker import TrackerState, TimedTrajectoryTracker


def trajectory():
    return TimedTrajectory.from_points(
        "track", "m1", "odom", 10.0, 20.0,
        [TimedPoint(0.0, 0.0, 0.0, 0.0, 0.2, 0.0),
         TimedPoint(2.0, 0.4, 0.0, 0.0, 0.2, 0.0)])


def test_tracker_interpolates_timed_target_and_uses_pose_feedback():
    tracker = TimedTrajectoryTracker(VehicleLimits(), longitudinal_gain=1.0,
                                     lateral_gain=2.0, heading_gain=1.0)
    command = tracker.command(trajectory(), TrackerState(-0.1, 0.2, 0.0), now=11.0,
                              expected_map_version="m1")
    assert command is not None
    assert math.isclose(command.target.x, 0.2)
    assert command.linear_x > 0.2
    assert command.angular_z < 0.0


def test_tracker_rejects_expired_or_invalid_trajectory():
    tracker = TimedTrajectoryTracker(VehicleLimits())
    assert tracker.command(trajectory(), TrackerState(0.0, 0.0, 0.0), now=20.1,
                           expected_map_version="m1") is None
    assert tracker.command(trajectory(), TrackerState(0.0, 0.0, 0.0), now=11.0,
                           expected_map_version="wrong") is None


def test_tracker_rejects_feedback_outside_vehicle_limits():
    tracker = TimedTrajectoryTracker(VehicleLimits(max_speed_mps=0.3, max_yaw_rate_rps=0.4),
                                     longitudinal_gain=100.0, lateral_gain=100.0,
                                     heading_gain=100.0)
    command = tracker.command(trajectory(), TrackerState(-10.0, 10.0, 0.0), now=10.0,
                              expected_map_version="m1")
    assert command is None


def test_preview_advances_acceleration_when_each_replan_starts_at_measured_rest():
    trajectory = TimedTrajectory.from_points("rest", "m1", "odom", 0, 1,
        [TimedPoint(0, 0, 0, 0, 0, 0, a=0.2), TimedPoint(0.1, 0.001, 0, 0, 0.02, 0, a=0.2)])
    state = TrackerState(0, 0, 0)
    without_preview = TimedTrajectoryTracker(VehicleLimits()).command(trajectory, state, now=0)
    with_preview = TimedTrajectoryTracker(VehicleLimits(), preview_s=0.05).command(trajectory, state, now=0)
    assert without_preview.linear_x == 0
    assert 0 < with_preview.linear_x < 0.02


def test_feedback_cannot_cross_actual_footprint_or_curvature_envelope():
    from research_runtime.grid_map import LocalObstacleGrid
    from research_runtime.physical_parameter_lock import LOCKED_FOOTPRINT
    cells=[0]*400;cells[14*20+11]=100
    grid=LocalObstacleGrid('odom','m1',.1,-1.,-1.,20,20,tuple(cells))
    tracker=TimedTrajectoryTracker(VehicleLimits(max_curvature_1pm=2.),lateral_gain=.1)
    assert tracker.command(trajectory(),TrackerState(0,.2,0),now=10.,expected_map_version='m1',
        footprint=LOCKED_FOOTPRINT,occupied=grid.occupied,resolution=.1,occupied_polygon=grid.polygon_occupied) is None
    assert tracker.last_rejection=='measured_footprint_or_feedback_sweep_blocked'
    tracker=TimedTrajectoryTracker(VehicleLimits(max_curvature_1pm=2.))
    assert tracker.command(trajectory(),TrackerState(0,.4,0),now=10.) is None
    assert tracker.last_rejection=='feedback_curvature_limit'
