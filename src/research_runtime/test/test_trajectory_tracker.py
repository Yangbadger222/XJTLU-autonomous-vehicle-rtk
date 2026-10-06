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


def test_tracker_clamps_feedback_command_to_vehicle_limits():
    tracker = TimedTrajectoryTracker(VehicleLimits(max_speed_mps=0.3, max_yaw_rate_rps=0.4),
                                     longitudinal_gain=100.0, lateral_gain=100.0,
                                     heading_gain=100.0)
    command = tracker.command(trajectory(), TrackerState(-10.0, 10.0, 0.0), now=10.0,
                              expected_map_version="m1")
    assert command is not None
    assert command.linear_x == 0.3
    assert abs(command.angular_z) == 0.4
