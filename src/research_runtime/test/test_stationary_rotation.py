from dataclasses import replace
from research_runtime.trajectory import TimedPoint,TimedTrajectory,VehicleLimits,validate_trajectory
from research_runtime.trajectory_tracker import TimedTrajectoryTracker,TrackerState
from research_runtime.command_smoother import slew_command


def rotation():
    return TimedTrajectory.from_points('rotation','map1','odom',1.,2.,
        [TimedPoint(i*.1,0.,0.,i*.03,0.,.3,curvature=0.,motion_mode=1) for i in range(11)])


def test_explicit_rotation_obeys_yaw_instead_of_dividing_by_zero_speed():
    limits=VehicleLimits(max_curvature_1pm=.346)
    assert validate_trajectory(rotation(),limits,now=1.).valid
    untagged=replace(rotation(),points=tuple(replace(p,motion_mode=0) for p in rotation().points))
    assert not validate_trajectory(untagged,limits,now=1.).valid


def test_rotation_cannot_translate_or_change_modes_without_a_stopped_boundary():
    points=list(rotation().points);points[5]=replace(points[5],x=.01)
    assert 'rotation_pivot_moved' in validate_trajectory(replace(rotation(),points=tuple(points)),VehicleLimits()).reasons
    points=list(rotation().points);points[5]=replace(points[5],motion_mode=0)
    assert 'motion_mode_transition_requires_stationary_boundary' in validate_trajectory(replace(rotation(),points=tuple(points)),VehicleLimits()).reasons


def test_rotation_tracking_remains_pure_yaw_and_rejects_real_translation():
    tracker=TimedTrajectoryTracker(VehicleLimits(max_curvature_1pm=.346),preview_s=.1)
    command=tracker.command(rotation(),TrackerState(0.,0.,0.),now=1.)
    assert command and command.linear_x==0. and 0<command.angular_z<=.7
    assert tracker.command(rotation(),TrackerState(0.,0.,0.,translation_speed=.02),now=1.) is None
    assert tracker.command(rotation(),TrackerState(0.,0.,0.,translation_speed=-.02),now=1.) is None
    assert tracker.command(rotation(),TrackerState(.03,0.,0.),now=1.) is None


def test_rotation_slew_retains_original_angular_caps_and_requires_zero_linear_state():
    limits=VehicleLimits(max_curvature_1pm=.346)
    v,w=slew_command((0.,0.),(0.,.7),.05,limits,.5,in_place_rotation=True)
    assert v==0. and 0<w<=1.4*.05*.5
    assert slew_command((.1,0.),(0.,.7),.05,limits,in_place_rotation=True)==(0.,0.)
