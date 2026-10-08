from dataclasses import replace
import pytest
from research_runtime.stopped_state import StopConfirmation
from research_runtime.trajectory_tracker import TrackerState,TimedTrajectoryTracker
from research_runtime.trajectory import VehicleLimits
from test_stationary_rotation import rotation


def confirmed(speed=.049,w=.034):
    stop=StopConfirmation(.2)
    assert not stop.update(1.,speed,w)
    for i in range(1,10):assert not stop.update(1.+i*.1,speed,w)
    assert stop.update(2.,speed,w)
    return stop


@pytest.mark.parametrize("stamp,speed,w",[(2.,0.,0.),(1.9,0.,0.),(2.3,0.,0.),
    (2.1,.051,0.),(2.1,0.,.035),(2.1,float("nan"),0.)])
def test_acquisition_or_rate_fault_requires_a_new_full_window(stamp,speed,w):
    stop=confirmed()
    assert not stop.update(stamp,speed,w)
    if stamp<=2. or stamp>2.2:assert not stop.continuity_valid
    assert not stop.update(2.4,0.,0.)


def test_tracker_proposal_does_not_commit_rotation_before_final_authority():
    tracker=TimedTrajectoryTracker(VehicleLimits())
    plan=rotation()
    admitted=TrackerState(0.,0.,0.,translation_speed=.049,yaw_rate=.034,stationary_confirmed=True)
    candidate=tracker.command(plan,admitted,now=1.,context="session1")
    assert candidate and candidate.linear_x==0.
    moving=replace(admitted,yaw_rate=.2,stationary_confirmed=False)
    assert tracker.command(plan,moving,now=1.1,context="session1") is None
    tracker.commit_command(plan,candidate,context="session1")
    assert tracker.command(plan,moving,now=1.1,context="session1") is not None
    tracker.revoke_rotation()
    assert tracker.command(plan,moving,now=1.1,context="session1") is None


@pytest.mark.parametrize("change",["generated_at","points","map","session"])
def test_rotation_admission_binds_motion_content_and_context(change):
    tracker=TimedTrajectoryTracker(VehicleLimits())
    plan=rotation()
    stopped=TrackerState(0.,0.,0.,stationary_confirmed=True)
    candidate=tracker.command(plan,stopped,now=1.,context="s")
    tracker.commit_command(plan,candidate,context="s")
    moving=replace(stopped,yaw_rate=.2,stationary_confirmed=False)
    context="s"
    if change=="generated_at":plan=replace(plan,generated_at=1.001)
    if change=="points":plan=replace(plan,points=tuple(replace(p,x=.001) for p in plan.points))
    if change=="map":plan=replace(plan,map_version="map2")
    if change=="session":context="new"
    assert tracker.command(plan,moving,now=1.1,context=context) is None
    assert tracker.last_rejection=="rotation_stop_confirmation_missing"


def test_original_stop_tolerance_is_not_permission_to_translate_or_drift():
    tracker=TimedTrajectoryTracker(VehicleLimits())
    assert tracker.command(rotation(),TrackerState(0.,0.,0.,.051,0.,True),now=1.) is None
    assert tracker.command(rotation(),TrackerState(.021,0.,0.,.049,.034,True),now=1.) is None


def test_normal_rotation_rate_leaves_continuity_valid():
    stop=confirmed()
    assert not stop.update(2.1,0.,.2)
    assert stop.continuity_valid


@pytest.mark.parametrize("bad_stamp",[2.,1.9,2.3])
def test_admitted_rotation_revokes_on_an_acquisition_fault(bad_stamp):
    stop=confirmed()
    tracker=TimedTrajectoryTracker(VehicleLimits())
    plan=rotation()
    state=TrackerState(0.,0.,0.,stationary_confirmed=True)
    command=tracker.command(plan,state,now=1.)
    tracker.commit_command(plan,command)
    stop.update(bad_stamp,0.,.2)
    assert not stop.continuity_valid
    tracker.revoke_rotation()  # Same explicit callback path as the ROS edge.
    assert tracker.command(plan,replace(state,yaw_rate=.2,stationary_confirmed=False),now=1.1) is None
