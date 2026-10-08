"""Receipt ordering regression on actual callback functions, without ROS setup."""
import ast
from dataclasses import replace
import math
import os
from pathlib import Path
import subprocess
import time
from types import SimpleNamespace
import pytest
from research_runtime.trajectory import VehicleLimits
from research_runtime.trajectory_tracker import TrackerState,TimedTrajectoryTracker
from research_runtime.stopped_state import StopConfirmation
from test_stationary_rotation import rotation


def callback_target():
    repo=Path(__file__).resolve().parents[3]
    relative="src/research_runtime/research_runtime/safety_bridge.py"
    commit=os.environ.get("ROTATION_CALLBACK_SOURCE_COMMIT")
    text=(subprocess.check_output(["git","show",commit+":"+relative],cwd=repo,text=True)
          if commit else (repo/relative).read_text())
    wanted={"_revoke_rotation","_authority","_authority_mode_cb","_health_cb","_tf_cb","_map_version_cb"}
    tree=ast.parse(text)
    functions=[n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name in wanted]
    assert {n.name for n in functions}==wanted
    scope=dict(time=time,math=math,replace=replace,_valid_map_version=lambda v:v not in ("","UNKNOWN"))
    exec(compile(ast.Module(body=functions,type_ignores=[]),relative,"exec"),scope)
    target=SimpleNamespace()
    for name in wanted:setattr(target,name,lambda *args,_name=name,**kwargs:scope[_name](target,*args,**kwargs))
    target._tracker=TimedTrajectoryTracker(VehicleLimits())
    target._state=TrackerState(0.,0.,0.,stationary_confirmed=True)
    target._stop_confirmation=StopConfirmation(.2)
    target._original_guard_preflight=SimpleNamespace(update_authority=lambda *a,**kw:None)
    target._map_version="map1"
    plan=rotation()
    command=target._tracker.command(plan,target._state,now=1.)
    target._tracker.commit_command(plan,command)
    return target,plan


@pytest.mark.parametrize("name,denied,restored",[
    ("_authority",False,True),
    ("_authority_mode_cb","RTK_HOLD","RTK_AUTHORITATIVE"),
    ("_health_cb","UNKNOWN:lost","OK:restored"),
    ("_tf_cb",False,True),
    ("_map_version_cb","UNKNOWN","map1")])
def test_denial_then_restore_before_next_tick_does_not_preserve_rotation(name,denied,restored):
    target,plan=callback_target()
    getattr(target,name)(SimpleNamespace(data=denied))
    getattr(target,name)(SimpleNamespace(data=restored))
    moving=TrackerState(0.,0.,0.,0.,.2,False)
    assert target._tracker.command(plan,moving,now=1.1) is None
    assert target._tracker.last_rejection=="rotation_stop_confirmation_missing"
