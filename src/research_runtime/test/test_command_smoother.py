import math

from research_runtime.command_smoother import slew_command
from research_runtime.trajectory import VehicleLimits


def test_corridor_component_acceleration_and_deceleration_values():
    limits = VehicleLimits()
    v, w = slew_command((0, 0), (0.85, 0.7), 0.05, limits)
    assert math.isclose(v, 0.85 * 0.05)
    assert math.isclose(w, 1.4 * 0.05)
    v, w = slew_command((0.85, 0.7), (0, 0), 0.05, limits)
    assert math.isclose(v, 0.85 - 1.2 * 0.05)
    assert math.isclose(w, 0.7 - 1.8 * 0.05)


def test_timer_gap_does_not_create_unbounded_recovery_step():
    assert slew_command((0, 0), (0.85, 0.7), 3.0, VehicleLimits())[0] <= 0.085
    assert slew_command((0, 0), (math.nan, 0), 0.05, VehicleLimits()) == (0.0, 0.0)


def test_shared_slew_preserves_curvature_cone_and_original_component_caps():
    limits=VehicleLimits(max_curvature_1pm=1.)
    v,w=slew_command((0,0),(.85,.7),.05,limits)
    assert 0<v<=.85*.05 and 0<w<=1.4*.05 and w/v<=1.
    assert math.isclose(w/v,.7/.85)
