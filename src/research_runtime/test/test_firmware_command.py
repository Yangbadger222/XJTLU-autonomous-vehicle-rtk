import math
import pytest
from research_runtime.firmware_command import motor_targets_rpm, within_firmware_command_envelope
from research_runtime.physical_parameter_lock import PHYSICAL_LIMITS, RESEARCH_EXECUTION_PROFILE


def test_stm_left_pair_and_reversed_right_pair_use_source_command_geometry():
    # STM expected signs: a positive pure yaw commands all four same sign.
    targets=motor_targets_rpm(0.,.7)
    assert targets == pytest.approx((-.7*.23/.1*19.2*9.55,)*4)
    assert motor_targets_rpm(.5,0.) == pytest.approx((916.8,916.8,-916.8,-916.8))


def test_entire_original_velocity_box_fits_source_firmware_rpm_cap():
    peaks=[max(abs(t) for t in motor_targets_rpm(v,w)) for v in (0.,.85) for w in (-.7,.7)]
    assert max(peaks) == pytest.approx(1853.7696)
    assert all(within_firmware_command_envelope(v,w) for v in (0.,.85) for w in (-.7,.7))


def test_nonfinite_and_overspeed_commands_are_rejected():
    assert not within_firmware_command_envelope(20.,0.)
    assert not within_firmware_command_envelope(float('nan'),.1)
    with pytest.raises(ValueError):motor_targets_rpm(.1,float('inf'))


def test_conservative_research_curvature_respects_speed_dependent_original_guard():
    cap=RESEARCH_EXECUTION_PROFILE['max_curvature_1pm']
    assert cap == pytest.approx(.25/.85**2)
    for v in (.01,.1,.3,.6,.85):
        assert cap <= min(PHYSICAL_LIMITS['max_yaw_rate_rps']/v,
                          PHYSICAL_LIMITS['max_lateral_accel_mps2']/v**2)+1e-12
        assert v*cap <= PHYSICAL_LIMITS['max_yaw_rate_rps']
        assert v*v*cap <= PHYSICAL_LIMITS['max_lateral_accel_mps2']+1e-12
