import pytest

from research_runtime.authority import AuthorityState, SafetyGate, format_serial


def test_rtk_authority_loss_reaches_final_mock_serial_sink():
    command = SafetyGate().command(0.4, 0.2, AuthorityState(False, 1.0, 1.1))
    assert not command.allowed
    assert command.reason == "rtk_authority_false"
    assert format_serial(command) == b"vcx=0,wc=0\n"


def test_stale_authority_and_unknown_lio_stop():
    command = SafetyGate().command(0.4, 0.2, AuthorityState(True, 0.0, 1.0, health="UNKNOWN"))
    assert not command.allowed
    assert "authority_stale" in command.reason
    assert "lio_health_not_ok" in command.reason


def test_allowed_command_keeps_serial_contract_and_scale():
    command = SafetyGate().command(0.4, -0.2, AuthorityState(True, 1.0, 1.1, max_linear_speed_mps=0.5))
    assert command.allowed
    assert format_serial(command, 1.0) == b"vcx=0.4,wc=-0.2\n"


@pytest.mark.parametrize("fault", [
    AuthorityState(True, 1.0, 1.1, tf_ok=False),
    AuthorityState(True, 1.0, 1.1, map_ok=False),
    AuthorityState(True, 1.0, 1.1, trajectory_ok=False),
    AuthorityState(True, 1.0, 1.1, stop_override=True),
    AuthorityState(True, 1.0, 1.1, manual_stop=True),
])
def test_fault_injection_reaches_final_mock_serial_sink(fault):
    command = SafetyGate().command(0.4, 0.2, fault)
    assert not command.allowed
    assert format_serial(command) == b"vcx=0,wc=0\n"


def test_nonfinite_command_reaches_final_mock_serial_sink():
    command = SafetyGate().command(float("nan"), 0.2, AuthorityState(True, 1.0, 1.1))
    assert not command.allowed
    assert "non_finite_command" in command.reason
    assert format_serial(command) == b"vcx=0,wc=0\n"
