import pytest

from research_runtime.authority import AuthorityState, SafetyCommand, SafetyGate, format_serial


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


@pytest.mark.parametrize("authority_stamp, now", [(float("nan"), 1.0), (1.0, float("inf"))])
def test_nonfinite_authority_time_reaches_final_mock_serial_sink(authority_stamp, now):
    command = SafetyGate().command(0.4, 0.2,
                                   AuthorityState(True, authority_stamp, now))
    assert not command.allowed
    assert "authority_time_non_finite" in command.reason
    assert format_serial(command) == b"vcx=0,wc=0\n"


def test_invalid_authority_timeout_is_rejected_before_commands():
    with pytest.raises(ValueError):
        SafetyGate(float("nan"))
    with pytest.raises(ValueError):
        SafetyGate(-0.1)


@pytest.mark.parametrize("speed_limit", [float("nan"), float("inf"), -0.1])
def test_invalid_linear_speed_override_reaches_final_mock_serial_sink(speed_limit):
    command = SafetyGate().command(0.4, 0.2,
                                   AuthorityState(True, 1.0, 1.1,
                                                 max_linear_speed_mps=speed_limit))
    assert not command.allowed
    assert "invalid_linear_speed_limit" in command.reason
    assert format_serial(command) == b"vcx=0,wc=0\n"


@pytest.mark.parametrize("scale", [float("nan"), float("inf"), -float("inf")])
def test_nonfinite_serial_scale_reaches_final_mock_serial_sink(scale):
    command = SafetyGate().command(0.4, 0.2, AuthorityState(True, 1.0, 1.1))
    assert format_serial(command, scale) == b"vcx=0,wc=0\n"


def test_format_serial_defensively_rejects_nonfinite_allowed_command():
    command = SafetyCommand(float("nan"), 0.2, True, "test")
    assert format_serial(command) == b"vcx=0,wc=0\n"
