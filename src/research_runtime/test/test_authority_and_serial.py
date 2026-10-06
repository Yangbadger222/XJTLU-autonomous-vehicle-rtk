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
