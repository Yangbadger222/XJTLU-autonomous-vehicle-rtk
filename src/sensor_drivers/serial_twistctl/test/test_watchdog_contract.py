from pathlib import Path


SOURCE = Path(__file__).resolve().parents[1] / "src" / "serial_twistctl_node.cpp"


def test_host_watchdog_repeatedly_enforces_zero_velocity():
    text = SOURCE.read_text(encoding="utf-8")

    assert 'declare_parameter<int>("command_timeout_ms", 300)' in text
    assert 'declare_parameter<int>("watchdog_period_ms", 100)' in text
    assert "watchdog_callback" in text
    assert "send_command(0.0, 0.0, false);" in text
    assert "No velocity command" in text


def test_shutdown_attempts_zero_before_closing_serial_port():
    text = SOURCE.read_text(encoding="utf-8")
    destructor = text.split("~SerialTwistCtlNode()", maxsplit=1)[1].split(
        "private:", maxsplit=1
    )[0]

    assert destructor.index("send_command(0.0, 0.0, false)") < destructor.index(
        "serial_port_.close()"
    )
