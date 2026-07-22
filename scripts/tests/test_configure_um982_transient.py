import importlib.util
import io
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "configure_um982_transient.py"
SPEC = importlib.util.spec_from_file_location("configure_um982_transient", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_mixed_profile_keeps_ascii_and_adds_all_supported_binary_inputs():
    profile = MODULE.mixed_profile("COM1", 0.1)

    for command in (
        "GPGGA COM1 0.1",
        "GPTHS COM1 0.1",
        "UNIHEADINGA COM1 0.1",
        "OBSVMB COM1 0.1",
        "OBSVHB COM1 0.1",
        "OBSVBASEB COM1 ONCHANGED",
        "GPSEPHB COM1 60",
        "GLOEPHB COM1 60",
        "BDSEPHB COM1 60",
        "GALEPHB COM1 60",
        "QZSSEPHB COM1 60",
    ):
        assert command in profile.commands
    assert profile.final_baud == 921600


def test_profiles_never_persist_receiver_state():
    profiles = (
        MODULE.mixed_profile("COM1", 0.1),
        MODULE.nmea_only_profile("COM1", 0.1, 115200),
    )

    for profile in profiles:
        MODULE.validate_commands(profile.commands)
        assert all("SAVE" not in command.upper() for command in profile.commands)


def test_persistent_command_is_rejected():
    with pytest.raises(ValueError, match="persistent command is forbidden"):
        MODULE.validate_commands(["SAVE" + "CONFIG"])


def test_restore_switches_baud_only_after_ascii_output_is_selected():
    profile = MODULE.nmea_only_profile("COM1", 0.2, 115200)

    assert profile.commands[0] == "UNLOG COM1"
    assert profile.commands[-1] == "CONFIG COM1 115200"
    assert all(command.endswith("COM1 115200") is False for command in profile.commands[:-1])


def test_mixed_dry_run_shows_baud_transition_and_never_opens_serial():
    args = SimpleNamespace(
        com="COM1",
        period=0.1,
        source_baud=115200,
        ephemeris_period=60.0,
        dry_run=True,
    )
    output = io.StringIO()

    with patch.object(MODULE, "open_serial") as open_serial, patch("sys.stdout", output):
        assert MODULE.enter_mixed(args) == 0

    open_serial.assert_not_called()
    assert "CONFIG COM1 921600" in output.getvalue()
    assert "receiver_state_saved=false" in output.getvalue()
