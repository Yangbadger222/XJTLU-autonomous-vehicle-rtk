import math
from pathlib import Path

import pytest

from gps_waypoint_dispatcher.corridor_cmd_guard import CorridorCommandGuard


def _ready_guard(**kwargs):
    guard = CorridorCommandGuard(**kwargs)
    guard.update_authority(True, received_s=10.0)
    guard.update_stop_override(False, received_s=10.0)
    return guard


def test_guard_preserves_straight_speed_and_limits_turn_product():
    guard = _ready_guard()
    guard.update_command(0.85, 0.0, received_s=10.0)
    straight = guard.evaluate(now_s=10.1)
    guard.update_command(0.85, 0.70, received_s=10.2)
    turning = guard.evaluate(now_s=10.3)

    assert straight.linear_x == pytest.approx(0.85)
    assert turning.linear_x == pytest.approx(0.25 / 0.70)
    assert turning.angular_z == pytest.approx(0.70)


def test_guard_applies_same_magnitude_limit_in_reverse():
    guard = _ready_guard()
    guard.update_command(-0.85, -0.70, received_s=10.0)

    result = guard.evaluate(now_s=10.1)

    assert result.linear_x == pytest.approx(-(0.25 / 0.70))
    assert result.angular_z == pytest.approx(-0.70)


@pytest.mark.parametrize(
    ("linear_x", "angular_z"),
    [(math.nan, 0.0), (0.2, math.inf), (-math.inf, 0.1)],
)
def test_guard_zeros_nonfinite_commands(linear_x, angular_z):
    guard = _ready_guard()
    guard.update_command(linear_x, angular_z, received_s=10.0)

    result = guard.evaluate(now_s=10.1)

    assert result.linear_x == 0.0
    assert result.angular_z == 0.0
    assert result.reason == "NONFINITE_COMMAND"


def test_guard_is_fail_closed_before_all_inputs_arrive():
    guard = CorridorCommandGuard()

    assert guard.evaluate(now_s=10.0).reason == "COMMAND_UNAVAILABLE"
    guard.update_command(0.2, 0.0, received_s=10.0)
    assert guard.evaluate(now_s=10.0).reason == "AUTHORITY_UNAVAILABLE"
    guard.update_authority(True, received_s=10.0)
    assert guard.evaluate(now_s=10.0).reason == "STOP_OVERRIDE_UNAVAILABLE"


@pytest.mark.parametrize(
    ("setup", "now_s", "reason"),
    [
        ("command_stale", 10.251, "COMMAND_STALE"),
        ("authority_stale", 10.501, "AUTHORITY_STALE"),
        ("stop_stale", 10.501, "STOP_OVERRIDE_STALE"),
        ("authority_false", 10.1, "MOTION_AUTHORITY_FALSE"),
        ("stop_true", 10.1, "STOP_OVERRIDE_TRUE"),
    ],
)
def test_guard_zeros_for_stale_or_blocking_inputs(setup, now_s, reason):
    guard = _ready_guard()
    guard.update_command(0.4, 0.2, received_s=10.0)
    if setup == "command_stale":
        guard.update_authority(True, received_s=10.1)
        guard.update_stop_override(False, received_s=10.1)
    elif setup == "authority_stale":
        guard.update_command(0.4, 0.2, received_s=10.4)
        guard.update_stop_override(False, received_s=10.4)
    elif setup == "stop_stale":
        guard.update_command(0.4, 0.2, received_s=10.4)
        guard.update_authority(True, received_s=10.4)
    elif setup == "authority_false":
        guard.update_authority(False, received_s=10.0)
    elif setup == "stop_true":
        guard.update_stop_override(True, received_s=10.0)

    result = guard.evaluate(now_s=now_s)

    assert (result.linear_x, result.angular_z) == (0.0, 0.0)
    assert result.reason == reason


def test_guard_node_contract_and_setup_entrypoint():
    package_root = Path(__file__).resolve().parents[1]
    node_text = (
        package_root
        / "gps_waypoint_dispatcher"
        / "corridor_cmd_vel_guard_node.py"
    ).read_text(encoding="utf-8")
    setup_text = (package_root / "setup.py").read_text(encoding="utf-8")

    assert '"/cmd_vel_nav"' in node_text
    assert '"/localization_authority/motion_allowed"' in node_text
    assert '"/gps_corridor/stop_override"' in node_text
    assert '"/cmd_vel"' in node_text
    assert "time.monotonic()" in node_text
    assert "create_timer(0.05" in node_text
    assert "corridor_cmd_vel_guard_node = " in setup_text
