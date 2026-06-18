import math

import pytest

from gps_waypoint_dispatcher.alignment_math import (
    choose_bootstrap_yaw,
    compute_bootstrap_alignment,
    heading_quaternion_yaw_to_enu_yaw,
)


def test_bootstrap_yaw_prefers_rtk_heading_over_route_launch_heading():
    choice = choose_bootstrap_yaw(
        route_launch_heading_deg=175.0,
        rtk_heading_yaw_rad=math.radians(0.0),
        use_rtk_heading=True,
        mismatch_warn_deg=10.0,
    )

    assert choice.source == "rtk_heading"
    assert choice.enu_yaw_rad == 0.0
    assert choice.route_mismatch_deg == pytest.approx(85.0)
    assert choice.warn_route_mismatch is True


def test_bootstrap_alignment_uses_selected_rtk_heading_for_theta():
    alignment = compute_bootstrap_alignment(
        map_x=10.0,
        map_y=20.0,
        map_yaw_rad=math.radians(30.0),
        anchor_enu_x=100.0,
        anchor_enu_y=5.0,
        selected_enu_yaw_rad=0.0,
    )

    assert alignment.theta == pytest.approx(math.radians(30.0))
    assert alignment.tx == pytest.approx(10.0 - (
        math.cos(math.radians(30.0)) * 100.0
        - math.sin(math.radians(30.0)) * 5.0
    ))
    assert alignment.ty == pytest.approx(20.0 - (
        math.sin(math.radians(30.0)) * 100.0
        + math.cos(math.radians(30.0)) * 5.0
    ))


def test_bootstrap_yaw_falls_back_to_route_when_rtk_heading_missing():
    choice = choose_bootstrap_yaw(
        route_launch_heading_deg=90.0,
        rtk_heading_yaw_rad=None,
        use_rtk_heading=True,
        mismatch_warn_deg=10.0,
    )

    assert choice.source == "route_launch_yaw"
    assert choice.enu_yaw_rad == 0.0
    assert choice.route_mismatch_deg is None
    assert choice.warn_route_mismatch is False


def test_heading_quaternion_yaw_defaults_to_compass_convention():
    enu_yaw = heading_quaternion_yaw_to_enu_yaw(
        math.radians(175.0),
        quaternion_yaw_is_compass=True,
    )

    assert math.degrees(enu_yaw) == pytest.approx(-85.0)


def test_heading_quaternion_yaw_can_use_ros_enu_convention():
    enu_yaw = heading_quaternion_yaw_to_enu_yaw(
        math.radians(15.0),
        quaternion_yaw_is_compass=False,
    )

    assert math.degrees(enu_yaw) == pytest.approx(15.0)
