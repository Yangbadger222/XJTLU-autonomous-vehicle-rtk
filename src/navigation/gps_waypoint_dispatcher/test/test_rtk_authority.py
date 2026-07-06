import math

import pytest

from gps_waypoint_dispatcher.rtk_authority import (
    Pose2D,
    compute_bootstrap_alignment_from_current_pose,
    compute_map_to_odom,
    compute_rtk_map_base,
    limit_pose_step,
    summarize_authority_inputs,
)


def test_compute_map_to_odom_maps_local_base_to_rtk_map_pose():
    map_base = Pose2D(x=10.0, y=4.0, yaw=math.radians(35.0))
    odom_base = Pose2D(x=2.0, y=-1.0, yaw=math.radians(15.0))

    map_odom = compute_map_to_odom(map_base, odom_base)

    cos_yaw = math.cos(map_odom.yaw)
    sin_yaw = math.sin(map_odom.yaw)
    projected_x = map_odom.x + cos_yaw * odom_base.x - sin_yaw * odom_base.y
    projected_y = map_odom.y + sin_yaw * odom_base.x + cos_yaw * odom_base.y
    projected_yaw = math.atan2(
        math.sin(map_odom.yaw + odom_base.yaw),
        math.cos(map_odom.yaw + odom_base.yaw),
    )

    assert projected_x == pytest.approx(map_base.x)
    assert projected_y == pytest.approx(map_base.y)
    assert projected_yaw == pytest.approx(map_base.yaw)


def test_compute_rtk_map_base_projects_enu_and_heading_through_alignment():
    map_base = compute_rtk_map_base(
        enu_x=10.0,
        enu_y=0.0,
        heading_enu_yaw=math.radians(20.0),
        alignment_theta=math.radians(30.0),
        alignment_tx=3.0,
        alignment_ty=-2.0,
    )

    assert map_base.x == pytest.approx(3.0 + math.cos(math.radians(30.0)) * 10.0)
    assert map_base.y == pytest.approx(-2.0 + math.sin(math.radians(30.0)) * 10.0)
    assert math.degrees(map_base.yaw) == pytest.approx(50.0)


def test_bootstrap_alignment_maps_current_rtk_pose_to_current_odom_pose():
    odom_base = Pose2D(x=1.5, y=-0.2, yaw=math.radians(8.0))

    alignment = compute_bootstrap_alignment_from_current_pose(
        odom_base=odom_base,
        enu_x=20.0,
        enu_y=3.0,
        heading_enu_yaw=math.radians(5.0),
    )
    map_base = compute_rtk_map_base(
        enu_x=20.0,
        enu_y=3.0,
        heading_enu_yaw=math.radians(5.0),
        alignment_theta=alignment.theta,
        alignment_tx=alignment.tx,
        alignment_ty=alignment.ty,
    )

    assert map_base.x == pytest.approx(odom_base.x)
    assert map_base.y == pytest.approx(odom_base.y)
    assert map_base.yaw == pytest.approx(odom_base.yaw)


def test_limit_pose_step_caps_translation_and_yaw():
    previous = Pose2D(x=0.0, y=0.0, yaw=0.0)
    target = Pose2D(x=3.0, y=4.0, yaw=math.radians(30.0))

    limited = limit_pose_step(
        previous,
        target,
        max_translation_step_m=1.0,
        max_yaw_step_rad=math.radians(5.0),
    )

    assert math.hypot(limited.pose.x, limited.pose.y) == pytest.approx(1.0)
    assert math.degrees(limited.pose.yaw) == pytest.approx(5.0)
    assert limited.limited is True
    assert limited.translation_step_m == pytest.approx(1.0)
    assert math.degrees(limited.yaw_step_rad) == pytest.approx(5.0)


def test_authority_inputs_accept_current_rtk_authority_source():
    summary = summarize_authority_inputs(
        alignment_valid=True,
        odom_available=True,
        fix_age_s=0.2,
        heading_age_s=0.1,
        target_jump_m=0.4,
        target_yaw_jump_rad=math.radians(2.0),
        max_fix_age_s=1.0,
        max_heading_age_s=1.0,
        max_target_jump_m=2.0,
        max_target_yaw_jump_rad=math.radians(20.0),
    )

    assert summary.ok is True
    assert summary.mode == "RTK_AUTHORITATIVE"
    assert summary.reason is None


@pytest.mark.parametrize(
    ("kwargs", "reason"),
    [
        ({"alignment_valid": False}, "NO_ALIGNMENT"),
        ({"odom_available": False}, "NO_ODOM_TF"),
        ({"fix_age_s": 2.0}, "STALE_FIX"),
        ({"heading_age_s": 2.0}, "STALE_HEADING"),
        ({"target_jump_m": 3.0}, "TARGET_JUMP"),
        ({"target_yaw_jump_rad": math.radians(30.0)}, "TARGET_YAW_JUMP"),
    ],
)
def test_authority_inputs_reject_unsafe_sources(kwargs, reason):
    params = {
        "alignment_valid": True,
        "odom_available": True,
        "fix_age_s": 0.2,
        "heading_age_s": 0.1,
        "target_jump_m": 0.4,
        "target_yaw_jump_rad": math.radians(2.0),
        "max_fix_age_s": 1.0,
        "max_heading_age_s": 1.0,
        "max_target_jump_m": 2.0,
        "max_target_yaw_jump_rad": math.radians(20.0),
    }
    params.update(kwargs)

    summary = summarize_authority_inputs(**params)

    assert summary.ok is False
    assert summary.mode == "RTK_DEGRADED"
    assert summary.reason == reason


def test_setup_exposes_rtk_map_odom_corrector_entry_point():
    setup_text = open("src/navigation/gps_waypoint_dispatcher/setup.py", encoding="utf-8").read()

    assert (
        "rtk_map_odom_corrector_node = "
        "gps_waypoint_dispatcher.rtk_map_odom_corrector_node:main"
    ) in setup_text


def test_rtk_map_odom_corrector_node_owns_authority_outputs():
    node_text = open(
        "src/navigation/gps_waypoint_dispatcher/gps_waypoint_dispatcher/"
        "rtk_map_odom_corrector_node.py",
        encoding="utf-8",
    ).read()

    assert 'super().__init__("rtk_map_odom_corrector")' in node_text
    assert "TransformBroadcaster" in node_text
    assert "lookup_transform(" in node_text
    assert "sendTransform" in node_text
    assert '"/localization_authority/mode"' in node_text
    assert '"/localization_authority/status"' in node_text
    assert '"/localization_authority/diagnostics"' in node_text
    assert "compute_map_to_odom" in node_text
    assert "compute_rtk_map_base" in node_text
