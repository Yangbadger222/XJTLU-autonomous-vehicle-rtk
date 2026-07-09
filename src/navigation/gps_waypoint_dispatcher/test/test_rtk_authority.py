import math

import pytest

import gps_waypoint_dispatcher.rtk_authority as authority
from gps_waypoint_dispatcher.rtk_authority import (
    Pose2D,
    blend_pose_target,
    compose_pose,
    compute_bootstrap_alignment_from_current_pose,
    compute_map_to_odom,
    compute_rtk_map_base,
    limit_map_to_odom_step_for_base,
    limit_pose_step,
    select_authority_alignment,
    should_publish_bootstrap_without_fixed,
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


def test_select_authority_alignment_reuses_bootstrap_when_external_missing():
    external = (math.radians(10.0), 1.0, 2.0, True)
    bootstrap = (math.radians(20.0), 3.0, 4.0, True)

    alignment, using_external = select_authority_alignment(
        latest_alignment=external,
        external_alignment_valid=False,
        bootstrap_alignment=bootstrap,
    )

    assert alignment == bootstrap
    assert using_external is False


def test_bootstrap_tf_can_publish_before_rtk_fixed_for_nav2_startup():
    assert should_publish_bootstrap_without_fixed(
        rtk_fixed_ok=False,
        using_external_alignment=False,
        bootstrap_alignment_valid=True,
    )
    assert not should_publish_bootstrap_without_fixed(
        rtk_fixed_ok=False,
        using_external_alignment=True,
        bootstrap_alignment_valid=True,
    )


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


def test_limit_map_to_odom_step_caps_base_motion_from_far_yaw_lever_arm():
    previous_map_odom = Pose2D(x=0.0, y=0.0, yaw=0.0)
    target_map_odom = Pose2D(x=0.0, y=0.0, yaw=math.radians(5.0))
    odom_base = Pose2D(x=9.0, y=67.0, yaw=0.0)

    limited = limit_map_to_odom_step_for_base(
        previous_map_odom,
        target_map_odom,
        odom_base=odom_base,
        max_translation_step_m=1.0,
        max_yaw_step_rad=math.radians(0.5),
        max_base_yaw_step_m=0.12,
    )

    previous_map_base = compose_pose(previous_map_odom, odom_base)
    limited_map_base = compose_pose(limited.pose, odom_base)
    base_shift_m = math.hypot(
        limited_map_base.x - previous_map_base.x,
        limited_map_base.y - previous_map_base.y,
    )

    assert base_shift_m <= 0.12 + 1e-6
    assert math.degrees(limited.yaw_step_rad) < 0.5
    assert limited.limited is True


def test_blend_pose_target_holds_small_map_odom_target_jitter():
    previous = Pose2D(x=10.0, y=-2.0, yaw=math.radians(5.0))
    jitter = Pose2D(x=10.03, y=-2.02, yaw=math.radians(5.15))

    held = blend_pose_target(
        previous,
        jitter,
        alpha=0.20,
        translation_deadband_m=0.05,
        yaw_deadband_rad=math.radians(0.25),
    )

    assert held == previous


def test_blend_pose_target_low_passes_larger_map_odom_target_changes():
    previous = Pose2D(x=0.0, y=0.0, yaw=0.0)
    target = Pose2D(x=1.0, y=0.0, yaw=math.radians(20.0))

    blended = blend_pose_target(
        previous,
        target,
        alpha=0.25,
        translation_deadband_m=0.05,
        yaw_deadband_rad=math.radians(0.25),
    )

    assert blended.x == pytest.approx(0.25)
    assert blended.y == pytest.approx(0.0)
    assert math.degrees(blended.yaw) == pytest.approx(5.0)


def test_target_jump_uses_last_trusted_raw_target_not_smoothed_output():
    last_trusted_raw = Pose2D(x=10.0, y=0.0, yaw=math.radians(2.0))
    current_raw = Pose2D(x=10.3, y=0.1, yaw=math.radians(2.4))
    lagged_output = Pose2D(x=0.0, y=0.0, yaw=0.0)

    translation_m, yaw_rad = authority.compute_pose_delta(last_trusted_raw, current_raw)
    lagged_translation_m, _ = authority.compute_pose_delta(lagged_output, current_raw)

    assert translation_m == pytest.approx(math.hypot(0.3, 0.1))
    assert math.degrees(yaw_rad) == pytest.approx(0.4)
    assert lagged_translation_m > 10.0


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


def test_authority_inputs_allow_yaw_reacquire_when_translation_is_safe():
    summary = summarize_authority_inputs(
        alignment_valid=True,
        odom_available=True,
        fix_age_s=0.2,
        heading_age_s=0.1,
        target_jump_m=1.6,
        target_yaw_jump_rad=math.radians(40.0),
        max_fix_age_s=1.0,
        max_heading_age_s=1.0,
        max_target_jump_m=2.0,
        max_target_yaw_jump_rad=math.radians(20.0),
        allow_yaw_reacquire=True,
        max_yaw_reacquire_jump_rad=math.radians(45.0),
    )

    assert summary.ok is True
    assert summary.mode == "RTK_AUTHORITATIVE"
    assert summary.reason == "YAW_REACQUIRE"


def test_authority_inputs_reject_yaw_reacquire_outside_window():
    summary = summarize_authority_inputs(
        alignment_valid=True,
        odom_available=True,
        fix_age_s=0.2,
        heading_age_s=0.1,
        target_jump_m=1.6,
        target_yaw_jump_rad=math.radians(55.0),
        max_fix_age_s=1.0,
        max_heading_age_s=1.0,
        max_target_jump_m=2.0,
        max_target_yaw_jump_rad=math.radians(20.0),
        allow_yaw_reacquire=True,
        max_yaw_reacquire_jump_rad=math.radians(45.0),
    )

    assert summary.ok is False
    assert summary.reason == "TARGET_YAW_JUMP"


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
    assert '"allow_yaw_reacquire"' in node_text
    assert '"YAW_REACQUIRE"' in node_text
    assert '"max_base_yaw_step_m"' in node_text
    assert "limit_map_to_odom_step_for_base" in node_text
    assert "self._last_raw_target: Pose2D | None = None" in node_text
    assert "compute_pose_delta(\n                self._last_raw_target," in node_text
    assert "raw_output_gap_m" in node_text


def test_rtk_map_odom_corrector_rebroadcasts_last_trusted_tf_when_degraded():
    node_text = open(
        "src/navigation/gps_waypoint_dispatcher/gps_waypoint_dispatcher/"
        "rtk_map_odom_corrector_node.py",
        encoding="utf-8",
    ).read()

    assert "def _rebroadcast_last_output(self) -> bool:" in node_text
    assert "self._publish_tf(self._last_output)" in node_text

    degraded_exit_markers = [
        "if not summary.ok:",
        "if not rtk_fixed_ok and not publish_bootstrap_without_fixed:",
        "if not valid_fix(self._latest_fix) or self._latest_heading_enu_yaw is None:",
        "if not jump_summary.ok:",
    ]
    for marker in degraded_exit_markers:
        marker_index = node_text.index(marker)
        return_index = node_text.index("return", marker_index)
        degraded_branch = node_text[marker_index:return_index]
        assert "_rebroadcast_last_output()" in degraded_branch


def test_rtk_map_odom_corrector_is_shutdown_safe():
    node_text = open(
        "src/navigation/gps_waypoint_dispatcher/gps_waypoint_dispatcher/"
        "rtk_map_odom_corrector_node.py",
        encoding="utf-8",
    ).read()

    assert "def _safe_publish(" in node_text
    assert "if not rclpy.ok():" in node_text
    assert "_safe_publish(self._mode_pub" in node_text
    assert "_safe_publish(self._status_pub" in node_text
    assert "_safe_publish(self._diagnostics_pub" in node_text
    assert "Ignoring publish during shutdown" in node_text
