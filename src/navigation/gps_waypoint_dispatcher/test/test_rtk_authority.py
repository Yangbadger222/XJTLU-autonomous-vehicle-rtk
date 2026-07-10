import math

import pytest

import gps_waypoint_dispatcher.rtk_authority as authority
from gps_waypoint_dispatcher.rtk_authority import (
    CorrectionGate,
    CorrectionGateState,
    Pose2D,
    StampedPoseHistory,
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


def _pose(x=0.0, y=0.0, yaw=0.0):
    return Pose2D(x=x, y=y, yaw=yaw)


def _append_history(history, stamp_s, pose=None, **frames):
    return history.append(
        stamp_s=stamp_s,
        pose=pose or _pose(),
        frame_id=frames.get("frame_id", "odom"),
        child_frame_id=frames.get("child_frame_id", "base_footprint"),
    )


def _lock_yaw_gate(gate, *, start_stamp_s=1.0, now_s=10.0, values=None):
    values = values or [0.0] * 5
    result = None
    for index, value in enumerate(values):
        result = gate.observe(
            stamp_s=start_stamp_s + 0.1 * index,
            value=value,
            now_s=now_s + 0.1 * index,
        )
    return result


@pytest.mark.parametrize(
    ("frames", "reason"),
    [
        ({"frame_id": "map"}, "INVALID_FRAME"),
        ({"child_frame_id": "base_link"}, "INVALID_CHILD_FRAME"),
    ],
)
def test_stamped_pose_history_rejects_wrong_frames(frames, reason):
    history = StampedPoseHistory()

    result = _append_history(history, 1.0, **frames)

    assert result.accepted is False
    assert result.reason == reason
    assert len(history) == 0


@pytest.mark.parametrize("stamp_s", [0.0, -1.0, math.inf, math.nan])
def test_stamped_pose_history_rejects_zero_or_nonfinite_stamp(stamp_s):
    history = StampedPoseHistory()

    result = _append_history(history, stamp_s)

    assert result.accepted is False
    assert result.reason == "INVALID_STAMP"
    assert len(history) == 0


@pytest.mark.parametrize(
    "pose",
    [
        _pose(x=math.nan),
        _pose(y=math.inf),
        _pose(yaw=-math.inf),
    ],
)
def test_stamped_pose_history_rejects_nonfinite_pose(pose):
    history = StampedPoseHistory()

    result = _append_history(history, 1.0, pose)

    assert result.accepted is False
    assert result.reason == "NONFINITE_POSE"
    assert len(history) == 0


def test_stamped_pose_history_rejects_duplicate_and_regressing_stamps():
    history = StampedPoseHistory()
    assert _append_history(history, 2.0).accepted

    duplicate = _append_history(history, 2.0, _pose(x=1.0))
    regressing = _append_history(history, 1.5, _pose(x=2.0))

    assert duplicate.reason == "DUPLICATE_STAMP"
    assert regressing.reason == "REGRESSING_STAMP"
    assert [sample.stamp_s for sample in history.samples] == [2.0]


def test_stamped_pose_history_bounds_samples_by_time():
    history = StampedPoseHistory(max_age_s=2.0)
    for stamp_s in [1.0, 2.0, 3.0, 3.01]:
        assert _append_history(history, stamp_s).accepted

    assert [sample.stamp_s for sample in history.samples] == [2.0, 3.0, 3.01]


def test_stamped_pose_history_bounds_samples_by_count():
    history = StampedPoseHistory(max_samples=3)
    for stamp_s in [1.0, 2.0, 3.0, 4.0]:
        assert _append_history(history, stamp_s).accepted

    assert [sample.stamp_s for sample in history.samples] == [2.0, 3.0, 4.0]


def test_stamped_pose_history_allows_exact_samples_without_extrapolation():
    history = StampedPoseHistory()
    exact_pose = _pose(x=2.0, y=-3.0, yaw=0.4)
    _append_history(history, 1.0, _pose())
    _append_history(history, 1.1, exact_pose)

    exact = history.interpolate(1.1)
    before = history.interpolate(0.9)
    after = history.interpolate(1.2)

    assert exact.ok is True
    assert exact.pose == exact_pose
    assert exact.reason is None
    assert before.reason == "ODOM_AT_STAMP_UNAVAILABLE"
    assert after.reason == "ODOM_AT_STAMP_UNAVAILABLE"


def test_stamped_pose_history_interpolates_translation_linearly():
    history = StampedPoseHistory()
    _append_history(history, 1.0, _pose(x=1.0, y=-2.0, yaw=0.0))
    _append_history(history, 1.2, _pose(x=3.0, y=4.0, yaw=0.0))

    result = history.interpolate(1.05)

    assert result.ok is True
    assert result.pose.x == pytest.approx(1.5)
    assert result.pose.y == pytest.approx(-0.5)


def test_stamped_pose_history_interpolates_yaw_across_wrap_on_shortest_arc():
    history = StampedPoseHistory()
    _append_history(history, 1.0, _pose(yaw=math.radians(179.0)))
    _append_history(history, 1.2, _pose(yaw=math.radians(-179.0)))

    result = history.interpolate(1.1)

    assert result.ok is True
    assert abs(math.degrees(result.pose.yaw)) == pytest.approx(180.0)


def test_stamped_pose_history_enforces_inclusive_bracket_limit():
    history = StampedPoseHistory()
    _append_history(history, 1.0)
    _append_history(history, 1.2, _pose(x=1.0))
    _append_history(history, 1.41, _pose(x=2.0))

    inclusive = history.interpolate(1.1, max_bracket_s=0.20)
    too_wide = history.interpolate(1.3, max_bracket_s=0.20)

    assert inclusive.ok is True
    assert too_wide.ok is False
    assert too_wide.reason == "ODOM_BRACKET_TOO_WIDE"


def test_stamped_pose_history_accepts_logical_bracket_limit_float_roundoff():
    history = StampedPoseHistory()
    _append_history(history, 1.0)
    _append_history(history, 1.2000000000000002, _pose(x=1.0))

    result = history.interpolate(1.1, max_bracket_s=0.20)

    assert result.ok is True


def test_stamped_pose_history_accepts_logical_bracket_at_ros_epoch_scale():
    base_stamp_s = 1783342965.0
    history = StampedPoseHistory()
    _append_history(history, base_stamp_s)
    _append_history(history, base_stamp_s + 0.20, _pose(x=1.0))

    result = history.interpolate(base_stamp_s + 0.10, max_bracket_s=0.20)

    assert result.ok is True


def test_correction_gate_state_values_match_diagnostic_contract():
    assert list(CorrectionGateState) == [
        CorrectionGateState.UNINITIALIZED,
        CorrectionGateState.LOCKED,
        CorrectionGateState.SUSPECT,
        CorrectionGateState.REACQUIRING,
        CorrectionGateState.DEGRADED,
    ]
    assert [state.value for state in CorrectionGateState] == list(range(5))


def test_first_eligible_observation_seeds_reacquisition():
    gate = CorrectionGate.yaw()

    result = gate.observe(stamp_s=1.0, value=0.2, now_s=5.0)

    assert result.processed is True
    assert result.accepted is False
    assert result.state is CorrectionGateState.REACQUIRING
    assert result.reason == "RECOVERY_PENDING"
    assert result.candidate_count == 1
    assert gate.target is None


def test_yaw_gate_locks_on_count_and_span_with_circular_mean():
    gate = CorrectionGate.yaw()
    values = [
        math.radians(178.0),
        math.radians(179.0),
        math.radians(-179.0),
        math.radians(-178.0),
        math.radians(180.0),
    ]

    result = _lock_yaw_gate(gate, values=values)

    assert result.accepted is True
    assert result.state is CorrectionGateState.LOCKED
    assert result.candidate_count == 5
    assert result.candidate_span_s == pytest.approx(0.4)
    assert abs(math.degrees(result.target)) == pytest.approx(180.0)


def test_translation_gate_locks_to_component_arithmetic_mean():
    gate = CorrectionGate.translation()
    values = [(0.0, 0.0), (0.1, 0.0), (0.2, 0.0), (0.1, 0.05), (0.1, -0.05)]
    result = None
    for index, value in enumerate(values):
        result = gate.observe(
            stamp_s=1.0 + 0.1 * index,
            value=value,
            now_s=10.0 + 0.1 * index,
        )

    assert result.state is CorrectionGateState.LOCKED
    assert result.target == pytest.approx((0.1, 0.0))


def test_high_rate_fifth_candidate_stays_reacquiring_until_span_passes():
    gate = CorrectionGate.yaw()
    result = None
    for index in range(5):
        result = gate.observe(
            stamp_s=1.0 + 0.05 * index,
            value=0.0,
            now_s=2.0 + 0.05 * index,
        )

    assert result.candidate_count == 5
    assert result.candidate_span_s == pytest.approx(0.2)
    assert result.state is CorrectionGateState.REACQUIRING

    locked = gate.observe(stamp_s=1.3, value=0.0, now_s=2.3)
    assert locked.state is CorrectionGateState.LOCKED


def test_recovery_locks_when_logical_span_has_negative_float_roundoff():
    gate = CorrectionGate.yaw()
    result = None
    for stamp_s in [1.0, 1.05, 1.1, 1.2, 1.2999999999999998]:
        result = gate.observe(stamp_s=stamp_s, value=0.0, now_s=stamp_s + 1.0)

    assert result.candidate_span_s == 0.2999999999999998
    assert result.state is CorrectionGateState.LOCKED


def test_recovery_locks_on_logical_span_at_ros_epoch_scale():
    base_stamp_s = 1783342965.0
    gate = CorrectionGate.yaw()
    result = None
    for offset_s in [0.0, 0.05, 0.10, 0.20, 0.30]:
        stamp_s = base_stamp_s + offset_s
        result = gate.observe(stamp_s=stamp_s, value=0.0, now_s=stamp_s)

    assert result.state is CorrectionGateState.LOCKED


def test_inconsistent_candidate_replaces_the_whole_window():
    gate = CorrectionGate.yaw()
    gate.observe(stamp_s=1.0, value=math.radians(0.0), now_s=2.0)
    gate.observe(stamp_s=1.1, value=math.radians(1.0), now_s=2.1)

    result = gate.observe(stamp_s=1.2, value=math.radians(8.0), now_s=2.2)

    assert result.state is CorrectionGateState.REACQUIRING
    assert result.reason == "RECOVERY_WINDOW_REPLACED"
    assert result.candidate_count == 1
    assert gate.candidate_values == (math.radians(8.0),)


def test_recovery_candidate_window_is_capped_at_twenty():
    gate = CorrectionGate.yaw()
    for index in range(25):
        result = gate.observe(
            stamp_s=1.0 + 0.01 * index,
            value=0.0,
            now_s=2.0 + 0.01 * index,
        )

    assert result.state is CorrectionGateState.REACQUIRING
    assert result.candidate_count == 20
    assert gate.candidate_stamps[0] == pytest.approx(1.05)


def test_locked_gate_accepts_an_in_gate_innovation():
    gate = CorrectionGate.yaw()
    _lock_yaw_gate(gate)

    result = gate.observe(
        stamp_s=1.5,
        value=math.radians(14.9),
        now_s=10.5,
    )

    assert result.accepted is True
    assert result.state is CorrectionGateState.LOCKED
    assert math.degrees(result.innovation) == pytest.approx(14.9)
    assert gate.target == pytest.approx(math.radians(14.9))


def test_one_locked_innovation_rejection_freezes_target_and_enters_suspect():
    gate = CorrectionGate.yaw()
    _lock_yaw_gate(gate)
    trusted = gate.target

    result = gate.observe(
        stamp_s=1.5,
        value=math.radians(15.1),
        now_s=10.5,
    )

    assert result.accepted is False
    assert result.state is CorrectionGateState.SUSPECT
    assert result.reason == "INNOVATION_REJECTED"
    assert result.candidate_count == 0
    assert result.target == trusted
    assert gate.last_trusted_target == trusted


def test_suspect_and_degraded_gates_seed_reacquisition_without_moving_target():
    suspect = CorrectionGate.yaw()
    _lock_yaw_gate(suspect)
    suspect.observe(stamp_s=1.5, value=math.radians(20.0), now_s=10.5)
    trusted = suspect.target

    suspect_result = suspect.observe(stamp_s=1.6, value=math.radians(20.0), now_s=10.6)

    degraded = CorrectionGate.translation()
    for index in range(5):
        degraded.prerequisite_failure(
            now_s=float(index),
            kind=authority.PrerequisiteFailureKind.ODOM_AT_STAMP_UNAVAILABLE,
        )
    degraded_result = degraded.observe(
        stamp_s=2.0,
        value=(3.0, 4.0),
        now_s=5.0,
    )

    assert suspect_result.state is CorrectionGateState.REACQUIRING
    assert suspect_result.target == trusted
    assert degraded_result.state is CorrectionGateState.REACQUIRING
    assert degraded_result.candidate_count == 1
    assert degraded_result.target is None


def test_duplicate_or_older_gate_observation_does_not_change_state():
    gate = CorrectionGate.translation()
    gate.observe(stamp_s=2.0, value=(1.0, 1.0), now_s=4.0)
    before = gate.snapshot()

    duplicate = gate.observe(stamp_s=2.0, value=(2.0, 2.0), now_s=4.1)
    older = gate.observe(stamp_s=1.9, value=(3.0, 3.0), now_s=4.2)

    assert duplicate.processed is False
    assert duplicate.reason == "DUPLICATE_STAMP"
    assert older.processed is False
    assert older.reason == "REGRESSING_STAMP"
    assert gate.snapshot() == before


def test_one_odom_timeout_moves_locked_gate_to_suspect_and_freezes_target():
    gate = CorrectionGate.yaw()
    _lock_yaw_gate(gate)
    trusted = gate.target

    result = gate.odom_timeout(stamp_s=2.0, now_s=11.0)

    assert result.state is CorrectionGateState.SUSPECT
    assert result.reason == "ODOM_AT_STAMP_UNAVAILABLE"
    assert result.target == trusted
    assert result.candidate_count == 0


@pytest.mark.parametrize(
    "kind",
    [
        authority.PrerequisiteFailureKind.HEADING_QUALITY_UNAVAILABLE,
        authority.PrerequisiteFailureKind.FIX_QUALITY_UNAVAILABLE,
    ],
)
def test_one_quality_miss_keeps_locked_gate_and_freezes_target(kind):
    gate = CorrectionGate.yaw()
    _lock_yaw_gate(gate)
    trusted = gate.target

    result = gate.prerequisite_failure(now_s=10.5, kind=kind)

    assert result.state is CorrectionGateState.LOCKED
    assert result.target == trusted
    assert result.consecutive_failures == 1


def test_five_quality_misses_degrade_and_clear_candidates():
    gate = CorrectionGate.yaw()
    _lock_yaw_gate(gate)
    trusted = gate.target

    for index in range(4):
        result = gate.prerequisite_failure(
            now_s=10.5 + index * 0.01,
            kind=authority.PrerequisiteFailureKind.HEADING_QUALITY_UNAVAILABLE,
        )
        assert result.state is CorrectionGateState.LOCKED

    degraded = gate.prerequisite_failure(
        now_s=10.54,
        kind=authority.PrerequisiteFailureKind.HEADING_QUALITY_UNAVAILABLE,
    )

    assert degraded.state is CorrectionGateState.DEGRADED
    assert degraded.target == trusted
    assert degraded.candidate_count == 0


def test_quality_miss_degrades_after_one_second_without_processable_observation():
    gate = CorrectionGate.yaw()
    _lock_yaw_gate(gate, now_s=10.0)

    result = gate.prerequisite_failure(
        now_s=11.4,
        kind=authority.PrerequisiteFailureKind.FIX_QUALITY_UNAVAILABLE,
    )

    assert result.state is CorrectionGateState.DEGRADED
    assert result.reason == "FIX_QUALITY_UNAVAILABLE"


def test_eligible_observation_resets_prerequisite_failure_tracking():
    gate = CorrectionGate.yaw()
    _lock_yaw_gate(gate, now_s=10.0)
    for now_s in [10.5, 10.6]:
        gate.prerequisite_failure(
            now_s=now_s,
            kind=authority.PrerequisiteFailureKind.HEADING_QUALITY_UNAVAILABLE,
        )

    accepted = gate.observe(stamp_s=1.5, value=0.0, now_s=10.7)
    before_timeout = gate.check_timeout(now_s=11.69)

    assert accepted.state is CorrectionGateState.LOCKED
    assert accepted.consecutive_failures == 0
    assert before_timeout.state is CorrectionGateState.LOCKED


@pytest.mark.parametrize(
    "kind",
    [
        authority.PrerequisiteFailureKind.STALE_INPUT,
        authority.PrerequisiteFailureKind.NON_FIXED_INPUT,
        authority.PrerequisiteFailureKind.MALFORMED_INPUT,
    ],
)
def test_terminal_input_failure_immediately_degrades_and_freezes(kind):
    gate = CorrectionGate.translation()
    values = [(0.0, 0.0), (0.1, 0.0), (0.2, 0.0), (0.1, 0.05), (0.1, -0.05)]
    result = None
    for index, value in enumerate(values):
        result = gate.observe(
            stamp_s=1.0 + index * 0.1,
            value=value,
            now_s=10.0 + index * 0.1,
        )
    trusted = result.target

    degraded = gate.prerequisite_failure(now_s=10.5, kind=kind)

    assert degraded.state is CorrectionGateState.DEGRADED
    assert degraded.target == trusted
    assert degraded.candidate_count == 0
    assert degraded.reason == kind.value


def test_invalid_odom_timeout_process_time_does_not_advance_gate_stamp():
    gate = CorrectionGate.yaw()
    gate.observe(stamp_s=1.0, value=0.0, now_s=2.0)
    before = gate.snapshot()

    result = gate.odom_timeout(stamp_s=1.1, now_s=math.nan)

    assert result.processed is False
    assert result.reason == "INVALID_PROCESS_TIME"
    assert gate.snapshot() == before


def test_five_consecutive_prerequisite_failures_degrade_and_clear_candidates():
    gate = CorrectionGate.translation()
    gate.observe(stamp_s=1.0, value=(0.0, 0.0), now_s=10.0)

    for index in range(4):
        result = gate.prerequisite_failure(
            now_s=10.1 + index * 0.1,
            kind=authority.PrerequisiteFailureKind.ODOM_AT_STAMP_UNAVAILABLE,
        )
        assert result.state is CorrectionGateState.REACQUIRING

    degraded = gate.prerequisite_failure(
        now_s=10.5,
        kind=authority.PrerequisiteFailureKind.ODOM_AT_STAMP_UNAVAILABLE,
    )

    assert degraded.state is CorrectionGateState.DEGRADED
    assert degraded.reason == "ODOM_AT_STAMP_UNAVAILABLE"
    assert degraded.candidate_count == 0


def test_one_second_without_processable_observation_degrades_gate():
    gate = CorrectionGate.yaw()
    _lock_yaw_gate(gate, now_s=10.0)

    still_locked = gate.check_timeout(now_s=11.399)
    degraded = gate.check_timeout(now_s=11.4)

    assert still_locked.state is CorrectionGateState.LOCKED
    assert degraded.state is CorrectionGateState.DEGRADED
    assert degraded.reason == "NO_PROCESSABLE_OBSERVATION"
    assert degraded.target == pytest.approx(0.0)


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


def test_target_jump_translation_uses_rtk_map_base_not_map_odom_lever_arm():
    previous_map_base = Pose2D(x=80.0, y=-10.0, yaw=math.radians(80.0))
    current_map_base = Pose2D(x=80.3, y=-9.9, yaw=math.radians(86.0))
    previous_odom_base = Pose2D(x=67.0, y=0.0, yaw=math.radians(0.0))
    current_odom_base = Pose2D(x=67.0, y=0.0, yaw=math.radians(0.0))

    previous_map_odom = compute_map_to_odom(previous_map_base, previous_odom_base)
    current_map_odom = compute_map_to_odom(current_map_base, current_odom_base)

    map_base_translation_m, map_base_yaw_rad = authority.compute_pose_delta(
        previous_map_base,
        current_map_base,
    )
    map_odom_translation_m, _ = authority.compute_pose_delta(
        previous_map_odom,
        current_map_odom,
    )
    gate_translation_m, gate_yaw_rad = authority.compute_authority_target_delta(
        previous_map_base=previous_map_base,
        current_map_base=current_map_base,
        previous_map_odom=previous_map_odom,
        current_map_odom=current_map_odom,
    )

    assert map_base_translation_m == pytest.approx(math.hypot(0.3, 0.1))
    assert math.degrees(map_base_yaw_rad) == pytest.approx(6.0)
    assert map_odom_translation_m > 6.0
    assert gate_translation_m == pytest.approx(map_base_translation_m)
    assert gate_yaw_rad == pytest.approx(map_base_yaw_rad)


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
    assert "self._last_raw_map_base: Pose2D | None = None" in node_text
    assert "compute_authority_target_delta(" in node_text
    assert "previous_map_base=self._last_raw_map_base" in node_text
    assert "previous_map_odom=self._last_raw_target" in node_text
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
