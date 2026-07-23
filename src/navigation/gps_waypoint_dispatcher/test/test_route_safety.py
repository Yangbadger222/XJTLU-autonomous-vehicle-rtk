import math

import pytest
from pathlib import Path

from gps_waypoint_dispatcher.route_safety import (
    BlockedRetryDecision,
    BlockedRetryState,
    ContinuousReadiness,
    GlobalCorrectionWatchdog,
    LocalOdomWatchdog,
    WatchdogDecision,
    is_rcl_context_shutdown_error_message,
    summarize_nav2_success_progress,
    summarize_map_gps_consistency,
    summarize_tf_freshness,
    summarize_tf_watchdog_gap,
)


ROUTE_RUNNER = (
    Path(__file__).resolve().parents[1]
    / "gps_waypoint_dispatcher"
    / "gps_route_runner_node.py"
)
GLOBAL_ALIGNER = (
    Path(__file__).resolve().parents[1]
    / "gps_waypoint_dispatcher"
    / "gps_global_aligner_node.py"
)


def test_map_gps_consistency_aborts_large_tf_divergence():
    summary = summarize_map_gps_consistency(
        map_xy=(582.57, -870.68),
        gps_map_xy=(-0.20, 0.05),
        warn_m=2.0,
        abort_m=5.0,
    )

    assert summary.ok is False
    assert summary.warn is True
    assert summary.distance_m == pytest.approx(1047.8, abs=0.1)


def test_map_gps_consistency_accepts_small_alignment_error():
    summary = summarize_map_gps_consistency(
        map_xy=(10.0, -4.0),
        gps_map_xy=(11.0, -5.0),
        warn_m=2.0,
        abort_m=5.0,
    )

    assert summary.ok is True
    assert summary.warn is False
    assert summary.distance_m == pytest.approx(2 ** 0.5)


def test_tf_freshness_rejects_old_pose_samples():
    summary = summarize_tf_freshness(now_s=1783342965.48, stamp_s=1783342960.30, max_age_s=0.75)

    assert summary.ok is False
    assert summary.age_s == pytest.approx(5.18, abs=0.01)


def test_tf_freshness_accepts_recent_pose_samples():
    summary = summarize_tf_freshness(now_s=1783342965.48, stamp_s=1783342965.36, max_age_s=0.75)

    assert summary.ok is True
    assert summary.age_s == pytest.approx(0.12, abs=0.01)


def test_route_runner_uses_tf_freshness_gate_before_pose_use():
    text = ROUTE_RUNNER.read_text(encoding="utf-8")

    assert "tf_pose_max_age_s" in text
    assert "summarize_tf_freshness" in text
    assert "LocalOdomWatchdog" in text
    assert "GlobalCorrectionWatchdog" in text
    assert "NAV2_FALSE_SUCCESS_ABORT" in text
    assert "_verify_nav2_success_progress" in text
    assert "_authority_age_s" in text


def test_nav2_success_progress_allows_near_goal_shortfall_with_guard_tolerance():
    summary = summarize_nav2_success_progress(
        target_progress_m=15.77,
        verified_progress_m=15.29,
        waypoint_tolerance_m=0.35,
        success_shortfall_tolerance_m=0.75,
    )

    assert summary.ok is True
    assert summary.shortfall_m == pytest.approx(0.48, abs=0.01)
    assert summary.tolerance_m == pytest.approx(0.75)


def test_nav2_success_progress_rejects_no_progress_false_success():
    summary = summarize_nav2_success_progress(
        target_progress_m=13.71,
        verified_progress_m=0.0,
        waypoint_tolerance_m=0.35,
        success_shortfall_tolerance_m=0.75,
    )

    assert summary.ok is False
    assert summary.shortfall_m == pytest.approx(13.71)


def test_route_runner_has_independent_nav2_success_shortfall_tolerance():
    text = ROUTE_RUNNER.read_text(encoding="utf-8")

    assert '"nav2_success_shortfall_tolerance_m"' in text
    assert "summarize_nav2_success_progress" in text


def test_route_runner_holds_zero_cmd_before_success_status():
    text = ROUTE_RUNNER.read_text(encoding="utf-8")
    stop_index = text.index('self._publish_status("STOPPING_BEFORE_EXIT")')
    hold_index = text.index("self._publish_terminal_stop_hold()", stop_index)
    success_index = text.index('self._publish_status("SUCCEEDED")', hold_index)

    assert 'self.declare_parameter("terminal_stop_hold_s", 1.2)' in text
    assert 'self.declare_parameter("terminal_stop_publish_hz", 20.0)' in text
    assert "def _publish_terminal_stop_hold(self) -> None:" in text
    assert stop_index < hold_index < success_index


def test_rcl_context_shutdown_error_is_not_reported_as_aligner_abort():
    text = GLOBAL_ALIGNER.read_text(encoding="utf-8")
    message = (
        "failed to initialize wait set: the given context is not valid, "
        "either rcl_init() was not called or rcl_shutdown() was called."
    )

    assert is_rcl_context_shutdown_error_message(message) is True
    assert is_rcl_context_shutdown_error_message("serial port failed") is False
    assert "is_rcl_context_shutdown_error_message" in text
    assert "ALIGNER_ABORTED" in text


def test_tf_watchdog_waits_for_stale_grace_duration():
    summary = summarize_tf_watchdog_gap(
        stale_count=3,
        stale_elapsed_s=0.3,
        abort_count=3,
        abort_after_s=2.0,
    )

    assert summary.abort is False
    assert summary.reason is None


def test_tf_watchdog_aborts_after_count_and_grace_duration():
    summary = summarize_tf_watchdog_gap(
        stale_count=3,
        stale_elapsed_s=2.1,
        abort_count=3,
        abort_after_s=2.0,
    )

    assert summary.abort is True
    assert summary.reason == "TF_STALE_2.10s"


def test_local_odom_watchdog_ignores_duplicate_stamps():
    watchdog = LocalOdomWatchdog()
    assert watchdog.update(1.0, 0.0, 0.0, 0.0).decision is WatchdogDecision.OK

    duplicate = watchdog.update(1.0, 100.0, 0.0, 0.0)

    assert duplicate.decision is WatchdogDecision.IGNORE
    assert duplicate.reason == "DUPLICATE_STAMP"


@pytest.mark.parametrize(
    ("stamp_s", "x", "y", "yaw", "reason"),
    [
        (0.9, 0.1, 0.0, 0.0, "REGRESSING_STAMP"),
        (1.1, math.nan, 0.0, 0.0, "NONFINITE_LOCAL_ODOM"),
        (1.1, 0.0, math.inf, 0.0, "NONFINITE_LOCAL_ODOM"),
        (1.1, 0.0, 0.0, math.nan, "NONFINITE_LOCAL_ODOM"),
    ],
)
def test_local_odom_watchdog_catastrophic_samples_abort_immediately(
    stamp_s, x, y, yaw, reason
):
    watchdog = LocalOdomWatchdog()
    watchdog.update(1.0, 0.0, 0.0, 0.0)

    result = watchdog.update(stamp_s, x, y, yaw)

    assert result.decision is WatchdogDecision.LOCAL_ABORT
    assert result.reason == reason


@pytest.mark.parametrize(
    ("x", "yaw", "reason"),
    [
        (1.01, 0.0, "CATASTROPHIC_LINEAR_RATE"),
        (0.0, 1.01, "CATASTROPHIC_YAW_RATE"),
    ],
)
def test_local_odom_watchdog_aborts_one_sample_above_catastrophic_rate(
    x, yaw, reason
):
    watchdog = LocalOdomWatchdog()
    watchdog.update(1.0, 0.0, 0.0, 0.0)

    result = watchdog.update(1.1, x, 0.0, yaw)

    assert result.decision is WatchdogDecision.LOCAL_ABORT
    assert result.reason == reason


def test_local_odom_watchdog_latches_abort_across_following_good_sample():
    watchdog = LocalOdomWatchdog()
    watchdog.update(1.0, 0.0, 0.0, 0.0)
    fault = watchdog.update(1.1, 1.01, 0.0, 0.0)

    following = watchdog.update(1.2, 1.02, 0.0, 0.0)

    assert fault.decision is WatchdogDecision.LOCAL_ABORT
    assert following == fault


def test_local_odom_watchdog_requires_three_ordinary_rate_violations():
    watchdog = LocalOdomWatchdog()
    watchdog.update(1.0, 0.0, 0.0, 0.0)

    first = watchdog.update(1.1, 0.31, 0.0, 0.0)
    second = watchdog.update(1.2, 0.62, 0.0, 0.0)
    third = watchdog.update(1.3, 0.93, 0.0, 0.0)

    assert first.decision is WatchdogDecision.OK
    assert second.decision is WatchdogDecision.OK
    assert third.decision is WatchdogDecision.LOCAL_ABORT
    assert third.reason == "REPEATED_LOCAL_RATE"


def test_local_odom_watchdog_good_sample_resets_ordinary_counter():
    watchdog = LocalOdomWatchdog()
    watchdog.update(1.0, 0.0, 0.0, 0.0)
    watchdog.update(1.1, 0.31, 0.0, 0.0)
    watchdog.update(1.2, 0.62, 0.0, 0.0)
    good = watchdog.update(1.3, 0.63, 0.0, 0.0)
    after_reset = watchdog.update(1.4, 0.94, 0.0, 0.0)

    assert good.decision is WatchdogDecision.OK
    assert after_reset.decision is WatchdogDecision.OK


def test_global_correction_rate_causes_hold_not_local_abort():
    watchdog = GlobalCorrectionWatchdog()
    watchdog.update(1.0, 0.0, 0.0, 0.0, authority_allowed=True, authority_age_s=0.0)

    result = watchdog.update(
        1.1,
        0.06,
        0.0,
        math.radians(0.6),
        authority_allowed=True,
        authority_age_s=0.0,
    )

    assert result.decision is WatchdogDecision.GLOBAL_HOLD
    assert result.reason == "GLOBAL_CORRECTION_RATE"


def test_global_correction_vehicle_space_step_ignores_far_origin_yaw_lever_arm():
    watchdog = GlobalCorrectionWatchdog()
    watchdog.update_step(
        1.0,
        0.0,
        0.0,
        authority_allowed=True,
        authority_age_s=0.0,
    )

    # A 2 deg/s yaw correction at a 240 m odom lever arm looks like about
    # 8.4 m/s in raw map->odom translation. The actual base-space correction
    # remains 0.02 m and 0.2 deg in this 100 ms release step.
    result = watchdog.update_step(
        1.1,
        0.02,
        math.radians(0.2),
        authority_allowed=True,
        authority_age_s=0.0,
    )

    assert result.decision is WatchdogDecision.OK
    assert result.linear_rate_mps == pytest.approx(0.2)
    assert math.degrees(result.yaw_rate_radps) == pytest.approx(2.0)


def test_global_correction_vehicle_space_step_holds_real_jump():
    watchdog = GlobalCorrectionWatchdog()
    watchdog.update_step(
        1.0,
        0.0,
        0.0,
        authority_allowed=True,
        authority_age_s=0.0,
    )

    result = watchdog.update_step(
        1.1,
        0.06,
        0.0,
        authority_allowed=True,
        authority_age_s=0.0,
    )

    assert result.decision is WatchdogDecision.GLOBAL_HOLD
    assert result.reason == "GLOBAL_CORRECTION_RATE"


@pytest.mark.parametrize(
    ("allowed", "age_s", "reason"),
    [
        (False, 0.0, "MOTION_AUTHORITY_FALSE"),
        (True, 0.51, "MOTION_AUTHORITY_STALE"),
    ],
)
def test_global_correction_watchdog_holds_on_false_or_stale_authority(
    allowed, age_s, reason
):
    watchdog = GlobalCorrectionWatchdog()

    result = watchdog.update(
        1.0, 0.0, 0.0, 0.0, authority_allowed=allowed, authority_age_s=age_s
    )

    assert result.decision is WatchdogDecision.GLOBAL_HOLD
    assert result.reason == reason


def test_route_runner_owns_stop_override_not_cmd_vel():
    text = ROUTE_RUNNER.read_text(encoding="utf-8")

    assert '"/gps_corridor/stop_override"' in text
    assert "create_publisher(Twist" not in text
    assert "cancel_goal_async()" in text
    assert "goals_canceling" in text
    assert '"GLOBAL_CORRECTION_HOLD"' in text
    assert 'self.declare_parameter("global_hold_timeout_s", 15.0)' in text
    assert 'self.declare_parameter("authority_ready_confirmation_s", 1.0)' in text
    assert 'self._tf_buffer.lookup_transform(' in text
    assert '"odom",\n                self._base_frame' in text
    assert "update_step(" in text


def test_authority_readiness_must_be_continuous_for_one_second():
    readiness = ContinuousReadiness(confirmation_s=1.0)

    assert readiness.update(ready=True, now_s=10.0) is False
    assert readiness.update(ready=True, now_s=10.9) is False
    assert readiness.update(ready=False, now_s=10.95) is False
    assert readiness.update(ready=True, now_s=11.0) is False
    assert readiness.update(ready=True, now_s=12.0) is True


def test_blocked_retry_waits_then_retries_only_with_authority():
    blocked = BlockedRetryState(
        retry_delay_s=2.0,
        timeout_s=60.0,
        recovery_confirmation_s=3.0,
    )

    entered = blocked.enter(10.0)
    no_authority = blocked.poll(12.5, retry_allowed=False)
    retry = blocked.poll(12.5, retry_allowed=True)

    assert entered.decision is BlockedRetryDecision.WAIT
    assert no_authority.decision is BlockedRetryDecision.WAIT
    assert retry.decision is BlockedRetryDecision.RETRY
    assert retry.retry_count == 1
    assert retry.elapsed_s == pytest.approx(2.5)


def test_blocked_retry_preserves_timeout_across_repeated_aborts():
    blocked = BlockedRetryState(retry_delay_s=2.0, timeout_s=10.0)
    blocked.enter(10.0)
    blocked.poll(12.0, retry_allowed=True)

    reentered = blocked.enter(12.2)
    second_retry = blocked.poll(14.2, retry_allowed=True)
    timeout = blocked.poll(20.0, retry_allowed=True)

    assert reentered.elapsed_s == pytest.approx(2.2)
    assert second_retry.retry_count == 2
    assert timeout.decision is BlockedRetryDecision.TIMEOUT
    assert timeout.retry_count == 2


def test_blocked_retry_clears_after_retry_runs_stably():
    blocked = BlockedRetryState(
        retry_delay_s=2.0,
        timeout_s=60.0,
        recovery_confirmation_s=3.0,
    )
    blocked.enter(10.0)
    blocked.poll(12.0, retry_allowed=True)

    assert blocked.confirm_action_running(13.0, moving=False) is False
    assert blocked.confirm_action_running(14.0, moving=True) is False
    assert blocked.confirm_action_running(16.9, moving=True) is False
    assert blocked.confirm_action_running(17.0, moving=True) is True
    assert blocked.active is False
    assert blocked.poll(16.0, retry_allowed=True).decision is BlockedRetryDecision.INACTIVE


def test_blocked_retry_requires_continuous_motion_for_recovery():
    blocked = BlockedRetryState(
        retry_delay_s=1.0,
        timeout_s=60.0,
        recovery_confirmation_s=2.0,
    )
    blocked.enter(10.0)
    blocked.poll(11.0, retry_allowed=True)

    assert blocked.confirm_action_running(12.0, moving=True) is False
    assert blocked.confirm_action_running(13.0, moving=False) is False
    assert blocked.confirm_action_running(14.0, moving=True) is False
    assert blocked.confirm_action_running(16.0, moving=True) is True
