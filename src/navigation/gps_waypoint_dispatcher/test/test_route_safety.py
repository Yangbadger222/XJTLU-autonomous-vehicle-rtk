import pytest
from pathlib import Path

from gps_waypoint_dispatcher.route_safety import (
    is_rcl_context_shutdown_error_message,
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
    assert "odom_watchdog_tf_stale_abort_s" in text
    assert "summarize_tf_watchdog_gap" in text
    assert "NAV2_FALSE_SUCCESS_ABORT" in text
    assert "_verify_nav2_success_progress" in text
    assert "TF_STALE" in text


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
