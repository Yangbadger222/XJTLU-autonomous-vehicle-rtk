import pytest

from gps_waypoint_dispatcher.route_safety import summarize_map_gps_consistency


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
