"""Counterexamples to false PASS on early exit, stalled source and wrong yaw."""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from replay_acceptance import quaternion_distance, requested_prefix_completed, stream_continuity


def stamps(start, end, step=.1):
    return [round((start+index*step)*1e9) for index in range(round((end-start)/step)+1)]


def test_premature_exit_is_not_repaired_by_later_drain_time():
    assert not requested_prefix_completed(85., 100., 99., 1, [None]*3)
    assert not requested_prefix_completed(85., 100., 102., 1, [None]*3)
    assert not requested_prefix_completed(85., 100., 102., None, [None, 1])
    assert requested_prefix_completed(85., 100., 100., None, [None]*3)


def test_fifty_one_good_pairs_followed_by_source_stall_cannot_pass_eof():
    result = stream_continuity(stamps(0., 80.), stamps(0., 80., .005), stamps(2., 7.), stamps(2., 7.))
    assert not result['checks']['native_input_terminal_covered']
    assert not result['checks']['vehicle_input_span_covered']


def test_continuous_source_and_vehicle_cover_real_input_tail():
    result = stream_continuity(stamps(0., 80.), stamps(0., 80., .005), stamps(2., 80.), stamps(2., 80.))
    assert all(result['checks'].values())


def test_lidar_and_source_stall_while_imu_continues_cannot_pass_eof():
    result = stream_continuity(stamps(0., 7.), stamps(0., 80.), stamps(2., 7.), stamps(2., 7.))
    assert not result['checks']['native_input_terminal_covered']
    assert not result['checks']['vehicle_input_terminal_covered']


def test_internal_output_gap_is_not_hidden_by_good_last_stamp():
    output = stamps(2., 7.)+stamps(9., 80.)
    result = stream_continuity(stamps(0., 80.), stamps(0., 80.), output, output)
    assert not result['checks']['native_no_prolonged_output_gap']


def test_identity_mean_comparison_catches_yaw_and_accepts_q_sign_equivalence():
    assert quaternion_distance((0., 0., 0., 1.), (0., 0., 0., -1.)) == 0.
    yaw = .3
    assert abs(quaternion_distance((0., 0., 0., 1.), (0., 0., math.sin(yaw/2), math.cos(yaw/2)))-yaw) < 1e-12
    assert math.isinf(quaternion_distance((0., 0., 0., 0.), (0., 0., 0., 1.)))
