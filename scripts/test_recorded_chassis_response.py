"""Independent excitation/clock checks for exploratory response estimates."""
import math
import json
from types import SimpleNamespace

import numpy as np

from audit_recorded_chassis_response import analyze_serial, fit_recorded_response, recorded_zero_responses, stats, yaw


def test_identifies_known_gain_and_recording_delay():
    time = np.arange(0., 12., .05)
    signal = .2+.15*np.sin(time*3.)+.1*np.cos(time*7.)
    commands = np.column_stack((time, signal, signal))
    stamps = time[20:-1]
    delayed = np.searchsorted(time, stamps-.2, side='right')-1
    measured = np.column_stack((stamps, .8*signal[delayed], .8*signal[delayed]))
    result = fit_recorded_response(commands, measured, 1)
    assert result['status'] == 'EXPLORATORY_RECORDING_CLOCK_FIT'
    assert abs(result['gain']-.8) < .03
    assert abs(result['recording_lag_s']-.2) < .03
    assert result['rmse'] < .02


def test_constant_command_does_not_identify_delay():
    t = np.arange(0., 3., .05)
    rows = np.column_stack((t, np.ones(len(t))*.55, np.zeros(len(t))))
    assert fit_recorded_response(rows, rows, 1)['status'] == 'INSUFFICIENT_EXCITATION'


def test_disjoint_recording_clocks_are_not_fitted():
    t = np.arange(0., 3., .05)
    rows = np.column_stack((t, .2+.1*np.sin(t*8), np.zeros(len(t))))
    late = rows.copy()
    late[:, 0] += 1000
    assert fit_recorded_response(rows, late, 1)['status'] == 'INSUFFICIENT_EXCITATION'


def test_rejects_nonunit_pose_orientation_and_nonfinite_statistics():
    assert math.isnan(yaw(SimpleNamespace(x=0, y=0, z=0, w=0)))
    assert stats([1., math.nan, math.inf, 3.]) == {'count': 2, 'min': 1., 'median': 2., 'p95': 2.9, 'max': 3.}


def test_nonfinite_commands_and_states_cannot_produce_a_successful_nan_fit():
    t = np.arange(0., 3., .05)
    commands = np.column_stack((t, np.full(len(t), math.inf), np.zeros(len(t))))
    states = np.column_stack((t, np.full(len(t), .2), np.zeros(len(t))))
    result = fit_recorded_response(commands, states, 1)
    assert result['status'] == 'INSUFFICIENT_SAMPLES'
    assert result['excluded_nonfinite_command_rows'] == len(t)
    json.dumps(result, allow_nan=False)
    commands[:, 1] = .2+.1*np.sin(t*8)
    states[:, 1] = math.nan
    assert fit_recorded_response(commands, states, 1)['excluded_nonfinite_state_rows'] == len(t)


def test_serial_forward_axis_and_gyro_sign_follow_actual_firmware_layout(tmp_path):
    tx, rx = [], []
    for i in range(120):
        stamp = 1000000000+i*50000000
        commanded = .3+.1*math.sin(i*.7)
        tx.append(f'ROS_timestamp: {stamp}, [SERIAL_TX] Sending command (1/1): vcx={commanded},wc={commanded}')
        # Source fixture: the encoder speed is in Y; the raw gyro sign differs from legacy odom.
        rx.append(f'linear_vel(0, {commanded*.8}, 0), angular_vel(0, 0, {-commanded*.6}), timestamp: {stamp}')
    (tmp_path/'serial_twistctl.log').write_text('\n'.join(tx)+'\n')
    (tmp_path/'serial_reader.log').write_text('\n'.join(rx)+'\n')
    result = analyze_serial(tmp_path)
    assert result['feedback_nonzero_forward_y_count'] == 120
    assert result['feedback_nonzero_placeholder_x_count'] == 0
    assert math.isclose(result['forward_fit_to_final_tx']['gain'], .8)
    assert math.isclose(result['gyro_fit_to_final_tx_legacy_odom_sign']['gain'], .6)


def measured_segments(record_dt=.1, header_dt=.1, v=0., lateral=0., count=12):
    return np.asarray([((i+.5)*record_dt, v, 0., lateral, i*header_dt, (i+1)*header_dt,
                        v*header_dt, i, i*record_dt, (i+1)*record_dt) for i in range(count)])


def test_rotation_zero_transition_is_included_and_same_stamp_order_preserved():
    # At one recorded stamp, a nonzero command followed by zero remains a stop.
    commands = np.asarray([(0., 0., .5), (0., 0., 0.)])
    result = recorded_zero_responses(commands, measured_segments())
    assert len(result) == 1
    assert result[0]['excitation'] == 'rotation'
    assert result[0]['quiet_measurement_window']['covered_measurement_seconds'] >= 1.


def test_quiet_requires_a_full_second_of_source_measurements_and_planar_stillness():
    commands = np.asarray([(0., .5, 0.), (0., 0., 0.)])
    assert recorded_zero_responses(commands, measured_segments(header_dt=.03))[0]['quiet_measurement_window'] is None
    assert recorded_zero_responses(commands, measured_segments(lateral=.2))[0]['quiet_measurement_window'] is None
    gap = measured_segments(count=20)
    gap[9:, 7] += 1  # Rejected pose interval: neither side covers one second.
    gap[9:, 4:6] += .1
    gap = gap[:18]
    assert recorded_zero_responses(commands, gap)[0]['quiet_measurement_window'] is None


def test_measured_distance_is_independent_of_recording_clock_rate():
    commands = np.asarray([(0., .5, 0.), (0., 0., 0.)])
    result = recorded_zero_responses(commands, measured_segments(record_dt=.1, header_dt=.02, v=.4, lateral=.3))
    assert math.isclose(result[0]['sampled_longitudinal_distance_after_zero_m'], .4*.02*12)
    assert math.isclose(result[0]['sampled_planar_distance_after_zero_m'], .5*.02*12)
