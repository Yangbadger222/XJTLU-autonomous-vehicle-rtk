#!/usr/bin/env python3
"""Read-only command/LIO/serial evidence; estimates never enable motion.

Firmware equations define the command interface. Recorded response does not
identify physical wheel radius, track, gear, slip and delay independently.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import sqlite3

import numpy as np
import yaml


def stats(values):
    a = np.asarray(values, dtype=float)
    a = a[np.isfinite(a)]
    if not len(a):
        return {'count': 0}
    return {'count': len(a), 'min': float(a.min()), 'median': float(np.median(a)),
            'p95': float(np.percentile(a, 95)), 'max': float(a.max())}


def yaw(q):
    norm = math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
    if not math.isfinite(norm) or abs(norm-1.) > .02:
        return math.nan
    x, y, z, w = q.x/norm, q.y/norm, q.z/norm, q.w/norm
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def fit_recorded_response(commands, states, axis):
    """Recording-clock association; not a hardware latency measurement."""
    command_valid = np.isfinite(commands[:, [0, axis]]).all(axis=1)
    state_valid = np.isfinite(states[:, [0, axis]]).all(axis=1)
    exclusions = {'excluded_nonfinite_command_rows': int((~command_valid).sum()),
                  'excluded_nonfinite_state_rows': int((~state_valid).sum())}
    commands = commands[command_valid]
    states = states[state_valid]
    commands = commands[np.argsort(commands[:, 0], kind='stable')]
    if len(commands) < 20 or len(states) < 20:
        return dict(status='INSUFFICIENT_SAMPLES', **exclusions)
    best = None
    for lag in np.arange(0., .501, .025):
        index = np.searchsorted(commands[:, 0], states[:, 0]-lag, side='right')-1
        clipped = np.clip(index, 0, len(commands)-1)
        x = commands[clipped, axis]
        y = states[:, axis]
        age = states[:, 0]-lag-commands[clipped, 0]
        valid = (index >= 0) & (age >= 0) & (age <= .25) & (np.abs(x) >= .05)
        # More than a constant command level is needed to identify delay/gain.
        with np.errstate(over='ignore', invalid='ignore'):
            excitation = float(np.std(x[valid])) if valid.any() else 0.
        if valid.sum() < 20 or not math.isfinite(excitation) or excitation < .02:
            continue
        a, b = x[valid], y[valid]
        with np.errstate(over='ignore', invalid='ignore', divide='ignore'):
            gain = float(a.dot(b)/a.dot(a))
            rmse = float(np.sqrt(np.mean((b-gain*a)**2)))
        if not math.isfinite(gain) or not math.isfinite(rmse):
            continue
        candidate = {'gain': gain, 'recording_lag_s': float(lag), 'rmse': rmse,
                     'sample_count': int(valid.sum()), 'command_range': [float(a.min()), float(a.max())]}
        if best is None or rmse < best['rmse']:
            best = candidate
    return dict(status='EXPLORATORY_RECORDING_CLOCK_FIT', **exclusions, **best) if best else dict(status='INSUFFICIENT_EXCITATION', **exclusions)


def recorded_zero_responses(commands, states):
    """Associate zero commands by record clock; assess motion by header intervals.

    State columns: record midpoint, longitudinal v, yaw rate, lateral v,
    header start/end, longitudinal displacement, original pose-interval index,
    record start/end. No distance or covered-duration uses the record clock.
    """
    events = []
    for i in range(1, len(commands)):
        previous, current = commands[i-1], commands[i]
        if not np.isfinite([*previous, *current]).all():
            continue
        longitudinal = abs(previous[1]) > .05
        rotation = abs(previous[2]) > math.radians(2)
        if not (longitudinal or rotation) or np.max(np.abs(current[1:])) > .001:
            continue
        t = current[0]
        next_move = next((c[0] for c in commands[i+1:] if
                          not np.isfinite(c).all() or np.max(np.abs(c[1:])) > .001), t+5.)
        window = states[(states[:, 8] >= t) & (states[:, 9] <= min(next_move, t+5.))]
        quiet_window = None
        for j, item in enumerate(window):
            covered = []
            for segment in window[j:]:
                if math.hypot(segment[1], segment[3]) > .05 or abs(segment[2]) > math.radians(2):
                    break
                if segment[5]-segment[4] > .2:
                    break
                if covered and (segment[7] != covered[-1][7]+1 or abs(segment[4]-covered[-1][5]) > 1e-6):
                    break
                covered.append(segment)
                if len(covered) >= 9 and segment[5]-item[4] >= 1.:
                    quiet_window = {'header_start_s': float(item[4]), 'header_end_s': float(segment[5]),
                                    'covered_measurement_seconds': float(segment[5]-item[4]),
                                    'recording_seconds_to_window_start': float(item[8]-t)}
                    break
            if quiet_window is not None:
                break
        events.append({'excitation': 'longitudinal_and_rotation' if longitudinal and rotation else
                                    'longitudinal' if longitudinal else 'rotation',
                       'measurement_samples': len(window), 'quiet_measurement_window': quiet_window,
                       'sampled_longitudinal_distance_after_zero_m': float(np.abs(window[:, 6]).sum()),
                       'sampled_planar_distance_after_zero_m': float(np.sum(np.hypot(window[:, 1], window[:, 3])*(window[:, 5]-window[:, 4]))),
                       'scope': 'Full measured intervals associated after recorded zero; gaps excluded; no hardware stop/button label'})
    return events


def analyze_bag(path):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    metadata = path/'metadata.yaml'
    info = yaml.safe_load(metadata.read_text())['rosbag2_bagfile_information']
    commands, poses, digest = [], [], hashlib.sha256()
    selected_counts = {}
    for name in info['relative_file_paths']:
        db_path = (path/name).resolve()
        with sqlite3.connect(db_path.as_uri()+'?mode=ro', uri=True) as db:
            topics = {row[0]: (row[1], row[2]) for row in db.execute('SELECT id,name,type FROM topics')}
            ids = [key for key, (topic, _) in topics.items() if topic in ('/cmd_vel', '/fastlio2/lio_odom')]
            if not ids:
                continue
            placeholders = ','.join('?' for _ in ids)
            for topic_id, record_stamp, blob in db.execute(
                    'SELECT topic_id,timestamp,data FROM messages WHERE topic_id IN ('+placeholders+') ORDER BY timestamp,id', ids):
                topic, kind = topics[topic_id]
                selected_counts[topic] = selected_counts.get(topic, 0)+1
                digest.update(topic.encode()+b'\0'+record_stamp.to_bytes(8, 'little', signed=True)+blob)
                m = deserialize_message(blob, get_message(kind))
                t = record_stamp/1e9
                if topic == '/cmd_vel':
                    commands.append((t, m.linear.x, m.angular.z))
                else:
                    p = m.pose.pose.position
                    poses.append((t, m.header.stamp.sec+m.header.stamp.nanosec/1e9, p.x, p.y, yaw(m.pose.pose.orientation)))
    # Keep database/message order for equal stamps; tuple-value sorting changes the final command.
    commands = np.asarray(sorted(commands, key=lambda row: row[0]), dtype=float).reshape((-1, 3))
    poses = np.asarray(sorted(poses, key=lambda row: row[0]), dtype=float).reshape((-1, 5))
    states, rejected = [], {'nonfinite': 0, 'measurement_interval': 0, 'pose_step': 0}
    for interval_index, (previous, current) in enumerate(zip(poses[:-1], poses[1:])):
        if not np.isfinite([*previous, *current]).all():
            rejected['nonfinite'] += 1
            continue
        dt = current[1]-previous[1]
        dx, dy = current[2]-previous[2], current[3]-previous[3]
        angle = math.atan2(math.sin(current[4]-previous[4]), math.cos(current[4]-previous[4]))
        if not .02 <= dt <= .3:
            rejected['measurement_interval'] += 1
            continue
        if math.hypot(dx, dy) > .5 or abs(angle) > math.radians(15):
            rejected['pose_step'] += 1
            continue
        heading = previous[4]+angle/2
        v = (dx*math.cos(heading)+dy*math.sin(heading))/dt
        lateral = (-dx*math.sin(heading)+dy*math.cos(heading))/dt
        states.append(((previous[0]+current[0])/2, v, angle/dt, lateral,
                       previous[1], current[1], v*dt, interval_index, previous[0], current[0]))
    states = np.asarray(states, dtype=float).reshape((-1, 10))
    moving = np.abs(states[:, 1]) > .05
    stop_events = recorded_zero_responses(commands, states)
    result = {'bag_path': str(path), 'selected_cdr_sha256': digest.hexdigest(),
              'metadata_sha256': hashlib.sha256(metadata.read_bytes()).hexdigest(),
              'counts': selected_counts, 'rejected_pose_intervals': rejected, 'measurement_intervals': len(states),
              'nonfinite_command_rows': int((~np.isfinite(commands).all(axis=1)).sum()),
              'equal_record_stamp_command_pairs': int((np.diff(commands[:, 0]) == 0).sum()),
              'command_abs_v': stats(np.abs(commands[:, 1])), 'command_abs_w': stats(np.abs(commands[:, 2])),
              'measured_abs_v': stats(np.abs(states[:, 1])), 'measured_abs_w': stats(np.abs(states[:, 2])),
              'legacy_imu_reference_abs_lateral_mps': stats(np.abs(states[:, 3])),
              'observed_abs_curvature_for_v_gt_0_05': stats(np.abs(states[moving, 2]/states[moving, 1])),
              'linear_fit': fit_recorded_response(commands, states, 1),
              'yaw_fit': fit_recorded_response(commands, states, 2), 'zero_command_responses': stop_events,
              'stamp_minus_record_s': stats(poses[:, 1]-poses[:, 0])}
    return result


def analyze_serial(directory):
    tx = directory/'serial_twistctl.log'
    rx = directory/'serial_reader.log'
    tx_rows = []
    for line in tx.read_text().splitlines():
        m = re.search(r'ROS_timestamp: (\d+).*vcx=([^,]+),wc=([^\s]+)', line)
        if m:
            tx_rows.append([int(m[1])/1e9, float(m[2]), float(m[3])])
    feedback, parse_rejected = [], 0
    for line in rx.read_text().splitlines():
        m = re.search(r'linear_vel\(([^)]+)\).*angular_vel\(([^)]+)\).*timestamp: (\d+)', line)
        if m:
            try:
                linear = [float(v) for v in m[1].split(',')]
                angular = [float(v) for v in m[2].split(',')]
                if len(linear) != 3 or len(angular) != 3:
                    raise ValueError('wrong vector size')
                feedback.append([int(m[3])/1e9, *linear, *angular])
            except ValueError:
                parse_rejected += 1
    tx_array = np.asarray(tx_rows, dtype=float).reshape((-1, 3))
    raw = np.asarray(feedback, dtype=float).reshape((-1, 7))
    # main.c Serial_Output sends [0, real_vc, 0] and gyro, not MOTORrpm2vw's real_w.
    finite = np.isfinite(raw).all(axis=1)
    layout_ok = finite & (np.abs(raw[:, 1]) <= 1e-6) & (np.abs(raw[:, 3]) <= 1e-6)
    forward_ok = layout_ok & (np.abs(raw[:, 2]) <= 3.)
    gyro_ok = layout_ok & (np.abs(raw[:, 6]) <= 10.)
    linear_states = np.column_stack((raw[forward_ok, 0], raw[forward_ok, 2], -raw[forward_ok, 6]))
    gyro_states = np.column_stack((raw[gyro_ok, 0], raw[gyro_ok, 2], -raw[gyro_ok, 6]))
    return {'directory': str(directory), 'tx_count': len(tx_rows), 'feedback_count': len(feedback),
            'parse_rejected_rows': parse_rejected, 'nonfinite_feedback_rows': int((~finite).sum()),
            'nonfinite_tx_rows': int((~np.isfinite(tx_array).all(axis=1)).sum()),
            'feedback_nonzero_forward_y_count': int((np.abs(raw[:, 2]) > 1e-6).sum()),
            'feedback_nonzero_placeholder_x_count': int((np.abs(raw[:, 1]) > 1e-6).sum()),
            'nonzero_expected_zero_xz_count': int((finite & ~layout_ok).sum()),
            'feedback_forward_y_abs_above_3_count': int((np.abs(raw[:, 2]) > 3.).sum()),
            'raw_gyro_z_abs_above_10_count': int((np.abs(raw[:, 6]) > 10.).sum()),
            'raw_abs_forward_y_mps': stats(np.abs(raw[:, 2])),
            'raw_abs_gyro_z_rps': stats(np.abs(raw[:, 6])),
            'forward_fit_to_final_tx': fit_recorded_response(tx_array, linear_states, 1),
            'gyro_fit_to_final_tx_legacy_odom_sign': fit_recorded_response(tx_array, gyro_states, 2),
            'field_semantics': {'forward': 'linear_vel.y = real_vc from MOTORrpm2vw',
                                'angular': 'angular_vel.z = gyroscope.z rad/s; legacy odom negates and LPF alpha .3; IMU topic keeps raw sign',
                                'wheel_yaw': 'MOTORrpm2vw real_w is not serialized; theoretical wheel ratio .929684 is not a correction to gyro'},
            'exploratory_quality_screen': {'expected_zero_xz_tolerance': 1e-6, 'abs_forward_y_mps': 3., 'abs_raw_gyro_z_rps': 10.,
                                           'forward_samples': int(forward_ok.sum()), 'gyro_samples': int(gyro_ok.sum()),
                                           'scope': 'Analyst rejection bands for these fits; no change to vehicle safety limits or reader'},
            'final_original_serial_commands': [[value if math.isfinite(value) else None for value in row] for row in tx_rows[-5:]],
            'log_sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in (tx, rx)},
            'qualification': 'EXPLORATORY_ONLY: forward feedback is present in Y; retain layout/speed/gyro outliers, no four raw wheel RPM or flashed identity. Coupled response can be estimated; individual wheel/track/slip/brake calibration is not certified'}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--catalog', type=Path, required=True)
    parser.add_argument('--serial-log-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    catalog = json.loads(args.catalog.read_text())
    selected = [Path(row['bag_path']) for row in catalog['bags'] if
                row.get('topics', {}).get('/cmd_vel', {}).get('count', 0) > 0 and
                row.get('topics', {}).get('/fastlio2/lio_odom', {}).get('count', 0) > 0]
    results = []
    for path in selected:
        try:
            results.append(analyze_bag(path))
            print('AUDITED', path, flush=True)
        except (OSError, ValueError, sqlite3.Error) as exc:
            results.append({'bag_path': str(path), 'error': str(exc)})
    serial = [analyze_serial(path/'data') for path in sorted(args.serial_log_root.iterdir())
              if (path/'data/serial_reader.log').is_file() and (path/'data/serial_twistctl.log').is_file()]
    data = {'status': 'PASS' if results and all('error' not in row for row in results) else 'FAIL',
            'scope': 'Read-only recorded command/LIO response audit and original serial-log qualification; no runtime parameters or motor output changed',
            'selection': 'Every catalog bag with nonempty cmd_vel and FAST odom; source time for pose derivatives, recording clock only for command association',
            'analysis_settings': {'pose_dt_s': [.02, .3], 'pose_step_m': .5, 'pose_step_deg': 15,
                                  'fit_lag_scan_s': [0., .5, .025], 'command_age_s': .25,
                                  'quiet_planar_speed_mps': .05, 'quiet_yaw_rps': math.radians(2),
                                  'quiet_header_coverage_s': 1., 'quiet_min_intervals': 9,
                                  'quiet_max_header_interval_s': .2,
                                  'prior_longitudinal_excitation_mps': .05, 'prior_rotation_excitation_rps': math.radians(2)},
            'limitations': ['cmd_vel is not proof of final MCU acceptance or motor targets; actual TX logs supplement only two July7 runs',
                            'FAST odom derivatives are not independent ground truth; IMU-origin lateral velocity includes a lever arm',
                            'recording-clock fit includes unmeasured transport latency and manual ownership; not physical actuator delay',
                            'observed extrema do not authorize new safety limits, curvature/jerk caps or minimum guaranteed braking'],
            'bag_count': len(results), 'bags': results, 'serial_logs': serial}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(data, indent=2, allow_nan=False)+'\n')
    return 0 if data['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
