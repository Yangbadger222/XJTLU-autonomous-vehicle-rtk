"""Strict replay evidence checks; these are test tolerances, not vehicle limits."""
import math


def requested_prefix_completed(duration_s, deadline, loop_exit_time, player_exit, child_exits):
    return (duration_s > 0 and loop_exit_time >= deadline and player_exit is None
            and all(value is None for value in child_exits))


def quaternion_distance(a, b):
    norms = [math.sqrt(sum(value * value for value in q)) for q in (a, b)]
    if not all(math.isfinite(value) and value > 1e-12 for value in norms):
        return math.inf
    dot = abs(sum(x * y for x, y in zip(a, b)) / (norms[0] * norms[1]))
    return 2 * math.acos(min(1., dot))


def stream_continuity(raw_lidar, raw_imu, native, vehicle, *, tolerance_s=.5,
                      startup_allowance_s=5., minimum_span_ratio=.95):
    streams = {name: sorted(set(values)) for name, values in (
        ('raw_lidar', raw_lidar), ('raw_imu', raw_imu), ('native', native), ('vehicle', vehicle))}
    summaries = {}
    for name, values in streams.items():
        summaries[name] = {'count': len(values), 'first_ns': values[0] if values else None,
                           'last_ns': values[-1] if values else None,
                           'span_s': (values[-1] - values[0]) * 1e-9 if values else 0.,
                           'max_gap_s': max((b-a for a, b in zip(values, values[1:])), default=0) * 1e-9}
    checks = {}
    for name in ('native', 'vehicle'):
        output, lidar, imu = summaries[name], summaries['raw_lidar'], summaries['raw_imu']
        present = all(item['count'] > 1 for item in (output, lidar, imu))
        start_lag = (output['first_ns'] - lidar['first_ns']) * 1e-9 if present else math.inf
        tail_lag = (lidar['last_ns'] - output['last_ns']) * 1e-9 if present else math.inf
        # End-of-packet time may be ahead of its header, but never seconds ahead
        # of the actual observed IMU/header stream. Keep the allowance explicit.
        future_lag = (output['last_ns'] - imu['last_ns']) * 1e-9 if present else math.inf
        expected_span = (lidar['last_ns'] - output['first_ns']) * 1e-9 if present else math.inf
        span_ratio = output['span_s'] / expected_span if present and expected_span > 0 else 0.
        output.update(start_lag_s=start_lag, raw_tail_lag_s=tail_lag,
                      ahead_of_last_imu_s=future_lag, post_start_span_ratio=span_ratio)
        checks[name+'_input_terminal_covered'] = present and abs(tail_lag) <= tolerance_s and abs(future_lag) <= tolerance_s
        checks[name+'_input_span_covered'] = present and -tolerance_s <= start_lag <= startup_allowance_s and span_ratio >= minimum_span_ratio
        checks[name+'_no_prolonged_output_gap'] = present and output['max_gap_s'] <= tolerance_s
    return {'checks': checks, 'streams': summaries,
            'tolerances': {'terminal_and_internal_gap_s': tolerance_s,
                           'initialization_allowance_s': startup_allowance_s,
                           'minimum_post_start_span_ratio': minimum_span_ratio},
            'scope': 'actual received input/output stamps; tolerances grant no motion or authority'}
