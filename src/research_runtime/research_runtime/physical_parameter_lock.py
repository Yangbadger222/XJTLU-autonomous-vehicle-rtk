"""Generated from approved parameter lock; SHA256 56a29e2bad5ddb8f96cd7cafa5724d9751709912ef68cc92fafa48e5a0ddb114."""
PHYSICAL_LIMITS = {'max_speed_mps': 0.85, 'min_speed_mps': 0.0, 'max_yaw_rate_rps': 0.7, 'max_accel_mps2': 0.85, 'max_decel_mps2': 1.2, 'max_yaw_accel_rps2': 1.4, 'max_yaw_decel_rps2': 1.8, 'max_lateral_accel_mps2': 0.25}
LOCKED_FOOTPRINT = ((0.33, 0.305), (0.33, -0.305), (-0.33, -0.305), (-0.33, 0.305))
STOP_CONFIRMATION = (0.05, 0.03490658503988659, 1.0)
STOP_CONFIRMATION_SOURCE_SHA256 = 'a332eea25d38058b330a9cce9da5cd3b8bb45993045da468c5cb66af83df3758'
AUTHORITY_HEARTBEAT_TIMEOUT_S = 0.5

def require_locked_motion_parameters(actual):
    import math
    for key, expected in PHYSICAL_LIMITS.items():
        value = actual.get(key)
        if not isinstance(value, (int,float)) or not math.isfinite(value) or abs(value-expected) > 1e-12:
            raise ValueError(f"protected corridor parameter override rejected: {key}")
