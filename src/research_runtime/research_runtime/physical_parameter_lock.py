"""Generated from approved parameter lock; SHA256 ac747e87ba5e098c4ea953b874552debfdfb2590ee90ffd4e245497675ced765."""
PHYSICAL_LIMITS = {'max_speed_mps': 0.85, 'min_speed_mps': 0.0, 'max_yaw_rate_rps': 0.7, 'max_accel_mps2': 0.85, 'max_decel_mps2': 1.2, 'max_yaw_accel_rps2': 1.4, 'max_yaw_decel_rps2': 1.8}
LOCKED_FOOTPRINT = ((0.33, 0.305), (0.33, -0.305), (-0.33, -0.305), (-0.33, 0.305))

def require_locked_motion_parameters(actual):
    import math
    for key, expected in PHYSICAL_LIMITS.items():
        value = actual.get(key)
        if not isinstance(value, (int,float)) or not math.isfinite(value) or abs(value-expected) > 1e-12:
            raise ValueError(f"protected corridor parameter override rejected: {key}")
