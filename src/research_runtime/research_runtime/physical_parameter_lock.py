"""Generated from approved parameter lock; SHA256 56a29e2bad5ddb8f96cd7cafa5724d9751709912ef68cc92fafa48e5a0ddb114."""
PHYSICAL_LIMITS = {'max_speed_mps': 0.85, 'min_speed_mps': 0.0, 'max_yaw_rate_rps': 0.7, 'max_accel_mps2': 0.85, 'max_decel_mps2': 1.2, 'max_yaw_accel_rps2': 1.4, 'max_yaw_decel_rps2': 1.8, 'max_lateral_accel_mps2': 0.25}
LOCKED_FOOTPRINT = ((0.33, 0.305), (0.33, -0.305), (-0.33, -0.305), (-0.33, 0.305))
STOP_CONFIRMATION = (0.05, 0.03490658503988659, 1.0)
STOP_CONFIRMATION_SOURCE_SHA256 = 'a332eea25d38058b330a9cce9da5cd3b8bb45993045da468c5cb66af83df3758'
AUTHORITY_HEARTBEAT_TIMEOUT_S = 0.5
FIRMWARE_COMMAND_MODEL = {'radius_m': 0.1, 'track_m': 0.46, 'gear_ratio': 19.2, 'radps_to_rpm': 9.55, 'max_motor_rpm': 20000.0}
RESEARCH_EXECUTION_PROFILE = {'max_curvature_1pm': 0.34602076124567477, 'max_lateral_speed_mps': 0.05, 'max_jerk_mps3': 1.7}
MID360_GROUND_REFERENCE = {'lidar_height_m': 0.447, 'lidar_in_imu_m': (-0.011, -0.02329, 0.04412)}
MID360_MOUNTING_NOTES_SHA256 = '68c13b201ca622ead792b388d4404c4344c0a73e1a7e56ea1d34ea121c9e95fa'
from super_lio_vehicle_adapter.control_reference_lock import CONTROL_REFERENCE_CONTRACT, CONTROL_ODOM_TOPIC, CONTROL_CHILD_FRAME

def require_locked_motion_parameters(actual):
    import math
    for key, expected in PHYSICAL_LIMITS.items():
        value = actual.get(key)
        if not isinstance(value, (int,float)) or not math.isfinite(value) or abs(value-expected) > 1e-12:
            raise ValueError(f"protected corridor parameter override rejected: {key}")
