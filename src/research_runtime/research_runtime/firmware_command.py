"""Read-only STM32 command model. This is not an estimate of skid or braking."""
import math
from .physical_parameter_lock import FIRMWARE_COMMAND_MODEL as MODEL


def motor_targets_rpm(linear_x: float, angular_z: float) -> tuple[float, ...]:
    if not all(math.isfinite(value) for value in (linear_x, angular_z)):
        raise ValueError("nonfinite firmware command")
    factor = MODEL['gear_ratio']*MODEL['radps_to_rpm']/MODEL['radius_m']
    left = (linear_x-angular_z*MODEL['track_m']/2)*factor
    right = -(linear_x+angular_z*MODEL['track_m']/2)*factor
    return left, left, right, right


def within_firmware_command_envelope(linear_x: float, angular_z: float) -> bool:
    try:
        return all(abs(target)<=MODEL['max_motor_rpm'] for target in motor_targets_rpm(linear_x,angular_z))
    except (TypeError,ValueError):
        return False
