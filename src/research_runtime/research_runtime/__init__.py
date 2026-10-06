"""Pure-Python research contracts; ROS transport stays at the edge."""

from .trajectory import VehicleLimits, TimedPoint, TimedTrajectory, ValidationResult
from .authority import AuthorityState, SafetyCommand, SafetyGate

__all__ = [
    "VehicleLimits", "TimedPoint", "TimedTrajectory", "ValidationResult",
    "AuthorityState", "SafetyCommand", "SafetyGate",
]
