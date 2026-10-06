"""Pure-Python research contracts; ROS transport stays at the edge."""

from .trajectory import VehicleLimits, TimedPoint, TimedTrajectory, ValidationResult
from .authority import AuthorityState, SafetyCommand, SafetyGate
from .geometry import CameraIntrinsics, RigidTransform, pixel_to_camera, pixel_to_odom
from .grid_map import LocalObstacleGrid, ProjectionStats, project_obstacle_points
from .trajectory_tracker import TrackerCommand, TrackerState, TimedTrajectoryTracker

__all__ = [
    "VehicleLimits", "TimedPoint", "TimedTrajectory", "ValidationResult",
    "AuthorityState", "SafetyCommand", "SafetyGate",
    "CameraIntrinsics", "RigidTransform", "pixel_to_camera", "pixel_to_odom",
    "LocalObstacleGrid", "ProjectionStats", "project_obstacle_points",
    "TrackerCommand", "TrackerState", "TimedTrajectoryTracker",
]
