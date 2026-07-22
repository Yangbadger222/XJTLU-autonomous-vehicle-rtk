"""ROS-free ECEF-to-corridor geometry helpers."""

from __future__ import annotations

import math
from dataclasses import dataclass


WGS84_A_M = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


@dataclass(frozen=True)
class GeodeticPose:
    latitude_deg: float
    longitude_deg: float
    altitude_m: float
    enu_yaw_rad: float


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _finite(values: tuple[float, ...]) -> bool:
    return all(math.isfinite(value) for value in values)


def ecef_to_geodetic(
    x_m: float, y_m: float, z_m: float
) -> tuple[float, float, float] | None:
    """Return WGS84 latitude, longitude, altitude for an ECEF position."""
    if not _finite((x_m, y_m, z_m)):
        return None
    p_m = math.hypot(x_m, y_m)
    if p_m < 1.0 or math.hypot(p_m, z_m) < 1.0e6:
        return None
    b_m = WGS84_A_M * (1.0 - WGS84_F)
    ep2 = (WGS84_A_M * WGS84_A_M - b_m * b_m) / (b_m * b_m)
    longitude_rad = math.atan2(y_m, x_m)
    theta = math.atan2(z_m * WGS84_A_M, p_m * b_m)
    sin_theta = math.sin(theta)
    cos_theta = math.cos(theta)
    latitude_rad = math.atan2(
        z_m + ep2 * b_m * sin_theta**3,
        p_m - WGS84_E2 * WGS84_A_M * cos_theta**3,
    )
    sin_latitude = math.sin(latitude_rad)
    radius_m = WGS84_A_M / math.sqrt(1.0 - WGS84_E2 * sin_latitude**2)
    cosine = math.cos(latitude_rad)
    altitude_m = abs(z_m) - b_m if abs(cosine) < 1.0e-12 else p_m / cosine - radius_m
    result = (math.degrees(latitude_rad), math.degrees(longitude_rad), altitude_m)
    return result if _finite(result) else None


def quaternion_xyzw_to_matrix(
    x: float, y: float, z: float, w: float
) -> tuple[tuple[float, float, float], ...] | None:
    if not _finite((x, y, z, w)):
        return None
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1.0e-12:
        return None
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return (
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
        (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
        (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
    )


def _matmul(
    first: tuple[tuple[float, float, float], ...],
    second: tuple[tuple[float, float, float], ...],
) -> tuple[tuple[float, float, float], ...]:
    return tuple(
        tuple(sum(first[row][k] * second[k][column] for k in range(3)) for column in range(3))
        for row in range(3)
    )


def _rotate(
    matrix: tuple[tuple[float, float, float], ...], vector: tuple[float, float, float]
) -> tuple[float, float, float]:
    return tuple(sum(matrix[row][column] * vector[column] for column in range(3)) for row in range(3))


def ecef_imu_pose_to_geodetic_base_pose(
    *,
    position_ecef_m: tuple[float, float, float],
    orientation_ecef_imu_xyzw: tuple[float, float, float, float],
    base_from_imu_m: tuple[float, float, float] = (0.0, 0.0, -0.02),
) -> GeodeticPose | None:
    """Convert nav_msgs ECEF<-imu pose to base position and local ENU yaw.

    The default lever arm matches the repository URDF: ``imu_link`` is 2 cm
    above ``base_link``.  Invalid geometry is rejected instead of guessed.
    """
    if not _finite((*position_ecef_m, *orientation_ecef_imu_xyzw, *base_from_imu_m)):
        return None
    ecef_from_imu = quaternion_xyzw_to_matrix(*orientation_ecef_imu_xyzw)
    if ecef_from_imu is None:
        return None
    offset_ecef = _rotate(ecef_from_imu, base_from_imu_m)
    base_ecef = tuple(position_ecef_m[i] + offset_ecef[i] for i in range(3))
    geodetic = ecef_to_geodetic(*base_ecef)
    if geodetic is None:
        return None
    latitude_deg, longitude_deg, altitude_m = geodetic
    latitude_rad, longitude_rad = math.radians(latitude_deg), math.radians(longitude_deg)
    enu_from_ecef = (
        (-math.sin(longitude_rad), math.cos(longitude_rad), 0.0),
        (-math.sin(latitude_rad) * math.cos(longitude_rad), -math.sin(latitude_rad) * math.sin(longitude_rad), math.cos(latitude_rad)),
        (math.cos(latitude_rad) * math.cos(longitude_rad), math.cos(latitude_rad) * math.sin(longitude_rad), math.sin(latitude_rad)),
    )
    enu_from_imu = _matmul(enu_from_ecef, ecef_from_imu)
    yaw_rad = normalize_angle(math.atan2(enu_from_imu[1][0], enu_from_imu[0][0]))
    if not math.isfinite(yaw_rad):
        return None
    return GeodeticPose(latitude_deg, longitude_deg, altitude_m, yaw_rad)
