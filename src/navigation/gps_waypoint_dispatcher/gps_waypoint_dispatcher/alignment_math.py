from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class BootstrapYawChoice:
    enu_yaw_rad: float
    source: str
    route_mismatch_deg: float | None
    warn_route_mismatch: bool


@dataclass(frozen=True)
class BootstrapAlignment:
    theta: float
    tx: float
    ty: float


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def compass_heading_to_enu_yaw_deg(heading_deg: float) -> float:
    return (90.0 - float(heading_deg)) % 360.0


def angle_delta_deg(a_rad: float, b_rad: float) -> float:
    return abs(math.degrees(normalize_angle(a_rad - b_rad)))


def route_launch_heading_to_enu_yaw_rad(route_launch_heading_deg: float) -> float:
    return math.radians(compass_heading_to_enu_yaw_deg(route_launch_heading_deg))


def heading_quaternion_yaw_to_enu_yaw(
    quaternion_yaw_rad: float,
    *,
    quaternion_yaw_is_compass: bool,
) -> float:
    if not quaternion_yaw_is_compass:
        return normalize_angle(quaternion_yaw_rad)
    compass_heading_deg = math.degrees(quaternion_yaw_rad) % 360.0
    return normalize_angle(math.radians(compass_heading_to_enu_yaw_deg(compass_heading_deg)))


def choose_bootstrap_yaw(
    *,
    route_launch_heading_deg: float,
    rtk_heading_yaw_rad: float | None,
    use_rtk_heading: bool,
    mismatch_warn_deg: float,
) -> BootstrapYawChoice:
    route_yaw_rad = route_launch_heading_to_enu_yaw_rad(route_launch_heading_deg)
    if not use_rtk_heading or rtk_heading_yaw_rad is None:
        return BootstrapYawChoice(
            enu_yaw_rad=route_yaw_rad,
            source="route_launch_yaw",
            route_mismatch_deg=None,
            warn_route_mismatch=False,
        )

    mismatch_deg = angle_delta_deg(rtk_heading_yaw_rad, route_yaw_rad)
    return BootstrapYawChoice(
        enu_yaw_rad=rtk_heading_yaw_rad,
        source="rtk_heading",
        route_mismatch_deg=mismatch_deg,
        warn_route_mismatch=mismatch_deg > mismatch_warn_deg,
    )


def compute_bootstrap_alignment(
    *,
    map_x: float,
    map_y: float,
    map_yaw_rad: float,
    anchor_enu_x: float,
    anchor_enu_y: float,
    selected_enu_yaw_rad: float,
) -> BootstrapAlignment:
    theta = normalize_angle(map_yaw_rad - selected_enu_yaw_rad)
    cos_theta = math.cos(theta)
    sin_theta = math.sin(theta)
    tx = map_x - (cos_theta * anchor_enu_x - sin_theta * anchor_enu_y)
    ty = map_y - (sin_theta * anchor_enu_x + cos_theta * anchor_enu_y)
    return BootstrapAlignment(theta=theta, tx=tx, ty=ty)
