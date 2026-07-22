import math

import pytest

from gps_waypoint_dispatcher.fgo_corridor_math import (
    ecef_imu_pose_to_geodetic_base_pose,
    ecef_to_geodetic,
)


def _geodetic_to_ecef(latitude_deg, longitude_deg, altitude_m=0.0):
    a = 6378137.0
    f = 1.0 / 298.257223563
    e2 = f * (2.0 - f)
    latitude = math.radians(latitude_deg)
    longitude = math.radians(longitude_deg)
    radius = a / math.sqrt(1.0 - e2 * math.sin(latitude) ** 2)
    x = (radius + altitude_m) * math.cos(latitude) * math.cos(longitude)
    y = (radius + altitude_m) * math.cos(latitude) * math.sin(longitude)
    z = (radius * (1.0 - e2) + altitude_m) * math.sin(latitude)
    return x, y, z


def test_ecef_to_geodetic_round_trip():
    ecef = _geodetic_to_ecef(31.274927, 120.737548, 14.2)
    result = ecef_to_geodetic(*ecef)

    assert result is not None
    assert result[0] == pytest.approx(31.274927, abs=1e-8)
    assert result[1] == pytest.approx(120.737548, abs=1e-8)
    assert result[2] == pytest.approx(14.2, abs=1e-4)


def test_ecef_imu_pose_converts_to_enu_heading():
    # At the equator/prime meridian, ECEF +Y is local east. A +90 degree
    # ECEF-Z rotation maps imu +X to ECEF +Y, therefore ENU yaw is zero.
    result = ecef_imu_pose_to_geodetic_base_pose(
        position_ecef_m=(6378137.0, 0.0, 0.0),
        orientation_ecef_imu_xyzw=(0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)),
        base_from_imu_m=(0.0, 0.0, 0.0),
    )

    assert result is not None
    assert result.latitude_deg == pytest.approx(0.0, abs=1e-9)
    assert result.longitude_deg == pytest.approx(0.0, abs=1e-9)
    assert result.enu_yaw_rad == pytest.approx(0.0, abs=1e-9)


def test_ecef_imu_pose_rejects_nonfinite_or_zero_quaternion():
    assert ecef_to_geodetic(math.nan, 0.0, 0.0) is None
    assert (
        ecef_imu_pose_to_geodetic_base_pose(
            position_ecef_m=(6378137.0, 0.0, 0.0),
            orientation_ecef_imu_xyzw=(0.0, 0.0, 0.0, 0.0),
        )
        is None
    )
