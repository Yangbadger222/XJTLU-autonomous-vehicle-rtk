import math

import numpy as np
import pytest

from scripts.calibrate_fgo_gil_ecef_world import (
    TimedPose,
    fit_rigid_transform,
    gga_quality,
    lidar_pose_to_local_antenna,
    rotation_quaternion_wxyz,
    wgs84_to_ecef,
)


def test_wgs84_to_ecef_matches_equator_axes():
    assert wgs84_to_ecef(0.0, 0.0, 0.0) == pytest.approx([6378137.0, 0.0, 0.0])
    assert wgs84_to_ecef(0.0, 90.0, 0.0) == pytest.approx(
        [0.0, 6378137.0, 0.0], abs=1.0e-9
    )


def test_gga_quality_rejects_non_gga_and_extracts_fixed():
    assert gga_quality("$GNGGA,010203.0,0,N,0,E,4,37,0.5,0,M,0,M,,*00") == 4
    assert gga_quality("$GNRMC,010203.0,A,0,N,0,E,0,0,010100,,,A*00") is None


def test_lidar_pose_applies_inverse_imu_lidar_and_master_lever_arm():
    pose = TimedPose(0, (10.0, 20.0, 30.0), (0.0, 0.0, 0.0, 1.0))
    antenna = lidar_pose_to_local_antenna(
        pose,
        (-0.011, -0.02329, 0.04412),
        (1.0, 0.0, 0.0, 0.0),
        (0.0, -0.184, 0.134),
    )
    assert antenna == pytest.approx([10.011, 19.83929, 30.08988])


def test_rigid_fit_recovers_rotation_and_large_ecef_translation():
    angle = math.radians(32.0)
    expected_rotation = np.asarray(
        [
            [math.cos(angle), -math.sin(angle), 0.0],
            [math.sin(angle), math.cos(angle), 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    expected_translation = np.asarray([-2700000.0, 4300000.0, 3800000.0])
    source = np.asarray(
        [(0.0, 0.0, 0.0), (10.0, 0.0, 0.0), (0.0, 12.0, 0.0), (8.0, 9.0, 1.0)]
    )
    destination = (expected_rotation @ source.T).T + expected_translation

    rotation, translation, _, residuals = fit_rigid_transform(source, destination)

    assert rotation == pytest.approx(expected_rotation, abs=1.0e-10)
    assert translation == pytest.approx(expected_translation, abs=1.0e-8)
    assert max(residuals) < 1.0e-8
    quaternion = rotation_quaternion_wxyz(rotation)
    assert quaternion == pytest.approx(
        [math.cos(angle / 2.0), 0.0, 0.0, math.sin(angle / 2.0)], abs=1.0e-10
    )
