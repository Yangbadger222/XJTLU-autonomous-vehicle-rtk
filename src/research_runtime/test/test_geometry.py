import math

import pytest

from research_runtime.active_road import GeoTransform
from research_runtime.geometry import CameraIntrinsics, RigidTransform, pixel_to_camera, pixel_to_odom


def test_pixel_center_round_trip_preserves_declared_geotransform():
    transform = GeoTransform("EPSG:32651", "WGS84", 100.0, 200.0, 0.5, 0.5)
    local = transform.pixel_to_local(3.0, 4.0)
    assert transform.local_to_pixel(*local) == (3.0, 4.0)


def test_optical_z_depth_and_acquisition_transform_chain():
    intrinsics = CameraIntrinsics(100.0, 100.0, 50.0, 50.0, 0.001, "optical_z")
    identity = RigidTransform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    base_to_odom = RigidTransform((10.0, -2.0, 0.5), (0.0, 0.0, 0.0, 1.0))
    assert pixel_to_camera(50.0, 50.0, 1000.0, intrinsics) == (0.0, 0.0, 1.0)
    assert pixel_to_odom(50.0, 50.0, 1000.0, intrinsics, identity, base_to_odom) == (10.0, -2.0, 1.5)


def test_ray_range_and_invalid_depth_are_explicit():
    intrinsics = CameraIntrinsics(100.0, 100.0, 0.0, 0.0, 1.0, "ray_range")
    point = pixel_to_camera(100.0, 0.0, 2.0, intrinsics)
    assert math.isclose(math.sqrt(sum(value * value for value in point)), 2.0)
    with pytest.raises(ValueError):
        pixel_to_camera(0.0, 0.0, 0.0, intrinsics)
    with pytest.raises(ValueError):
        pixel_to_camera(0.0, 0.0, float("nan"), intrinsics)
