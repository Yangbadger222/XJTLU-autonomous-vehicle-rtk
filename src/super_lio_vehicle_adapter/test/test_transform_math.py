import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parents[1]))
from super_lio_vehicle_adapter.adapter_node import (  # noqa: E402
    _covariance_is_known, _normalize_quaternion, _parameter_bool, _qrotate,
    _rotate_covariance, _stamp_is_set,
)
from super_lio_vehicle_adapter.cloud_frame_node import _stamp_is_set as _cloud_stamp_is_set


def test_identity_transform_preserves_vector():
    assert _qrotate((0.0, 0.0, 0.0, 1.0), (1.0, 2.0, 3.0)) == (1.0, 2.0, 3.0)


def test_quaternion_rotation_is_explicit():
    q = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    x, y, z = _qrotate(q, (1.0, 0.0, 0.0))
    assert abs(x) < 1e-9
    assert abs(y - 1.0) < 1e-9
    assert z == 0.0


def test_non_unit_quaternion_is_normalized_before_use():
    assert _normalize_quaternion((0.0, 0.0, 0.0, 2.0)) == (0.0, 0.0, 0.0, 1.0)
    assert _normalize_quaternion((0.0, 0.0, 0.0, 0.0)) is None


def test_pose_covariance_rotates_with_stamped_world_to_odom_tf():
    covariance = [0.0] * 36
    covariance[0] = 1.0
    covariance[7] = 4.0
    covariance[14] = 9.0
    covariance[21] = 16.0
    covariance[28] = 25.0
    covariance[35] = 36.0
    q = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    rotated = _rotate_covariance(covariance, q)
    assert rotated is not None
    # A 90-degree yaw swaps x/y variances in each 3x3 pose block.
    assert math.isclose(rotated[0], 4.0, abs_tol=1e-9)
    assert math.isclose(rotated[7], 1.0, abs_tol=1e-9)
    assert math.isclose(rotated[14], 9.0, abs_tol=1e-9)
    assert math.isclose(rotated[21], 25.0, abs_tol=1e-9)
    assert math.isclose(rotated[28], 16.0, abs_tol=1e-9)
    assert math.isclose(rotated[35], 36.0, abs_tol=1e-9)


def test_pose_covariance_rejects_wrong_shape_or_nonfinite_values():
    assert _rotate_covariance([0.0] * 35, (0.0, 0.0, 0.0, 1.0)) is None
    covariance = [0.0] * 36
    covariance[0] = float("nan")
    assert _rotate_covariance(covariance, (0.0, 0.0, 0.0, 1.0)) is None


def test_ros_unknown_covariance_sentinel_is_motion_blocking():
    known = [0.0] * 72
    known[0] = known[7] = known[14] = 1.0
    known[36] = known[43] = known[50] = 1.0
    assert _covariance_is_known(known)
    unknown = list(known)
    unknown[0] = -1.0
    assert not _covariance_is_known(unknown)


def test_string_false_cannot_enable_verified_extrinsic():
    assert _parameter_bool("false") is False
    assert _parameter_bool("TRUE") is True


def test_unknown_boolean_is_rejected_instead_of_silently_disabling_a_gate():
    import pytest
    with pytest.raises(ValueError):
        _parameter_bool("maybe")


class _Stamp:
    def __init__(self, sec, nanosec):
        self.sec = sec
        self.nanosec = nanosec


def test_zero_or_malformed_stamps_are_not_usable_for_tf():
    for stamp in (_Stamp(0, 0), _Stamp(-1, 1), _Stamp(1, 1_000_000_000)):
        assert not _stamp_is_set(stamp)
        assert not _cloud_stamp_is_set(stamp)
    assert _stamp_is_set(_Stamp(0, 1))
    assert _cloud_stamp_is_set(_Stamp(1, 0))
