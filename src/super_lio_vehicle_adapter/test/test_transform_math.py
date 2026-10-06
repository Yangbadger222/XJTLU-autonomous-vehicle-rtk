import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parents[1]))
from super_lio_vehicle_adapter.adapter_node import (  # noqa: E402
    _normalize_quaternion, _parameter_bool, _qrotate,
)


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


def test_string_false_cannot_enable_verified_extrinsic():
    assert _parameter_bool("false") is False
    assert _parameter_bool("TRUE") is True
