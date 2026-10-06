import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parents[1]))
from super_lio_vehicle_adapter.adapter_node import _qrotate  # noqa: E402


def test_identity_transform_preserves_vector():
    assert _qrotate((0.0, 0.0, 0.0, 1.0), (1.0, 2.0, 3.0)) == (1.0, 2.0, 3.0)


def test_quaternion_rotation_is_explicit():
    q = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    x, y, z = _qrotate(q, (1.0, 0.0, 0.0))
    assert abs(x) < 1e-9
    assert abs(y - 1.0) < 1e-9
    assert z == 0.0
