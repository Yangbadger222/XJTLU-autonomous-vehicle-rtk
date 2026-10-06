import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parents[2] / "active_road_mapping"))
sys.path.insert(0, str(Path(__file__).parents[1]))
from active_road_mapping.map_node import _valid_version  # noqa: E402


def test_map_boundary_does_not_treat_unknown_as_a_valid_version():
    assert not _valid_version("")
    assert not _valid_version("UNKNOWN")
    assert not _valid_version(" unknown ")
    assert _valid_version("prior-v3")
