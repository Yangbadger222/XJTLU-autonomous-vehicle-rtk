import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parents[1]))
from research_runtime.local_obstacle_grid_node import _valid_map_version  # noqa: E402


def test_unknown_map_versions_are_rejected_before_cloud_projection():
    assert not _valid_map_version("")
    assert not _valid_map_version("UNKNOWN")
    assert not _valid_map_version(" unknown ")
    assert _valid_map_version("map-2026-10-07")
