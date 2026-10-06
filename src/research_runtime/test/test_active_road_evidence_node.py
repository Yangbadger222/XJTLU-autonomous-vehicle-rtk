import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parents[2] / "active_road_mapping"))

from active_road_mapping.evidence_node import _valid_version  # noqa: E402


def test_evidence_node_requires_known_persisted_map_version():
    assert not _valid_version("")
    assert not _valid_version("UNKNOWN")
    assert _valid_version("magr-v3")
