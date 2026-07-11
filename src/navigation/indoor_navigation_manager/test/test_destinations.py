import sys
from pathlib import Path

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from indoor_navigation_manager.destinations import load_destinations, resolve_destination


def test_destination_aliases_resolve_case_and_whitespace(tmp_path):
    path = tmp_path / "destinations.yaml"
    path.write_text(
        yaml.safe_dump(
            {
                "schema_version": 1,
                "map_id": "floor_1",
                "destinations": {
                    "lab_101": {
                        "display_name": "Laboratory 101",
                        "aliases": ["实验室", "Lab 101"],
                        "pose": {"x": 1.2, "y": 3.4, "yaw": 1.57},
                    }
                },
            }
        ),
        encoding="utf-8",
    )

    catalog = load_destinations(path)

    assert resolve_destination(catalog, "  LAB   101 ")["id"] == "lab_101"
    assert resolve_destination(catalog, "实验室")["x"] == 1.2


def test_duplicate_alias_is_rejected(tmp_path):
    path = tmp_path / "destinations.yaml"
    path.write_text(
        yaml.safe_dump(
            {
                "schema_version": 1,
                "map_id": "floor_1",
                "destinations": {
                    "a": {"aliases": ["same"], "pose": {"x": 0, "y": 0}},
                    "b": {"aliases": ["same"], "pose": {"x": 1, "y": 1}},
                },
            }
        ),
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="duplicate destination alias"):
        load_destinations(path)


def test_invalid_map_aliases_and_pose_are_rejected(tmp_path):
    path = tmp_path / "destinations.yaml"
    path.write_text(
        yaml.safe_dump(
            {
                "schema_version": 1,
                "map_id": "",
                "destinations": {
                    "bad": {"aliases": "not-a-list", "pose": {"x": float("nan"), "y": 0}}
                },
            }
        ),
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="map_id is required"):
        load_destinations(path)

    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    payload["map_id"] = "floor_1"
    path.write_text(yaml.safe_dump(payload), encoding="utf-8")
    with pytest.raises(ValueError, match="aliases must be a list"):
        load_destinations(path)

    payload["destinations"]["bad"]["aliases"] = []
    path.write_text(yaml.safe_dump(payload), encoding="utf-8")
    with pytest.raises(ValueError, match="pose must be finite"):
        load_destinations(path)
