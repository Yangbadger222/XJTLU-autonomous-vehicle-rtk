import importlib.util
import json
from pathlib import Path

import yaml
import pytest


SCRIPT = Path(__file__).resolve().parent / "compile_qgis_scene.py"
spec = importlib.util.spec_from_file_location("compile_qgis_scene", SCRIPT)
compile_qgis_scene = importlib.util.module_from_spec(spec)
spec.loader.exec_module(compile_qgis_scene)


def _write_sample_geojson(path: Path) -> None:
    data = {
        "type": "FeatureCollection",
        "features": [
            {
                "type": "Feature",
                "properties": {"feature_type": "route"},
                "geometry": {
                    "type": "LineString",
                    "coordinates": [
                        [120.0000000, 31.0000000],
                        [120.0003000, 31.0000000],
                        [120.0003000, 31.0003000],
                    ],
                },
            },
            {
                "type": "Feature",
                "properties": {
                    "feature_type": "building",
                    "name:en": "Test Building",
                },
                "geometry": {
                    "type": "Polygon",
                    "coordinates": [
                        [
                            [120.0002500, 31.0002500],
                            [120.0002600, 31.0002500],
                            [120.0002600, 31.0002600],
                            [120.0002500, 31.0002600],
                            [120.0002500, 31.0002500],
                        ]
                    ],
                },
            },
        ],
    }
    path.write_text(json.dumps(data), encoding="utf-8")


def test_compile_qgis_scene_densifies_routes_and_adds_destinations(tmp_path):
    geojson_path = tmp_path / "qgis.geojson"
    _write_sample_geojson(geojson_path)

    bundle = compile_qgis_scene.compile_qgis_scene(
        geojson_path,
        scene_name="unit_qgis",
        densify_step_m=5.0,
        building_snap_max_m=80.0,
    )

    assert bundle["scene_name"] == "unit_qgis"
    assert bundle["fixed_origin_node_id"] in bundle["nodes"]
    assert len(bundle["nodes"]) > 3
    assert len(bundle["edges"]) >= len(bundle["nodes"]) - 1
    assert any(node["anchor"] for node in bundle["nodes"].values())
    assert not all(node["anchor"] for node in bundle["nodes"].values())

    destinations = {node["name"] for node in bundle["nodes"].values() if node["dest"]}
    assert "test_building" in destinations
    assert any(name.startswith("route_end_") for name in destinations)

    max_edge_m = compile_qgis_scene.max_edge_length_m(bundle)
    assert max_edge_m <= 5.5


def test_compile_qgis_scene_cli_writes_scene_bundle(tmp_path):
    geojson_path = tmp_path / "qgis.geojson"
    output_path = tmp_path / "scene_gps_bundle.yaml"
    _write_sample_geojson(geojson_path)

    rc = compile_qgis_scene.main(
        [
            "--input",
            str(geojson_path),
            "--output",
            str(output_path),
            "--scene-name",
            "unit_qgis",
            "--densify-step-m",
            "5.0",
        ]
    )

    assert rc == 0
    written = yaml.safe_load(output_path.read_text(encoding="utf-8"))
    assert written["scene_name"] == "unit_qgis"
    assert written["nodes"]
    assert written["edges"]


def test_compile_qgis_scene_rejects_disconnected_route_network(tmp_path):
    geojson_path = tmp_path / "disconnected.geojson"
    geojson_path.write_text(
        json.dumps(
            {
                "type": "FeatureCollection",
                "features": [
                    {
                        "type": "Feature",
                        "properties": {"feature_type": "route"},
                        "geometry": {
                            "type": "LineString",
                            "coordinates": [[120.0, 31.0], [120.0001, 31.0]],
                        },
                    },
                    {
                        "type": "Feature",
                        "properties": {"feature_type": "route"},
                        "geometry": {
                            "type": "LineString",
                            "coordinates": [[120.01, 31.01], [120.0101, 31.01]],
                        },
                    },
                ],
            }
        ),
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="route graph is disconnected"):
        compile_qgis_scene.compile_qgis_scene(
            geojson_path,
            scene_name="disconnected",
        )
