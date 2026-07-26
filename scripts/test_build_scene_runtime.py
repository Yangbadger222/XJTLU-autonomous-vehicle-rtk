import importlib.util
from pathlib import Path


SCRIPT = Path(__file__).resolve().parent / "build_scene_runtime.py"
spec = importlib.util.spec_from_file_location("build_scene_runtime", SCRIPT)
build_scene_runtime = importlib.util.module_from_spec(spec)
spec.loader.exec_module(build_scene_runtime)


def _node(name, lon, lat, *, anchor=False, dest=False):
    return {
        "name": name,
        "lon": lon,
        "lat": lat,
        "alt": 0.0,
        "anchor": anchor,
        "dest": dest,
    }


def test_sanitize_bundle_splits_proper_route_crossings_into_a_junction():
    raw_bundle = {
        "fixed_origin_node_id": 1,
        "nodes": {
            1: _node("west", 120.0000, 31.0000, anchor=True),
            2: _node("east", 120.0002, 31.0002, dest=True),
            3: _node("north", 120.0000, 31.0002),
            4: _node("south", 120.0002, 31.0000),
        },
        "edges": [[1, 2], [3, 4]],
    }

    nodes, edges, meta = build_scene_runtime.sanitize_bundle(raw_bundle)

    assert meta["intersection_node_count"] == 1
    intersection_id = max(nodes)
    intersection = nodes[intersection_id]
    assert intersection["name"] == f"intersection_{intersection_id}"
    assert intersection["anchor"] is True
    assert intersection["dest"] is False
    assert abs(intersection["lon"] - 120.0001) < 1e-9
    assert abs(intersection["lat"] - 31.0001) < 1e-9
    assert {tuple(edge) for edge in edges} == {
        (1, intersection_id),
        (2, intersection_id),
        (3, intersection_id),
        (4, intersection_id),
    }
