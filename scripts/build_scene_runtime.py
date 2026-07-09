#!/usr/bin/env python3
"""
Compile a collected scene bundle into runtime files for nav-gps mode.
"""

from __future__ import annotations

import math
import re
import shutil
import sys
from datetime import datetime
import json
from pathlib import Path

from pyproj import Transformer
import yaml

RUNTIME_ROOT = Path.home() / "XJTLU-autonomous-vehicle/runtime-data"
DEFAULT_BUNDLE = RUNTIME_ROOT / "gnss" / "scene_gps_bundle.yaml"
CURRENT_SCENE_DIR = RUNTIME_ROOT / "gnss" / "current_scene"
REPO_ROOT = Path(__file__).resolve().parents[1]
MASTER_PARAMS_TEMPLATE = REPO_ROOT / "src" / "bringup" / "config" / "master_params.yaml"
SCENE_POINTS_FILE = CURRENT_SCENE_DIR / "scene_points.yaml"
SCENE_GRAPH_FILE = CURRENT_SCENE_DIR / "scene_route_graph.geojson"
MASTER_PARAMS_SCENE_FILE = CURRENT_SCENE_DIR / "master_params_scene.yaml"
SCENE_BUNDLE_COPY = CURRENT_SCENE_DIR / "scene_gps_bundle.yaml"
ENGLISH_NAME_RE = re.compile(r"^[A-Za-z0-9_-]+$")


def load_yaml(path: Path) -> dict:
    with open(path, "r", encoding="utf-8") as input_file:
        return yaml.safe_load(input_file) or {}


def save_yaml(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as output_file:
        yaml.safe_dump(data, output_file, allow_unicode=True, sort_keys=False)


def build_transformer(origin_lat: float, origin_lon: float, origin_alt: float) -> Transformer:
    pipeline = (
        "+proj=pipeline "
        "+step +proj=cart +ellps=WGS84 "
        f"+step +proj=topocentric +ellps=WGS84 +lat_0={origin_lat} "
        f"+lon_0={origin_lon} +h_0={origin_alt}"
    )
    return Transformer.from_pipeline(pipeline)


def latlon_to_enu(transformer: Transformer, lat: float, lon: float, alt: float) -> tuple[float, float, float]:
    x, y, z = transformer.transform(lon, lat, alt, radians=False)
    return float(x), float(y), float(z)


def sanitize_bundle(raw_bundle: dict) -> tuple[dict[int, dict], list[list[int]], dict]:
    raw_nodes = raw_bundle.get("nodes", {})
    if not isinstance(raw_nodes, dict) or not raw_nodes:
        raise ValueError("scene bundle has no nodes")

    node_names_lower: set[str] = set()
    nodes: dict[int, dict] = {}
    for raw_id, raw_node in raw_nodes.items():
        node_id = int(raw_id)
        if not isinstance(raw_node, dict):
            raise ValueError(f"node {raw_id} must be a map")

        name = str(raw_node.get("name", "")).strip()
        if not ENGLISH_NAME_RE.fullmatch(name):
            raise ValueError(f"node {node_id} has invalid english name '{name}'")
        if name.lower() in node_names_lower:
            raise ValueError(f"duplicate node name '{name}'")
        node_names_lower.add(name.lower())

        lat = float(raw_node["lat"])
        lon = float(raw_node["lon"])
        alt = float(raw_node.get("alt", 0.0))
        if not all(math.isfinite(value) for value in (lat, lon, alt)):
            raise ValueError(f"node {node_id} contains non-finite coordinates")

        nodes[node_id] = {
            "id": node_id,
            "name": name,
            "lat": lat,
            "lon": lon,
            "alt": alt,
            "anchor": bool(raw_node.get("anchor", False)),
            "dest": bool(raw_node.get("dest", False)),
            "samples": int(raw_node.get("samples", 0)),
            "spread_m": float(raw_node.get("spread_m", 0.0)),
            "source": str(raw_node.get("source", "/fix")),
            "time": str(raw_node.get("time", "")),
        }

    raw_edges = raw_bundle.get("edges", [])
    if not isinstance(raw_edges, list) or not raw_edges:
        raise ValueError("scene bundle has no edges")

    edges: list[list[int]] = []
    seen_edges: set[tuple[int, int]] = set()
    for raw_edge in raw_edges:
        if not isinstance(raw_edge, (list, tuple)) or len(raw_edge) != 2:
            raise ValueError(f"invalid edge entry: {raw_edge!r}")
        a = int(raw_edge[0])
        b = int(raw_edge[1])
        if a == b:
            raise ValueError(f"self-loop edge is invalid: {raw_edge!r}")
        if a not in nodes or b not in nodes:
            raise ValueError(f"edge references missing node: {raw_edge!r}")
        normalized = (min(a, b), max(a, b))
        if normalized in seen_edges:
            continue
        seen_edges.add(normalized)
        edges.append([normalized[0], normalized[1]])

    origin_id = raw_bundle.get("fixed_origin_node_id")
    if origin_id is None:
        raise ValueError("scene bundle is missing fixed_origin_node_id")
    origin_id = int(origin_id)
    if origin_id not in nodes:
        raise ValueError(f"fixed_origin_node_id {origin_id} does not exist in nodes")

    anchors = [node_id for node_id, node in nodes.items() if node["anchor"]]
    if not anchors:
        raise ValueError("scene bundle must contain at least one anchor node")

    destinations = [node_id for node_id, node in nodes.items() if node["dest"]]
    if not destinations:
        raise ValueError("scene bundle must contain at least one destination node")

    return nodes, edges, {"origin_id": origin_id, "anchor_ids": anchors, "destination_ids": destinations}


def build_scene_points(bundle: dict, nodes: dict[int, dict], edges: list[list[int]], origin_id: int) -> dict:
    origin_node = nodes[origin_id]
    transformer = build_transformer(origin_node["lat"], origin_node["lon"], origin_node["alt"])

    compiled_nodes: dict[str, dict] = {}
    for node_id, node in sorted(nodes.items()):
        x, y, z = latlon_to_enu(transformer, node["lat"], node["lon"], node["alt"])
        compiled_nodes[str(node_id)] = {
            "id": node_id,
            "name": node["name"],
            "lat": round(node["lat"], 7),
            "lon": round(node["lon"], 7),
            "alt": round(node["alt"], 2),
            "x": round(x, 3),
            "y": round(y, 3),
            "z": round(z, 3),
            "anchor": node["anchor"],
            "dest": node["dest"],
            "samples": node["samples"],
            "spread_m": round(node["spread_m"], 2),
            "source": node["source"],
            "time": node["time"],
        }

    destination_names = {
        node["name"]: node["id"]
        for node in sorted(compiled_nodes.values(), key=lambda item: item["name"])
        if node["dest"]
    }

    return {
        "scene_name": str(bundle.get("scene_name", "unknown_scene")),
        "compiled_at": datetime.now().isoformat(),
        "coordinate_source": str(bundle.get("coordinate_source", "/fix")),
        "fixed_origin": {
            "node_id": origin_node["id"],
            "name": origin_node["name"],
            "lat": round(origin_node["lat"], 7),
            "lon": round(origin_node["lon"], 7),
            "alt": round(origin_node["alt"], 2),
        },
        "nodes": compiled_nodes,
        "edges": edges,
        "anchor_ids": [node["id"] for node in compiled_nodes.values() if node["anchor"]],
        "destination_ids": [node["id"] for node in compiled_nodes.values() if node["dest"]],
        "destination_names": destination_names,
    }


def build_route_graph(scene_points: dict) -> dict:
    features: list[dict] = []

    for node in scene_points["nodes"].values():
        features.append(
            {
                "type": "Feature",
                "properties": {
                    "id": int(node["id"]),
                    "frame": "map",
                    "metadata": {
                        "name": node["name"],
                        "anchor": bool(node["anchor"]),
                        "dest": bool(node["dest"]),
                    },
                },
                "geometry": {
                    "type": "Point",
                    "coordinates": [float(node["x"]), float(node["y"])],
                },
            }
        )

    edge_id = 1000
    for a, b in scene_points["edges"]:
        node_a = scene_points["nodes"][str(a)]
        node_b = scene_points["nodes"][str(b)]
        distance = math.hypot(float(node_b["x"]) - float(node_a["x"]), float(node_b["y"]) - float(node_a["y"]))

        for start_id, end_id, start_node, end_node in (
            (a, b, node_a, node_b),
            (b, a, node_b, node_a),
        ):
            features.append(
                {
                    "type": "Feature",
                    "properties": {
                        "id": edge_id,
                        "startid": int(start_id),
                        "endid": int(end_id),
                        "cost": round(distance, 3),
                        "metadata": {
                            "distance_m": round(distance, 3),
                        },
                    },
                    "geometry": {
                        "type": "MultiLineString",
                        "coordinates": [
                            [
                                [float(start_node["x"]), float(start_node["y"])],
                                [float(end_node["x"]), float(end_node["y"])],
                            ]
                        ],
                    },
                }
            )
            edge_id += 1

    return {
        "type": "FeatureCollection",
        "name": scene_points["scene_name"],
        "features": features,
    }


def build_master_params_scene(scene_points: dict) -> dict:
    params = load_yaml(MASTER_PARAMS_TEMPLATE)
    origin = scene_points["fixed_origin"]

    pgo_params = params.setdefault("/pgo", {}).setdefault("pgo_node", {}).setdefault("ros__parameters", {})
    pgo_params["gps.origin_mode"] = "fixed"
    pgo_params["gps.origin_lat"] = origin["lat"]
    pgo_params["gps.origin_lon"] = origin["lon"]
    pgo_params["gps.origin_alt"] = origin["alt"]
    pgo_params["gps.topic"] = "/gnss"

    params["/gps_anchor_localizer"] = {
        "ros__parameters": {
            "scene_points_file": str(SCENE_POINTS_FILE),
            "enu_origin_lat": origin["lat"],
            "enu_origin_lon": origin["lon"],
            "enu_origin_alt": origin["alt"],
            "anchor_match_radius_m": 8.0,
            "ambiguity_margin_m": 3.0,
            "fix_sample_count": 10,
            "fix_spread_max_m": 2.0,
            "fix_sigma_xy_max_m": 6.0,
            "nav_ready_map_residual_m": 4.0,
            "nav_ready_required_consecutive_samples": 3,
            "map_frame": "map",
            "base_frame": "base_link",
        }
    }

    params["/gps_waypoint_dispatcher"] = {
        "ros__parameters": {
            "scene_points_file": str(SCENE_POINTS_FILE),
            "route_frame": "map",
            "base_frame": "base_link",
            "navigate_to_anchor_tolerance_m": 2.5,
            "controller_id": "FollowPath",
            "goal_checker_id": "general_goal_checker",
        }
    }

    params["/rtk_map_odom_corrector"] = params.get(
        "/rtk_map_odom_corrector",
        {"ros__parameters": {}},
    )
    rtk_authority_params = params["/rtk_map_odom_corrector"].setdefault("ros__parameters", {})
    rtk_authority_params.update(
        {
            "scene_points_file": str(SCENE_POINTS_FILE),
            "use_scene_identity_alignment": True,
            "alignment_topic": "/gps_scene/enu_to_map",
            "enu_origin_lat": origin["lat"],
            "enu_origin_lon": origin["lon"],
            "enu_origin_alt": origin["alt"],
        }
    )

    return params


def main() -> None:
    bundle_path = Path(sys.argv[1]).expanduser() if len(sys.argv) > 1 else DEFAULT_BUNDLE
    if not bundle_path.exists():
        raise SystemExit(f"Scene bundle not found: {bundle_path}")

    raw_bundle = load_yaml(bundle_path)
    nodes, edges, meta = sanitize_bundle(raw_bundle)
    scene_points = build_scene_points(raw_bundle, nodes, edges, meta["origin_id"])
    route_graph = build_route_graph(scene_points)
    master_params_scene = build_master_params_scene(scene_points)

    CURRENT_SCENE_DIR.mkdir(parents=True, exist_ok=True)
    save_yaml(SCENE_POINTS_FILE, scene_points)
    save_yaml(MASTER_PARAMS_SCENE_FILE, master_params_scene)
    with open(SCENE_GRAPH_FILE, "w", encoding="utf-8") as output_file:
        json.dump(route_graph, output_file, ensure_ascii=False, indent=2)
    shutil.copy2(bundle_path, SCENE_BUNDLE_COPY)

    print("Scene runtime compiled successfully:")
    print(f"  bundle:       {bundle_path}")
    print(f"  scene_points: {SCENE_POINTS_FILE}")
    print(f"  route_graph:  {SCENE_GRAPH_FILE}")
    print(f"  params:       {MASTER_PARAMS_SCENE_FILE}")
    print(f"  anchors:      {len(scene_points['anchor_ids'])}")
    print(f"  destinations: {len(scene_points['destination_ids'])}")
    print(f"  edges:        {len(scene_points['edges'])}")


if __name__ == "__main__":
    main()
