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

import yaml

try:
    from pyproj import Transformer

    PYPROJ_AVAILABLE = True
except ImportError:
    Transformer = None
    PYPROJ_AVAILABLE = False

RUNTIME_ROOT = Path.home() / "XJTLU-autonomous-vehicle/runtime-data"
DEFAULT_BUNDLE = RUNTIME_ROOT / "gnss" / "scene_gps_bundle.yaml"
CURRENT_SCENE_DIR = RUNTIME_ROOT / "gnss" / "current_scene"
REPO_ROOT = Path(__file__).resolve().parents[1]
MASTER_PARAMS_TEMPLATE = REPO_ROOT / "src" / "bringup" / "config" / "master_params.yaml"
SCENE_POINTS_FILE = CURRENT_SCENE_DIR / "scene_points.yaml"
SCENE_GRAPH_FILE = CURRENT_SCENE_DIR / "scene_route_graph.geojson"
MASTER_PARAMS_SCENE_FILE = CURRENT_SCENE_DIR / "master_params_scene.yaml"
SCENE_BUNDLE_COPY = CURRENT_SCENE_DIR / "scene_gps_bundle.yaml"
ROAD_KEEPOUT_YAML = CURRENT_SCENE_DIR / "road_keepout.yaml"
ROAD_KEEPOUT_IMAGE = CURRENT_SCENE_DIR / "road_keepout.pgm"
ENGLISH_NAME_RE = re.compile(r"^[A-Za-z0-9_-]+$")
EARTH_RADIUS_M = 6378137.0
ROUTE_TOPOLOGY_EPSILON_M = 1e-4
ROUTE_TOPOLOGY_RATIO_EPSILON = 1e-6
ROUTE_NEAR_JUNCTION_TOLERANCE_M = 0.25


class LocalENUProjector:
    def __init__(self, origin_lat: float, origin_lon: float, origin_alt: float) -> None:
        self.origin_lat = float(origin_lat)
        self.origin_lon = float(origin_lon)
        self.origin_alt = float(origin_alt)
        self._origin_lat_rad = math.radians(self.origin_lat)

    def transform(
        self,
        lon: float,
        lat: float,
        alt: float,
        radians: bool = False,
    ) -> tuple[float, float, float]:
        if radians:
            lon_deg = math.degrees(float(lon))
            lat_deg = math.degrees(float(lat))
        else:
            lon_deg = float(lon)
            lat_deg = float(lat)
        x = (
            math.radians(lon_deg - self.origin_lon)
            * EARTH_RADIUS_M
            * math.cos(self._origin_lat_rad)
        )
        y = math.radians(lat_deg - self.origin_lat) * EARTH_RADIUS_M
        z = float(alt) - self.origin_alt
        return float(x), float(y), float(z)


def load_yaml(path: Path) -> dict:
    with open(path, "r", encoding="utf-8") as input_file:
        return yaml.safe_load(input_file) or {}


def save_yaml(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as output_file:
        yaml.safe_dump(data, output_file, allow_unicode=True, sort_keys=False)


def build_transformer(origin_lat: float, origin_lon: float, origin_alt: float):
    if not PYPROJ_AVAILABLE:
        return LocalENUProjector(origin_lat, origin_lon, origin_alt)

    pipeline = (
        "+proj=pipeline "
        "+step +proj=cart +ellps=WGS84 "
        f"+step +proj=topocentric +ellps=WGS84 +lat_0={origin_lat} "
        f"+lon_0={origin_lon} +h_0={origin_alt}"
    )
    return Transformer.from_pipeline(pipeline)


def latlon_to_enu(
    transformer: Transformer, lat: float, lon: float, alt: float
) -> tuple[float, float, float]:
    x, y, z = transformer.transform(lon, lat, alt, radians=False)
    return float(x), float(y), float(z)


def _segment_intersection(
    a: tuple[float, float],
    b: tuple[float, float],
    c: tuple[float, float],
    d: tuple[float, float],
) -> tuple[float, float] | None:
    """Return a proper segment crossing, excluding shared endpoints."""

    ab_x, ab_y = b[0] - a[0], b[1] - a[1]
    cd_x, cd_y = d[0] - c[0], d[1] - c[1]
    denominator = ab_x * cd_y - ab_y * cd_x
    if abs(denominator) <= ROUTE_TOPOLOGY_EPSILON_M**2:
        return None

    ac_x, ac_y = c[0] - a[0], c[1] - a[1]
    first_ratio = (ac_x * cd_y - ac_y * cd_x) / denominator
    second_ratio = (ac_x * ab_y - ac_y * ab_x) / denominator
    if not (
        ROUTE_TOPOLOGY_RATIO_EPSILON < first_ratio < 1.0 - ROUTE_TOPOLOGY_RATIO_EPSILON
        and ROUTE_TOPOLOGY_RATIO_EPSILON < second_ratio < 1.0 - ROUTE_TOPOLOGY_RATIO_EPSILON
    ):
        return None
    return first_ratio, second_ratio


def _project_to_segment(
    point: tuple[float, float],
    start: tuple[float, float],
    end: tuple[float, float],
) -> tuple[float, float]:
    delta_x, delta_y = end[0] - start[0], end[1] - start[1]
    length_sq = delta_x * delta_x + delta_y * delta_y
    if length_sq <= ROUTE_TOPOLOGY_EPSILON_M:
        return 0.0, math.hypot(point[0] - start[0], point[1] - start[1])
    ratio = ((point[0] - start[0]) * delta_x + (point[1] - start[1]) * delta_y) / length_sq
    projected = (start[0] + ratio * delta_x, start[1] + ratio * delta_y)
    return ratio, math.hypot(point[0] - projected[0], point[1] - projected[1])


def split_route_intersections(
    nodes: dict[int, dict], edges: list[list[int]]
) -> tuple[list[list[int]], int]:
    """Insert graph nodes where two route segments meet.

    Scene bundles often originate from separate QGIS LineStrings. A visual
    crossing is not a graph connection unless it has an explicit shared node.
    Resolving proper crossings and sub-25cm endpoint gaps at runtime keeps
    legacy bundles traversable without changing their road geometry.
    """

    normalized_edges = [tuple(sorted((int(a), int(b)))) for a, b in edges]
    normalized_edges = sorted(set(normalized_edges))
    split_points: dict[tuple[int, int], list[tuple[float, int]]] = {
        edge: [(0.0, edge[0]), (1.0, edge[1])] for edge in normalized_edges
    }
    intersection_ids: dict[tuple[float, float], int] = {}
    used_names = {str(node["name"]).lower() for node in nodes.values()}
    next_id = max(nodes) + 1
    mean_lat_rad = math.radians(
        sum(float(node["lat"]) for node in nodes.values()) / len(nodes)
    )
    lon_scale = EARTH_RADIUS_M * math.pi / 180.0 * math.cos(mean_lat_rad)
    lat_scale = EARTH_RADIUS_M * math.pi / 180.0
    connector_edges: set[tuple[int, int]] = set()

    def local_point(node_id: int) -> tuple[float, float]:
        node = nodes[node_id]
        return float(node["lon"]) * lon_scale, float(node["lat"]) * lat_scale

    def make_intersection_node(
        edge: tuple[int, int], ratio: float
    ) -> int:
        nonlocal next_id
        start_id, end_id = edge
        start, end = nodes[start_id], nodes[end_id]
        lon = float(start["lon"]) + ratio * (float(end["lon"]) - float(start["lon"]))
        lat = float(start["lat"]) + ratio * (float(end["lat"]) - float(start["lat"]))
        key = (round(lon, 9), round(lat, 9))
        node_id = intersection_ids.get(key)
        if node_id is not None:
            return node_id
        name = f"intersection_{next_id}"
        suffix = 2
        while name.lower() in used_names:
            name = f"intersection_{next_id}_{suffix}"
            suffix += 1
        used_names.add(name.lower())
        node_id = next_id
        next_id += 1
        intersection_ids[key] = node_id
        start_alt = float(start["alt"])
        end_alt = float(end["alt"])
        nodes[node_id] = {
            "id": node_id,
            "name": name,
            "lat": lat,
            "lon": lon,
            "alt": start_alt + ratio * (end_alt - start_alt),
            "anchor": True,
            "dest": False,
            "samples": 0,
            "spread_m": 0.0,
            "source": "route_intersection",
            "time": "",
        }
        return node_id

    for index, first_edge in enumerate(normalized_edges):
        first_start, first_end = first_edge
        first_a = local_point(first_start)
        first_b = local_point(first_end)
        for second_edge in normalized_edges[index + 1 :]:
            if set(first_edge) & set(second_edge):
                continue
            second_start, second_end = second_edge
            second_a = local_point(second_start)
            second_b = local_point(second_end)
            crossing = _segment_intersection(first_a, first_b, second_a, second_b)
            if crossing is not None:
                first_ratio, second_ratio = crossing
                node_id = make_intersection_node(first_edge, first_ratio)
                split_points[first_edge].append((first_ratio, node_id))
                split_points[second_edge].append((second_ratio, node_id))
                continue

            for endpoint_id, endpoint in (
                (first_start, first_a),
                (first_end, first_b),
            ):
                second_ratio, distance_m = _project_to_segment(
                    endpoint, second_a, second_b
                )
                if (
                    ROUTE_TOPOLOGY_RATIO_EPSILON
                    < second_ratio
                    < 1.0 - ROUTE_TOPOLOGY_RATIO_EPSILON
                    and distance_m <= ROUTE_NEAR_JUNCTION_TOLERANCE_M
                ):
                    node_id = make_intersection_node(second_edge, second_ratio)
                    split_points[second_edge].append((second_ratio, node_id))
                    connector_edges.add(tuple(sorted((endpoint_id, node_id))))
            for endpoint_id, endpoint in (
                (second_start, second_a),
                (second_end, second_b),
            ):
                first_ratio, distance_m = _project_to_segment(
                    endpoint, first_a, first_b
                )
                if (
                    ROUTE_TOPOLOGY_RATIO_EPSILON
                    < first_ratio
                    < 1.0 - ROUTE_TOPOLOGY_RATIO_EPSILON
                    and distance_m <= ROUTE_NEAR_JUNCTION_TOLERANCE_M
                ):
                    node_id = make_intersection_node(first_edge, first_ratio)
                    split_points[first_edge].append((first_ratio, node_id))
                    connector_edges.add(tuple(sorted((endpoint_id, node_id))))

    if not intersection_ids:
        return [list(edge) for edge in normalized_edges], 0

    expanded_edges: set[tuple[int, int]] = set()
    for edge in normalized_edges:
        ordered = sorted(split_points[edge], key=lambda item: item[0])
        ordered_ids: list[int] = []
        for _, node_id in ordered:
            if not ordered_ids or ordered_ids[-1] != node_id:
                ordered_ids.append(node_id)
        for start_id, end_id in zip(ordered_ids, ordered_ids[1:]):
            if start_id != end_id:
                expanded_edges.add(tuple(sorted((start_id, end_id))))
    expanded_edges.update(connector_edges)
    return [list(edge) for edge in sorted(expanded_edges)], len(intersection_ids)


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

    edges, intersection_node_count = split_route_intersections(nodes, edges)

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

    return nodes, edges, {
        "origin_id": origin_id,
        "anchor_ids": anchors,
        "destination_ids": destinations,
        "intersection_node_count": intersection_node_count,
    }


def build_scene_points(
    bundle: dict,
    nodes: dict[int, dict],
    edges: list[list[int]],
    origin_id: int,
) -> dict:
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
        "drivable_area": dict(bundle.get("drivable_area", {})),
    }


def install_keepout_assets(bundle: dict, bundle_path: Path) -> Path | None:
    drivable_area = bundle.get("drivable_area", {})
    source_yaml_name = str(drivable_area.get("keepout_map_yaml", "")).strip()
    if not source_yaml_name:
        return None
    source_yaml = Path(source_yaml_name).expanduser()
    if not source_yaml.is_absolute():
        source_yaml = bundle_path.parent / source_yaml
    if not source_yaml.exists():
        raise ValueError(f"road keepout YAML not found: {source_yaml}")
    map_metadata = load_yaml(source_yaml)
    source_image = Path(str(map_metadata.get("image", ""))).expanduser()
    if not source_image.is_absolute():
        source_image = source_yaml.parent / source_image
    if not source_image.exists():
        raise ValueError(f"road keepout image not found: {source_image}")

    if source_image.resolve() != ROAD_KEEPOUT_IMAGE.resolve():
        shutil.copy2(source_image, ROAD_KEEPOUT_IMAGE)
    map_metadata["image"] = ROAD_KEEPOUT_IMAGE.name
    save_yaml(ROAD_KEEPOUT_YAML, map_metadata)
    return ROAD_KEEPOUT_YAML


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
        distance = math.hypot(
            float(node_b["x"]) - float(node_a["x"]),
            float(node_b["y"]) - float(node_a["y"]),
        )

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

    pgo_params = (
        params.setdefault("/pgo", {})
        .setdefault("pgo_node", {})
        .setdefault("ros__parameters", {})
    )
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
            "require_nav_ready": False,
            "controller_id": "FollowPath",
            "goal_checker_id": "general_goal_checker",
            "max_route_snap_distance_m": 8.0,
            "path_density_m": 0.20,
            "goal_success_tolerance_m": 1.0,
            "goal_pose_topic": "/goal_pose",
            "geo_goal_topic": "/gps_goal",
            "stop_override_topic": "/gps_nav/stop_override",
            "motion_allowed_topic": "/localization_authority/motion_allowed",
            "authority_status_topic": "/localization_authority/status",
            "lio_odom_topic": "/fastlio2/lio_odom",
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
    keepout_yaml = install_keepout_assets(raw_bundle, bundle_path)
    save_yaml(SCENE_POINTS_FILE, scene_points)
    save_yaml(MASTER_PARAMS_SCENE_FILE, master_params_scene)
    with open(SCENE_GRAPH_FILE, "w", encoding="utf-8") as output_file:
        json.dump(route_graph, output_file, ensure_ascii=False, indent=2)
    if bundle_path.resolve() != SCENE_BUNDLE_COPY.resolve():
        shutil.copy2(bundle_path, SCENE_BUNDLE_COPY)

    print("Scene runtime compiled successfully:")
    print(f"  bundle:       {bundle_path}")
    print(f"  scene_points: {SCENE_POINTS_FILE}")
    print(f"  route_graph:  {SCENE_GRAPH_FILE}")
    print(f"  params:       {MASTER_PARAMS_SCENE_FILE}")
    print(f"  anchors:      {len(scene_points['anchor_ids'])}")
    print(f"  destinations: {len(scene_points['destination_ids'])}")
    print(f"  edges:        {len(scene_points['edges'])}")
    print(f"  intersections:{meta['intersection_node_count']}")
    print(f"  road_keepout: {keepout_yaml if keepout_yaml else 'not configured'}")


if __name__ == "__main__":
    main()
