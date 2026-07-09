#!/usr/bin/env python3
"""
Compile a QGIS-exported GeoJSON route layer into a nav-gps scene bundle.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import unicodedata
from datetime import datetime
from pathlib import Path

import yaml


RUNTIME_SCENE_BUNDLE = (
    Path.home() / "XJTLU-autonomous-vehicle/runtime-data/gnss/scene_gps_bundle.yaml"
)
EARTH_RADIUS_M = 6378137.0
NON_ASCII_NAME_MAP = {
    "影视艺术学院": "film_school",
}


def normalize_name(name: str) -> str:
    mapped = NON_ASCII_NAME_MAP.get(name, name)
    ascii_name = unicodedata.normalize("NFKD", mapped).encode("ascii", "ignore").decode("ascii")
    normalized = re.sub(r"[^A-Za-z0-9]+", "_", ascii_name).strip("_").lower()
    return normalized or "dest"


def route_key(coordinate: list[float] | tuple[float, float]) -> tuple[float, float]:
    return round(float(coordinate[0]), 7), round(float(coordinate[1]), 7)


def haversine_m(a: tuple[float, float], b: tuple[float, float]) -> float:
    lon1, lat1 = map(math.radians, a)
    lon2, lat2 = map(math.radians, b)
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    h = (
        math.sin(dlat * 0.5) ** 2
        + math.cos(lat1) * math.cos(lat2) * math.sin(dlon * 0.5) ** 2
    )
    return 2.0 * EARTH_RADIUS_M * math.atan2(math.sqrt(h), math.sqrt(1.0 - h))


def densify_segment(
    start: tuple[float, float],
    end: tuple[float, float],
    max_step_m: float,
) -> list[tuple[float, float]]:
    distance_m = haversine_m(start, end)
    steps = max(1, int(math.ceil(distance_m / max(max_step_m, 0.1))))
    points = []
    for index in range(1, steps + 1):
        ratio = index / steps
        points.append(
            (
                start[0] + (end[0] - start[0]) * ratio,
                start[1] + (end[1] - start[1]) * ratio,
            )
        )
    return points


def polygon_points(geometry: dict) -> list[list[float]]:
    if geometry.get("type") == "Polygon":
        return list(geometry.get("coordinates", [[]])[0])
    if geometry.get("type") == "MultiPolygon":
        return list(geometry.get("coordinates", [[[[]]]])[0][0])
    return []


def polygon_centroid(geometry: dict) -> tuple[float, float] | None:
    points = polygon_points(geometry)
    if not points:
        return None
    if points[0] == points[-1]:
        points = points[:-1]
    if not points:
        return None
    return (
        sum(float(point[0]) for point in points) / len(points),
        sum(float(point[1]) for point in points) / len(points),
    )


def add_named_destination(
    nodes: dict[int, dict],
    node_id: int,
    desired_name: str,
    used_names: set[str],
) -> None:
    node = nodes[node_id]
    if node["dest"]:
        return

    base_name = normalize_name(desired_name)
    name = base_name
    suffix = 2
    while name in used_names:
        name = f"{base_name}_{suffix}"
        suffix += 1

    node["name"] = name
    node["dest"] = True
    used_names.add(name)


def nearest_node_id(
    coordinates: list[tuple[float, float]],
    target: tuple[float, float],
) -> tuple[int, float]:
    best_id = 1
    best_distance = math.inf
    for index, coordinate in enumerate(coordinates, start=1):
        distance = haversine_m(coordinate, target)
        if distance < best_distance:
            best_id = index
            best_distance = distance
    return best_id, best_distance


def compile_qgis_scene(
    geojson_path: str | Path,
    *,
    scene_name: str,
    densify_step_m: float = 5.0,
    building_snap_max_m: float = 80.0,
) -> dict:
    with open(Path(geojson_path).expanduser(), "r", encoding="utf-8") as geojson_file:
        geojson = json.load(geojson_file)

    route_features = [
        feature
        for feature in geojson.get("features", [])
        if feature.get("properties", {}).get("feature_type") == "route"
        and feature.get("geometry", {}).get("type") == "LineString"
    ]
    if not route_features:
        raise ValueError("QGIS GeoJSON has no LineString features with feature_type=route")

    coordinates: list[tuple[float, float]] = []
    coordinate_to_id: dict[tuple[float, float], int] = {}
    edges: set[tuple[int, int]] = set()

    def get_node_id(coordinate: tuple[float, float]) -> int:
        key = route_key(coordinate)
        if key not in coordinate_to_id:
            coordinate_to_id[key] = len(coordinates) + 1
            coordinates.append(key)
        return coordinate_to_id[key]

    for feature in route_features:
        raw_coords = [route_key(coord) for coord in feature["geometry"]["coordinates"]]
        if len(raw_coords) < 2:
            continue

        previous_id = get_node_id(raw_coords[0])
        for start, end in zip(raw_coords, raw_coords[1:]):
            for point in densify_segment(start, end, densify_step_m):
                current_id = get_node_id(point)
                if current_id != previous_id:
                    edges.add(tuple(sorted((previous_id, current_id))))
                previous_id = current_id

    if not edges:
        raise ValueError("QGIS route layer did not produce any graph edges")

    mean_lon = sum(lon for lon, _ in coordinates) / len(coordinates)
    mean_lat = sum(lat for _, lat in coordinates) / len(coordinates)
    fixed_origin_id, _ = nearest_node_id(coordinates, (mean_lon, mean_lat))

    degree = {node_id: 0 for node_id in range(1, len(coordinates) + 1)}
    for a, b in edges:
        degree[a] += 1
        degree[b] += 1

    timestamp = datetime.now().isoformat()
    nodes: dict[int, dict] = {}
    for node_id, (lon, lat) in enumerate(coordinates, start=1):
        nodes[node_id] = {
            "name": f"node_{node_id:03d}",
            "lat": float(lat),
            "lon": float(lon),
            "alt": 0.0,
            "anchor": node_id == fixed_origin_id or degree[node_id] == 1 or degree[node_id] >= 3,
            "dest": False,
            "samples": 0,
            "spread_m": 0.0,
            "source": "qgis_geojson",
            "time": timestamp,
        }

    used_names = {node["name"] for node in nodes.values()}
    for feature in geojson.get("features", []):
        properties = feature.get("properties", {})
        if properties.get("feature_type") != "building":
            continue
        raw_name = (
            properties.get("name:en")
            or properties.get("name")
            or properties.get("name:zh")
        )
        if not raw_name:
            continue
        centroid = polygon_centroid(feature.get("geometry", {}))
        if centroid is None:
            continue
        node_id, distance_m = nearest_node_id(coordinates, route_key(centroid))
        if distance_m <= building_snap_max_m:
            add_named_destination(nodes, node_id, raw_name, used_names)

    for node_id in sorted(degree):
        if degree[node_id] == 1:
            add_named_destination(nodes, node_id, f"route_end_{node_id}", used_names)
    for node_id in sorted(degree):
        if degree[node_id] >= 3:
            add_named_destination(nodes, node_id, f"junction_{node_id}", used_names)

    if not any(node["dest"] for node in nodes.values()):
        add_named_destination(nodes, fixed_origin_id, "route_origin", used_names)

    return {
        "scene_name": scene_name,
        "coordinate_source": "qgis_geojson",
        "fixed_origin_node_id": fixed_origin_id,
        "nodes": nodes,
        "edges": [list(edge) for edge in sorted(edges)],
    }


def max_edge_length_m(bundle: dict) -> float:
    nodes = bundle["nodes"]
    max_length = 0.0
    for a, b in bundle["edges"]:
        node_a = nodes[int(a)]
        node_b = nodes[int(b)]
        max_length = max(
            max_length,
            haversine_m(
                (float(node_a["lon"]), float(node_a["lat"])),
                (float(node_b["lon"]), float(node_b["lat"])),
            ),
        )
    return max_length


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Compile QGIS route GeoJSON into nav-gps scene bundle.")
    parser.add_argument("--input", required=True, help="QGIS GeoJSON containing feature_type=route lines.")
    parser.add_argument(
        "--output",
        default=str(RUNTIME_SCENE_BUNDLE),
        help="Output scene_gps_bundle.yaml path.",
    )
    parser.add_argument("--scene-name", default="qgis_scene", help="Scene name written to the bundle.")
    parser.add_argument(
        "--densify-step-m",
        type=float,
        default=5.0,
        help="Maximum graph edge length after densification.",
    )
    parser.add_argument(
        "--building-snap-max-m",
        type=float,
        default=80.0,
        help="Maximum distance for snapping building names to nearby route nodes.",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    bundle = compile_qgis_scene(
        args.input,
        scene_name=args.scene_name,
        densify_step_m=args.densify_step_m,
        building_snap_max_m=args.building_snap_max_m,
    )
    output_path = Path(args.output).expanduser()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "w", encoding="utf-8") as output_file:
        yaml.safe_dump(bundle, output_file, allow_unicode=True, sort_keys=False)

    anchors = sum(1 for node in bundle["nodes"].values() if node["anchor"])
    destinations = sorted(node["name"] for node in bundle["nodes"].values() if node["dest"])
    print("QGIS scene bundle compiled successfully:")
    print(f"  input:        {Path(args.input).expanduser()}")
    print(f"  output:       {output_path}")
    print(f"  scene:        {bundle['scene_name']}")
    print(f"  nodes:        {len(bundle['nodes'])}")
    print(f"  edges:        {len(bundle['edges'])}")
    print(f"  anchors:      {anchors}")
    print(f"  destinations: {len(destinations)}")
    print(f"  max_edge_m:   {max_edge_length_m(bundle):.2f}")
    for name in destinations[:30]:
        print(f"    - {name}")
    if len(destinations) > 30:
        print(f"    ... {len(destinations) - 30} more")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
