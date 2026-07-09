from __future__ import annotations

import math
from pathlib import Path

import yaml

try:
    from pyproj import CRS, Transformer

    PYPROJ_AVAILABLE = True
except ImportError:
    CRS = None
    Transformer = None
    PYPROJ_AVAILABLE = False


EARTH_RADIUS_M = 6378137.0


def default_runtime_root() -> Path:
    return Path.home() / "XJTLU-autonomous-vehicle/runtime-data"


def default_scene_points_file() -> Path:
    return default_runtime_root() / "gnss" / "current_scene" / "scene_points.yaml"


def default_route_file() -> Path:
    return default_runtime_root() / "gnss" / "current_route.yaml"


def load_scene_points(scene_points_file: str | Path) -> dict:
    path = Path(scene_points_file).expanduser()
    with open(path, "r", encoding="utf-8") as scene_file:
        data = yaml.safe_load(scene_file) or {}

    raw_nodes = data.get("nodes", {})
    if not isinstance(raw_nodes, dict) or not raw_nodes:
        raise RuntimeError(f"scene points file has no nodes: {path}")

    nodes: dict[int, dict] = {}
    name_to_id: dict[str, int] = {}
    for raw_id, raw_node in raw_nodes.items():
        node_id = int(raw_id)
        node = {
            "id": node_id,
            "name": str(raw_node["name"]),
            "lat": float(raw_node["lat"]),
            "lon": float(raw_node["lon"]),
            "alt": float(raw_node.get("alt", 0.0)),
            "x": float(raw_node["x"]),
            "y": float(raw_node["y"]),
            "z": float(raw_node.get("z", 0.0)),
            "anchor": bool(raw_node.get("anchor", False)),
            "dest": bool(raw_node.get("dest", False)),
        }
        nodes[node_id] = node
        name_to_id[node["name"]] = node_id

    return {
        "scene_name": str(data.get("scene_name", "unknown_scene")),
        "fixed_origin": dict(data.get("fixed_origin", {})),
        "nodes": nodes,
        "name_to_id": name_to_id,
        "edges": [list(edge) for edge in data.get("edges", [])],
        "anchor_ids": [int(node_id) for node_id, node in nodes.items() if node["anchor"]],
        "destination_ids": [int(node_id) for node_id, node in nodes.items() if node["dest"]],
        "destination_names": {
            node["name"]: node_id for node_id, node in nodes.items() if node["dest"]
        },
    }


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    radius_m = 6371000.0
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (
        math.sin(dlat / 2.0) ** 2
        + math.cos(math.radians(lat1))
        * math.cos(math.radians(lat2))
        * math.sin(dlon / 2.0) ** 2
    )
    return radius_m * 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def compass_heading_to_enu_yaw_deg(heading_deg: float) -> float:
    # Compass heading is 0=north, clockwise positive; ENU yaw is 0=east, CCW positive.
    return (90.0 - float(heading_deg)) % 360.0


class LocalENUProjector:
    def __init__(self, origin_lat: float, origin_lon: float, origin_alt: float = 0.0) -> None:
        self.origin_lat = float(origin_lat)
        self.origin_lon = float(origin_lon)
        self.origin_alt = float(origin_alt)
        self._origin_lat_rad = math.radians(self.origin_lat)

    def forward(self, lat: float, lon: float) -> tuple[float, float]:
        x = math.radians(float(lon) - self.origin_lon) * EARTH_RADIUS_M * math.cos(self._origin_lat_rad)
        y = math.radians(float(lat) - self.origin_lat) * EARTH_RADIUS_M
        return float(x), float(y)


class FixedENUProjector:
    def __init__(self, origin_lat: float, origin_lon: float, origin_alt: float = 0.0) -> None:
        self.origin_lat = float(origin_lat)
        self.origin_lon = float(origin_lon)
        self.origin_alt = float(origin_alt)
        if PYPROJ_AVAILABLE:
            local_crs = CRS.from_proj4(
                f"+proj=aeqd +lat_0={self.origin_lat} +lon_0={self.origin_lon} "
                "+datum=WGS84 +units=m +no_defs"
            )
            self._forward = Transformer.from_crs("EPSG:4326", local_crs, always_xy=True)
        else:
            self._forward = LocalENUProjector(self.origin_lat, self.origin_lon, self.origin_alt)

    def forward(self, lat: float, lon: float) -> tuple[float, float]:
        if PYPROJ_AVAILABLE:
            x, y = self._forward.transform(float(lon), float(lat))
            return float(x), float(y)
        x, y = self._forward.forward(float(lat), float(lon))
        return float(x), float(y)
