import math
from pathlib import Path

import yaml


def normalize_name(value):
    return " ".join(str(value).strip().casefold().split())


def load_destinations(path):
    source = Path(path).expanduser().resolve()
    data = yaml.safe_load(source.read_text(encoding="utf-8")) or {}
    if int(data.get("schema_version", 0)) != 1:
        raise ValueError("unsupported destinations schema")
    map_id = str(data.get("map_id", "")).strip()
    if not map_id:
        raise ValueError("map_id is required")
    destinations = data.get("destinations", {})
    if not isinstance(destinations, dict):
        raise ValueError("destinations must be a mapping")
    by_name = {}
    canonical = {}
    for destination_id, raw in destinations.items():
        if not isinstance(raw, dict) or not isinstance(raw.get("pose"), dict):
            raise ValueError(f"destination {destination_id} is missing pose")
        pose = raw["pose"]
        aliases = raw.get("aliases", [])
        if not isinstance(aliases, list):
            raise ValueError(f"destination {destination_id} aliases must be a list")
        entry = {
            "id": str(destination_id),
            "display_name": str(raw.get("display_name", destination_id)),
            "x": float(pose["x"]),
            "y": float(pose["y"]),
            "yaw": float(pose.get("yaw", 0.0)),
            "approach": str(raw.get("approach", "any")),
        }
        if not all(math.isfinite(entry[key]) for key in ("x", "y", "yaw")):
            raise ValueError(f"destination {destination_id} pose must be finite")
        canonical[entry["id"]] = entry
        names = [entry["id"], entry["display_name"], *aliases]
        for name in names:
            key = normalize_name(name)
            if not key:
                continue
            if key in by_name and by_name[key]["id"] != entry["id"]:
                raise ValueError(f"duplicate destination alias: {name}")
            by_name[key] = entry
    return {"map_id": map_id, "canonical": canonical, "by_name": by_name}


def resolve_destination(catalog, name):
    return catalog["by_name"].get(normalize_name(name))
