#!/usr/bin/env python3
"""Classify existing bags and identify identical raw inputs without replay.

Only reads sealed SQLite bags. Derived odometry is useful for diagnostics,
but never qualifies as input evidence for a replacement LIO front end.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sqlite3

import yaml


import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"src/research_runtime"))
from research_runtime.bag_identity import RAW_TYPES,raw_fingerprint


def catalog(records: list[dict]) -> dict:
    bags, groups = [], {}
    for record in records:
        path = Path(record["metadata_path"])
        row = {"metadata_path": str(path), "bag_path": str(path.parent),
               "category": "UNREADABLE", "error": None}
        try:
            content = path.read_bytes()
            info = yaml.safe_load(content)["rosbag2_bagfile_information"]
            topics = {item["topic_metadata"]["name"]: {
                "type": item["topic_metadata"]["type"], "count": item["message_count"]}
                for item in info["topics_with_message_count"]}
            row.update(metadata_sha256=hashlib.sha256(content).hexdigest(),
                       duration_s=info["duration"]["nanoseconds"] * 1e-9,
                       message_count=info["message_count"], topics=topics)
            complete_raw = all(topics.get(name, {}).get("type") == kind and
                               topics[name]["count"] > 0 for name, kind in RAW_TYPES.items())
            if complete_raw and info["storage_identifier"] == "sqlite3":
                row.update(raw_fingerprint(path.parent, info))
                derived = any(part.startswith("replay") or part in {"derived", "converted", "resampled"}
                              for part in path.parts)
                row["category"] = "REPLAY_DERIVED_RAW" if derived else "ORIGINAL_RAW_LIO"
            elif complete_raw:
                row["category"] = "RAW_STORAGE_UNSUPPORTED"
            elif any(value["count"] and value["type"] == "nav_msgs/msg/Odometry"
                     for value in topics.values()):
                row["category"] = "LOCALIZATION_LOG_ONLY"
            else:
                row["category"] = "NO_COMPLETE_RAW_PAIR"
            if path.read_bytes()!=content:
                raise ValueError("metadata changed while cataloging; not sealed")
            if row["category"] in ("ORIGINAL_RAW_LIO","REPLAY_DERIVED_RAW"):
                groups.setdefault(row["raw_input_sha256"], []).append(row)
        except (OSError, KeyError, TypeError, ValueError, sqlite3.Error, yaml.YAMLError) as exc:
            row["error"] = str(exc)
            row["category"] = "UNREADABLE"
        bags.append(row)
    selected = []
    for members in groups.values():
        originals = [row for row in members if row["category"] == "ORIGINAL_RAW_LIO"]
        preferred = min(originals, key=lambda row: (len(Path(row["bag_path"]).parts), row["bag_path"])) if originals else None
        for row in members:
            row["identical_raw_input_paths"] = [other["bag_path"] for other in members
                                                if other is not row]
            row["selected_original"] = row is preferred
        if preferred:
            selected.append(preferred["bag_path"])
    return {"scope": "read-only sealed metadata and full ordered raw CDR hashes; no source bag modifications",
            "comparison_rule": "recording timestamps/rates do not make identical raw payloads independent experiments",
            "bag_count": len(bags), "distinct_raw_input_count": len(groups),
            "selected_original_bags": selected, "bags": bags}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--inventory", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = catalog(json.loads(args.inventory.read_text())["bags"])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({key: result[key] for key in
                      ("bag_count", "distinct_raw_input_count", "selected_original_bags")}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
