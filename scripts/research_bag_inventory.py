#!/usr/bin/env python3
"""Read bag metadata without replaying messages or accessing devices."""
import argparse
import hashlib
import json
from pathlib import Path

import yaml


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("roots", nargs="+", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    records = []
    for path in sorted({p for root in args.roots for p in root.rglob("metadata.yaml")}):
        try:
            content = path.read_bytes()
            info = yaml.safe_load(content)["rosbag2_bagfile_information"]
            topics = [{"name": item["topic_metadata"]["name"],
                       "type": item["topic_metadata"]["type"],
                       "count": item["message_count"]}
                      for item in info["topics_with_message_count"]]
            raw = [item for item in topics if item["count"] and
                   ("livox" in item["name"].lower() or "CustomMsg" in item["type"] or
                    item["type"] in {"sensor_msgs/msg/Imu", "sensor_msgs/msg/PointCloud2"})]
            records.append({"metadata_path": str(path),
                            "metadata_sha256": hashlib.sha256(content).hexdigest(),
                            "duration_s": info["duration"]["nanoseconds"] * 1e-9,
                            "message_count": info["message_count"],
                            "raw_sensor_candidates": raw, "topics": topics})
        except (KeyError, TypeError, ValueError, yaml.YAMLError) as error:
            records.append({"metadata_path": str(path), "error": str(error)})
    result = {"scope": "read-only metadata; derived/replay bags are not raw LIO validation",
              "bags": records}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({"bags": len(records), "raw_sensor_candidates": [
        {"path": r["metadata_path"], "topics": r["raw_sensor_candidates"]}
        for r in records if r.get("raw_sensor_candidates")]}, indent=2))


if __name__ == "__main__":
    main()
