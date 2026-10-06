#!/usr/bin/env python3
"""Extract the three parameter layers without importing ROS launch.

Layer 1 is the YAML parse, layer 2 is the corridor launch rewrite visible in
source, and layer 3 is deliberately ``PENDING_JETSON`` until a real
``ros2 param dump`` is captured from the target process.
"""
from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

import yaml


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    root = args.root
    master_path = root / "src/bringup/config/master_params.yaml"
    launch_path = root / "src/bringup/launch/system_gps_corridor.launch.py"
    master = yaml.safe_load(master_path.read_text())
    params = master["/fastlio2"]["lio_node"]["ros__parameters"]
    launch_text = launch_path.read_text()

    def launch_value(name: str):
        match = re.search(rf"{re.escape(name)}'\]\s*=\s*\[([^]]+)\]", launch_text)
        if not match:
            raise AssertionError(f"corridor launch override missing: {name}")
        return [float(v.strip()) for v in match.group(1).split(",")]

    report = {
        "source_layer": {k: params[k] for k in ["imu_topic", "lidar_topic", "lidar_min_range", "lidar_max_range", "map_resolution", "esti_il", "t_il", "r_il"]},
        "parsed_layer": {k: type(params[k]).__name__ for k in ["imu_topic", "lidar_topic", "lidar_min_range", "lidar_max_range", "map_resolution", "esti_il", "t_il", "r_il"]},
        "corridor_launch_override_layer": {
            "max_velocity": launch_value("max_velocity"),
            "min_velocity": launch_value("min_velocity"),
            "max_accel": launch_value("max_accel"),
            "max_decel": launch_value("max_decel"),
        },
        "runtime_layer": {"status": "PENDING_JETSON", "command": "ros2 param dump /fastlio2/lio_node and /corridor_cmd_vel_guard"},
    }
    assert report["corridor_launch_override_layer"]["max_velocity"] == [0.85, 0.0, 0.70]
    assert report["corridor_launch_override_layer"]["max_accel"] == [0.85, 0.0, 1.4]
    rendered = json.dumps(report, indent=2, sort_keys=True) + "\n"
    output = args.output or (root / "audit/vehicle_baseline/PARAMETER_LAYERS.json")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(rendered)
    print(rendered, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
