"""Metadata-derived replay budget; no ROS or actuator dependencies."""
import math
from pathlib import Path

import yaml


def replay_contract(bag):
    info = yaml.safe_load((Path(bag) / "metadata.yaml").read_text())["rosbag2_bagfile_information"]
    duration = info["duration"]["nanoseconds"] * 1e-9
    counts = {entry["topic_metadata"]["name"]: entry["message_count"]
              for entry in info["topics_with_message_count"]}
    types = {entry["topic_metadata"]["name"]: entry["topic_metadata"]["type"]
             for entry in info["topics_with_message_count"]}
    expected = {"/livox/lidar": "livox_ros_driver2/msg/CustomMsg", "/livox/imu": "sensor_msgs/msg/Imu"}
    if not math.isfinite(duration) or duration <= 0 or any(
            types.get(name) != kind or counts.get(name, 0) <= 0 for name, kind in expected.items()):
        raise ValueError("requires a sealed positive-duration raw Livox/IMU bag")
    if any(not (Path(bag) / filename).is_file() for filename in info["relative_file_paths"]):
        raise ValueError("bag storage is incomplete")
    return {"duration_s": duration, "expected_raw_counts": {name: counts[name] for name in expected},
            "wall_budget_s": 2.0 + duration + max(20.0, duration * .1),
            "rule": "2 s startup + full recorded duration at 1x + max(20 s, 10%) EOF margin"}
