#!/usr/bin/env python3
"""Read original Livox/IMU bag fields; no ROS publishing and no truth claims."""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import statistics

from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(args.bag), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    classes = {name: get_message(types[name]) for name in ("/livox/lidar", "/livox/imu")}
    imu, scans, lines, tags = [], [], Counter(), Counter()
    while reader.has_next() and (len(imu) < 1000 or len(scans) < 50):
        topic, data, timestamp = reader.read_next()
        if topic not in classes:
            continue
        msg = deserialize_message(data, classes[topic])
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if topic == "/livox/imu" and len(imu) < 1000:
            a, w = msg.linear_acceleration, msg.angular_velocity
            imu.append({"stamp": stamp, "record_minus_header_s": timestamp * 1e-9 - stamp,
                        "a": [a.x, a.y, a.z], "w": [w.x, w.y, w.z]})
        elif topic == "/livox/lidar" and len(scans) < 50:
            points = list(msg.points)
            offsets = [p.offset_time for p in points]
            lines.update(p.line for p in points)
            tags.update(p.tag & 0x30 for p in points)
            old_count = new_count = 0
            for p in points[::4]:
                radius2 = p.x * p.x + p.y * p.y + p.z * p.z
                tag_ok = p.tag & 0x30 in (0, 0x10)
                old_count += int(p.line < 4 and tag_ok and 0.5 ** 2 <= radius2 <= 25 ** 2)
                new_count += int(tag_ok and 0.5 ** 2 < radius2 < 25 ** 2)
            scans.append({"stamp": stamp, "timebase": msg.timebase, "point_num": msg.point_num,
                          "array_size": len(points), "offset_max_s": max(offsets, default=0) * 1e-9,
                          "offsets_monotonic": all(a <= b for a, b in zip(offsets, offsets[1:])),
                          "old_filter_count": old_count, "upstream_filter_count": new_count,
                          "record_minus_header_s": timestamp * 1e-9 - stamp})
    file_hashes = {}
    for path in args.bag.glob("*.db3"):
        digest = hashlib.sha256()
        with path.open("rb") as stream:
            for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
                digest.update(block)
        file_hashes[path.name] = digest.hexdigest()
    result = {"bag": str(args.bag), "db3_sha256": file_hashes,
              "imu_samples": len(imu), "scan_samples": len(scans),
              "acceleration_norm_median_raw": statistics.median(math.sqrt(sum(v*v for v in m["a"])) for m in imu),
              "gyro_norm_median_raw": statistics.median(math.sqrt(sum(v*v for v in m["w"])) for m in imu),
              "imu_period_s_median": statistics.median(b["stamp"] - a["stamp"] for a, b in zip(imu, imu[1:])),
              "line_histogram": dict(lines), "tag_histogram": dict(tags),
              "sample_filter_counts_equal": all(s["old_filter_count"] == s["upstream_filter_count"] for s in scans),
              "scans": scans, "first_imu": imu[:20],
              "interpretation": "Raw bag scale/clock evidence only. Source FAST-LIO multiplies acceleration by 10; Super-LIO initial gravity normalization is a different convention. No calibration changed."}
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({k: v for k, v in result.items() if k not in {"scans", "first_imu"}}, indent=2))


if __name__ == "__main__":
    main()
