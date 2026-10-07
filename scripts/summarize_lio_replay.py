#!/usr/bin/env python3
"""Compact paired replay metrics; discrepancy is not ground truth accuracy."""
import argparse
import bisect
import json
import math
from pathlib import Path
import statistics


def quantile(values, fraction):
    if not values:
        return None
    values = sorted(values)
    return values[min(len(values)-1, round((len(values)-1)*fraction))]


def yaw(q):
    x, y, z, w = q
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("fastlio", type=Path)
    parser.add_argument("superlio", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    datasets = [json.loads(args.fastlio.read_text()), json.loads(args.superlio.read_text())]
    runs = []
    for data in datasets:
        odom, resources = data["odometry"], data["resources"]
        elapsed = resources[-1]["elapsed_s"] - resources[0]["elapsed_s"]
        cpu = resources[-1]["cpu_seconds"] - resources[0]["cpu_seconds"]
        delays = [p["sim_clock_minus_stamp_s"] for p in odom]
        runs.append({k: data[k] for k in ("kind", "binary_sha256", "config_sha256", "bag",
                                         "playback_exit", "odometry_count", "imu_received", "health_counts", "status")})
        runs[-1].update({"output_rate_hz": (len(odom)-1)/(odom[-1]["stamp"]-odom[0]["stamp"]),
                         "cpu_percent_one_core": 100*cpu/elapsed,
                         "rss_peak_mib": max(r["rss_kb"] for r in resources)/1024,
                         "sim_clock_minus_stamp_s": {"p50": quantile(delays, 0.50), "p95": quantile(delays, 0.95), "p99": quantile(delays, 0.99)},
                         "nonfinite_positions": sum(not all(math.isfinite(x) for x in p["position"]) for p in odom),
                         "header_monotonic": all(a["stamp"] < b["stamp"] for a,b in zip(odom,odom[1:]))})
        runs[-1].update({"duplicate_adjacent_stamps": sum(a["stamp"] == b["stamp"] for a,b in zip(odom,odom[1:])),
                         "decreasing_adjacent_stamps": sum(a["stamp"] > b["stamp"] for a,b in zip(odom,odom[1:]))})
    old, new = (d["odometry"] for d in datasets)
    old = sorted(old, key=lambda point: point["stamp"])
    stamps = [p["stamp"] for p in old]
    pairs = []
    for point in new:
        i = bisect.bisect_left(stamps, point["stamp"])
        options = [old[j] for j in (i-1,i) if 0 <= j < len(old)]
        nearest = min(options, key=lambda p: abs(p["stamp"]-point["stamp"]))
        if abs(nearest["stamp"]-point["stamp"]) <= 0.06:
            pairs.append((nearest, point))
    angle = yaw(pairs[0][0]["quaternion_xyzw"]) - yaw(pairs[0][1]["quaternion_xyzw"])
    c, s = math.cos(angle), math.sin(angle)
    old0, new0 = pairs[0]
    errors = []
    for a,b in pairs:
        dx,dy = b["position"][0]-new0["position"][0], b["position"][1]-new0["position"][1]
        transformed = (old0["position"][0]+c*dx-s*dy, old0["position"][1]+s*dx+c*dy)
        errors.append(math.dist(transformed, a["position"][:2]))
    result = {"runs": runs, "raw_scan_count": 2867, "alignment": "one fixed initial planar yaw/translation, no segment realignment",
              "paired_samples": len(pairs), "trajectory_discrepancy_m": {"rmse": math.sqrt(statistics.mean(e*e for e in errors)),
                    "p50": quantile(errors,.5), "p95": quantile(errors,.95), "max": max(errors), "last": errors[-1]},
              "limitations": ["FAST-LIO is not ground truth", "sensor/vehicle lever-arm equivalence remains unverified",
                               "RTK reference accuracy and static-period identification not evaluated",
                               "sim-clock minus source stamp includes scan-end convention and 50 Hz clock quantization; it is not compute latency",
                               "raw input scan count minus output count includes initialization; not labelled packet loss"]}
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
