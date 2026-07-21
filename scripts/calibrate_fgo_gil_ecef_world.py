#!/usr/bin/env python3
"""Fit the FGO LiDAR-world to ECEF transform from an RTK Fixed driving bag."""

from __future__ import annotations

import argparse
import bisect
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Sequence

import yaml


@dataclass(frozen=True)
class TimedPosition:
    timestamp_ns: int
    position: tuple[float, float, float]


@dataclass(frozen=True)
class TimedPose:
    timestamp_ns: int
    position: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]


def wgs84_to_ecef(latitude_deg: float, longitude_deg: float, altitude_m: float):
    import numpy as np

    semi_major_axis_m = 6378137.0
    eccentricity_squared = 6.69437999014e-3
    latitude = math.radians(latitude_deg)
    longitude = math.radians(longitude_deg)
    sin_latitude = math.sin(latitude)
    prime_vertical = semi_major_axis_m / math.sqrt(
        1.0 - eccentricity_squared * sin_latitude * sin_latitude
    )
    return np.asarray(
        [
            (prime_vertical + altitude_m) * math.cos(latitude) * math.cos(longitude),
            (prime_vertical + altitude_m) * math.cos(latitude) * math.sin(longitude),
            (prime_vertical * (1.0 - eccentricity_squared) + altitude_m) * sin_latitude,
        ],
        dtype=float,
    )


def quaternion_rotation_xyzw(quaternion: Sequence[float]):
    import numpy as np

    x, y, z, w = (float(value) for value in quaternion)
    magnitude = math.sqrt(w * w + x * x + y * y + z * z)
    if not math.isfinite(magnitude) or magnitude < 1.0e-12:
        raise ValueError("invalid zero or non-finite quaternion")
    x, y, z, w = x / magnitude, y / magnitude, z / magnitude, w / magnitude
    return np.asarray(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def rotation_quaternion_wxyz(rotation) -> tuple[float, float, float, float]:
    import numpy as np

    matrix = np.asarray(rotation, dtype=float)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = (
            0.25 * scale,
            (matrix[2, 1] - matrix[1, 2]) / scale,
            (matrix[0, 2] - matrix[2, 0]) / scale,
            (matrix[1, 0] - matrix[0, 1]) / scale,
        )
    else:
        axis = int(np.argmax(np.diag(matrix)))
        if axis == 0:
            scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
            quaternion = (
                (matrix[2, 1] - matrix[1, 2]) / scale,
                0.25 * scale,
                (matrix[0, 1] + matrix[1, 0]) / scale,
                (matrix[0, 2] + matrix[2, 0]) / scale,
            )
        elif axis == 1:
            scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
            quaternion = (
                (matrix[0, 2] - matrix[2, 0]) / scale,
                (matrix[0, 1] + matrix[1, 0]) / scale,
                0.25 * scale,
                (matrix[1, 2] + matrix[2, 1]) / scale,
            )
        else:
            scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
            quaternion = (
                (matrix[1, 0] - matrix[0, 1]) / scale,
                (matrix[0, 2] + matrix[2, 0]) / scale,
                (matrix[1, 2] + matrix[2, 1]) / scale,
                0.25 * scale,
            )
    if quaternion[0] < 0.0:
        quaternion = tuple(-value for value in quaternion)
    return tuple(float(value) for value in quaternion)


def lidar_pose_to_local_antenna(
    pose: TimedPose,
    imu_lidar_translation: Sequence[float],
    imu_lidar_rotation_wxyz: Sequence[float],
    master_in_imu: Sequence[float],
):
    import numpy as np

    world_lidar_rotation = quaternion_rotation_xyzw(pose.quaternion_xyzw)
    w, x, y, z = (float(value) for value in imu_lidar_rotation_wxyz)
    imu_lidar_rotation = quaternion_rotation_xyzw((x, y, z, w))
    antenna_lidar = imu_lidar_rotation.T @ (
        np.asarray(master_in_imu, dtype=float)
        - np.asarray(imu_lidar_translation, dtype=float)
    )
    return np.asarray(pose.position, dtype=float) + world_lidar_rotation @ antenna_lidar


def fit_rigid_transform(source, destination):
    import numpy as np

    source = np.asarray(source, dtype=float)
    destination = np.asarray(destination, dtype=float)
    if source.shape != destination.shape or source.ndim != 2 or source.shape[1] != 3:
        raise ValueError("source and destination must be matching Nx3 arrays")
    if source.shape[0] < 3 or not np.all(np.isfinite(source)) or not np.all(np.isfinite(destination)):
        raise ValueError("at least three finite point pairs are required")
    source_center = source.mean(axis=0)
    destination_center = destination.mean(axis=0)
    covariance = (source - source_center).T @ (destination - destination_center)
    left, singular_values, right_transpose = np.linalg.svd(covariance)
    rotation = right_transpose.T @ left.T
    if np.linalg.det(rotation) < 0.0:
        right_transpose[-1, :] *= -1.0
        rotation = right_transpose.T @ left.T
    translation = destination_center - rotation @ source_center
    residuals = np.linalg.norm((rotation @ source.T).T + translation - destination, axis=1)
    return rotation, translation, singular_values, residuals


def robust_fit(source, destination):
    import numpy as np

    rotation, translation, singular_values, residuals = fit_rigid_transform(source, destination)
    median = float(np.median(residuals))
    mad = float(np.median(np.abs(residuals - median)))
    threshold = max(0.20, median + 3.0 * 1.4826 * mad)
    inliers = residuals <= threshold
    if int(np.count_nonzero(inliers)) < 10:
        raise ValueError("fewer than ten calibration inliers remain")
    rotation, translation, singular_values, residuals = fit_rigid_transform(
        np.asarray(source)[inliers], np.asarray(destination)[inliers]
    )
    return rotation, translation, singular_values, residuals, inliers, threshold


def gga_quality(sentence: str) -> int | None:
    payload = sentence.strip().split("*", 1)[0]
    fields = payload.split(",")
    if len(fields) < 7 or not fields[0].lstrip("$").endswith("GGA"):
        return None
    try:
        return int(fields[6])
    except ValueError:
        return None


def message_stamp_ns(message: Any, fallback_ns: int) -> int:
    stamp = getattr(getattr(message, "header", None), "stamp", None)
    value = int(getattr(stamp, "sec", 0)) * 1_000_000_000 + int(
        getattr(stamp, "nanosec", 0)
    )
    return value if value > 0 else int(fallback_ns)


def nearest(items: Sequence[Any], timestamp_ns: int, tolerance_ns: int):
    stamps = [item.timestamp_ns for item in items]
    index = bisect.bisect_left(stamps, timestamp_ns)
    candidates = items[max(0, index - 1) : min(len(items), index + 1)]
    if not candidates:
        return None
    selected = min(candidates, key=lambda item: abs(item.timestamp_ns - timestamp_ns))
    return selected if abs(selected.timestamp_ns - timestamp_ns) <= tolerance_ns else None


def read_bag_samples(
    bag: Path,
    fix_topic: str,
    lio_topic: str,
    nmea_topic: str,
    required_quality: int,
    quality_tolerance_s: float,
) -> tuple[list[TimedPosition], list[TimedPose], dict[str, int]]:
    try:
        import rosbag2_py  # type: ignore
        from rclpy.serialization import deserialize_message  # type: ignore
        from rosidl_runtime_py.utilities import get_message  # type: ignore
    except ImportError as error:
        raise RuntimeError("source ROS 2 and the workspace before reading a bag") from error

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    missing = [topic for topic in (fix_topic, lio_topic, nmea_topic) if topic not in topic_types]
    if missing:
        raise RuntimeError("bag is missing calibration topics: " + ", ".join(missing))
    classes = {
        topic: get_message(topic_types[topic]) for topic in (fix_topic, lio_topic, nmea_topic)
    }
    fixes: list[TimedPosition] = []
    poses: list[TimedPose] = []
    quality_stamps: list[int] = []
    counts = {"fix_messages": 0, "fixed_gga": 0, "lio_messages": 0}
    while reader.has_next():
        topic, data, storage_ns = reader.read_next()
        if topic not in classes:
            continue
        message = deserialize_message(data, classes[topic])
        timestamp_ns = message_stamp_ns(message, storage_ns)
        if topic == nmea_topic:
            if gga_quality(str(message.sentence)) == required_quality:
                quality_stamps.append(timestamp_ns)
                counts["fixed_gga"] += 1
        elif topic == fix_topic:
            counts["fix_messages"] += 1
            values = (float(message.latitude), float(message.longitude), float(message.altitude))
            if int(message.status.status) >= 0 and all(math.isfinite(value) for value in values):
                fixes.append(TimedPosition(timestamp_ns, tuple(wgs84_to_ecef(*values))))
        else:
            counts["lio_messages"] += 1
            position = message.pose.pose.position
            orientation = message.pose.pose.orientation
            values = (
                float(position.x),
                float(position.y),
                float(position.z),
                float(orientation.x),
                float(orientation.y),
                float(orientation.z),
                float(orientation.w),
            )
            if all(math.isfinite(value) for value in values):
                poses.append(TimedPose(timestamp_ns, values[:3], values[3:]))

    quality_stamps.sort()
    tolerance_ns = int(quality_tolerance_s * 1.0e9)
    fixed_fixes = []
    for fix in fixes:
        index = bisect.bisect_left(quality_stamps, fix.timestamp_ns)
        candidates = quality_stamps[max(0, index - 1) : min(len(quality_stamps), index + 1)]
        if candidates and min(abs(value - fix.timestamp_ns) for value in candidates) <= tolerance_ns:
            fixed_fixes.append(fix)
    counts["quality_verified_fixes"] = len(fixed_fixes)
    return fixed_fixes, sorted(poses, key=lambda item: item.timestamp_ns), counts


def parse_vector(value: str, expected: int) -> tuple[float, ...]:
    parsed = tuple(float(item) for item in value.split(","))
    if len(parsed) != expected or not all(math.isfinite(item) for item in parsed):
        raise argparse.ArgumentTypeError(f"expected {expected} finite comma-separated values")
    return parsed


def main(argv: list[str] | None = None) -> int:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--template", type=Path, default=root / "src/bringup/config/fgo_gil.yaml")
    parser.add_argument("--fix-topic", default="/fix")
    parser.add_argument("--lio-topic", default="/fastlio2/lio_odom")
    parser.add_argument("--nmea-topic", default="/rtk/nmea_sentence")
    parser.add_argument("--required-gga-quality", type=int, default=4)
    parser.add_argument("--match-tolerance-s", type=float, default=0.10)
    parser.add_argument("--quality-tolerance-s", type=float, default=0.05)
    parser.add_argument("--minimum-pairs", type=int, default=30)
    parser.add_argument("--minimum-span-m", type=float, default=10.0)
    parser.add_argument("--maximum-rmse-m", type=float, default=0.50)
    parser.add_argument("--maximum-residual-m", type=float, default=1.00)
    parser.add_argument("--imu-lidar-translation", default="-0.011,-0.02329,0.04412")
    parser.add_argument("--imu-lidar-rotation-wxyz", default="1,0,0,0")
    parser.add_argument("--master-in-imu", default="0,-0.184,0.134")
    args = parser.parse_args(argv)
    try:
        import numpy as np

        imu_lidar_translation = parse_vector(args.imu_lidar_translation, 3)
        imu_lidar_rotation = parse_vector(args.imu_lidar_rotation_wxyz, 4)
        master_in_imu = parse_vector(args.master_in_imu, 3)
        fixes, poses, counts = read_bag_samples(
            args.bag,
            args.fix_topic,
            args.lio_topic,
            args.nmea_topic,
            args.required_gga_quality,
            args.quality_tolerance_s,
        )
        tolerance_ns = int(args.match_tolerance_s * 1.0e9)
        source = []
        destination = []
        for fix in fixes:
            pose = nearest(poses, fix.timestamp_ns, tolerance_ns)
            if pose is None:
                continue
            source.append(
                lidar_pose_to_local_antenna(
                    pose, imu_lidar_translation, imu_lidar_rotation, master_in_imu
                )
            )
            destination.append(np.asarray(fix.position, dtype=float))
        if len(source) < args.minimum_pairs:
            raise RuntimeError(
                f"only {len(source)} matched RTK Fixed/LIO pairs; need {args.minimum_pairs}"
            )
        source_array = np.asarray(source)
        span_m = float(np.max(np.linalg.norm(source_array - source_array[0], axis=1)))
        if span_m < args.minimum_span_m:
            raise RuntimeError(f"trajectory span {span_m:.3f} m is below {args.minimum_span_m:.3f} m")
        rotation, translation, singular_values, residuals, inliers, threshold = robust_fit(
            source_array, np.asarray(destination)
        )
        if singular_values[0] <= 0.0 or singular_values[1] / singular_values[0] < 1.0e-3:
            raise RuntimeError("trajectory geometry is effectively straight; add a broad turn")
        rmse_m = float(math.sqrt(np.mean(residuals * residuals)))
        maximum_residual_m = float(np.max(residuals))
        if rmse_m > args.maximum_rmse_m or maximum_residual_m > args.maximum_residual_m:
            raise RuntimeError(
                f"calibration residuals fail limits: rmse={rmse_m:.3f} m "
                f"max={maximum_residual_m:.3f} m"
            )
        quaternion = rotation_quaternion_wxyz(rotation)
        config = yaml.safe_load(args.template.read_text(encoding="utf-8"))
        calibration = config["fgo_gil_float_fgo"]["ros__parameters"]["calibration"][
            "ecef_from_lidar_world"
        ]
        calibration["calibrated"] = True
        calibration["translation_m"] = [float(value) for value in translation]
        calibration["rotation_wxyz"] = [float(value) for value in quaternion]
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(yaml.safe_dump(config, sort_keys=False), encoding="utf-8")
        report = {
            "status": "CALIBRATED",
            "bag": str(args.bag),
            "required_gga_quality": args.required_gga_quality,
            "counts": counts,
            "matched_pairs": len(source),
            "inliers": int(np.count_nonzero(inliers)),
            "trajectory_span_m": span_m,
            "outlier_threshold_m": threshold,
            "residual_rmse_m": rmse_m,
            "maximum_residual_m": maximum_residual_m,
            "singular_values": [float(value) for value in singular_values],
            "translation_m": calibration["translation_m"],
            "rotation_wxyz": calibration["rotation_wxyz"],
        }
        report_path = args.out.with_suffix(args.out.suffix + ".report.json")
        report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(report, indent=2))
    except (OSError, RuntimeError, ValueError, KeyError, yaml.YAMLError) as error:
        print(f"calibration rejected: {error}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
