#!/usr/bin/env python3
"""Audit sensor_msgs/Imu timing and finite measurements in a rosbag2 SQLite bag."""

import argparse
import json
import math
from pathlib import Path
import sqlite3
import struct
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


IMU_TYPE = "sensor_msgs/msg/Imu"


class CdrDecodeError(ValueError):
    pass


class CdrReader:
    def __init__(self, data: bytes):
        if len(data) < 4 or data[:4] != b"\x00\x01\x00\x00":
            raise CdrDecodeError("only little-endian CDR encapsulation is supported")
        self.data = data
        self.offset = 4
        self.origin = 4

    def align(self, boundary: int) -> None:
        relative = self.offset - self.origin
        self.offset += (-relative) % boundary

    def unpack(self, format_code: str, boundary: int):
        self.align(boundary)
        size = struct.calcsize("<" + format_code)
        if self.offset + size > len(self.data):
            raise CdrDecodeError("truncated CDR payload")
        value = struct.unpack_from("<" + format_code, self.data, self.offset)[0]
        self.offset += size
        return value

    def int32(self) -> int:
        return self.unpack("i", 4)

    def uint32(self) -> int:
        return self.unpack("I", 4)

    def float64(self) -> float:
        return self.unpack("d", 8)

    def string(self) -> str:
        length = self.uint32()
        if length == 0 or self.offset + length > len(self.data):
            raise CdrDecodeError("invalid CDR string length")
        raw = self.data[self.offset : self.offset + length]
        self.offset += length
        if raw[-1] != 0:
            raise CdrDecodeError("CDR string lacks null terminator")
        return raw[:-1].decode("utf-8")

    def float64_array(self, count: int) -> Tuple[float, ...]:
        return tuple(self.float64() for _ in range(count))


def decode_imu_cdr(data: bytes) -> Dict[str, object]:
    reader = CdrReader(data)
    seconds = reader.int32()
    nanoseconds = reader.uint32()
    if nanoseconds >= 1_000_000_000:
        raise CdrDecodeError("header nanoseconds outside ROS range")
    frame_id = reader.string()
    reader.float64_array(4)  # orientation
    reader.float64_array(9)  # orientation covariance
    angular_velocity = reader.float64_array(3)
    reader.float64_array(9)  # angular velocity covariance
    linear_acceleration = reader.float64_array(3)
    reader.float64_array(9)  # linear acceleration covariance
    return {
        "stamp_s": float(seconds) + float(nanoseconds) * 1.0e-9,
        "frame_id": frame_id,
        "angular_velocity": angular_velocity,
        "linear_acceleration": linear_acceleration,
    }


def _bag_databases(bag_path: Path) -> List[Path]:
    if bag_path.is_file() and bag_path.suffix == ".db3":
        return [bag_path]
    if not bag_path.is_dir():
        raise FileNotFoundError(f"bag path does not exist: {bag_path}")
    databases = sorted(bag_path.glob("*.db3"))
    if not databases:
        raise FileNotFoundError(f"no .db3 files found under {bag_path}")
    return databases


def _read_imu_rows(database: Path, topic: str) -> Iterable[Tuple[int, bytes]]:
    connection = sqlite3.connect(f"file:{database}?mode=ro", uri=True)
    try:
        topic_row = connection.execute(
            "SELECT id, type, serialization_format FROM topics WHERE name = ?", (topic,)
        ).fetchone()
        if topic_row is None:
            return []
        topic_id, message_type, serialization_format = topic_row
        if message_type != IMU_TYPE or serialization_format != "cdr":
            raise ValueError(
                f"{topic} must be {IMU_TYPE} with cdr serialization, got "
                f"{message_type} with {serialization_format}"
            )
        return connection.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp, id",
            (topic_id,),
        ).fetchall()
    finally:
        connection.close()


def percentile(values: Sequence[float], quantile: float) -> Optional[float]:
    if not values:
        return None
    ordered = sorted(values)
    position = quantile * float(len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    fraction = position - float(lower)
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def analyze_imu_bag(
    bag_path: Path,
    topic: str = "/livox/imu",
    max_gap_s: float = 0.05,
) -> Dict[str, object]:
    if not math.isfinite(max_gap_s) or max_gap_s <= 0.0:
        raise ValueError("max_gap_s must be positive and finite")
    rows: List[Tuple[int, bytes]] = []
    for database in _bag_databases(bag_path):
        rows.extend(_read_imu_rows(database, topic))
    rows.sort(key=lambda row: row[0])
    if not rows:
        raise ValueError(f"topic {topic} has no messages in {bag_path}")

    stamps: List[float] = []
    positive_deltas: List[float] = []
    storage_latency_s: List[float] = []
    frame_ids = set()
    duplicates = 0
    time_reversals = 0
    gaps = 0
    nonfinite_measurements = 0
    decode_failures = 0
    segment_count = 1
    current_segment_samples = 0
    maximum_segment_samples = 0
    previous_stamp: Optional[float] = None

    for storage_timestamp_ns, data in rows:
        try:
            message = decode_imu_cdr(data)
        except (CdrDecodeError, UnicodeDecodeError, struct.error):
            decode_failures += 1
            continue
        stamp_s = float(message["stamp_s"])
        angular_velocity = message["angular_velocity"]
        linear_acceleration = message["linear_acceleration"]
        frame_ids.add(str(message["frame_id"]))
        stamps.append(stamp_s)
        storage_latency_s.append(float(storage_timestamp_ns) * 1.0e-9 - stamp_s)
        if not all(math.isfinite(value) for value in angular_velocity + linear_acceleration):
            nonfinite_measurements += 1

        if previous_stamp is not None:
            delta_s = stamp_s - previous_stamp
            if delta_s == 0.0:
                duplicates += 1
            elif delta_s < 0.0:
                time_reversals += 1
                segment_count += 1
                maximum_segment_samples = max(maximum_segment_samples, current_segment_samples)
                current_segment_samples = 0
            else:
                positive_deltas.append(delta_s)
                if delta_s > max_gap_s:
                    gaps += 1
                    segment_count += 1
                    maximum_segment_samples = max(maximum_segment_samples, current_segment_samples)
                    current_segment_samples = 0
        previous_stamp = stamp_s
        current_segment_samples += 1
    maximum_segment_samples = max(maximum_segment_samples, current_segment_samples)

    duration_s = stamps[-1] - stamps[0] if len(stamps) >= 2 else 0.0
    effective_rate_hz = (len(stamps) - 1) / duration_s if duration_s > 0.0 else None
    return {
        "bag": str(bag_path),
        "topic": topic,
        "message_count": len(rows),
        "decoded_count": len(stamps),
        "decode_failures": decode_failures,
        "frame_ids": sorted(frame_ids),
        "first_stamp_s": stamps[0] if stamps else None,
        "last_stamp_s": stamps[-1] if stamps else None,
        "duration_s": duration_s,
        "effective_rate_hz": effective_rate_hz,
        "dt_s": {
            "minimum": min(positive_deltas) if positive_deltas else None,
            "p50": percentile(positive_deltas, 0.50),
            "p95": percentile(positive_deltas, 0.95),
            "p99": percentile(positive_deltas, 0.99),
            "maximum": max(positive_deltas) if positive_deltas else None,
        },
        "duplicates": duplicates,
        "time_reversals": time_reversals,
        "gaps_over_threshold": gaps,
        "gap_threshold_s": max_gap_s,
        "nonfinite_measurements": nonfinite_measurements,
        "segment_count": segment_count,
        "maximum_segment_samples": maximum_segment_samples,
        "storage_latency_s": {
            "p50": percentile(storage_latency_s, 0.50),
            "p95": percentile(storage_latency_s, 0.95),
            "maximum": max(storage_latency_s) if storage_latency_s else None,
        },
        "continuous_preintegration_ready": decode_failures == 0
        and nonfinite_measurements == 0
        and time_reversals == 0
        and gaps == 0,
    }


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="rosbag2 directory or .db3 file")
    parser.add_argument("--topic", default="/livox/imu")
    parser.add_argument("--max-gap-s", type=float, default=0.05)
    parser.add_argument("--json-output", type=Path)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_argument_parser().parse_args(argv)
    try:
        metrics = analyze_imu_bag(args.bag, args.topic, args.max_gap_s)
    except (FileNotFoundError, ValueError, sqlite3.Error) as error:
        print(f"error: {error}")
        return 2
    output = json.dumps(metrics, indent=2, sort_keys=True, allow_nan=False)
    print(output)
    if args.json_output is not None:
        args.json_output.parent.mkdir(parents=True, exist_ok=True)
        args.json_output.write_text(output + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
