#!/usr/bin/env python3
"""Summarize RTK FGO shadow-mode diagnostics from a ROS 2 bag."""

from __future__ import annotations

import argparse
import json
import re
import sys
from collections import Counter
from pathlib import Path
from statistics import median
from typing import Any, Iterable


KEY_RE = re.compile(r"\b([A-Za-z0-9_]+)=")


def parse_status_line(text: str) -> dict[str, str]:
    """Parse `/rtk_fgo/status` text into state/gate/reason fields."""
    out = {"state": "", "gate": "", "reason": ""}
    matches = list(KEY_RE.finditer(text))
    for index, match in enumerate(matches):
        key = match.group(1)
        start = match.end()
        end = matches[index + 1].start() if index + 1 < len(matches) else len(text)
        value = text[start:end].strip().strip('"')
        if key in out:
            out[key] = value
    return out


def parse_correction(values: list[float]) -> dict[str, Any]:
    """Parse `/rtk_fgo/correction_status` Float32MultiArray payload."""
    committed = bool(values and values[0] >= 0.5)
    correction_norm_m = float(values[1]) if len(values) > 1 else 0.0
    strong_sample_count = float(values[2]) if len(values) > 2 else 0.0
    return {
        "committed": committed,
        "correction_norm_m": correction_norm_m,
        "strong_sample_count": strong_sample_count,
    }


def parse_numeric(value: str) -> float | int | None:
    if value in {"", "na"}:
        return None
    try:
        number = float(value)
    except ValueError:
        return None
    if number.is_integer():
        return int(number)
    return number


def parse_diagnostic_values(message: Any) -> dict[str, float | int]:
    """Extract numeric RTK/FGO diagnostic KeyValue fields."""
    wanted = {
        "rtk_gga_quality",
        "rtk_satellites",
        "rtk_hdop",
        "position_innovation_m",
        "heading_innovation_rad",
        "implied_fix_speed_mps",
        "fix_age_s",
        "nmea_age_s",
        "heading_age_s",
        "fastlio_age_s",
        "wheel_age_s",
    }
    out: dict[str, float | int] = {}
    for status in getattr(message, "status", []):
        for item in getattr(status, "values", []):
            key = getattr(item, "key", "")
            if key not in wanted:
                continue
            parsed = parse_numeric(getattr(item, "value", ""))
            if parsed is not None:
                out[key] = parsed
    return out


def empty_metrics() -> dict[str, Any]:
    return {
        "status_count": 0,
        "state_counts": {},
        "gate_counts": {},
        "shadow_commit_count": 0,
        "max_correction_norm_m": 0.0,
        "rejection_reasons": {},
        "rtk_quality_counts": {},
        "rtk_satellites": {},
        "rtk_hdop": {},
        "position_innovation_m": {},
        "heading_innovation_rad": {},
        "implied_fix_speed_mps": {},
        "input_age_s": {},
        "first_timestamp_ns": None,
        "last_timestamp_ns": None,
    }


def update_timestamp(metrics: dict[str, Any], timestamp_ns: int) -> None:
    if metrics["first_timestamp_ns"] is None or timestamp_ns < metrics["first_timestamp_ns"]:
        metrics["first_timestamp_ns"] = timestamp_ns
    if metrics["last_timestamp_ns"] is None or timestamp_ns > metrics["last_timestamp_ns"]:
        metrics["last_timestamp_ns"] = timestamp_ns


def summarize_numbers(values: list[float | int]) -> dict[str, float | int]:
    if not values:
        return {}
    ordered = sorted(values)
    return {
        "count": len(values),
        "min": ordered[0],
        "median": median(ordered),
        "max": ordered[-1],
    }


def summarize_events(events: Iterable[tuple[str, Any, int]]) -> dict[str, Any]:
    metrics = empty_metrics()
    state_counts: Counter[str] = Counter()
    gate_counts: Counter[str] = Counter()
    rejection_reasons: Counter[str] = Counter()
    rtk_quality_counts: Counter[str] = Counter()
    numeric_values: dict[str, list[float | int]] = {
        "rtk_satellites": [],
        "rtk_hdop": [],
        "position_innovation_m": [],
        "heading_innovation_rad": [],
        "implied_fix_speed_mps": [],
        "fix_age_s": [],
        "nmea_age_s": [],
        "heading_age_s": [],
        "fastlio_age_s": [],
        "wheel_age_s": [],
    }

    for topic, message, timestamp_ns in events:
        update_timestamp(metrics, timestamp_ns)
        if topic == "/rtk_fgo/status":
            parsed = parse_status_line(getattr(message, "data", ""))
            metrics["status_count"] += 1
            if parsed["state"]:
                state_counts[parsed["state"]] += 1
            if parsed["gate"]:
                gate_counts[parsed["gate"]] += 1
            if parsed["gate"] == "REJECTED":
                rejection_reasons[parsed["reason"] or "unknown"] += 1
        elif topic == "/rtk_fgo/correction_status":
            correction = parse_correction(list(getattr(message, "data", [])))
            if correction["committed"]:
                metrics["shadow_commit_count"] += 1
            metrics["max_correction_norm_m"] = max(
                metrics["max_correction_norm_m"],
                correction["correction_norm_m"],
            )
        elif topic == "/rtk_fgo/factor_diagnostics":
            parsed = parse_diagnostic_values(message)
            quality = parsed.get("rtk_gga_quality")
            if quality is not None:
                rtk_quality_counts[str(int(quality))] += 1
            for key in numeric_values:
                if key in parsed:
                    numeric_values[key].append(parsed[key])

    metrics["state_counts"] = dict(state_counts)
    metrics["gate_counts"] = dict(gate_counts)
    metrics["rejection_reasons"] = dict(rejection_reasons)
    metrics["rtk_quality_counts"] = dict(rtk_quality_counts)
    metrics["rtk_satellites"] = summarize_numbers(numeric_values["rtk_satellites"])
    metrics["rtk_hdop"] = summarize_numbers(numeric_values["rtk_hdop"])
    metrics["position_innovation_m"] = summarize_numbers(numeric_values["position_innovation_m"])
    metrics["heading_innovation_rad"] = summarize_numbers(numeric_values["heading_innovation_rad"])
    metrics["implied_fix_speed_mps"] = summarize_numbers(numeric_values["implied_fix_speed_mps"])
    metrics["input_age_s"] = {
        "fix": summarize_numbers(numeric_values["fix_age_s"]),
        "nmea": summarize_numbers(numeric_values["nmea_age_s"]),
        "heading": summarize_numbers(numeric_values["heading_age_s"]),
        "fastlio": summarize_numbers(numeric_values["fastlio_age_s"]),
        "wheel": summarize_numbers(numeric_values["wheel_age_s"]),
    }
    return metrics


def read_rosbag_events(bag_path: Path) -> Iterable[tuple[str, Any, int]]:
    try:
        import rosbag2_py  # type: ignore
        from rclpy.serialization import deserialize_message  # type: ignore
        from rosidl_runtime_py.utilities import get_message  # type: ignore
    except ImportError as error:
        raise RuntimeError(
            "rosbag2_py is required; source ROS 2 Humble before running this script."
        ) from error

    storage_options = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = {
        topic.name: topic.type for topic in reader.get_all_topics_and_types()
    }
    wanted_topics = {
        "/rtk_fgo/status",
        "/rtk_fgo/correction_status",
        "/rtk_fgo/factor_diagnostics",
    }
    message_types = {
        topic: get_message(type_name)
        for topic, type_name in topic_types.items()
        if topic in wanted_topics
    }

    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        if topic not in message_types:
            continue
        yield topic, deserialize_message(data, message_types[topic]), timestamp_ns


def write_metrics(metrics: dict[str, Any], out_path: Path) -> None:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(metrics, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, type=Path, help="Path to a ROS 2 bag directory")
    parser.add_argument("--out", required=True, type=Path, help="Output JSON path")
    args = parser.parse_args(argv)

    try:
        metrics = summarize_events(read_rosbag_events(args.bag))
    except RuntimeError as error:
        print(str(error), file=sys.stderr)
        return 2

    write_metrics(metrics, args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
