#!/usr/bin/env python3
"""Evaluate FGO-GIL shadow outputs and fail closed when evidence is missing."""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from statistics import mean, median
from typing import Any, Iterable, Sequence

import yaml


RAW_OBSERVATION_TOPIC = "/gnss/raw/observation_epoch"
AMBIGUITY_TOPIC = "/fgo_gil/ambiguity_status"
PERFORMANCE_TOPIC = "/fgo_gil/performance"
RECEIVER_MASTER = 1
RECEIVER_BASE = 3
GNSS_WEEK_MILLISECONDS = 604_800_000


@dataclass(frozen=True)
class PoseSample:
    timestamp_ns: int
    position: tuple[float, float, float]


@dataclass(frozen=True)
class RawEpochSample:
    reception_timestamp_ns: int
    week: int
    milliseconds_of_week: int

    @property
    def gnss_timestamp_ms(self) -> int:
        return self.week * GNSS_WEEK_MILLISECONDS + self.milliseconds_of_week


def percentile(values: Sequence[float], fraction: float) -> float:
    if not values:
        raise ValueError("percentile requires at least one value")
    ordered = sorted(values)
    position = fraction * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def summarize_numbers(values: Sequence[float]) -> dict[str, float | int]:
    finite = [float(value) for value in values if math.isfinite(float(value))]
    if not finite:
        return {}
    return {
        "count": len(finite),
        "min": min(finite),
        "mean": mean(finite),
        "median": median(finite),
        "p95": percentile(finite, 0.95),
        "max": max(finite),
        "rmse": math.sqrt(mean([value * value for value in finite])),
    }


def diagnostic_values(message: Any, status_name: str | None = None) -> dict[str, str]:
    values: dict[str, str] = {}
    for status in getattr(message, "status", []):
        if status_name is not None and getattr(status, "name", "") != status_name:
            continue
        for item in getattr(status, "values", []):
            values[str(getattr(item, "key", ""))] = str(getattr(item, "value", ""))
    return values


def parse_float(value: str | None) -> float | None:
    if value is None:
        return None
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def message_stamp_ns(message: Any, fallback_ns: int) -> int:
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    seconds = int(getattr(stamp, "sec", 0))
    nanoseconds = int(getattr(stamp, "nanosec", 0))
    stamp_ns = seconds * 1_000_000_000 + nanoseconds
    return stamp_ns if stamp_ns > 0 else int(fallback_ns)


def pose_sample(message: Any, fallback_ns: int) -> PoseSample | None:
    pose_with_covariance = getattr(message, "pose", None)
    pose = getattr(pose_with_covariance, "pose", pose_with_covariance)
    position = getattr(pose, "position", None)
    if position is None:
        return None
    values = (float(position.x), float(position.y), float(position.z))
    if not all(math.isfinite(value) for value in values):
        return None
    return PoseSample(message_stamp_ns(message, fallback_ns), values)


def match_poses(
    estimates: Sequence[PoseSample],
    references: Sequence[PoseSample],
    tolerance_s: float,
) -> list[tuple[PoseSample, PoseSample]]:
    tolerance_ns = int(tolerance_s * 1.0e9)
    ordered_estimates = sorted(estimates, key=lambda sample: sample.timestamp_ns)
    ordered_references = sorted(references, key=lambda sample: sample.timestamp_ns)
    if not ordered_estimates or not ordered_references:
        return []

    matches: list[tuple[PoseSample, PoseSample]] = []
    reference_index = 0
    for estimate in ordered_estimates:
        while (
            reference_index + 1 < len(ordered_references)
            and abs(ordered_references[reference_index + 1].timestamp_ns - estimate.timestamp_ns)
            <= abs(ordered_references[reference_index].timestamp_ns - estimate.timestamp_ns)
        ):
            reference_index += 1
        reference = ordered_references[reference_index]
        if abs(reference.timestamp_ns - estimate.timestamp_ns) <= tolerance_ns:
            matches.append((estimate, reference))
    return matches


def trajectory_metrics(
    estimates: Sequence[PoseSample],
    references: Sequence[PoseSample],
    tolerance_s: float,
) -> tuple[dict[str, Any], list[tuple[int, tuple[float, float, float]]]]:
    matches = match_poses(estimates, references, tolerance_s)
    if len(matches) < 3:
        return {
            "status": "INSUFFICIENT_MATCHED_POSES",
            "matched_poses": len(matches),
            "ape_m": {},
            "rpe_m": {},
        }, []
    try:
        import numpy as np
    except ImportError:
        return {
            "status": "NUMPY_UNAVAILABLE",
            "matched_poses": len(matches),
            "ape_m": {},
            "rpe_m": {},
        }, []

    estimate_matrix = np.asarray([pair[0].position for pair in matches], dtype=float)
    reference_matrix = np.asarray([pair[1].position for pair in matches], dtype=float)
    estimate_center = estimate_matrix.mean(axis=0)
    reference_center = reference_matrix.mean(axis=0)
    centered_estimate = estimate_matrix - estimate_center
    centered_reference = reference_matrix - reference_center
    u_matrix, _, vt_matrix = np.linalg.svd(centered_estimate.T @ centered_reference)
    rotation = vt_matrix.T @ u_matrix.T
    if np.linalg.det(rotation) < 0.0:
        vt_matrix[-1, :] *= -1.0
        rotation = vt_matrix.T @ u_matrix.T
    translation = reference_center - rotation @ estimate_center
    aligned = (rotation @ estimate_matrix.T).T + translation
    error_vectors = aligned - reference_matrix
    ape_values = np.linalg.norm(error_vectors, axis=1).tolist()
    estimate_deltas = aligned[1:] - aligned[:-1]
    reference_deltas = reference_matrix[1:] - reference_matrix[:-1]
    rpe_values = np.linalg.norm(estimate_deltas - reference_deltas, axis=1).tolist()
    timed_errors = [
        (pair[0].timestamp_ns, tuple(float(value) for value in error_vectors[index]))
        for index, pair in enumerate(matches)
    ]
    return {
        "status": "OK",
        "matched_poses": len(matches),
        "alignment": "SE3_UMEYAMA_NO_SCALE",
        "ape_m": summarize_numbers(ape_values),
        "rpe_m": summarize_numbers(rpe_values),
    }, timed_errors


def availability_fraction(
    timestamps_ns: Sequence[int],
    start_ns: int | None,
    end_ns: int | None,
    stale_timeout_s: float,
) -> float | None:
    if not timestamps_ns or start_ns is None or end_ns is None or end_ns <= start_ns:
        return None
    timeout_ns = int(stale_timeout_s * 1.0e9)
    intervals = sorted(
        (max(start_ns, stamp), min(end_ns, stamp + timeout_ns))
        for stamp in timestamps_ns
        if stamp <= end_ns and stamp + timeout_ns >= start_ns
    )
    covered = 0
    current_start = None
    current_end = None
    for interval_start, interval_end in intervals:
        if interval_end <= interval_start:
            continue
        if current_start is None:
            current_start, current_end = interval_start, interval_end
        elif interval_start <= current_end:
            current_end = max(current_end, interval_end)
        else:
            covered += current_end - current_start
            current_start, current_end = interval_start, interval_end
    if current_start is not None:
        covered += current_end - current_start
    return covered / float(end_ns - start_ns)


def outage_drift_metrics(
    raw_timestamps_ns: Sequence[int],
    timed_errors: Sequence[tuple[int, tuple[float, float, float]]],
    outage_threshold_s: float,
) -> dict[str, Any]:
    if len(raw_timestamps_ns) < 2:
        return {"status": "DD_GNSS_UNAVAILABLE", "outage_count": 0, "drift_m": {}}
    if not timed_errors:
        return {"status": "REFERENCE_UNAVAILABLE", "outage_count": 0, "drift_m": {}}
    threshold_ns = int(outage_threshold_s * 1.0e9)
    outages = [
        (left, right)
        for left, right in zip(sorted(raw_timestamps_ns), sorted(raw_timestamps_ns)[1:])
        if right - left > threshold_ns
    ]
    drifts: list[float] = []
    for start_ns, end_ns in outages:
        before = min(timed_errors, key=lambda sample: abs(sample[0] - start_ns))
        after = min(timed_errors, key=lambda sample: abs(sample[0] - end_ns))
        start_error = before[1]
        end_error = after[1]
        drifts.append(
            math.sqrt(
                sum((end_error[index] - start_error[index]) ** 2 for index in range(3))
            )
        )
    return {
        "status": "OK" if outages else "NO_OUTAGE_OVER_THRESHOLD",
        "outage_count": len(outages),
        "drift_m": summarize_numbers(drifts),
    }


def align_receiver_epochs(
    master_epochs: Sequence[RawEpochSample],
    base_epochs: Sequence[RawEpochSample],
    tolerance_s: float = 0.05,
) -> list[int]:
    master = sorted(master_epochs, key=lambda sample: sample.gnss_timestamp_ms)
    base = sorted(base_epochs, key=lambda sample: sample.gnss_timestamp_ms)
    tolerance_ms = int(round(tolerance_s * 1.0e3))
    aligned: list[int] = []
    master_index = 0
    base_index = 0
    while master_index < len(master) and base_index < len(base):
        delta_ms = (
            master[master_index].gnss_timestamp_ms
            - base[base_index].gnss_timestamp_ms
        )
        if abs(delta_ms) <= tolerance_ms:
            aligned.append(
                (
                    master[master_index].reception_timestamp_ns
                    + base[base_index].reception_timestamp_ns
                )
                // 2
            )
            master_index += 1
            base_index += 1
        elif delta_ms < 0:
            master_index += 1
        else:
            base_index += 1
    return aligned


def parse_tegrastats(lines: Iterable[str]) -> dict[str, Any]:
    ram_values: list[float] = []
    cpu_values: list[float] = []
    for line in lines:
        ram_match = re.search(r"\bRAM\s+(\d+)/(\d+)MB", line)
        if ram_match:
            ram_values.append(float(ram_match.group(1)))
        cpu_match = re.search(r"\bCPU\s*\[([^]]+)\]", line)
        if cpu_match:
            percentages = [
                float(value)
                for value in re.findall(r"(\d+(?:\.\d+)?)%", cpu_match.group(1))
            ]
            if percentages:
                cpu_values.append(mean(percentages))
    if not ram_values and not cpu_values:
        return {"status": "UNAVAILABLE", "ram_used_mb": {}, "cpu_mean_percent": {}}
    return {
        "status": "OK",
        "ram_used_mb": summarize_numbers(ram_values),
        "cpu_mean_percent": summarize_numbers(cpu_values),
    }


def summarize_events(
    events: Iterable[tuple[str, Any, int]],
    topic_names: set[str],
    bag_start_ns: int | None = None,
    bag_end_ns: int | None = None,
    estimate_topic: str = "/fgo_gil/odom",
    reference_topic: str = "/fastlio2/lio_odom",
    match_tolerance_s: float = 0.10,
    output_stale_timeout_s: float = 1.0,
    outage_threshold_s: float = 2.0,
) -> dict[str, Any]:
    estimates: list[PoseSample] = []
    references: list[PoseSample] = []
    output_timestamps: list[int] = []
    raw_epochs: dict[int, list[RawEpochSample]] = {
        RECEIVER_MASTER: [],
        RECEIVER_BASE: [],
    }
    solution_counts = {"FLOAT": 0, "FIXED": 0}
    ratios: list[float] = []
    real_time_factors: list[float] = []
    optimization_latency_ms: list[float] = []
    event_start = bag_start_ns
    event_end = bag_end_ns

    for topic, message, timestamp_ns in events:
        event_start = timestamp_ns if event_start is None else min(event_start, timestamp_ns)
        event_end = timestamp_ns if event_end is None else max(event_end, timestamp_ns)
        if topic == estimate_topic:
            sample = pose_sample(message, timestamp_ns)
            if sample is not None:
                estimates.append(sample)
                output_timestamps.append(sample.timestamp_ns)
        elif topic == reference_topic:
            sample = pose_sample(message, timestamp_ns)
            if sample is not None:
                references.append(sample)
        elif topic == RAW_OBSERVATION_TOPIC:
            receiver = int(getattr(message, "receiver", 0))
            week = int(getattr(message, "week", 0))
            milliseconds_of_week = int(getattr(message, "milliseconds_of_week", -1))
            if (
                receiver in raw_epochs
                and week > 0
                and 0 <= milliseconds_of_week < GNSS_WEEK_MILLISECONDS
            ):
                raw_epochs[receiver].append(
                    RawEpochSample(timestamp_ns, week, milliseconds_of_week)
                )
        elif topic == AMBIGUITY_TOPIC:
            values = diagnostic_values(message, "fgo_gil/ambiguity")
            solution = values.get("solution_status", "")
            if solution in solution_counts:
                solution_counts[solution] += 1
            ratio = parse_float(values.get("ratio"))
            if ratio is not None:
                ratios.append(ratio)
        elif topic == PERFORMANCE_TOPIC:
            values = diagnostic_values(message, "fgo_gil/performance")
            real_time_factor = parse_float(values.get("real_time_factor"))
            latency = parse_float(values.get("optimization_latency_ms"))
            if real_time_factor is not None:
                real_time_factors.append(real_time_factor)
            if latency is not None:
                optimization_latency_ms.append(latency)

    trajectory, timed_errors = trajectory_metrics(
        estimates, references, match_tolerance_s
    )
    solution_samples = solution_counts["FLOAT"] + solution_counts["FIXED"]
    aligned_raw_timestamps = align_receiver_epochs(
        raw_epochs[RECEIVER_MASTER], raw_epochs[RECEIVER_BASE]
    )
    if RAW_OBSERVATION_TOPIC not in topic_names or not any(raw_epochs.values()):
        raw_status = "RAW_GNSS_UNAVAILABLE"
    elif not raw_epochs[RECEIVER_MASTER] or not raw_epochs[RECEIVER_BASE]:
        raw_status = "RAW_GNSS_INCOMPLETE"
    elif not aligned_raw_timestamps:
        raw_status = "RAW_GNSS_UNALIGNED"
    else:
        raw_status = "RAW_GNSS_AVAILABLE"
    reference_timestamps = [sample.timestamp_ns for sample in references]
    availability = availability_fraction(
        output_timestamps,
        min(reference_timestamps) if reference_timestamps else None,
        max(reference_timestamps) if reference_timestamps else None,
        output_stale_timeout_s,
    )
    return {
        "raw_gnss_status": raw_status,
        "topic_presence": {topic: topic in topic_names for topic in sorted({
            estimate_topic,
            reference_topic,
            RAW_OBSERVATION_TOPIC,
            AMBIGUITY_TOPIC,
            PERFORMANCE_TOPIC,
        })},
        "trajectory": trajectory,
        "availability": {
            "status": "OK" if availability is not None else "OUTPUT_UNAVAILABLE",
            "fraction": availability,
            "output_samples": len(output_timestamps),
            "stale_timeout_s": output_stale_timeout_s,
        },
        "ambiguity": {
            "solution_counts": solution_counts,
            "fixing_rate": (
                solution_counts["FIXED"] / solution_samples if solution_samples else None
            ),
            "ratio": summarize_numbers(ratios),
        },
        "outage_drift": outage_drift_metrics(
            aligned_raw_timestamps, timed_errors, outage_threshold_s
        ),
        "raw_receiver_epochs": {
            "master": len(raw_epochs[RECEIVER_MASTER]),
            "base": len(raw_epochs[RECEIVER_BASE]),
            "aligned": len(aligned_raw_timestamps),
            "alignment_clock": "GNSS_WEEK_TOW",
        },
        "performance": {
            "real_time_factor": summarize_numbers(real_time_factors),
            "optimization_latency_ms": summarize_numbers(optimization_latency_ms),
        },
        "bag_start_ns": event_start,
        "bag_end_ns": event_end,
    }


def read_metadata(bag_path: Path) -> dict[str, Any]:
    metadata_path = bag_path / "metadata.yaml"
    if not metadata_path.exists():
        raise RuntimeError(f"missing rosbag metadata: {metadata_path}")
    root = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
    if not isinstance(root, dict):
        raise RuntimeError(f"invalid rosbag metadata root: {metadata_path}")
    info = root.get("rosbag2_bagfile_information", root)
    if not isinstance(info, dict):
        raise RuntimeError(f"invalid rosbag information block: {metadata_path}")
    topic_counts = {
        entry["topic_metadata"]["name"]: int(entry.get("message_count", 0))
        for entry in info.get("topics_with_message_count", [])
    }
    start = info.get("starting_time", {}).get("nanoseconds_since_epoch")
    duration = info.get("duration", {}).get("nanoseconds")
    return {
        "storage_id": info.get("storage_identifier", "sqlite3"),
        "topic_names": set(topic_counts),
        "topic_counts": topic_counts,
        "start_ns": int(start) if start is not None else None,
        "end_ns": (
            int(start) + int(duration)
            if start is not None and duration is not None
            else None
        ),
    }


def read_rosbag_events(
    bag_path: Path,
    storage_id: str,
    wanted_topics: set[str],
) -> Iterable[tuple[str, Any, int]]:
    try:
        import rosbag2_py  # type: ignore
        from rclpy.serialization import deserialize_message  # type: ignore
        from rosidl_runtime_py.utilities import get_message  # type: ignore
    except ImportError as error:
        raise RuntimeError(
            "rosbag2_py is required; source ROS 2 Humble or use --metadata-only"
        ) from error
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=storage_id),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    message_types = {
        topic: get_message(topic_types[topic])
        for topic in wanted_topics
        if topic in topic_types
    }
    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        if topic in message_types:
            yield topic, deserialize_message(data, message_types[topic]), timestamp_ns


def metadata_only_metrics(metadata: dict[str, Any]) -> dict[str, Any]:
    topics = metadata["topic_names"]
    topic_counts = metadata.get("topic_counts", {})
    raw_count = topic_counts.get(RAW_OBSERVATION_TOPIC)
    return {
        "mode": "METADATA_ONLY",
        "raw_gnss_status": (
            "RAW_GNSS_AVAILABLE"
            if RAW_OBSERVATION_TOPIC in topics and raw_count != 0
            else "RAW_GNSS_UNAVAILABLE"
        ),
        "topic_count": len(topics),
        "message_counts": topic_counts,
        "topics": sorted(topics),
        "trajectory": {"status": "NOT_DECODED", "ape_m": {}, "rpe_m": {}},
        "availability": {"status": "NOT_DECODED", "fraction": None},
        "ambiguity": {"fixing_rate": None},
        "outage_drift": {"status": "NOT_DECODED", "drift_m": {}},
        "performance": {"real_time_factor": {}, "optimization_latency_ms": {}},
    }


def write_metrics(metrics: dict[str, Any], out_path: Path) -> None:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(metrics, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--estimate-topic", default="/fgo_gil/odom")
    parser.add_argument("--reference-topic", default="/fastlio2/lio_odom")
    parser.add_argument("--match-tolerance-s", type=float, default=0.10)
    parser.add_argument("--output-stale-timeout-s", type=float, default=1.0)
    parser.add_argument("--outage-threshold-s", type=float, default=2.0)
    parser.add_argument("--tegrastats", type=Path)
    parser.add_argument("--metadata-only", action="store_true")
    args = parser.parse_args(argv)

    try:
        for name, value in (
            ("match tolerance", args.match_tolerance_s),
            ("output stale timeout", args.output_stale_timeout_s),
            ("outage threshold", args.outage_threshold_s),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        metadata = read_metadata(args.bag)
        if args.metadata_only:
            metrics = metadata_only_metrics(metadata)
        else:
            wanted_topics = {
                args.estimate_topic,
                args.reference_topic,
                RAW_OBSERVATION_TOPIC,
                AMBIGUITY_TOPIC,
                PERFORMANCE_TOPIC,
            }
            metrics = summarize_events(
                read_rosbag_events(args.bag, metadata["storage_id"], wanted_topics),
                metadata["topic_names"],
                metadata["start_ns"],
                metadata["end_ns"],
                args.estimate_topic,
                args.reference_topic,
                args.match_tolerance_s,
                args.output_stale_timeout_s,
                args.outage_threshold_s,
            )
            metrics["mode"] = "FULL_DECODE"
        tegrastats_path = args.tegrastats
        if tegrastats_path is None:
            candidate = args.bag.parent.parent / "system" / "tegrastats.log"
            tegrastats_path = candidate if candidate.exists() else None
        metrics["resources"] = (
            parse_tegrastats(tegrastats_path.read_text(encoding="utf-8").splitlines())
            if tegrastats_path is not None and tegrastats_path.exists()
            else {"status": "UNAVAILABLE", "ram_used_mb": {}, "cpu_mean_percent": {}}
        )
        write_metrics(metrics, args.out)
    except (OSError, RuntimeError, ValueError, yaml.YAMLError) as error:
        print(str(error), file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
