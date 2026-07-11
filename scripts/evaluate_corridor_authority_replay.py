#!/usr/bin/env python3
"""Replay corridor authority safety rules against synthetic records or ROS 2 bags."""

from __future__ import annotations

import argparse
import bisect
from collections import Counter
import json
import math
import os
import sys
from pathlib import Path

from gps_waypoint_dispatcher.alignment_math import (
    heading_quaternion_yaw_to_enu_yaw,
)
from gps_waypoint_dispatcher.route_safety import (
    GlobalCorrectionWatchdog,
    LocalOdomWatchdog,
    WatchdogDecision,
)
from gps_waypoint_dispatcher.rtk_authority import (
    CorrectionGate,
    CorrectionGateState,
    CorrectionReleaseState,
    Pose2D,
    StampedPoseHistory,
    normalize_angle,
)


BAG_MANIFEST = {
    "2026-07-10-13-34-36": {"expected": "LOCAL_NO_PROGRESS"},
    "2026-07-10-13-36-24": {"expected": "HEADING_OUTLIER"},
    "2026-07-10-13-46-03": {"expected": "STABLE_RELEASE"},
    "2026-07-10-13-48-43": {"expected": "STABLE_RELEASE"},
}


def _pose(record):
    return Pose2D(float(record["x"]), float(record["y"]), float(record["yaw"]))


def _project_enu(latitude, longitude, origin_lat=31.274927, origin_lon=120.737548):
    earth_radius_m = 6378137.0
    x = math.radians(longitude - origin_lon) * earth_radius_m * math.cos(
        math.radians(origin_lat)
    )
    y = math.radians(latitude - origin_lat) * earth_radius_m
    return x, y


def _release_metrics(samples):
    ordered = sorted(samples, key=lambda item: item["stamp_s"])
    max_linear = 0.0
    max_yaw = 0.0
    max_saturated = 0
    saturated_run = 0
    previous = None
    for sample in ordered:
        if sample.get("saturated", False):
            saturated_run += 1
            max_saturated = max(max_saturated, saturated_run)
        else:
            saturated_run = 0
        if previous is not None:
            dt_s = float(sample["stamp_s"]) - float(previous["stamp_s"])
            if dt_s > 0.0:
                max_linear = max(
                    max_linear,
                    math.hypot(
                        float(sample["x"]) - float(previous["x"]),
                        float(sample["y"]) - float(previous["y"]),
                    )
                    / dt_s,
                )
                max_yaw = max(
                    max_yaw,
                    abs(
                        normalize_angle(
                            float(sample["yaw"]) - float(previous["yaw"])
                        )
                    )
                    / dt_s,
                )
        previous = sample
    return {
        "sample_count": len(ordered),
        "max_translation_rate_mps": max_linear,
        "max_yaw_rate_degps": math.degrees(max_yaw),
        "max_consecutive_saturated": max_saturated,
    }


def _heading_metrics(observations):
    gate = CorrectionGate.yaw()
    rejected = False
    max_rejected = 0.0
    samples_after_reject = 0
    minimum_reacquire = 0
    for item in sorted(observations, key=lambda value: value["stamp_s"]):
        result = gate.observe(
            float(item["stamp_s"]),
            float(item["correction_yaw_rad"]),
            now_s=float(item["stamp_s"]),
        )
        if result.reason == "INNOVATION_REJECTED":
            rejected = True
            max_rejected = max(max_rejected, math.degrees(result.innovation or 0.0))
            samples_after_reject = 0
        elif rejected and gate.state is not CorrectionGateState.LOCKED:
            samples_after_reject += 1
        elif rejected and gate.state is CorrectionGateState.LOCKED:
            samples_after_reject += 1
            minimum_reacquire = (
                samples_after_reject
                if minimum_reacquire == 0
                else min(minimum_reacquire, samples_after_reject)
            )
            rejected = False
    return {
        "observation_count": len(observations),
        "outlier_rejected": max_rejected > 0.0,
        "max_rejected_innovation_deg": max_rejected,
        "minimum_reacquire_samples": minimum_reacquire,
    }


def _watchdog_metrics(local_records, global_records):
    local = LocalOdomWatchdog()
    local_aborts = []
    for item in sorted(local_records, key=lambda value: value["stamp_s"]):
        result = local.update(
            float(item["stamp_s"]),
            float(item["x"]),
            float(item["y"]),
            float(item["yaw"]),
        )
        if result.decision is WatchdogDecision.LOCAL_ABORT:
            local_aborts.append(result.reason)
            break

    global_watchdog = GlobalCorrectionWatchdog()
    holds = []
    for item in sorted(global_records, key=lambda value: value["stamp_s"]):
        result = global_watchdog.update(
            float(item["stamp_s"]),
            float(item["x"]),
            float(item["y"]),
            float(item["yaw"]),
            authority_allowed=True,
            authority_age_s=0.0,
        )
        if result.decision is WatchdogDecision.GLOBAL_HOLD:
            holds.append(result.reason)
    return (
        {"abort_count": len(local_aborts), "reasons": dict(Counter(local_aborts))},
        {"hold_count": len(holds), "reasons": dict(Counter(holds))},
    )


def _progress_metrics(samples):
    started_s = None
    max_duration_s = 0.0
    for item in sorted(samples, key=lambda value: value["stamp_s"]):
        stalled = (
            abs(float(item["command_speed_mps"])) > 0.5
            and abs(float(item["lio_speed_mps"])) < 0.05
            and abs(float(item["gnss_speed_mps"])) < 0.05
        )
        stamp_s = float(item["stamp_s"])
        if not stalled:
            started_s = None
            continue
        if started_s is None:
            started_s = stamp_s
        max_duration_s = max(max_duration_s, stamp_s - started_s)
    return {
        "local_no_progress": max_duration_s >= 15.0,
        "max_no_progress_duration_s": max_duration_s,
    }


def evaluate_records(records):
    heading = _heading_metrics(records.get("heading_observations", []))
    release = _release_metrics(records.get("release_samples", []))
    local, global_correction = _watchdog_metrics(
        records.get("local_odom", []), records.get("global_correction", [])
    )
    progress = _progress_metrics(records.get("progress_samples", []))
    assertions = {
        "heading_outlier_fail_closed": (
            not heading["outlier_rejected"]
            or heading["minimum_reacquire_samples"] >= 5
        ),
        "release_rates_within_limits": (
            release["max_translation_rate_mps"] <= 0.205 + 1e-9
            and release["max_yaw_rate_degps"] <= 2.05 + 1e-9
        ),
        "global_never_classified_as_local_abort": local["abort_count"] == 0,
    }
    return {
        "heading": heading,
        "release": release,
        "local_watchdog": local,
        "global_correction": global_correction,
        "progress": progress,
        "assertions": assertions,
        "passed": all(assertions.values()),
    }


def _message_stamp_s(message, fallback_s):
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return fallback_s
    value = float(stamp.sec) + float(stamp.nanosec) * 1e-9
    return value if value > 0.0 and math.isfinite(value) else fallback_s


def _yaw_from_quaternion(quaternion):
    siny = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy = 1.0 - 2.0 * (quaternion.y**2 + quaternion.z**2)
    return math.atan2(siny, cosy)


def _interpolate_history(history, stamp_s):
    result = history.interpolate(stamp_s, max_bracket_s=0.20)
    return result.pose if result.ok else None


def _nearest_prior(items, bag_time_s):
    if not items:
        return None
    stamps = [item[0] for item in items]
    index = bisect.bisect_right(stamps, bag_time_s) - 1
    return items[index] if index >= 0 else None


def _extract_bag_records(bag_path):
    try:
        from rosbags.highlevel import AnyReader
    except ImportError as error:
        raise RuntimeError(
            "Install the Python rosbags package or source ROS 2 Humble."
        ) from error

    lio = []
    headings = []
    fixes = []
    alignments = []
    commands = []
    map_odom = []
    wanted = {
        "/fastlio2/lio_odom",
        "/heading",
        "/fix",
        "/gps_corridor/enu_to_map",
        "/cmd_vel",
        "/tf",
    }
    with AnyReader([Path(bag_path)]) as reader:
        connections = [item for item in reader.connections if item.topic in wanted]
        for connection, timestamp_ns, raw in reader.messages(connections=connections):
            message = reader.deserialize(raw, connection.msgtype)
            bag_s = timestamp_ns * 1e-9
            topic = connection.topic
            if topic == "/fastlio2/lio_odom":
                pose = message.pose.pose
                lio.append(
                    {
                        "stamp_s": _message_stamp_s(message, bag_s),
                        "bag_s": bag_s,
                        "x": float(pose.position.x),
                        "y": float(pose.position.y),
                        "yaw": _yaw_from_quaternion(pose.orientation),
                    }
                )
            elif topic == "/heading":
                headings.append(
                    {
                        "stamp_s": _message_stamp_s(message, bag_s),
                        "bag_s": bag_s,
                        "yaw": _yaw_from_quaternion(message.quaternion),
                    }
                )
            elif topic == "/fix":
                fixes.append(
                    {
                        "stamp_s": _message_stamp_s(message, bag_s),
                        "bag_s": bag_s,
                        "lat": float(message.latitude),
                        "lon": float(message.longitude),
                    }
                )
            elif topic == "/gps_corridor/enu_to_map" and len(message.data) >= 4:
                if float(message.data[3]) >= 0.5:
                    alignments.append(
                        (
                            bag_s,
                            (
                                float(message.data[0]),
                                float(message.data[1]),
                                float(message.data[2]),
                            ),
                        )
                    )
            elif topic == "/cmd_vel":
                commands.append((bag_s, abs(float(message.linear.x))))
            elif topic == "/tf":
                for transform in message.transforms:
                    if transform.header.frame_id == "map" and transform.child_frame_id == "odom":
                        map_odom.append(
                            {
                                "stamp_s": _message_stamp_s(transform, bag_s),
                                "x": float(transform.transform.translation.x),
                                "y": float(transform.transform.translation.y),
                                "yaw": _yaw_from_quaternion(transform.transform.rotation),
                            }
                        )
    return lio, headings, fixes, alignments, commands, map_odom


def _build_replay_records(bag_path):
    lio, headings, fixes, alignments, commands, map_odom = _extract_bag_records(
        bag_path
    )
    history = StampedPoseHistory(max_age_s=1e9, max_samples=max(1, len(lio)))
    local_records = []
    for item in sorted(lio, key=lambda value: value["stamp_s"]):
        pose = _pose(item)
        appended = history.append(
            item["stamp_s"], pose, frame_id="odom", child_frame_id="base_footprint"
        )
        if not appended.accepted:
            continue
        local_records.append(item)

    yaw_gate = CorrectionGate.yaw()
    position_gate = CorrectionGate.translation()
    accepted_heading = []
    heading_observations = []
    target_events = []
    observations = [
        (item["bag_s"], "heading", item) for item in headings
    ] + [(item["bag_s"], "fix", item) for item in fixes]
    for bag_s, kind, item in sorted(observations):
        alignment_record = _nearest_prior(alignments, bag_s)
        local_pose = _interpolate_history(history, item["stamp_s"])
        if alignment_record is None or local_pose is None:
            continue
        theta, tx, ty = alignment_record[1]
        if kind == "heading":
            enu_yaw = heading_quaternion_yaw_to_enu_yaw(
                item["yaw"], quaternion_yaw_is_compass=True
            )
            correction = normalize_angle(theta + enu_yaw - local_pose.yaw)
            heading_observations.append(
                {"stamp_s": item["stamp_s"], "correction_yaw_rad": correction}
            )
            result = yaw_gate.observe(item["stamp_s"], correction, now_s=bag_s)
            if result.accepted and yaw_gate.state is CorrectionGateState.LOCKED:
                accepted_heading.append((item["stamp_s"], float(yaw_gate.target)))
            continue

        if yaw_gate.state is not CorrectionGateState.LOCKED or not accepted_heading:
            continue
        heading_stamps = [value[0] for value in accepted_heading]
        heading_index = bisect.bisect_right(heading_stamps, item["stamp_s"]) - 1
        if heading_index < 0:
            continue
        heading_stamp, yaw_correction = accepted_heading[heading_index]
        if item["stamp_s"] - heading_stamp > 0.30:
            continue
        enu_x, enu_y = _project_enu(item["lat"], item["lon"])
        map_x = math.cos(theta) * enu_x - math.sin(theta) * enu_y + tx
        map_y = math.sin(theta) * enu_x + math.cos(theta) * enu_y + ty
        correction_xy = (
            map_x
            - (
                math.cos(yaw_correction) * local_pose.x
                - math.sin(yaw_correction) * local_pose.y
            ),
            map_y
            - (
                math.sin(yaw_correction) * local_pose.x
                + math.cos(yaw_correction) * local_pose.y
            ),
        )
        position_gate.observe(item["stamp_s"], correction_xy, now_s=bag_s)
        if position_gate.state is CorrectionGateState.LOCKED:
            target_events.append(
                (
                    item["stamp_s"],
                    Pose2D(
                        float(position_gate.target[0]),
                        float(position_gate.target[1]),
                        float(yaw_gate.target),
                    ),
                )
            )

    release_samples = []
    if target_events and local_records:
        release = CorrectionReleaseState()
        output = Pose2D(0.0, 0.0, 0.0)
        cumulative_translation_m = 0.0
        cumulative_yaw_rad = 0.0
        target_stamps = [value[0] for value in target_events]
        previous_local = None
        for item in local_records:
            target_index = bisect.bisect_right(target_stamps, item["stamp_s"]) - 1
            if target_index < 0:
                continue
            local_pose = _pose(item)
            linear_rate = 0.0
            yaw_rate = 0.0
            if previous_local is not None:
                dt_s = item["stamp_s"] - previous_local["stamp_s"]
                if dt_s > 0.0:
                    linear_rate = math.hypot(
                        item["x"] - previous_local["x"],
                        item["y"] - previous_local["y"],
                    ) / dt_s
                    yaw_rate = abs(
                        normalize_angle(item["yaw"] - previous_local["yaw"])
                    ) / dt_s
            result = release.update(
                previous_output_map_odom=output,
                target_map_odom=target_events[target_index][1],
                local_pose=local_pose,
                now_s=item["stamp_s"],
                lio_stamp_s=item["stamp_s"],
                lio_age_s=0.0,
                local_linear_rate_mps=linear_rate,
                local_yaw_rate_radps=yaw_rate,
                gates_locked=True,
            )
            output = result.output_map_odom
            cumulative_translation_m += result.translation_step_m
            cumulative_yaw_rad += result.yaw_step_rad
            release_samples.append(
                {
                    "stamp_s": item["stamp_s"],
                    "x": cumulative_translation_m,
                    "y": 0.0,
                    "yaw": cumulative_yaw_rad,
                    "saturated": (
                        result.translation_step_m >= result.dt_s * 0.20 - 1e-6
                        or result.yaw_step_rad
                        >= result.dt_s * math.radians(2.0) - 1e-6
                    )
                    and result.dt_s > 0.0,
                }
            )
            previous_local = item

    progress_samples = []
    if commands and local_records:
        start_s = math.floor(max(commands[0][0], local_records[0]["bag_s"]))
        end_s = math.ceil(min(commands[-1][0], local_records[-1]["bag_s"]))
        for second in range(int(start_s), int(end_s) + 1):
            command_values = [
                value for stamp, value in commands if second <= stamp < second + 1
            ]
            positions = [
                item
                for item in local_records
                if second <= item["bag_s"] < second + 1
            ]
            if not command_values or len(positions) < 2:
                continue
            elapsed_s = positions[-1]["bag_s"] - positions[0]["bag_s"]
            lio_speed = (
                math.hypot(
                    positions[-1]["x"] - positions[0]["x"],
                    positions[-1]["y"] - positions[0]["y"],
                )
                / elapsed_s
                if elapsed_s > 0.0
                else math.inf
            )
            progress_samples.append(
                {
                    "stamp_s": float(second),
                    "command_speed_mps": max(command_values),
                    "lio_speed_mps": lio_speed,
                    "gnss_speed_mps": lio_speed,
                }
            )

    return {
        "heading_observations": heading_observations,
        "release_samples": release_samples,
        "local_odom": local_records,
        "global_correction": map_odom,
        "progress_samples": progress_samples,
    }


def evaluate_bag(bag_path, fixture_name=None):
    result = evaluate_records(_build_replay_records(Path(bag_path)))
    name = fixture_name or Path(bag_path).parent.name
    expectation = BAG_MANIFEST.get(name, {}).get("expected")
    common = {
        "global_never_classified_as_local_abort": result["assertions"][
            "global_never_classified_as_local_abort"
        ]
    }
    if expectation == "HEADING_OUTLIER":
        common["fixture_heading_outlier_detected"] = (
            result["heading"]["outlier_rejected"]
            and result["heading"]["max_rejected_innovation_deg"] >= 45.0
        )
    elif expectation == "STABLE_RELEASE":
        common["release_rates_within_limits"] = result["assertions"][
            "release_rates_within_limits"
        ]
        common["fixture_release_available"] = (
            result["release"]["sample_count"] > 0
        )
    elif expectation == "LOCAL_NO_PROGRESS":
        common["fixture_local_no_progress"] = result["progress"][
            "local_no_progress"
        ]
    result["assertions"] = common
    result["fixture"] = name
    result["expected"] = expectation
    result["passed"] = all(result["assertions"].values())
    return result


def _write_json(payload, path):
    text = json.dumps(payload, indent=2, sort_keys=True) + "\n"
    if path is None:
        print(text, end="")
    else:
        Path(path).write_text(text, encoding="utf-8")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--records", type=Path)
    source.add_argument("--bag", type=Path)
    source.add_argument("--manifest", action="store_true")
    parser.add_argument("--bag-root", type=Path)
    parser.add_argument("--out", type=Path)
    args = parser.parse_args(argv)

    try:
        if args.records:
            payload = evaluate_records(json.loads(args.records.read_text(encoding="utf-8")))
        elif args.bag:
            payload = evaluate_bag(args.bag)
        else:
            root = args.bag_root or Path(
                os.environ.get("FYP_CORRIDOR_BAG_ROOT", "")
            )
            if not str(root):
                raise RuntimeError("Set FYP_CORRIDOR_BAG_ROOT or pass --bag-root")
            payload = {}
            for name in BAG_MANIFEST:
                bag = root / name / "bag"
                payload[name] = (
                    evaluate_bag(bag, fixture_name=name)
                    if bag.exists()
                    else {"fixture": name, "available": False, "passed": False}
                )
            payload = {
                "fixtures": payload,
                "passed": all(item["passed"] for item in payload.values()),
            }
    except (OSError, RuntimeError, ValueError) as error:
        print(str(error), file=sys.stderr)
        return 2
    _write_json(payload, args.out)
    return 0 if payload["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
