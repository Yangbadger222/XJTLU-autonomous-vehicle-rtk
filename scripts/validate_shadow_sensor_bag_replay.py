#!/usr/bin/env python3
"""Finite actual bag sensor ingress proof, without localization or actuators."""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from geometry_msgs.msg import QuaternionStamped
from livox_ros_driver2.msg import CustomMsg
from nmea_msgs.msg import Sentence
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu, NavSatFix, PointCloud2
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
import yaml

SENSORS = (
    ("/livox/lidar", CustomMsg, "livox_ros_driver2/msg/CustomMsg"),
    ("/livox/imu", Imu, "sensor_msgs/msg/Imu"),
    ("/fix", NavSatFix, "sensor_msgs/msg/NavSatFix"),
    ("/heading", QuaternionStamped, "geometry_msgs/msg/QuaternionStamped"),
    ("/rtk/nmea_sentence", Sentence, "nmea_msgs/msg/Sentence"),
    ("/rtk/status", String, "std_msgs/msg/String"),
    ("/rtk/health", DiagnosticArray, "diagnostic_msgs/msg/DiagnosticArray"),
)
EXCLUDED = (("/tf", TFMessage), ("/fastlio2/lio_odom", Odometry),
            ("/fastlio2/body_cloud", PointCloud2))


def file_sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while block := stream.read(1024 * 1024):
            digest.update(block)
    return digest.hexdigest()


def stop_child(child):
    if child is not None and child.poll() is None:
        child.send_signal(signal.SIGINT)
        try:
            child.wait(timeout=5)
        except subprocess.TimeoutExpired:
            child.kill()
            child.wait(timeout=2)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ("launcher", "executable", "source-root", "bag", "output"):
        parser.add_argument("--" + key, type=Path, required=True)
    parser.add_argument("--duration-s", type=float, default=20.)
    args = parser.parse_args()
    if os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        parser.error("localhost-only required")
    if not math.isfinite(args.duration_s) or not 1 <= args.duration_s <= 120:
        parser.error("bounded prefix between 1 and 120 seconds required")
    metadata_path = args.bag / "metadata.yaml"
    metadata = yaml.safe_load(metadata_path.read_text())["rosbag2_bagfile_information"]
    inventory = {entry["topic_metadata"]["name"]: entry
                 for entry in metadata["topics_with_message_count"]}
    selected = [(topic, kind, wire) for topic, kind, wire in SENSORS if topic in inventory
                and inventory[topic]["message_count"] > 0]
    for topic, _, wire in selected:
        if inventory[topic]["topic_metadata"]["type"] != wire:
            parser.error("recorded sensor wire type differs from allowlist: " + topic)
    if not {"/livox/imu", "/livox/lidar"} <= {t for t, _, _ in selected}:
        parser.error("actual MID360 IMU and CustomMsg inputs required")
    excluded = [(topic, kind) for topic, kind in EXCLUDED if topic in inventory
                and inventory[topic]["message_count"] > 0]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    attempt_dir = args.output.parent / (args.output.stem + "-attempt-" + str(time.monotonic_ns()))
    attempt_dir.mkdir()
    hashes = {str(metadata_path): file_sha256(metadata_path)}
    for relative in metadata["relative_file_paths"]:
        path = args.bag / relative
        hashes[str(path)] = file_sha256(path)

    contexts, nodes, executors, logs = [], [], [], []
    child = player = None
    result = {}
    source_payloads, target_payloads = defaultdict(Counter), defaultdict(list)
    source_counts, target_counts = Counter(), Counter()
    source_excluded, target_excluded = Counter(), Counter()
    samples, first_source = {}, {}
    chains = {}
    started = time.monotonic()
    try:
        for name, domain in (("shadow_bag_source_probe", 134), ("shadow_bag_target_probe", 135)):
            context = Context()
            context.init(args=[], domain_id=domain, initialize_logging=False)
            node = Node(name, context=context, use_global_arguments=False,
                        enable_rosout=False, start_parameter_services=False)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            contexts.append(context)
            nodes.append(node)
            executors.append(executor)
        src, dst = nodes
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        for topic, _kind, _wire in selected:
            def source(payload, t=topic):
                payload = bytes(payload)
                source_counts[t] += 1
                source_payloads[t][(len(payload), hashlib.sha256(payload).hexdigest())] += 1
                first_source.setdefault(t, payload)
                samples[("source_last", t)] = payload

            def target(payload, t=topic):
                payload = bytes(payload)
                target_counts[t] += 1
                target_payloads[t].append(tuple(
                    (len(payload)-padding, hashlib.sha256(payload[:len(payload)-padding]).hexdigest())
                    for padding in range(4) if len(payload) > padding))
                chain = chains.setdefault(t, hashlib.sha256())
                chain.update(len(payload).to_bytes(8, "little"))
                chain.update(payload)
                samples.setdefault(("target_first", t), payload)
                samples[("target_last", t)] = payload

            src.create_subscription(_kind, topic, source, qos, raw=True)
            dst.create_subscription(_kind, topic, target, qos, raw=True)
        for topic, kind in excluded:
            src.create_subscription(kind, topic,
                lambda _payload, t=topic: source_excluded.update([t]), qos, raw=True)
            dst.create_subscription(kind, topic,
                lambda _payload, t=topic: target_excluded.update([t]), qos, raw=True)

        def pump(duration):
            until = time.monotonic() + duration
            while time.monotonic() < until:
                for executor in executors:
                    executor.spin_once(timeout_sec=.001)

        receipt = attempt_dir / "ingress-process.json"
        relay_log = (attempt_dir / "ingress.log").open("w")
        logs.append(relay_log)
        relay_command = [sys.executable, str(args.launcher), "--executable", str(args.executable), "--source-domain", "134", "--research-domain", "135",
                         "--duration-s", str(args.duration_s + 15), "--audit-payloads", "true",
                         "--output", str(receipt)]
        child = subprocess.Popen(relay_command, stdout=relay_log, stderr=subprocess.STDOUT)
        pump(1.)
        if child.poll() is not None:
            raise RuntimeError("native ingress exited during discovery")
        source_writers = src.get_publisher_names_and_types_by_node(
            "research_shadow_source_tap", "/")
        player_log = (attempt_dir / "player.log").open("w")
        logs.append(player_log)
        player_command = ["ros2", "bag", "play", str(args.bag), "--rate", "1.0",
                          "--disable-keyboard-controls", "--topics",
                          *[t for t, _, _ in selected], *[t for t, _ in excluded]]
        player_env = dict(os.environ, ROS_DOMAIN_ID="134", ROS_LOCALHOST_ONLY="1")
        player = subprocess.Popen(player_command, env=player_env, stdout=player_log,
                                  stderr=subprocess.STDOUT)
        startup_deadline = time.monotonic() + 5
        while not (source_counts["/livox/imu"] and source_counts["/livox/lidar"]):
            if player.poll() is not None or child.poll() is not None or time.monotonic() > startup_deadline:
                raise RuntimeError("actual player raw MID360 startup failed")
            pump(.02)
        prefix_start = time.monotonic()
        while time.monotonic() - prefix_start < args.duration_s:
            if player.poll() is not None or child.poll() is not None:
                raise RuntimeError("player/ingress exited before the requested prefix")
            pump(.02)
        prefix_completed_before_cleanup = True
        player_exit_before_cleanup = player.poll()
        stop_child(player)
        pump(.6)
        before = dict(target_counts)
        pump(.5)
        silence_unchanged = before == dict(target_counts)
        stop_child(child)
        launcher_receipt = json.loads(receipt.read_text())
        process = launcher_receipt["native_receipt"]

        mismatches, matching_counts = [], {}
        for topic, candidates_list in target_payloads.items():
            used = Counter()
            for candidates in candidates_list:
                matches = [key for key in candidates if used[key] < source_payloads[topic][key]]
                if not matches:
                    mismatches.append({"topic": topic, "candidate_lengths": [k[0] for k in candidates]})
                else:
                    used[matches[0]] += 1
            matching_counts[topic] = sum(used.values())
        spans, first_field_equality, last_field_equality = {}, {}, {}
        for topic, kind, _wire in selected:
            first, last = deserialize_message(first_source[topic], kind), deserialize_message(
                samples[("source_last", topic)], kind)
            if hasattr(first, "header"):
                stamp = lambda msg: msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                spans[topic] = stamp(last) - stamp(first)
            first_field_equality[topic] = deserialize_message(
                samples[("target_first", topic)], kind) == first
            last_field_equality[topic] = deserialize_message(
                samples[("target_last", topic)], kind) == last
        received_chains = {topic: chain.hexdigest() for topic, chain in chains.items()}
        ratios = {t: target_counts[t]/source_counts[t] if source_counts[t] else 0.
                  for t, _, _ in selected}
        checks = {
            "actual_requested_prefix_completed_before_cleanup": prefix_completed_before_cleanup,
            "prefix_is_explicitly_not_full_eof": player_exit_before_cleanup is None,
            "all_recorded_allowlist_inputs_have_actual_source_and_target_samples":
                all(source_counts[t] and target_counts[t] for t, _, _ in selected),
            "every_target_original_cdr_payload_matches_actual_source_with_only_tail_alignment":
                not mismatches and matching_counts == dict(target_counts),
            "complete_native_forwarded_cdr_streams_and_counts_match_target":
                received_chains == process["input_cdr_chains"] and dict(target_counts) == process["forwarded"],
            "per_topic_observed_ingress_delivery_at_least_98_percent": all(v >= .98 for v in ratios.values()),
            "actual_raw_lidar_and_imu_header_spans_cover_prefix":
                all(spans.get(t, 0) >= args.duration_s - .2 for t in ("/livox/lidar", "/livox/imu")),
            "recorded_legacy_tf_and_lio_inputs_observed_but_never_forwarded":
                bool(excluded) and all(source_excluded[t] > 0 and target_excluded[t] == 0 for t, _ in excluded),
            "source_tap_has_zero_publishers": source_writers == [],
            "source_silence_does_not_replay_or_refresh": silence_unchanged,
            "finite_relay_stopped_without_false_completion":
                child.returncode == 130 and launcher_receipt["status"] == "INTERRUPTED" and process["status"] == "INTERRUPTED" and process["termination"] == "SIGINT",
            "no_source_owner_or_sample_writer_denials":
                not process["denials"],
        }
        result = {
            "status": "PASS" if all(checks.values()) else "FAIL",
            "checks": checks, "source_counts": dict(source_counts), "target_counts": dict(target_counts),
            "source_excluded_counts": dict(source_excluded), "target_excluded_counts": dict(target_excluded),
            "observed_delivery_ratios": ratios, "source_header_spans_s": spans,
            "mismatches": mismatches[:20], "original_payload_matching_counts": matching_counts,
            "first_fields_exact_equality": first_field_equality,
            "last_fields_exact_equality": last_field_equality,
            "received_cdr_chains": received_chains, "process_receipt": process, "launcher_receipt": launcher_receipt,
            "source_tap_publishers": source_writers,
            "bag": str(args.bag), "bag_hashes": hashes,
            "bag_metadata_duration_s": metadata["duration"]["nanoseconds"] * 1e-9,
            "selected_topics": [t for t, _, _ in selected],
            "absent_or_zero_count_allowlist_topics": [t for t, _, _ in SENSORS if t not in {s[0] for s in selected}],
            "requested_prefix_s": args.duration_s, "replay_scope": "PREFIX", "rate": 1.,
            "player_exit_before_cleanup": player_exit_before_cleanup,
            "domains": [134, 135], "relay_command": relay_command, "player_command": player_command,
            "executable_sha256": file_sha256(args.executable),
            "launcher_sha256": file_sha256(args.launcher),
            "native_source_sha256": file_sha256(args.source_root / "sensor_ingress.cpp"),
            "cmake_sha256": file_sha256(args.source_root / "CMakeLists.txt"),
            "validator_sha256": file_sha256(Path(__file__)),
            "wall_s_excluding_bag_hashing": time.monotonic() - started,
            "scope": "Actual recorded MID360 and RTK sensor data through native DDS ingress between lab domains134/135; explicitly bounded prefix. Exact native serialized streams and original source payloads are checked. No localization, ground, actuator, target machine, live freshness, authentication or physical acceptance claim."
        }
    except Exception as error:
        result = {"status": "FAIL", "error": f"{type(error).__name__}: {error}",
                  "source_counts": dict(source_counts), "target_counts": dict(target_counts),
                  "bag_hashes": hashes}
    finally:
        stop_child(player)
        stop_child(child)
        for log in logs:
            log.close()
        for executor, node in zip(executors, nodes):
            executor.remove_node(node)
            executor.shutdown()
            node.destroy_node()
        for context in contexts:
            if context.ok():
                context.shutdown()
        args.output.write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result, indent=2))
    return 0 if result.get("status") == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
