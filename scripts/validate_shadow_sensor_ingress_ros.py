#!/usr/bin/env python3
"""Actual two-domain Humble qualification using nontrivial analytical fixtures."""
import argparse
from collections import defaultdict
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import QuaternionStamped, Twist
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from nmea_msgs.msg import Sentence
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

SENSORS = (
    ("/livox/lidar", CustomMsg, "livox_ros_driver2/msg/CustomMsg"),
    ("/livox/imu", Imu, "sensor_msgs/msg/Imu"),
    ("/fix", NavSatFix, "sensor_msgs/msg/NavSatFix"),
    ("/heading", QuaternionStamped, "geometry_msgs/msg/QuaternionStamped"),
    ("/rtk/nmea_sentence", Sentence, "nmea_msgs/msg/Sentence"),
    ("/rtk/status", String, "std_msgs/msg/String"),
    ("/rtk/health", DiagnosticArray, "diagnostic_msgs/msg/DiagnosticArray"),
)


def sample(topic, kind):
    msg = kind()
    if hasattr(msg, "header"):
        msg.header.stamp.sec = 123
        msg.header.stamp.nanosec = 456
        msg.header.frame_id = "unchanged-fixture"
    if topic == "/livox/lidar":
        msg.timebase = 1700000000123456789
        msg.lidar_id = 3
        point = CustomPoint()
        point.offset_time = 9234567
        point.x, point.y, point.z = 1.25, -2.5, 0.447
        point.reflectivity, point.tag, point.line = 37, 16, 2
        msg.points, msg.point_num = [point], 1
    if topic == "/livox/imu":
        msg.orientation.x, msg.orientation.y = .1, -.2
        msg.orientation.z, msg.orientation.w = .3, .9
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = .04, -.06, .2
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = 1.2, -2.3, 9.80665
        msg.orientation_covariance = [.01, 0., 0., 0., .02, 0., 0., 0., .03]
        msg.angular_velocity_covariance = [.04, 0., 0., 0., .05, 0., 0., 0., .06]
        msg.linear_acceleration_covariance = [.07, 0., 0., 0., .08, 0., 0., 0., .09]
    if topic == "/fix":
        msg.status.status, msg.status.service = 2, 1
        msg.latitude, msg.longitude, msg.altitude = 31.2741, 120.7377, 42.125
        msg.position_covariance = [.01, .002, 0., .002, .02, 0., 0., 0., .03]
        msg.position_covariance_type = 3
    if topic == "/heading":
        msg.quaternion.x, msg.quaternion.y = .1, -.2
        msg.quaternion.z, msg.quaternion.w = .3, .9
    if topic == "/rtk/nmea_sentence":
        msg.sentence = "$GPGGA,123519,4807.038,N,01131.000,E,4,08,0.9,545.4,M,46.9,M,,*47"
    if topic == "/rtk/status":
        msg.data = "analytical fixture, do not grant authority"
    if topic == "/rtk/health":
        status = DiagnosticStatus()
        status.level = bytes([1])
        status.name, status.message = "analytical_fixture", "do not grant authority"
        status.hardware_id = "fixture-only"
        value = KeyValue()
        value.key, value.value = "source_time", "123.000000456"
        status.values, msg.status = [value], [status]
    return msg


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--launcher", type=Path, required=True)
    parser.add_argument("--executable", type=Path, required=True)
    parser.add_argument("--source-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    launcher_command = [sys.executable, str(args.launcher), "--executable", str(args.executable)]
    if os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        parser.error("localhost-only required")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    attempt_dir = args.output.parent / (args.output.stem + "-attempt-" + str(time.monotonic_ns()))
    attempt_dir.mkdir()
    contexts, nodes, executors = [], [], []
    child = log_file = None
    result = {}
    try:
        for name, domain in (("shadow_source_fixture", 130), ("shadow_target_fixture", 131)):
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
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        pubs, wire, seen = {}, {}, defaultdict(list)
        source_controls, target_forbidden = [], []
        for topic, kind, _wire_type in SENSORS:
            pubs[topic] = src.create_publisher(kind, topic, qos)
            wire[topic] = bytes(serialize_message(sample(topic, kind)))
            dst.create_subscription(kind, topic,
                lambda payload, t=topic: seen[t].append(bytes(payload)), qos, raw=True)
        src.create_subscription(Twist, "/cmd_vel", source_controls.append, qos)
        research_cmd = dst.create_publisher(Twist, "/cmd_vel", qos)
        source_forbidden = [
            (src.create_publisher(Twist, "/cmd_vel_safe", qos), Twist()),
            (src.create_publisher(TFMessage, "/tf", qos), TFMessage()),
            (src.create_publisher(String, "/research/operator_consent", qos),
             String(data="fixture only")),
        ]
        for topic, kind in (("/cmd_vel_safe", Twist), ("/tf", TFMessage),
                            ("/research/operator_consent", String)):
            dst.create_subscription(kind, topic, target_forbidden.append, qos)

        def pump(duration):
            until = time.monotonic() + duration
            while time.monotonic() < until:
                for executor in executors:
                    executor.spin_once(timeout_sec=.003)

        pump(.5)
        receipt = attempt_dir / "process.json"
        log_file = args.output.with_suffix(".relay.log").open("w")
        command = launcher_command + [ "--source-domain", "130", "--research-domain", "131",
                   "--duration-s", "40", "--discovery-delay-s", "4", "--audit-payloads", "true", "--output", str(receipt)]
        child = subprocess.Popen(command, stdout=log_file, stderr=subprocess.STDOUT)
        subscriptions = []
        deadline = time.monotonic() + 2.
        while time.monotonic() < deadline:
            try:
                subscriptions = src.get_subscriber_names_and_types_by_node(
                    "research_shadow_source_tap", "/")
            except Exception as error:
                if type(error).__name__ != "NodeNameNonExistentError":
                    raise
                subscriptions = []
            if {(t, tuple(types)) for t, types in subscriptions} >= {(t, (w,)) for t, _, w in SENSORS}:
                break
            pump(.03)
        if not {(t, tuple(types)) for t, types in subscriptions} >= {(t, (w,)) for t, _, w in SENSORS}:
            raise RuntimeError("native source tap discovery failed before queue test: " + repr(subscriptions))

        # Queue a rival sample while the relay's finite discovery delay keeps
        # its executor idle. DDS threads still receive and update the graph.
        rival = src.create_publisher(Imu, "/livox/imu", qos)
        pump(.3)
        rival_sample = Imu()
        rival_sample.header.stamp.sec = 999
        rival_sample.header.frame_id = "removed-writer-fixture"
        rival_wire = bytes(serialize_message(rival_sample))
        endpoints = src.get_publishers_info_by_topic("/livox/imu")
        rival_graph_count = len(endpoints)
        rival.publish(rival_wire)
        pump(.15)
        src.destroy_publisher(rival)
        pump(.6)
        graph_count_before_dispatch = len(src.get_publishers_info_by_topic("/livox/imu"))
        pump(4.)
        queued_forwarded = any(payload == rival_wire for payload in seen["/livox/imu"])

        for _ in range(12):
            for topic, pub in pubs.items():
                pub.publish(wire[topic])
            research_cmd.publish(Twist())
            for pub, msg in source_forbidden:
                pub.publish(msg)
            pump(.12)
        pump(.3)
        checks = {}
        checks["all7_original_cdr_bytes_preserved_with_rmw_tail_alignment"] = all(
            seen[topic] and all(payload.startswith(wire[topic]) and len(payload) == (len(wire[topic]) + 3) // 4 * 4 for payload in seen[topic])
            for topic, _, _ in SENSORS)
        checks["all7_decoded_fields_unchanged"] = all(
            seen[topic] and all(deserialize_message(payload, kind) ==
                deserialize_message(wire[topic], kind) for payload in seen[topic])
            for topic, kind, _ in SENSORS)
        byte_diagnostics = {}
        for topic, _kind, _wire_type in SENSORS:
            if seen[topic]:
                actual, expected = seen[topic][0], wire[topic]
                byte_diagnostics[topic] = {
                    "source_bytes": len(expected), "target_bytes": len(actual),
                    "first_different_offsets": [i for i, (a, b) in enumerate(zip(expected, actual)) if a != b][:16],
                    "expected_at_differences": [expected[i] for i, (a, b) in enumerate(zip(expected, actual)) if a != b][:16],
                    "actual_at_differences": [actual[i] for i, (a, b) in enumerate(zip(expected, actual)) if a != b][:16],
                }
        source_writers = src.get_publisher_names_and_types_by_node(
            "research_shadow_source_tap", "/")
        output_writers = dst.get_publisher_names_and_types_by_node(
            "research_shadow_sensor_ingress", "/")
        checks["source_tap_zero_publishers_including_metadata"] = source_writers == []
        checks["target_has_exact_sensor_writer_allowlist"] = {
            (topic, tuple(types)) for topic, types in output_writers
        } == {(topic, (wire_type,)) for topic, _, wire_type in SENSORS}
        checks["research_control_cannot_cross_to_source"] = not source_controls
        checks["source_tf_control_consent_cannot_cross_to_research"] = not target_forbidden
        before = {topic: len(payloads) for topic, payloads in seen.items()}
        pump(.7)
        checks["source_silence_never_refreshes_or_replays"] = before == {
            topic: len(payloads) for topic, payloads in seen.items()}
        duplicate = src.create_publisher(Imu, "/livox/imu", qos)
        pump(.5)
        base = len(seen["/livox/imu"])
        for _ in range(8):
            pubs["/livox/imu"].publish(wire["/livox/imu"])
            duplicate.publish(wire["/livox/imu"])
            pump(.1)
        checks["converged_duplicate_sources_withhold_input"] = len(seen["/livox/imu"]) == base
        src.destroy_publisher(duplicate)
        pump(.5)
        pubs["/livox/imu"].publish(wire["/livox/imu"])
        pump(.3)
        checks["unique_source_resumes_exact_payload"] = (
            len(seen["/livox/imu"]) > base and deserialize_message(seen["/livox/imu"][-1], Imu) == deserialize_message(wire["/livox/imu"], Imu))

        invalid_receipt = attempt_dir / "invalid-domain.json"
        invalid_receipt.unlink(missing_ok=True)
        invalid = subprocess.run(launcher_command + [ "--source-domain", "130",
            "--research-domain", "130", "--duration-s", "1", "--output", str(invalid_receipt)],
            capture_output=True, text=True, timeout=4)
        checks["same_domain_rejected_before_ros_init"] = (
            invalid.returncode == 2 and "two distinct" in invalid.stderr and not invalid_receipt.exists())
        failed_receipt = attempt_dir / "startup-failed.json"
        failed_receipt.unlink(missing_ok=True)
        failed_env = dict(os.environ, AMENT_PREFIX_PATH="/opt/ros/humble")
        failed = subprocess.run(launcher_command + [ "--source-domain", "132",
            "--research-domain", "133", "--duration-s", "1", "--output", str(failed_receipt)],
            env=failed_env, capture_output=True, text=True, timeout=6)
        failure = json.loads(failed_receipt.read_text()) if failed_receipt.exists() else {}
        checks["actual_typesupport_lookup_exception_is_failed_receipt"] = (
            failed.returncode == 1 and failure.get("status") == "FAILED" and
            (failure.get("native_receipt") or {}).get("termination") == "exception" and bool((failure.get("native_receipt") or {}).get("error")))

        fatal_receipt = attempt_dir / "fatal-rmw.json"
        fatal_receipt.unlink(missing_ok=True)
        fatal = subprocess.run(launcher_command + [ "--source-domain", "132",
            "--research-domain", "133", "--duration-s", "1", "--output", str(fatal_receipt)],
            env=dict(os.environ, RMW_IMPLEMENTATION="codex_intentionally_missing_rmw"),
            capture_output=True, text=True, timeout=6)
        checks["fatal_rcl_missing_rmw_never_claims_completed"] = (
            fatal.returncode == 1 and fatal_receipt.exists() and json.loads(fatal_receipt.read_text()).get("status") == "FAILED" and bool(json.loads(fatal_receipt.read_text()).get("native_stderr_tail")))
        old_receipt = attempt_dir / "previous-completed.json"
        old_bytes = json.dumps({"status":"COMPLETED","previous_attempt_fixture":True}).encode() + bytes([10])
        old_receipt.write_bytes(old_bytes)
        reused = subprocess.run(launcher_command + [ "--source-domain", "136",
            "--research-domain", "137", "--duration-s", "1", "--output", str(old_receipt)],
            env=dict(os.environ, RMW_IMPLEMENTATION="codex_intentionally_missing_rmw"),
            capture_output=True, text=True, timeout=4)
        checks["old_receipt_rejected_before_ros_and_preserved"] = (
            reused.returncode == 2 and "receipt already exists" in reused.stderr and old_receipt.read_bytes() == old_bytes)
        duration_receipt = attempt_dir / "duration.json"
        duration_run = subprocess.run(launcher_command + [ "--source-domain", "136",
            "--research-domain", "137", "--duration-s", ".2", "--output", str(duration_receipt)],
            capture_output=True, text=True, timeout=4)
        completed = json.loads(duration_receipt.read_text())
        checks["finite_duration_without_input_is_process_completion_only"] = (
            duration_run.returncode == 0 and completed["status"] == "COMPLETED" and
            completed["termination"] == "duration" and completed["native_receipt"]["input_observed"] is False)
        child.send_signal(signal.SIGINT)
        process_exit = child.wait(timeout=5)
        log_file.flush()
        launcher_receipt = json.loads(receipt.read_text())
        process = launcher_receipt["native_receipt"]
        received_chains = {}
        for topic, payloads in seen.items():
            digest = hashlib.sha256()
            for payload in payloads:
                digest.update(len(payload).to_bytes(8, "little"))
                digest.update(payload)
            received_chains[topic] = digest.hexdigest()
        checks["all7_complete_native_received_cdr_streams_unchanged"] = (
            received_chains == process["input_cdr_chains"] and
            {t: len(v) for t, v in seen.items()} == process["forwarded"])
        checks["signal_exit_never_claims_completed"] = (
            process_exit == 130 and launcher_receipt["status"] == "INTERRUPTED" and process["status"] == "INTERRUPTED" and
            process["termination"] == "SIGINT" and process["cleanup_error"] is None)
        checks["queued_removed_writer_rejected_by_actual_sample_gid"] = (
            graph_count_before_dispatch == 1 and not queued_forwarded and
            process["denials"].get("/livox/imu:sample_writer", 0) >= 1)
        result = {
            "status": "PASS" if all(checks.values()) else "FAIL", "checks": checks,
            "counts": {topic: len(payloads) for topic, payloads in seen.items()}, "byte_diagnostics": byte_diagnostics,
            "queued_writer_probe": {"graph_count_before_dispatch": graph_count_before_dispatch,
                "rival_graph_count_before_enqueue": rival_graph_count,
                "removed_writer_forwarded": queued_forwarded,
                "actual_sample_writer_denials": process["denials"].get("/livox/imu:sample_writer", 0)},
            "source_tap_publishers": source_writers, "source_subscriptions": subscriptions, "output_publishers": output_writers,
            "process_receipt": process, "launcher_receipt": launcher_receipt, "process_exit": process_exit, "command": command,
            "received_cdr_chains": received_chains, "existing_output_exit": reused.returncode, "existing_output_stderr": reused.stderr, "duration_receipt": completed, "fatal_rmw_receipt": json.loads(fatal_receipt.read_text()), "fatal_rmw_exit": fatal.returncode, "fatal_rmw_stderr": fatal.stderr, "startup_failure_receipt": failure, "startup_failure_exit": failed.returncode, "startup_failure_stdout": failed.stdout, "startup_failure_stderr": failed.stderr, "domains": [130, 131],
            "executable_sha256": hashlib.sha256(args.executable.read_bytes()).hexdigest(),
            "launcher_sha256": hashlib.sha256(args.launcher.read_bytes()).hexdigest(),
            "native_source_sha256": hashlib.sha256(
                (args.source_root / "sensor_ingress.cpp").read_bytes()).hexdigest(),
            "cmake_sha256": hashlib.sha256(
                (args.source_root / "CMakeLists.txt").read_bytes()).hexdigest(),
            "validator_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            "scope": "Actual Humble C++ serialized callback, writer metadata and DDS between domains 130/131; nontrivial analytical sensor fixtures. No physical inputs, production domain, authentication, target shadow or physical acceptance claim."
        }
    except Exception as error:
        result = {"status": "FAIL", "error": f"{type(error).__name__}: {error}"}
    finally:
        if child is not None and child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait(timeout=2)
        if log_file is not None:
            log_file.close()
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
