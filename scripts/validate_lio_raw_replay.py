#!/usr/bin/env python3
"""Sequential real sensor playback into one LIO binary, actuator-free."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile, ReliabilityPolicy
from livox_ros_driver2.msg import CustomMsg
from raw_replay_contract import replay_contract


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=5)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--kind", choices=("fastlio", "superlio"), required=True)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    contract = replay_contract(args.bag)
    if os.environ.get("ROS_DOMAIN_ID") != "92" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise SystemExit("requires domain 92 and localhost-only isolation")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node("lio_raw_replay_probe", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
    odometry, health, received_imu, received_lidar, resources = [], {}, [0], [0], []

    def odom_cb(msg):
        q, p = msg.pose.pose.orientation, msg.pose.pose.position
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        odometry.append({"stamp": stamp, "position": [p.x, p.y, p.z],
                         "quaternion_xyzw": [q.x, q.y, q.z, q.w],
                         "linear": [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z],
                         "frame": msg.header.frame_id, "child": msg.child_frame_id,
                         "sim_clock_minus_stamp_s": node.get_clock().now().nanoseconds * 1e-9 - stamp})

    def health_cb(msg):
        health[msg.data] = health.get(msg.data, 0) + 1

    def imu_cb(msg):
        received_imu[0] += 1

    topic = "/fastlio2/lio_odom" if args.kind == "fastlio" else "/lio/odom"
    node.create_subscription(Odometry, topic, odom_cb, 100)
    node.create_subscription(String, "/lio/health", health_cb, 100)
    node.create_subscription(Imu, "/livox/imu", imu_cb,
        QoSProfile(depth=1024,reliability=ReliabilityPolicy.BEST_EFFORT))
    node.create_subscription(CustomMsg, "/livox/lidar", lambda msg: received_lidar.__setitem__(0, received_lidar[0]+1), qos_profile_sensor_data)
    command = [str(args.binary), "--ros-args", "--params-file", str(args.config), "-p", "use_sim_time:=true"]
    if args.kind == "fastlio":
        command += ["-r", "__ns:=/fastlio2"]
    logs = [args.output.with_suffix(".node.log").open("w"), args.output.with_suffix(".play.log").open("w")]
    algorithm = subprocess.Popen(command, stdout=logs[0], stderr=subprocess.STDOUT, start_new_session=True)
    playback = None
    started = time.monotonic()
    try:
        until = time.monotonic() + 2.0
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.05)
        playback = subprocess.Popen(["ros2", "bag", "play", str(args.bag), "--rate", "1.0", "--clock", "50",
                                     "--topics", "/livox/lidar", "/livox/imu"],
                                    stdout=logs[1], stderr=subprocess.STDOUT, start_new_session=True)
        sample_at = time.monotonic()
        deadline = started + contract["wall_budget_s"]
        while playback.poll() is None and algorithm.poll() is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
            if time.monotonic() >= sample_at:
                try:
                    fields = Path(f"/proc/{algorithm.pid}/stat").read_text().split()
                    status = Path(f"/proc/{algorithm.pid}/status").read_text()
                    rss_kb = int(next(line.split()[1] for line in status.splitlines() if line.startswith("VmRSS:")))
                    resources.append({"elapsed_s": time.monotonic() - started,
                                      "cpu_seconds": (int(fields[13]) + int(fields[14])) / os.sysconf("SC_CLK_TCK"),
                                      "rss_kb": rss_kb})
                except (OSError, StopIteration):
                    pass
                sample_at += 1.0
        playback_exit = playback.poll()
        until = time.monotonic() + 2.0
        while time.monotonic() < until and algorithm.poll() is None:
            rclpy.spin_once(node, timeout_sec=0.02)
        result = {"kind": args.kind, "binary": str(args.binary),
                  "binary_sha256": hashlib.sha256(args.binary.read_bytes()).hexdigest(),
                  "config_sha256": hashlib.sha256(args.config.read_bytes()).hexdigest(),
                  "bag": str(args.bag), "domain": 92, "play_rate": 1.0,
                  "topics_played": ["/livox/lidar", "/livox/imu"],
                  "playback_exit": playback_exit, "algorithm_exit_before_cleanup": algorithm.poll(),
                  "replay_contract": contract, "timed_out": playback_exit is None and time.monotonic() >= deadline,
                  "lidar_received": received_lidar[0],
                  "imu_received": received_imu[0], "odometry_count": len(odometry),
                  "health_counts": health, "wall_elapsed_s": time.monotonic() - started,
                  "resources": resources, "odometry": odometry,
                  "status": "PASS" if playback_exit == 0 and algorithm.poll() is None and len(odometry) > 100 else "FAIL",
                  "scope": "real raw bag algorithm execution only; no vehicle frame/health equivalence or RTK accuracy acceptance"}
    finally:
        if playback:
            stop(playback)
        stop(algorithm)
        for log in logs:
            log.close()
        node.destroy_node()
        rclpy.shutdown()
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({k: v for k, v in result.items() if k not in {"resources", "odometry"}}, indent=2))
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
