#!/usr/bin/env python3
"""Exercise the unmodified EGO/fake_sim binaries in an isolated ROS domain."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path as RosPath
from visualization_msgs.msg import MarkerArray


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--install", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if os.environ.get("ROS_DOMAIN_ID") != "91" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise SystemExit("requires domain 91 and localhost-only isolation")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    children, logs = [], []
    rclpy.init()
    node = rclpy.create_node("raw_ego_acceptance_probe")
    result = {"source_commit": "7f5be6d4cee34871e85aa1f15285cfaf17b23877",
              "source_modifications": False, "domain": 91,
              "rviz_gui": "NOT_RUN_HEADLESS", "local_paths": 0,
              "astar_marker_messages": 0, "pose_messages": 0, "max_path_points": 0,
              "scenario": "RViz input topics: initial pose, two route points, obstacle box"}
    poses = []

    def path_cb(msg):
        if msg.poses:
            result["local_paths"] += 1
            result["max_path_points"] = max(result["max_path_points"], len(msg.poses))

    def pose_cb(msg):
        result["pose_messages"] += 1
        poses.append((msg.pose.position.x, msg.pose.position.y))

    def astar_cb(msg):
        if msg.markers:
            result["astar_marker_messages"] += 1

    node.create_subscription(RosPath, "/visual_local_trajectory", path_cb, 10)
    node.create_subscription(PoseStamped, "/current_pose", pose_cb, 10)
    node.create_subscription(MarkerArray, "/trajectories", astar_cb, 10)
    point_pub = node.create_publisher(PointStamped, "/clicked_point", 10)
    obstacle_pub = node.create_publisher(PoseStamped, "/goal_pose", 10)
    reset_pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)

    def spin_for(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)

    try:
        for package, binary in [("ego_planner", "motion_plan"), ("rviz_car_sim", "fake_sim_node")]:
            log = (args.output.parent / (binary + "-raw.log")).open("w")
            logs.append(log)
            process = subprocess.Popen([str(args.install / package / "lib" / package / binary)],
                                       stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            children.append(process)
        spin_for(2.0)
        reset = PoseWithCovarianceStamped()
        reset.header.frame_id = "map"
        reset.pose.pose.orientation.w = 1.0
        reset_pub.publish(reset)
        spin_for(0.3)
        obstacle = PoseStamped()
        obstacle.header.frame_id = "map"
        obstacle.pose.position.x, obstacle.pose.position.y = 3.0, 0.0
        obstacle.pose.orientation.w = 1.0
        obstacle_pub.publish(obstacle)
        for x, y in [(0.0, 0.0), (6.0, 0.0)]:
            point = PointStamped()
            point.header.frame_id = "map"
            point.header.stamp = node.get_clock().now().to_msg()
            point.point.x, point.point.y = x, y
            point_pub.publish(point)
            spin_for(0.2)
        spin_for(12.0)
        result["simulated_displacement_m"] = (math.dist(poses[0], poses[-1]) if poses else 0.0)
        result["child_exit_before_cleanup"] = [p.poll() for p in children]
        result["status"] = "PASS" if (result["local_paths"] > 0 and
                result["astar_marker_messages"] > 0 and result["simulated_displacement_m"] > 0.1 and
                all(p.poll() is None for p in children)) else "FAIL"
    finally:
        for process in children:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for process in children:
            try:
                process.wait(timeout=4)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=4)
        for log in logs:
            log.close()
        node.destroy_node()
        rclpy.shutdown()
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
    return 0 if result.get("status") == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
