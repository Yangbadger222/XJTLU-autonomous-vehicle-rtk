#!/usr/bin/env python3
"""Actual patched EGO ROS input/output checks with labelled synthetic inputs."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path as RosPath
from std_msgs.msg import String
from research_interfaces.msg import TimedTrajectory2D
from research_interfaces.srv import PlanRoadReference


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--install", type=Path, required=True)
    parser.add_argument("--repo", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if os.environ.get("ROS_DOMAIN_ID") != "91" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise SystemExit("requires isolated domain 91")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node("ego_vehicle_input_probe")
    messages = []
    node.create_subscription(TimedTrajectory2D, "/research/ego_trajectory", messages.append, 100)
    odom_pub = node.create_publisher(Odometry, "/lio/odom_vehicle", 10)
    grid_pub = node.create_publisher(OccupancyGrid, "/research/local_obstacle_grid", 10)
    ref_pub = node.create_publisher(RosPath, "/research/road_reference", 10)
    ver_pub = node.create_publisher(String, "/research/map_version", 10)
    log = args.output.with_suffix(".node.log").open("w")
    command = [str(args.install / "ego_planner/lib/ego_planner/motion_plan"), "--ros-args",
               "--params-file", str(args.repo / "src/bringup/config/ego_vehicle_adapter.yaml"),
               "-p", "max_curvature_1pm:=1.0", "-p", "max_lateral_speed_mps:=0.05",
               "-p", "max_jerk_mps3:=3.0", "-p", "inflate_radius_m:=" + str(math.hypot(0.33, 0.305))]
    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    cases = []
    query_client = node.create_client(PlanRoadReference, "/research/ego_plan_query")

    def phase(speed, yaw, scenario, seconds=2.0):
        messages.clear()
        started = time.monotonic()
        deadline = started + seconds
        while time.monotonic() < deadline and process.poll() is None:
            stamp = node.get_clock().now().to_msg()
            ver_pub.publish(String(data="UNKNOWN" if scenario == "unknown_map" else "sim-input-map"))
            odom = Odometry()
            odom.header.frame_id = "map" if scenario == "wrong_odom_frame" else "odom"
            odom.header.stamp, odom.child_frame_id = stamp, "base_footprint"
            odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
            odom.twist.twist.linear.x = speed
            if scenario == "arc_moving":
                angle = speed/3*(time.monotonic()-started)
                odom.pose.pose.position.x = 3*math.sin(angle)
                odom.pose.pose.position.y = 3*(1-math.cos(angle))
                odom.pose.pose.orientation.z,odom.pose.pose.orientation.w = math.sin(angle/2),math.cos(angle/2)
                odom.twist.twist.angular.z = speed/3
            odom_pub.publish(odom)
            grid = OccupancyGrid()
            grid.header.frame_id, grid.header.stamp = "odom", stamp
            grid.info.resolution, grid.info.width, grid.info.height = 0.3, 100, 100
            grid.info.origin.position.x = grid.info.origin.position.y = -15.0
            grid.info.origin.orientation.w = 1.0
            grid.data = [-1 if scenario == "unknown_ground" else 0] * 10000
            grid_pub.publish(grid)
            reference = RosPath()
            reference.header.frame_id, reference.header.stamp = "odom", stamp
            for i in range(12):
                point = PoseStamped()
                point.header = reference.header
                point.pose.position.x, point.pose.position.y = i*0.3*math.cos(yaw), i*0.3*math.sin(yaw)
                if scenario.startswith("arc_"):
                    point.pose.position.x = 3*math.sin(i*.1)
                    point.pose.position.y = 3*(1-math.cos(i*.1))
                point.pose.orientation.w = 1.0
                reference.poses.append(point)
            ref_pub.publish(reference)
            rclpy.spin_once(node, timeout_sec=0.025)
            time.sleep(0.025)
        ok = [m for m in messages if m.status == m.STATUS_OK and m.points]
        cases.append({"scenario": scenario, "input_body_speed": speed, "input_yaw": yaw,
                      "received": len(messages), "ok_count": len(ok),
                      "failure_reasons": sorted({m.failure_reason for m in messages if m.failure_reason}),
                      "first_output_speed": ok[-1].points[0].v if ok else None,
                      "max_output_speed": max((p.v for m in ok for p in m.points), default=None),
                      "max_output_yaw_rate": max((abs(p.w) for m in ok for p in m.points), default=None),
                      "max_output_accel": max((abs(p.a) for m in ok for p in m.points), default=None),
                      "status": "PASS" if ((scenario in ("free","arc_rest","arc_moving") and ok) or
                          (scenario not in ("free","arc_rest","arc_moving") and messages and not ok)) else "FAIL"})

    try:
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        for speed, yaw in [(0.0, 0.0), (0.12, 0.0), (0.2, 0.3)]:
            phase(speed, yaw, "free")
        phase(0.0,0.0,"arc_rest")
        phase(.2,0.0,"arc_moving")
        for scenario in ("wrong_odom_frame", "unknown_map", "unknown_ground"):
            phase(0.0, 0.0, scenario)
        query_cases = []
        if query_client.wait_for_service(timeout_sec=2):
            for version in ("sim-input-map", "stale-candidate-map"):
                phase(0.0, 0.0, "free", seconds=0.5)
                request = PlanRoadReference.Request()
                request.request_id, request.map_version = "candidate-query", version
                request.road_reference.header.frame_id = "odom"
                request.road_reference.header.stamp = node.get_clock().now().to_msg()
                for i in range(12):
                    point = PoseStamped()
                    point.header = request.road_reference.header
                    point.pose.position.x, point.pose.orientation.w = i*.15, 1.0
                    request.road_reference.poses.append(point)
                future = query_client.call_async(request)
                rclpy.spin_until_future_complete(node, future, timeout_sec=.18)
                response = future.result() if future.done() else None
                output = response.trajectory if response else None
                candidate_published = any(m.trajectory_id == "candidate-query" for m in messages)
                expected_ok = version == "sim-input-map"
                passed = (output is not None and not candidate_published and
                          ((expected_ok and output.status == output.STATUS_OK and output.points) or
                           (not expected_ok and output.status == output.STATUS_UNKNOWN_MAP)))
                query_cases.append({"map_version": version, "status": "PASS" if passed else "FAIL",
                                    "output_status": output.status if output else None,
                                    "candidate_published_to_controller": candidate_published,
                                    "output_end_x": output.points[-1].x if output and output.points else None})
        else:
            query_cases.append({"status": "FAIL", "reason": "query_service_unavailable"})
        result = {"source_commit": "7f5be6d4cee34871e85aa1f15285cfaf17b23877", "patches": [1, 2, 3, 4],
                  "simulation_only_settings": {"max_curvature_1pm": 1.0, "max_lateral_speed_mps": 0.05,
                                               "max_jerk_mps3": 3.0},
                  "inflate_radius_from_locked_footprint_m": math.hypot(0.33, 0.305),
                  "scope": "synthetic real ROS transport through actual EGO core; no vehicle deployment/closed-loop claim",
                  "cases": cases, "process_exit_before_cleanup": process.poll(),
                  "query_cases": query_cases,
                  "status": "PASS" if all(c["status"] == "PASS" for c in cases+query_cases) and process.poll() is None else "FAIL"}
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=5)
        log.close()
        node.destroy_node()
        rclpy.shutdown()
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
