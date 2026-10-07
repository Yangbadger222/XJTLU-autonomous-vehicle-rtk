#!/usr/bin/env python3
"""Inject faults through research tracker -> original guard -> original serial.

The only serial endpoint is an allocated PTY. Synthetic kinematic settings
are labelled as such and never written to the vehicle configuration.
"""
import argparse
import json
import math
import os
from pathlib import Path
import pty
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import Twist, QuaternionStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path as RosPath
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Bool, Float32, String, Float32MultiArray, Float64MultiArray
from sensor_msgs.msg import NavSatFix
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nmea_msgs.msg import Sentence
from rclpy.qos import QoSProfile, DurabilityPolicy
from research_interfaces.msg import TimedTrajectory2D, TimedTrajectoryPoint2D, ResearchStatus


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--install", type=Path, required=True)
    parser.add_argument("--repo", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--ego-loop", action="store_true", help="Use actual EGO and PTY-driven planar simulator")
    parser.add_argument("--arc-loop",action="store_true")
    parser.add_argument("--rtk-classifier",action="store_true")
    parser.add_argument("--tf-static-fault",action="store_true")
    parser.add_argument("--loop-budget-s", type=float, default=30.0)
    args = parser.parse_args()
    if args.arc_loop:args.ego_loop=True
    if os.environ.get("ROS_DOMAIN_ID") != "91" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise SystemExit("requires domain 91 and localhost-only isolation")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    master_fd, slave_fd = pty.openpty()
    slave_path = os.ttyname(slave_fd)
    os.set_blocking(master_fd, False)
    rclpy.init()
    node = rclpy.create_node("mock_serial_acceptance_probe")
    children, logs, wire, cases = [], [], [], []
    buffer = b""
    simulated_pose = [0.0, 0.0, 0.0]
    wire_command = [0.0, 0.0]
    simulated_velocity = [0.0, 0.0]
    last_physics_time = node.get_clock().now().nanoseconds
    trajectory_counts = {"ok": 0, "failed": 0}
    tracker_reasons = {}
    master_config = args.repo / "src/bringup/config/master_params.yaml"
    safety_config = args.repo / "src/bringup/config/research_safety_bridge.yaml"
    pubs = {
        "authority": node.create_publisher(Bool, "/localization_authority/motion_allowed", 10),
        "authority_mode":node.create_publisher(String,"/localization_authority/mode",10),
        "speed": node.create_publisher(Float32, "/localization_authority/max_linear_speed_mps", 10),
        "stop": node.create_publisher(Bool, "/gps_corridor/stop_override", 10),
        "health": node.create_publisher(String, "/lio/vehicle_health", 10),
        "odom": node.create_publisher(Odometry, "/lio/odom_vehicle", 10),
        "grid": node.create_publisher(OccupancyGrid, "/research/local_obstacle_grid", 10),
        "version": node.create_publisher(String, "/research/map_version", 10),
        "trajectory": node.create_publisher(TimedTrajectory2D, "/research/ego_trajectory", 10),
        "reference": node.create_publisher(RosPath, "/research/road_reference", 10),
        "permission": node.create_publisher(OccupancyGrid, "/research/permission_grid", 10),
        "tf": node.create_publisher(TFMessage, "/tf", 10),
    }
    competing_tf_node = rclpy.create_node("mock_competing_tf_owner")
    static_tf_pub=competing_tf_node.create_publisher(TFMessage,"/tf_static",QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    gnss={"fix":node.create_publisher(NavSatFix,"/fix",10),
          "heading":node.create_publisher(QuaternionStamped,"/heading",10),
          "nmea":node.create_publisher(Sentence,"/rtk/nmea_sentence",10),
          "health":node.create_publisher(DiagnosticArray,"/rtk/health",10),
          "alignment":node.create_publisher(Float64MultiArray,"/gps_corridor/enu_to_map",10),
          "degeneracy":node.create_publisher(Float32MultiArray,"/fastlio2/degeneracy",10)}
    authority_modes=[]
    node.create_subscription(String,"/localization_authority/mode",lambda msg:authority_modes.append(msg.data),100)
    competing_tf_pub = competing_tf_node.create_publisher(TFMessage, "/tf", 10)

    def trajectory_cb(msg):
        trajectory_counts["ok" if msg.status == msg.STATUS_OK else "failed"] += 1

    if args.ego_loop:
        node.create_subscription(TimedTrajectory2D, "/research/ego_trajectory", trajectory_cb, 100)
    def status_cb(msg):
        tracker_reasons[msg.reason] = tracker_reasons.get(msg.reason, 0) + 1
    node.create_subscription(ResearchStatus, "/research/status", status_cb, 100)

    def drain():
        nonlocal buffer
        while True:
            try:
                data = os.read(master_fd, 65536)
            except BlockingIOError:
                break
            if not data:
                break
            buffer += data
        while b"\n" in buffer:
            line, buffer = buffer.split(b"\n", 1)
            wire.append(line.decode("ascii", errors="replace") + "\n")
            fields = line.decode("ascii").strip().split(",")
            if len(fields) == 2 and fields[0].startswith("vcx=") and fields[1].startswith("wc="):
                wire_command[:] = [float(fields[0][4:]), float(fields[1][3:])]

    def tick(fault="normal"):
        nonlocal last_physics_time
        stamp = node.get_clock().now().to_msg()
        now = stamp.sec * 1_000_000_000 + stamp.nanosec
        if args.ego_loop:
            dt = min((now-last_physics_time) * 1e-9, 0.1)
            simulated_velocity[0] += max(-1.20*dt, min(0.85*dt, wire_command[0]-simulated_velocity[0]))
            simulated_velocity[1] += max(-1.80*dt, min(1.40*dt, wire_command[1]-simulated_velocity[1]))
            simulated_pose[0] += simulated_velocity[0] * math.cos(simulated_pose[2]) * dt
            simulated_pose[1] += simulated_velocity[0] * math.sin(simulated_pose[2]) * dt
            simulated_pose[2] += simulated_velocity[1] * dt
        last_physics_time = now
        if not args.rtk_classifier and fault != "authority_stale":
            pubs["authority"].publish(Bool(data=fault not in {"authority_false", "rtk_authority_loss"}))
        if not args.rtk_classifier and fault!="authority_mode_stale":
            pubs["authority_mode"].publish(String(data="LIO_BRIDGE" if fault=="lio_bridge_motion_denied" else "RTK_AUTHORITATIVE"))
        if not args.rtk_classifier:pubs["speed"].publish(Float32(data=0.0 if fault == "speed_permission_zero" else 0.85))
        if args.rtk_classifier:
            observation=rclpy.time.Time.from_msg(stamp)-rclpy.duration.Duration(seconds=.04)
            observed=observation.to_msg()
            q=1 if fault=="gnss_non_fixed" else 4
            fix=NavSatFix();fix.header.stamp=observed;fix.header.frame_id="gps"
            fix.status.status=2;fix.latitude,fix.longitude,fix.altitude=31.274927,120.737548,0.
            gnss["fix"].publish(fix)
            heading=QuaternionStamped();heading.header.stamp=observed;heading.header.frame_id="gps"
            heading.quaternion.z=math.sin(math.pi/4);heading.quaternion.w=math.cos(math.pi/4);gnss["heading"].publish(heading)
            payload=f"GPGGA,120000.00,3116.49562,N,12044.25288,E,{q},15,0.8,0,M,0,M,,"
            checksum=0
            for character in payload:checksum^=ord(character)
            sentence=Sentence();sentence.header.stamp=observed;sentence.sentence=f"${payload}*{checksum:02X}";gnss["nmea"].publish(sentence)
            diagnostic=DiagnosticArray();diagnostic.header.stamp=stamp;status=DiagnosticStatus();status.name="um982_rtk_driver/health"
            values=dict(fix_quality=str(q),satellites="1" if fault=="gnss_low_satellites" else "15",
                hdop="99" if fault=="gnss_bad_hdop" else "0.8",heading_valid="true",
                heading_control_eligible="false" if fault=="gnss_heading_float" else "true",
                uniheading_position_type="NARROW_FLOAT" if fault=="gnss_heading_float" else "NARROW_INT",
                uniheading_status="SOL_COMPUTED",ntrip_state="RTCM_STALE" if fault=="gnss_rtcm_stale" else "RTCM_FRESH",
                rtcm_age_s="99" if fault=="gnss_rtcm_stale" else "0.1")
            status.values=[KeyValue(key=k,value=v) for k,v in values.items()];diagnostic.status=[status];gnss["health"].publish(diagnostic)
            gnss["alignment"].publish(Float64MultiArray(data=[0.,0.,0.,1.]))
            gnss["degeneracy"].publish(Float32MultiArray(data=[100.,1.,0.])) # Explicit healthy legacy-metric classifier fixture, not Super-LIO equivalence.
        if fault != "stop_heartbeat_lost":
            pubs["stop"].publish(Bool(data=fault == "stop_override"))
        pubs["health"].publish(String(data="UNKNOWN" if fault == "lio_health_unknown" else "OK: SIMULATED"))
        pubs["version"].publish(String(data="m2" if fault == "map_version_changed" else "m1"))
        if fault != "odom_stream_lost":
            odom = Odometry()
            odom.header.frame_id, odom.child_frame_id = ("map" if fault == "wrong_odom_frame" else "odom"), "base_footprint"
            odom.header.stamp = stamp
            if fault == "old_odom_stamp":
                odom.header.stamp.sec -= 2
            odom.pose.pose.position.x = simulated_pose[0] if args.ego_loop else (3.0 if fault == "pose_jump" else 0.0)
            odom.pose.pose.position.y = simulated_pose[1] if args.ego_loop else 0.0
            odom.pose.pose.orientation.z = math.sin(simulated_pose[2]/2)
            odom.pose.pose.orientation.w = 0.0 if fault == "zero_quaternion" else math.cos(simulated_pose[2]/2)
            odom.twist.twist.linear.x = simulated_velocity[0] if args.ego_loop else 0.2
            odom.twist.twist.angular.z = simulated_velocity[1] if args.ego_loop else 0.0
            pubs["odom"].publish(odom)
        if fault != "tf_stream_lost":
            transforms = []
            for parent, child in ((("odom", "base_footprint"),) if args.rtk_classifier else (("map", "odom"), ("odom", "base_footprint"))):
                transform = TransformStamped()
                transform.header.stamp, transform.header.frame_id, transform.child_frame_id = stamp, parent, child
                transform.transform.rotation.w = 1.0
                if child == "base_footprint":
                    transform.transform.translation.x, transform.transform.translation.y = simulated_pose[:2]
                    transform.transform.rotation.z = math.sin(simulated_pose[2]/2)
                    transform.transform.rotation.w = math.cos(simulated_pose[2]/2)
                transforms.append(transform)
            pubs["tf"].publish(TFMessage(transforms=transforms))
            if fault=="tf_static_competitor":static_tf_pub.publish(TFMessage(transforms=transforms))
            if fault == "tf_double_publisher":
                competing_tf_pub.publish(TFMessage(transforms=transforms))
        if fault != "grid_stream_lost":
            grid = OccupancyGrid()
            grid.header.frame_id, grid.header.stamp = "odom", stamp
            grid.info.resolution = 0.3
            grid.info.width = grid.info.height = 40
            grid.info.origin.position.x = grid.info.origin.position.y = -6.0
            grid.info.origin.orientation.w = 1.0
            grid.data = [-1 if fault == "unknown_ground" else 0] * 1600
            if fault == "footprint_interior_obstacle":
                grid.data[20 * 40 + 20] = 100
            pubs["grid"].publish(grid)
        if fault != "permission_stream_lost":
            permission = OccupancyGrid()
            permission.header.frame_id, permission.header.stamp = "odom", stamp
            permission.info.resolution, permission.info.width, permission.info.height = .3, 40, 40
            permission.info.origin.position.x = permission.info.origin.position.y = -6.0
            permission.info.origin.orientation.w = 1.0
            permission.data = [100 if fault == "keepout" else 0]*1600
            pubs["permission"].publish(permission)
        if args.ego_loop:
            reference = RosPath()
            reference.header.frame_id, reference.header.stamp = "odom", stamp
            for i in range(12):
                point = PoseStamped()
                point.header = reference.header
                if args.arc_loop:
                    angle=(math.pi/3)*i/11
                    point.pose.position.x=3*math.sin(angle);point.pose.position.y=3*(1-math.cos(angle))
                    point.pose.orientation.z,point.pose.orientation.w=math.sin(angle/2),math.cos(angle/2)
                else:point.pose.position.x, point.pose.orientation.w = i*0.3, 1.0
                reference.poses.append(point)
            pubs["reference"].publish(reference)
        elif fault != "planner_stream_lost":
            msg = TimedTrajectory2D()
            msg.header.frame_id, msg.header.stamp = "odom", stamp
            msg.trajectory_id, msg.map_version = "synthetic-safety-fixture", "m1"
            msg.generated_at = stamp
            msg.valid_until.sec = stamp.sec + (2 if fault != "expired_trajectory" else -1)
            msg.valid_until.nanosec = stamp.nanosec
            msg.status = msg.STATUS_INFEASIBLE if fault == "planning_failed" else msg.STATUS_OK
            for t, x in [(0.0, 0.0), (2.0, 0.4)]:
                point = TimedTrajectoryPoint2D()
                point.t, point.x, point.v = t, x, (2.0 if fault == "infeasible_speed" else 0.2)
                msg.points.append(point)
            pubs["trajectory"].publish(msg)
        rclpy.spin_once(node, timeout_sec=0.015)
        drain()

    def phase(fault, seconds):
        start = len(wire)
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            tick(fault)
            time.sleep(0.025)
        return wire[start:]

    try:
        commands = [
            ("research_runtime", "research_safety_bridge", ["--params-file", str(safety_config),
                "-p", "mode:=live", "-p", "actuator_enabled:=true", "-p", "max_curvature_1pm:=1.0",
                "-p", "max_lateral_speed_mps:=0.05"]),
            ("gps_waypoint_dispatcher", "corridor_cmd_vel_guard_node", ["--params-file", str(master_config)]),
            ("serial_twistctl", "serial_twistctl_node", ["--params-file", str(master_config),
                "-p", "port:=" + slave_path, "-r", "/cmd_vel:=/cmd_vel_guarded"]),
        ]
        commands.append(("ego_planner", "research_tf_guard", []))
        if args.rtk_classifier:
            commands.append(("gps_waypoint_dispatcher","rtk_map_odom_corrector_node",["--params-file",str(master_config),"-p","lio_odom_topic:=/lio/odom_vehicle"]))
        if args.ego_loop:
            commands.append(("ego_planner", "motion_plan", ["--params-file",
                str(args.repo / "src/bringup/config/ego_vehicle_adapter.yaml"),
                "-p", "max_curvature_1pm:=1.0", "-p", "max_lateral_speed_mps:=0.05",
                "-p", "max_jerk_mps3:=3.0", "-p", "inflate_radius_m:=" + str(math.hypot(0.33, 0.305))]))
        for package, binary, ros_args in commands:
            log = (args.output.parent / (binary + "-mock.log")).open("w")
            logs.append(log)
            children.append(subprocess.Popen([str(args.install / package / "lib" / package / binary),
                                              "--ros-args"] + ros_args, stdout=log,
                                             stderr=subprocess.STDOUT, start_new_session=True))
        phase("authority_false", 1.5)
        runtime_parameters = {}
        for name in ("research_safety_bridge", "corridor_cmd_vel_guard", "serial_twistctl_node"):
            dumped = subprocess.run(["ros2", "param", "dump", "/" + name], text=True,
                                    capture_output=True, timeout=10)
            runtime_parameters[name] = {"exit": dumped.returncode, "yaml": dumped.stdout}
        if args.ego_loop:
            normal = phase("normal", args.loop_budget_s)
            stop_lines = phase("authority_false", 0.8)
            tail = stop_lines[-5:]
            target=(3*math.sin(math.pi/3),1.5) if args.arc_loop else (3.3,0.)
            reached = math.dist(simulated_pose[:2], target) < 0.25
            stopped = len(tail) == 5 and all(line == "vcx=0.000,wc=0.000\n" for line in tail)
            result = {"domain": 91, "sink": "original serial binary -> allocated PTY -> synthetic unicycle physics",
                      "source": "actual four-patch pinned EGO binary and installed research tracker",
                      "simulation_only_settings": {"max_curvature_1pm": 1.0, "max_lateral_speed_mps": 0.05, "max_jerk_mps3": 3.0},
                      "scope": "planar mock closed loop with confirmed free test floor; no slip/real vehicle/Super-LIO policy comparison claim",
                      "final_simulated_pose": simulated_pose, "reached_goal": reached,"target":target,"arc_loop":args.arc_loop,
                      "final_wire_max_speed":max([abs(float(line.split(",")[0][4:])) for line in normal] or [0]),
                      "final_wire_max_yaw_rate":max([abs(float(line.split(",")[1][3:])) for line in normal] or [0]),
                      "authority_false_final_serial_tail": tail, "runtime_parameters": runtime_parameters,
                      "trajectory_counts": trajectory_counts, "wire_message_count": len(normal),
                      "loop_budget_s": args.loop_budget_s,
                      "tracker_reason_counts": tracker_reasons,
                      "status": "PASS" if reached and stopped and trajectory_counts["ok"] > 10 else "FAIL"}
            args.output.write_text(json.dumps(result, indent=2) + "\n")
            print(json.dumps({k:v for k,v in result.items() if k != "runtime_parameters"}, indent=2))
            return 0 if result["status"] == "PASS" else 1
        faults=("authority_false", "authority_stale", "rtk_authority_loss", "lio_bridge_motion_denied", "authority_mode_stale", "lio_health_unknown",
                      "planning_failed", "planner_stream_lost", "expired_trajectory", "infeasible_speed",
                      "map_version_changed", "grid_stream_lost", "odom_stream_lost", "unknown_ground",
                      "footprint_interior_obstacle", "stop_override", "stop_heartbeat_lost", "speed_permission_zero",
                      "wrong_odom_frame", "old_odom_stamp", "zero_quaternion", "pose_jump",
                      "tf_stream_lost", "tf_double_publisher", "permission_stream_lost", "keepout")
        if args.tf_static_fault:faults=("tf_static_competitor",)
        if args.rtk_classifier:faults=("gnss_non_fixed","gnss_low_satellites","gnss_bad_hdop","gnss_heading_float","gnss_rtcm_stale")
        for fault in faults:
            normal = phase("normal", 8.0 if args.rtk_classifier else .7)
            nominal_ok = any(line.startswith("vcx=0.2") for line in normal)
            fault_lines = phase(fault, 0.85)
            tail = fault_lines[-5:]
            zero_ok = len(tail) == 5 and all(line == "vcx=0.000,wc=0.000\n" for line in tail)
            cases.append({"fault": fault, "nominal_nonzero_reached_serial": nominal_ok,
                          "final_serial_tail": tail, "status": "PASS" if nominal_ok and zero_ok else "FAIL"})
            if any(child.poll() is not None for child in children):
                break
        if not args.tf_static_fault and not args.rtk_classifier:
            normal = phase("normal", 0.7)
            os.killpg(children[0].pid, signal.SIGINT)
            children[0].wait(timeout=5)
            fault_lines = phase("normal", 0.8)
            tail = fault_lines[-5:]
            cases.append({"fault": "controller_process_exit", "nominal_nonzero_reached_serial": any(
                line.startswith("vcx=0.2") for line in normal), "final_serial_tail": tail,
                "status": "PASS" if any(line.startswith("vcx=0.2") for line in normal) and
                len(tail) == 5 and all(line == "vcx=0.000,wc=0.000\n" for line in tail) else "FAIL"})
        result = {"domain": 91, "sink": "allocated PTY; no physical device", "serial_profile": "original master YAML, only port remapped",
                  "simulation_only_settings": {"max_curvature_1pm": 1.0, "max_lateral_speed_mps": 0.05},
                  "scope": "actual original serial binary; physical KEY/joystick/firmware emergency stop remains pending",
                  "rtk_classifier":args.rtk_classifier,"authority_mode_counts":{mode:authority_modes.count(mode) for mode in set(authority_modes)},
                  "runtime_parameters": runtime_parameters, "cases": cases, "tracker_reason_counts": tracker_reasons,
                  "child_exit_before_cleanup": [child.poll() for child in children],
                  "status": "PASS" if all(case["status"] == "PASS" for case in cases) else "FAIL"}
    finally:
        for child in children:
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGTERM)
                child.wait(timeout=5)
        for log in logs:
            log.close()
        os.close(master_fd)
        os.close(slave_fd)
        node.destroy_node()
        competing_tf_node.destroy_node()
        rclpy.shutdown()
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({"status": result["status"], "cases": cases}, indent=2))
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
