#!/usr/bin/env python3
"""Compare source lock, parsed research YAML and actual isolated ROS dumps.

Run with the default-entry/PTY evidence. Mutated launch/runtime fields cause
FAIL even when all original source hashes still match. This does not replace
the absent Jetson deployment dump.
"""
import argparse
import hashlib
import json
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


def parameters(text):
    payload = yaml.safe_load(text)
    if not isinstance(payload, dict) or len(payload) != 1:
        raise ValueError("one actual ROS node parameter dump required")
    return next(iter(payload.values()))["ros__parameters"]


def audit(root, default_entry, serial):
    lock = json.loads((root/"audit/vehicle_baseline/VEHICLE_PARAMETER_LOCK.json").read_text())
    locked = {item["name"]: item["value"] for item in lock["parameters"]}
    maximum, minimum = locked["corridor.smoother.max_velocity"], locked["corridor.smoother.min_velocity"]
    acceleration, deceleration = locked["corridor.smoother.max_accel"], locked["corridor.smoother.max_decel"]
    expected = {"max_speed_mps": maximum[0], "min_speed_mps": minimum[0],
        "max_yaw_rate_rps": maximum[2], "max_accel_mps2": acceleration[0],
        "max_decel_mps2": -deceleration[0], "max_yaw_accel_rps2": acceleration[2],
        "max_yaw_decel_rps2": -deceleration[2]}
    checks = []
    def checked(name, expected_value, parsed_value, runtime_value):
        checks.append({"name": name, "locked": expected_value, "parsed": parsed_value,
                       "runtime": runtime_value,
                       "status": "PASS" if expected_value == parsed_value == runtime_value else "FAIL"})
    evidence = default_entry["evidence"]
    for node, label in (("research_safety_bridge", "safety_parameters"), ("ego_vehicle_adapter", "ego_parameters")):
        runtime = parameters(evidence[label]["output"])
        parsed = yaml.safe_load((root/f"src/bringup/config/{node}.yaml").read_text())[node]["ros__parameters"]
        for key, value in expected.items():
            checked(f"{node}.{key}", value, parsed.get(key), runtime.get(key))
    expected_footprint=[value for point in locked["vehicle.corridor_footprint_xy"] for value in point]
    parsed_safety=yaml.safe_load((root/"src/bringup/config/research_safety_bridge.yaml").read_text())["research_safety_bridge"]["ros__parameters"]
    checked("corridor.footprint_xy",expected_footprint,parsed_safety["footprint_xy"],
            parameters(evidence["safety_parameters"]["output"])["footprint_xy"])
    super_params=parameters(evidence["superlio_parameters"]["output"])
    super_yaml=yaml.safe_load((root/"src/bringup/config/super_lio_vehicle.yaml").read_text())["/**"]["ros__parameters"]
    def flattened(fields,prefix=""):
        result={}
        for key,value in fields.items():
            path=prefix+key
            if isinstance(value,dict):result.update(flattened(value,path+"."))
            else:result[path]=value
        return result
    super_params=flattened(super_params)
    for source_key,target_key in (("lio.imu_topic","lio.ros.imu_topic"),("lio.lidar_topic","lio.ros.lidar_topic"),
            ("lio.lidar_min_range","lio.sensor.blind"),("lio.lidar_max_range","lio.sensor.maxrange"),
            ("lio.lidar_filter_num","lio.sensor.filter_rate")):
        checked(target_key,locked[source_key],super_yaml.get(target_key),super_params.get(target_key))
    expected_extrinsic=locked["lio.t_il"]+locked["lio.r_il"]
    checked("lio.extrinsic.lidar_imu",expected_extrinsic,super_yaml["lio.extrinsic.lidar_imu"],super_params.get("lio.extrinsic.lidar_imu"))
    actual_serial = parameters(serial["runtime_parameters"]["serial_twistctl_node"]["yaml"])
    master = yaml.safe_load((root/"src/bringup/config/master_params.yaml").read_text())
    parsed_serial = master["/serial_twistctl_node"]["ros__parameters"]
    for key in ("baudrate", "angular_z_scale"):
        checked(f"serial.{key}", locked[f"serial.{key}"], parsed_serial[key], actual_serial[key])
    for node,label in (("rtk_map_odom_corrector","authority_parameters"),("corridor_cmd_vel_guard","guard_parameters")):
        actual=parameters(evidence[label]["output"])
        for key,value in master["/"+node]["ros__parameters"].items():
            effective="/lio/odom_vehicle" if node=="rtk_map_odom_corrector" and key=="lio_odom_topic" else value
            checked(node+"."+key,effective,effective,actual.get(key))
            if key=="lio_odom_topic":checks[-1]["conversion"]="topic-only Super-LIO compatibility producer; original master unchanged"
    parsed_cloud=yaml.safe_load((root/"src/bringup/config/super_lio_cloud_frame.yaml").read_text())["super_lio_cloud_frame_adapter"]["ros__parameters"]
    cloud_runtime=parameters(evidence["cloud_frame_parameters"]["output"])
    for key,value in zip(("obstacle_min_z_m","obstacle_max_z_m"),locked["cloud.nav2_obstacle_height_window"]):
        checked("obstacle_height."+key,value,parsed_cloud[key],cloud_runtime[key])
    # Compare the actual robot_description geometry with expansion of the
    # protected original Xacro, ignoring only nonsemantic comments/formatting.
    source=root/"src/bringup/urdf/rosbot/rosbot.urdf.xacro"
    expanded=subprocess.run(["xacro",str(source)],capture_output=True,text=True,check=True).stdout
    def canonical(text):
        tree=ET.fromstring(text)
        return [(element.tag,sorted(element.attrib.items()),(element.text or '').strip()) for element in tree.iter()]
    runtime_xml=parameters(evidence["robot_description_parameters"]["output"])["robot_description"]
    checks.append({"name":"robot_description.protected_source_geometry","source":str(source.relative_to(root)),
        "status":"PASS" if canonical(expanded)==canonical(runtime_xml) else "FAIL",
        "scope":"original model geometry equality, not a new physical six-wheel calibration"})
    # Port is the deliberate PTY isolation override, not a changed physical alias.
    checks.append({"name": "serial.physical_alias", "locked": locked["serial.port"],
                   "parsed": parsed_serial["port"], "runtime": "NOT_RUN: physical device excluded",
                   "status": "PASS" if locked["serial.port"] == parsed_serial["port"] else "FAIL",
                   "scope": "source/parsed alias only; runtime port is deliberately a PTY"})
    checks.append({"name": "serial.mock_port_is_pty", "runtime": actual_serial["port"],
                   "status": "PASS" if actual_serial["port"].startswith("/dev/pts/") else "FAIL"})
    for filename in ("PROTECTED_FILES.sha256", "PROTECTED_ADDITIONAL_FILES.sha256"):
        for line in (root/"audit/vehicle_baseline"/filename).read_text().splitlines():
            digest, path = line.split("  ", 1)
            actual = hashlib.sha256((root/path).read_bytes()).hexdigest()
            if digest != actual:
                checks.append({"name": "source_bytes:"+path, "status": "FAIL"})
    return {"status": "PASS" if all(c["status"] == "PASS" for c in checks) else "FAIL",
            "checks": checks, "scope": "source baseline/parsed research YAML/actual non-Jetson runtime",
            "pending": ["Jetson current deployment overlays, firmware runtime and measured hardware calibration"]}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--default-entry", type=Path, required=True)
    parser.add_argument("--serial", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = audit(args.root, json.loads(args.default_entry.read_text()), json.loads(args.serial.read_text()))
    args.output.write_text(json.dumps(result, indent=2)+"\n")
    print(json.dumps({"status": result["status"], "checks": len(result["checks"])}))
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
