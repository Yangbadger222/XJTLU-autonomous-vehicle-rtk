#!/usr/bin/env python3
"""Start the default entry with no drivers/actuator, inspect, stop owned group."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import re
import yaml

import rclpy


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--override-probes",action="store_true")
    parser.add_argument("--repo",type=Path)
    parser.add_argument("--serial-evidence",type=Path)
    parser.add_argument("--console-port",type=int,default=8876,help="task-only HTTP port; preserves an existing user preview")
    args = parser.parse_args()
    if args.override_probes and (args.repo is None or args.serial_evidence is None):
        parser.error('--override-probes requires --repo and --serial-evidence before starting any launch')
    if os.environ.get("ROS_DOMAIN_ID") != "93" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise SystemExit("requires isolated domain 93")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    log = args.output.with_suffix(".log").open("w")
    process = subprocess.Popen(["ros2", "launch", "bringup", "system_active_road_research.launch.py",
                                "console_port:="+str(args.console_port)],
                               stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    time.sleep(4)
    evidence = {}
    try:
        for label, command in {
            "nodes": ["ros2", "node", "list", "--no-daemon"],
            "topics": ["ros2", "topic", "list", "--no-daemon", "-t"],
            "safety_parameters": ["ros2", "param", "dump", "/research_safety_bridge"],
            "console_parameters": ["ros2", "param", "dump", "/research_operator_console"],
            "ego_parameters": ["ros2", "param", "dump", "/ego_vehicle_adapter"],
            "authority_parameters": ["ros2", "param", "dump", "/rtk_map_odom_corrector"],
            "guard_parameters": ["ros2", "param", "dump", "/corridor_cmd_vel_guard"],
            "adapter_parameters": ["ros2", "param", "dump", "/super_lio_vehicle_adapter"],
            "tf_guard_parameters": ["ros2", "param", "dump", "/research_tf_integrity_guard"],
            "cloud_frame_parameters": ["ros2", "param", "dump", "/super_lio_cloud_frame_adapter"],
            "ground_parameters": ["ros2","param","dump","/research_observed_ground"],
            "local_grid_parameters": ["ros2","param","dump","/research_local_obstacle_grid"],
            "superlio_parameters": ["ros2", "param", "dump", "/super_lio_node"],
            "robot_description_parameters": ["ros2","param","dump","/robot_state_publisher"],
        }.items():
            for attempt in range(3):
                try:
                    output = subprocess.run(command,capture_output=True,text=True,timeout=20)
                    if output.returncode==0:break
                except subprocess.TimeoutExpired:
                    if attempt==2:raise
            evidence[label] = {"exit": output.returncode, "output": output.stdout, "stderr": output.stderr}
        override_results=[]
        if args.override_probes:
            from audit_research_runtime_parameters import audit,parameters
            serial=json.loads(args.serial_evidence.read_text())
            baseline=audit(args.repo,{"evidence":evidence},serial)
            override_results.append({"case":"normal_three_layer","status":baseline['status'],"check_count":len(baseline['checks'])})
            def mutate(case,node,key,value,label):
                applied=subprocess.run(['ros2','param','set','/'+node,key,value],capture_output=True,text=True,timeout=10)
                dumped=subprocess.run(['ros2','param','dump','/'+node],capture_output=True,text=True,timeout=10)
                changed=dict(evidence);changed[label]={"exit":dumped.returncode,"output":dumped.stdout,"stderr":dumped.stderr}
                checked=audit(args.repo,{"evidence":changed},serial)
                failed=[check['name'] for check in checked['checks'] if check['status']=='FAIL']
                override_results.append({"case":case,"parameter_set_exit":applied.returncode,"audit_status":checked['status'],
                    "rejected_checks":failed,"status":"PASS" if applied.returncode==0 and failed else "FAIL"})
            mutate('runtime_lidar_imu_extrinsic','super_lio_node','lio.extrinsic.lidar_imu',
                '[0.123,-0.02329,0.04412,1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0]','superlio_parameters')
            xml=parameters(evidence['robot_description_parameters']['output'])['robot_description']
            import xml.etree.ElementTree as ET
            tree=ET.fromstring(xml)
            joint=tree.find("joint[@name='left_front_wheel_joint']")
            if joint is None:
                joint=next(j for j in tree.findall('joint') if 'left_front_wheel' in j.attrib['name'])
            joint.find('origin').set('xyz','0.24 0.35 -0.13')
            mutate('runtime_model_wheel_offset','robot_state_publisher','robot_description',ET.tostring(tree,encoding='unicode'),'robot_description_parameters')
            mutate('runtime_guard_speed','corridor_cmd_vel_guard','straight_max_mps','1.2','guard_parameters')
            install=Path(__import__('ament_index_python.packages',fromlist=['get_package_prefix']).get_package_prefix('research_runtime')).parent
            config=args.repo/'src/bringup/config/research_safety_bridge.yaml'
            for case,extra in [('CLI_speed',['--params-file',str(config),'-p','max_speed_mps:=2.0']),
                               ('CLI_footprint',['--params-file',str(config),'-p','footprint_xy:=[0.1,0.1,-0.1,0.1,-0.1,-0.1,0.1,-0.1]'])]:
                child=subprocess.run([str(install/'research_runtime/lib/research_runtime/research_safety_bridge'),'--ros-args']+extra,
                    capture_output=True,text=True,timeout=10)
                override_results.append({"case":case,"exit":child.returncode,"status":"PASS" if child.returncode!=0 else "FAIL"})
            changed=yaml.safe_load(config.read_text());changed['research_safety_bridge']['ros__parameters']['max_yaw_rate_rps']=2.0
            temporary=args.output.parent/'deliberate-invalid-physical-limit.yaml';temporary.write_text(yaml.safe_dump(changed))
            child=subprocess.run([str(install/'research_runtime/lib/research_runtime/research_safety_bridge'),'--ros-args','--params-file',str(temporary)],
                capture_output=True,text=True,timeout=10)
            override_results.append({"case":"YAML_yaw_limit","exit":child.returncode,"status":"PASS" if child.returncode!=0 else "FAIL"})
        # Discovery can lag a started rclpy process. Keep the initial snapshot
        # and check the inventory again after parameter responses prove startup.
        expected = {"/research_tf_integrity_guard", "/super_lio_node", "/super_lio_vehicle_adapter", "/super_lio_cloud_frame_adapter",
                    "/active_road_map", "/active_road_evidence", "/active_observation", "/research_local_obstacle_grid", "/research_observed_ground",
                    "/ego_vehicle_adapter", "/rtk_map_odom_corrector", "/corridor_cmd_vel_guard",
                    "/research_safety_bridge", "/research_operator_console", "/robot_state_publisher", "/joint_state_publisher"}
        rclpy.init()
        probe = rclpy.create_node("entry_acceptance_graph_probe")
        deadline = time.monotonic()+15
        names = []
        while time.monotonic() < deadline:
            rclpy.spin_once(probe, timeout_sec=.1)
            names = [namespace.rstrip("/")+"/"+name for name,namespace in probe.get_node_names_and_namespaces()
                     if name != "entry_acceptance_graph_probe"]
            if expected <= set(names):
                break
        evidence["persistent_graph_probe"] = {"exit": 0, "nodes": names}
        owners = probe.get_publishers_info_by_topic("/cmd_vel")
        evidence["cmd_vel_publishers"] = [{"node": owner.node_name, "namespace": owner.node_namespace,
                                            "type": owner.topic_type} for owner in owners]
        probe.destroy_node()
        rclpy.shutdown()
        log.flush()
        log_text = args.output.with_suffix(".log").read_text()
        owned_pids = [int(pid) for pid in re.findall(r"process started with pid \[(\d+)\]", log_text)]
        evidence["owned_child_processes"] = [{"pid": pid, "alive": Path(f"/proc/{pid}").exists()}
                                              for pid in owned_pids]
        forbidden = ("fastlio", "nav2", "mppi", "slam", "fake_sim", "frc", "fgo", "serial_twistctl", "livox_ros_driver", "um982")
        result = {"domain": 93, "mode": "default replay", "test_only_overrides":{"console_port":args.console_port},
                  "evidence": evidence,"deliberate_override_probes":override_results,
                  "missing_expected_nodes": sorted(expected - set(names)),
                  "forbidden_nodes": [n for n in names if any(word in n.lower() for word in forbidden)],
                  "launch_exit_before_cleanup": process.poll()}
        result["expected_child_count"] = len(expected)
        result["status"] = "PASS" if (not result["missing_expected_nodes"] and not result["forbidden_nodes"] and
                process.poll() is None and all(c["status"]=="PASS" for c in override_results) and all(v["exit"] == 0 for v in evidence.values() if isinstance(v,dict) and "exit" in v)
                and len(owned_pids) == len(expected) and all(p["alive"] for p in evidence["owned_child_processes"])
                and len(owners) == 1 and owners[0].node_name == "research_safety_bridge") else "FAIL"
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5)
        log.close()
    result["cleanup_exit"] = process.returncode
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({k:v for k,v in result.items() if k != "evidence"}, indent=2))
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
