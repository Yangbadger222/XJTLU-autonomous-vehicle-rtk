#!/usr/bin/env python3
"""Compare shallow/deep audit receivers on one actual raw bag prefix.

This diagnoses monitor coverage, not estimator packet loss or accuracy.
The player publishes only raw sensor topics/clock in isolated domain 92.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu
from livox_ros_driver2.msg import CustomMsg
import yaml


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--bag",type=Path,required=True)
    parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args()
    if os.environ.get("ROS_DOMAIN_ID")!="92" or os.environ.get("ROS_LOCALHOST_ONLY")!="1":raise SystemExit("requires isolated domain 92")
    rclpy.init();node=rclpy.create_node("raw_transport_audit_probe")
    streams={"depth5":[],"depth1024":[]}
    stamp_ns=lambda msg:msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
    for name,depth in (("depth5",5),("depth1024",1024)):
        node.create_subscription(Imu,"/livox/imu",lambda msg,name=name:streams[name].append(stamp_ns(msg)),
            QoSProfile(depth=depth,reliability=ReliabilityPolicy.BEST_EFFORT))
    node.create_subscription(CustomMsg,"/livox/lidar",lambda msg:None,qos_profile_sensor_data)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    log=args.output.with_suffix('.play.log').open('w')
    player=None
    try:
        until=time.monotonic()+1.
        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.02)
        player=subprocess.Popen(["ros2","bag","play",str(args.bag),"--rate","1.0","--clock","50","--topics","/livox/lidar","/livox/imu"],
            stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        until=time.monotonic()+20.
        while time.monotonic()<until and player.poll() is None:rclpy.spin_once(node,timeout_sec=.01)
    finally:
        if player and player.poll() is None:
            os.killpg(player.pid,signal.SIGINT)
            try:player.wait(timeout=3)
            except subprocess.TimeoutExpired:os.killpg(player.pid,signal.SIGTERM);player.wait(timeout=3)
        until=time.monotonic()+2.
        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.01)
        log.close();node.destroy_node();rclpy.shutdown()
    if not streams['depth1024']:raise RuntimeError('no real IMU prefix received')
    first,last=min(streams['depth1024']),max(streams['depth1024'])
    info=yaml.safe_load((args.bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    expected=0
    for name in info['relative_file_paths']:
        path=args.bag/name
        with sqlite3.connect(path.resolve().as_uri()+'?mode=ro',uri=True) as db:
            for (blob,) in db.execute("SELECT m.data FROM messages m JOIN topics t ON m.topic_id=t.id WHERE t.name='/livox/imu'"):
                stamp=stamp_ns(deserialize_message(blob,Imu))
                expected+=int(first<=stamp<=last)
    counts={name:sum(first<=stamp<=last for stamp in values) for name,values in streams.items()}
    result={'scope':'20 s actual raw bag prefix; simultaneous audit receivers with real Livox deserialization workload; no estimator or motion',
        'bag':str(args.bag),'domain':92,'prefix_header_interval_ns':[first,last],'expected_imu_messages_in_interval':expected,
        'received_in_interval':counts,'status':'PASS' if counts['depth1024']==expected else 'FAIL',
        'interpretation':'Audit receiver QoS coverage only. Original paired runs are retained; estimator IMU consumption is not inferred from Python audit counts.'}
    args.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
    return 0 if result['status']=='PASS' else 1


if __name__=='__main__':raise SystemExit(main())
