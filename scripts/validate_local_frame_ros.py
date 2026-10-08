#!/usr/bin/env python3
"""Actual native TF guard: perception continues, global motion gate stays false."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage


def main():
    parser=argparse.ArgumentParser()
    for name in ('repo','install','output'):parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='107' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise SystemExit('requires domain107 localhost-only')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    rclpy.init();node=rclpy.create_node('native_local_frame_probe');second=rclpy.create_node('second_local_owner')
    dynamic=node.create_publisher(TFMessage,'/tf',10);competing=second.create_publisher(TFMessage,'/tf',10)
    qos=QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    static=node.create_publisher(TFMessage,'/tf_static',qos);wrong=second.create_publisher(TFMessage,'/tf_static',qos)
    local=[];full=[];cases=[]
    node.create_subscription(Bool,'/research/local_frame_integrity',lambda m:local.append(m.data),100)
    node.create_subscription(Bool,'/research/tf_integrity',lambda m:full.append(m.data),100)
    log=args.output.with_suffix('.log').open('w')
    child=subprocess.Popen([str(args.install/'ego_planner/lib/ego_planner/research_tf_guard'),
        '--ros-args','-p','protect_world_gauge:=true'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    def transform(parent,frame):
        t=TransformStamped();t.header.stamp=node.get_clock().now().to_msg()
        t.header.frame_id,t.child_frame_id=parent,frame;t.transform.rotation.w=1.;return t
    def phase(name,want_local,want_full,*,base=True,map_edge=False,duplicate=False):
        l0,f0=len(local),len(full);end=time.monotonic()+1.2
        while time.monotonic()<end:
            edges=[]
            if base:edges.append(transform('odom','base_footprint'))
            if map_edge:edges.append(transform('map','odom'))
            if edges:dynamic.publish(TFMessage(transforms=edges))
            if duplicate:competing.publish(TFMessage(transforms=[transform('odom','base_footprint')]))
            until=time.monotonic()+.025
            while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.005)
        lt,ft=local[l0:][-5:],full[f0:][-5:]
        cases.append(dict(case=name,ok=len(lt)==5 and len(ft)==5 and
            all(v==want_local for v in lt) and all(v==want_full for v in ft),local_tail=lt,full_tail=ft))
    try:
        static.publish(TFMessage(transforms=[transform('odom','world')]))
        phase('local_perception_without_global_TF',True,False)
        phase('both_chains_valid',True,True,map_edge=True)
        phase('global_TF_expiry_preserves_only_local_perception',True,False)
        phase('local_pose_expiry_denies_both',False,False,base=False)
        phase('duplicate_local_owner_denies_both',False,False,map_edge=True,duplicate=True)
        wrong.publish(TFMessage(transforms=[transform('odom','world')]))
        phase('second_world_owner_latches_both',False,False,map_edge=True)
        phase('world_conflict_does_not_self_recover',False,False,map_edge=True)
        result=dict(status='PASS' if all(c['ok'] for c in cases) and child.poll() is None else 'FAIL',
            cases=cases,domain=107,child_exit_before_cleanup=child.poll(),
            source_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=args.repo,text=True).strip(),
            scope='Actual compiled C++ guard; local Bool never grants actuator/RTK authority')
    finally:
        if child.poll() is None:os.killpg(child.pid,signal.SIGINT)
        try:child.wait(timeout=5)
        except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=5)
        log.close();node.destroy_node();second.destroy_node();rclpy.shutdown()
    args.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
    return 0 if result['status']=='PASS' else 1


if __name__=='__main__':raise SystemExit(main())
