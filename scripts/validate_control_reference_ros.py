#!/usr/bin/env python3
"""Actual installed adapter with analytical recorded-mount motion; no actuator."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from super_lio_vehicle_adapter.control_reference_lock import (
    CONTROL_ODOM_TOPIC, CONTROL_CHILD_FRAME, IMU_TO_CONTROL_TRANSLATION_M)


def main():
    p=argparse.ArgumentParser()
    for name in ('repo','install','output'):p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='108' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise SystemExit('requires isolated domain108/localhost-only')
    a.output.parent.mkdir(parents=True,exist_ok=True)
    rclpy.init();node=rclpy.create_node('recorded_control_reference_probe')
    source=node.create_publisher(Odometry,'/lio/odom',100)
    health=node.create_publisher(String,'/lio/health',100)
    received={};legacy={};sent={};states=[]
    stamp=lambda m:m.header.stamp.sec*10**9+m.header.stamp.nanosec
    node.create_subscription(Odometry,CONTROL_ODOM_TOPIC,lambda m:received.__setitem__(stamp(m),m),100)
    node.create_subscription(Odometry,'/lio/odom_vehicle',lambda m:legacy.__setitem__(stamp(m),m),100)
    node.create_subscription(String,'/lio/vehicle_health',lambda m:states.append(m.data),100)
    log=a.output.with_suffix('.node.log').open('w')
    child=subprocess.Popen([str(a.install/'super_lio_vehicle_adapter/lib/super_lio_vehicle_adapter/super_lio_vehicle_adapter'),
        '--ros-args','--params-file',str(a.repo/'src/bringup/config/super_lio_reference.yaml'),
        '-p','publish_unhealthy_odometry:=false'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    checks={};r=np.asarray(IMU_TO_CONTROL_TRANSLATION_M)
    # Non-diagonal PSD covariance exercises all cross blocks independently.
    lower=np.diag([.01,.02,.03,.04,.05,.06]);lower[0,5]=.003;lower[2,3]=-.004
    cov=lower@lower.T
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
    def publish(yaw,valid=True):
        m=Odometry();m.header.frame_id='world';m.child_frame_id='imu';m.header.stamp=node.get_clock().now().to_msg()
        c,s=math.cos(yaw),math.sin(yaw);rotation=np.array([[c,-s,0.],[s,c,0.],[0.,0.,1.]])
        position=np.array([2.,-1.,0.])-rotation@r
        m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z=map(float,position)
        m.pose.pose.orientation.z,m.pose.pose.orientation.w=math.sin(yaw/2),math.cos(yaw/2)
        v=-np.cross([0.,0.,.35],r)
        m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.linear.z=map(float,v)
        m.twist.twist.angular.z=.35;m.pose.covariance=list(cov.ravel());m.twist.covariance=list(cov.ravel())
        sent[stamp(m)]=(m,rotation)
        certificate=String(data=json.dumps(dict(source='super_lio/f89f48dc',
            certificate='fixed_extrinsic_observation_lower_bound_v1',twist_convention='imu_body_full_state_v1',
            stamp_ns=stamp(m),status='OK' if valid else 'FAIL',minimum_observation_information=100. if valid else 0.,
            legacy_min_eig_lower_bound=100. if valid else 0.,effective_points=100,iterations=2,
            reason='ANALYTICAL_RECORDED_MOUNT_TEST_ONLY')))
        # Exercise both DDS orders; production adapter must match acquisition.
        if len(sent)%2:source.publish(m);health.publish(certificate)
        else:health.publish(certificate);source.publish(m)
    try:
        spin(1.5)
        started=time.monotonic()
        while time.monotonic()-started<5.:
            publish(.35*(time.monotonic()-started));spin(.05)
        spin(.25)
        matched=set(received)&set(legacy)&set(sent)
        checks['actual_adapter_alive']=child.poll() is None
        checks['sustained_same_acquisition_two_references']=len(matched)>50 and set(received)==set(legacy)
        failures=[]
        skew=lambda v:np.array([[0.,-v[2],v[1]],[v[2],0.,-v[0]],[-v[1],v[0],0.]])
        for key in matched:
            m,rotation=sent[key];b=received[key];old=legacy[key]
            jp=np.eye(6);jp[:3,3:]=-skew(rotation@r)
            jt=np.eye(6);jt[:3,3:]=-skew(r)
            values=[b.pose.pose.position.x,b.pose.pose.position.y,b.pose.pose.position.z,
                b.twist.twist.linear.x,b.twist.twist.linear.y,b.twist.twist.linear.z,b.twist.twist.angular.z]
            if (not np.allclose(values,[2.,-1.,0.,0.,0.,0.,.35],rtol=0.,atol=1e-10) or
                not np.allclose(b.pose.covariance,(jp@cov@jp.T).ravel(),rtol=0.,atol=1e-10) or
                not np.allclose(b.twist.covariance,(jt@cov@jt.T).ravel(),rtol=0.,atol=1e-10) or
                b.header!=old.header or b.child_frame_id!=CONTROL_CHILD_FRAME or
                old.child_frame_id!='base_footprint' or old.twist!=m.twist or
                not np.allclose([old.pose.pose.position.x,old.pose.pose.position.y,old.pose.pose.position.z],
                    [m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z],rtol=0.,atol=1e-10) or
                not np.allclose([old.pose.pose.orientation.z,old.pose.pose.orientation.w],
                    [m.pose.pose.orientation.z,m.pose.pose.orientation.w],rtol=0.,atol=1e-10)):
                failures.append(key)
        checks['fixed_control_pivot_zero_speed_original_imu_unchanged_full_covariance']=bool(matched) and not failures
        count=len(received);old_count=len(legacy)
        for _ in range(16):publish(1.75,False);spin(.05)
        checks['ineligible_source_withholds_both_references']=len(received)==count and len(legacy)==old_count and states[-1].startswith('UNKNOWN')
        result=dict(status='PASS' if all(checks.values()) else 'FAIL',checks=checks,pairs=len(matched),mismatches=failures,
            imu_translation_speed_mps=float(np.linalg.norm(np.cross([0.,0.,.35],r))),
            scope='actual ROS adapter; analytical recorded mounting/configured attitude and covariance; no physical acceptance',
            source_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=a.repo,text=True).strip())
    finally:
        if child.poll() is None:os.killpg(child.pid,signal.SIGINT)
        child.wait(timeout=5);log.close();node.destroy_node();rclpy.shutdown()
    a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
    return 0 if result['status']=='PASS' else 1


if __name__=='__main__':raise SystemExit(main())
