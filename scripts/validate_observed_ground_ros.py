#!/usr/bin/env python3
"""Actual installed support/local-grid nodes, analytical ROS fixtures, no sink."""
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
from nav_msgs.msg import OccupancyGrid,Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header,Bool,String
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock
from research_interfaces.msg import LocalEvidenceGrid2D


def main():
    parser=argparse.ArgumentParser()
    for key in ('repo','install','output'):
        parser.add_argument('--'+key,type=Path,required=True)
    args=parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='106' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise SystemExit('requires isolated domain106 and localhost-only')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    rclpy.init();node=rclpy.create_node('observed_ground_fixture')
    pubs={key:node.create_publisher(kind,topic,100) for key,kind,topic in (
        ('cloud',PointCloud2,'/lio/cloud_world'),('odom',Odometry,'/lio/odom_vehicle'),
        ('health',String,'/lio/health'),('version',String,'/research/map_version'),
        ('integrity',Bool,'/research/tf_integrity'),('old_cloud',PointCloud2,'/lio/cloud_odom'),
        ('clock',Clock,'/clock'))}
    static=node.create_publisher(TFMessage,'/tf_static',QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    ground_injection=node.create_publisher(LocalEvidenceGrid2D,'/research/observed_ground_grid',10)
    ground=[];combined=[]
    node.create_subscription(LocalEvidenceGrid2D,'/research/observed_ground_grid',lambda msg:ground.append(msg.grid),100)
    node.create_subscription(OccupancyGrid,'/research/local_obstacle_grid',combined.append,100)
    children=[];logs=[];cases=[];measurement_ns=10_000_000_000
    def spawn(binary,config):
        stream=args.output.with_name(args.output.stem+'-'+binary+'.log').open('w');logs.append(stream)
        child=subprocess.Popen([str(args.install/'research_runtime/lib/research_runtime'/binary),
            '--ros-args','--params-file',str(args.repo/'src/bringup/config'/config),'-p','use_sim_time:=true',
            '-p','localization_session_id:=analytical-fixture'],
            stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        children.append(child)
    def publish(points,kind='normal',advance=True):
        nonlocal measurement_ns
        if advance:measurement_ns+=30_000_000
        clock=Clock();clock.clock.sec,clock.clock.nanosec=divmod(measurement_ns,1_000_000_000)
        pubs['clock'].publish(clock)
        stamp=clock.clock
        version='fixture-v2' if kind=='version' else 'fixture-v1'
        pubs['version'].publish(String(data=version))
        pubs['integrity'].publish(Bool(data=kind!='tf_denied'))
        pose=Odometry();pose.header=Header(stamp=stamp,frame_id='odom');pose.child_frame_id='base_footprint'
        pose.pose.pose.orientation.w=1.;pubs['odom'].publish(pose)
        bound=0. if kind=='health_denied' else 1000.
        cert=dict(source='super_lio/f89f48dc',certificate='fixed_extrinsic_observation_lower_bound_v1',
            twist_convention='imu_body_full_state_v1',stamp_ns=measurement_ns+(1 if kind=='unmatched' else 0),
            status='FAIL' if not bound else 'OK',minimum_observation_information=bound,
            legacy_min_eig_lower_bound=bound,effective_points=100,iterations=1,reason='analytical-fixture')
        pubs['health'].publish(String(data=json.dumps(cert)))
        header=Header(stamp=stamp,frame_id='wrong' if kind=='frame' else 'world')
        pubs['cloud'].publish(point_cloud2.create_cloud_xyz32(header,points))
        pubs['old_cloud'].publish(point_cloud2.create_cloud_xyz32(Header(stamp=stamp,frame_id='odom'),[]))
    def run(points,kind='normal',seconds=.8):
        g0,c0=len(ground),len(combined);end=time.monotonic()+seconds
        while time.monotonic()<end:
            publish(points,kind)
            until=time.monotonic()+.03
            while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.005)
        return ground[g0:],combined[c0:]
    def value(msg,x=.15,y=.15):
        col=int((x-msg.info.origin.position.x)//msg.info.resolution)
        row=int((y-msg.info.origin.position.y)//msg.info.resolution)
        return msg.data[row*msg.info.width+col]
    patch=[(.0375+.075*i,.0375+.075*j,-.40288) for i in range(4) for j in range(4)]
    try:
        spawn('research_observed_ground','research_observed_ground.yaml')
        spawn('research_local_obstacle_grid','research_local_grid.yaml')
        edge=TransformStamped();edge.header.frame_id='odom';edge.child_frame_id='world';edge.transform.rotation.w=1.
        static.publish(TFMessage(transforms=[edge]));run([],seconds=2.)
        g,c=run(patch,seconds=1.2)
        cases.append(dict(case='dense_supported_ground_reaches_combined_grid',ok=bool(g and c and value(g[-1])==0 and value(c[-1])==0)))
        # Force the filtered obstacle cloud to arrive before the matching raw
        # support observation: previous-scan support must not be refreshed.
        measurement_ns+=30_000_000
        clock=Clock();clock.clock.sec,clock.clock.nanosec=divmod(measurement_ns,1_000_000_000)
        pubs['clock'].publish(clock);before=len(combined)
        pubs['old_cloud'].publish(point_cloud2.create_cloud_xyz32(Header(stamp=clock.clock,frame_id='odom'),[]))
        until=time.monotonic()+.12
        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.005)
        cases.append(dict(case='reverse_order_never_refreshes_previous_support',ok=len(combined)>before and value(combined[-1])==-1))
        g,c=run(patch[:-1]);cases.append(dict(case='missing_subcell_unknown',ok=bool(g and c and value(g[-1])==-1 and value(c[-1])==-1)))
        g,c=run(patch+[(.15,.15,-.34288)]);cases.append(dict(case='six_cm_obstacle_not_lost_by_old_height_filter',ok=bool(g and c and value(g[-1])==100 and value(c[-1])==100)))
        g,c=run(patch+[(.15,.15,-.60288)]);cases.append(dict(case='measured_drop_blocked',ok=bool(g and c and value(g[-1])==100 and value(c[-1])==100)))
        stair=[(x,y,z-.049) for x,y,z in patch]+[(x+.3,y,z+.049) for x,y,z in patch]
        g,c=run(stair);cases.append(dict(case='adjacent_flat_nine_cm_step_blocked',ok=bool(g and c and value(g[-1])==100 and value(c[-1])==100)))
        for kind in ('health_denied','tf_denied','unmatched','frame'):
            run(patch);g,c=run(patch,kind,seconds=1.1)
            cases.append(dict(case=kind+'_does_not_preserve_support',ok=bool(g and c and all(x==-1 for x in g[-1].data) and value(c[-1])==-1)))
        run(patch);before=len(ground)
        end=time.monotonic()+.9
        while time.monotonic()<end:
            # Clock paused and publishers silent: wall watchdog must invalidate.
            rclpy.spin_once(node,timeout_sec=.02)
        cases.append(dict(case='paused_clock_wall_expiry',ok=len(ground)>before and all(x==-1 for x in ground[-1].data) and value(combined[-1])==-1))
        run(patch);measurement_ns-=5_000_000_000
        g,c=run(patch,seconds=.9)
        cases.append(dict(case='clock_regression_latched',ok=bool(g and all(x==-1 for x in g[-1].data))))
        g,c=run(patch,seconds=.8)
        cases.append(dict(case='clock_fault_does_not_self_clear',ok=all(x==-1 for x in ground[-1].data)))
        # Stop only the owned provider, then reproduce queued old-version and
        # wrong-session ground arriving after the version callback.
        os.killpg(children[0].pid,signal.SIGINT);children[0].wait(timeout=5)
        measurement_ns+=100_000_000
        clock=Clock();clock.clock.sec,clock.clock.nanosec=divmod(measurement_ns,1_000_000_000)
        pubs['clock'].publish(clock);pubs['version'].publish(String(data='fixture-v2'))
        until=time.monotonic()+.12
        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.005)
        payload=LocalEvidenceGrid2D();payload.header=Header(stamp=clock.clock,frame_id='odom')
        payload.grid.header=payload.header;payload.grid.info.resolution=.3
        payload.grid.info.width=payload.grid.info.height=100;payload.grid.info.origin.position.x=payload.grid.info.origin.position.y=-15.
        payload.grid.info.origin.orientation.w=1.;payload.grid.data=[0]*10000
        payload.support_model='single_scan_flat_dense_v1';payload.localization_session_id='analytical-fixture'
        for case,version,session in [('queued_old_version','fixture-v1','analytical-fixture'),('wrong_session','fixture-v2','other-session')]:
            payload.map_version,payload.localization_session_id=version,session;ground_injection.publish(payload)
            pubs['old_cloud'].publish(point_cloud2.create_cloud_xyz32(payload.header,[]))
            until=time.monotonic()+.12
            while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.005)
            cases.append(dict(case=case+'_cannot_rebind_free',ok=value(combined[-1])==-1))
        bad=subprocess.run([str(args.install/'research_runtime/lib/research_runtime/research_observed_ground'),
            '--ros-args','-p','lidar_height_m:=0.9'],capture_output=True,text=True,timeout=10)
        cases.append(dict(case='CLI_mount_override_rejected',ok=bad.returncode!=0 and 'override rejected' in bad.stderr))
        result=dict(status='PASS' if all(c['ok'] for c in cases) and children[0].returncode==0 and children[1].poll() is None else 'FAIL',
            cases=cases,provider_owned_stop_exit=children[0].returncode,local_grid_alive=children[1].poll() is None,domain=106,
            ground_messages=len(ground),combined_messages=len(combined),
            source_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=args.repo,text=True).strip(),
            scope='Actual installed ROS support/local-grid nodes with analytical fixtures; no physical terrain, LIO accuracy, authority or actuator claim')
    finally:
        for p in reversed(children):
            if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
            try:p.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
        for stream in logs:stream.close()
        node.destroy_node();rclpy.shutdown()
    args.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
    return 0 if result['status']=='PASS' else 1


if __name__=='__main__':raise SystemExit(main())
