#!/usr/bin/env python3
"""Actual installed ROS receipt callbacks -> original guard -> original serial PTY.

State/grid/TF/consent are explicit analytical fixtures. Denial and restoration
are called before any timer runs to exercise the ordering boundary exactly.
This is software stop qualification, never physical braking/KEY acceptance.
"""
import argparse
import copy
import errno
import json
import math
import os
from pathlib import Path
import pty
import signal
import subprocess
import time
import rclpy
from nav_msgs.msg import Odometry,OccupancyGrid
from std_msgs.msg import Bool,String,Float32
from research_interfaces.msg import LocalEvidenceGrid2D,OperatorPermit,TimedTrajectory2D,TimedTrajectoryPoint2D
from research_runtime.safety_bridge import SafetyBridgeNode
from research_runtime.physical_parameter_lock import CONTROL_CHILD_FRAME,CONTROL_REFERENCE_CONTRACT


def main():
    parser=argparse.ArgumentParser()
    for key in ("repo","install","output"):parser.add_argument("--"+key,type=Path,required=True)
    args=parser.parse_args()
    if os.environ.get("ROS_DOMAIN_ID")!="109" or os.environ.get("ROS_LOCALHOST_ONLY")!="1":
        parser.error("requires isolated localhost domain109")
    args.output.parent.mkdir(parents=True,exist_ok=True)
    source=subprocess.check_output(["git","rev-parse","HEAD"],cwd=args.repo,text=True).strip()
    results=[]
    cases=("authority","mode","health","tf","stop","speed","map","duplicate_stamp","out_of_order_stamp","acquisition_gap")
    for case in cases:
        master,slave=pty.openpty();slave_path=os.ttyname(slave);os.set_blocking(master,False)
        children=[];logs=[];buffer=b"";wire=[];sequence=0;last_stamp=None;last_wire_w=0.;yaw=0.;previous=time.monotonic()
        rclpy.init(args=["--ros-args","--params-file",str(args.repo/"src/bringup/config/research_safety_bridge.yaml"),
            "-p","mode:=live","-p","actuator_enabled:=true","-p","localization_session_id:=mock-only",
            "-p","allow_analytical_grid_fixture:=true"])
        node=SafetyBridgeNode()
        pubs={name:node.create_publisher(kind,topic,10) for name,kind,topic in (
            ("authority",Bool,"/localization_authority/motion_allowed"),
            ("speed",Float32,"/localization_authority/max_linear_speed_mps"),
            ("stop",Bool,"/gps_corridor/stop_override"),
            ("operator",OperatorPermit,"/research/operator_permit"))}
        started=node.get_clock().now().to_msg()
        plan=TimedTrajectory2D()
        plan.header.frame_id="odom";plan.generated_at=started;plan.trajectory_id="ordering-"+case;plan.map_version="m1"
        plan.status=plan.STATUS_OK;plan.control_reference_contract=CONTROL_REFERENCE_CONTRACT
        plan.points=[TimedTrajectoryPoint2D(t=i*.1,x=0.,y=0.,yaw=i*.015,v=0.,w=.15,motion_mode=1) for i in range(201)]
        def drain():
            nonlocal buffer,last_wire_w
            while True:
                try:data=os.read(master,65536)
                except BlockingIOError:break
                except OSError as error:
                    if error.errno==errno.EIO:break
                    raise
                if not data:break
                buffer+=data
            while b"\n" in buffer:
                line,buffer=buffer.split(b"\n",1)
                text=line.decode(errors="replace")+"\n"
                if text.startswith("vcx="):
                    wire.append(text);last_wire_w=float(text.split(",")[1][3:])
        def feed(ready=False,skip_odom=False):
            nonlocal sequence,last_stamp,yaw,previous
            now=time.monotonic();yaw+=last_wire_w*(now-previous);previous=now
            stamp=node.get_clock().now().to_msg()
            node._map_version_cb(String(data="m1"))
            node._authority(Bool(data=True));node._authority_mode_cb(String(data="RTK_AUTHORITATIVE"))
            node._health_cb(String(data="OK:analytical_fixture"));node._tf_cb(Bool(data=True))
            node._stop_override_cb(Bool(data=False));node._speed_limit_cb(Float32(data=.85))
            pubs["authority"].publish(Bool(data=True));pubs["stop"].publish(Bool(data=False));pubs["speed"].publish(Float32(data=.85))
            grid=OccupancyGrid();grid.header.frame_id="odom";grid.header.stamp=stamp
            grid.info.resolution=.3;grid.info.width=grid.info.height=40
            grid.info.origin.position.x=grid.info.origin.position.y=-6.;grid.info.origin.orientation.w=1.;grid.data=[0]*1600
            node._grid_cb(LocalEvidenceGrid2D(header=grid.header,grid=grid,map_version="m1",
                localization_session_id="mock-only",support_model="analytical_fixture_v1"))
            node._permission_cb(grid)
            sequence+=1
            permit=OperatorPermit(header=grid.header,session_id="ordering-fixture",sequence=sequence,
                execution_mode="live",map_version="m1",state="READY" if ready else "AUTONOMOUS",
                lease_active=True,motion_requested=not ready)
            node._operator_cb(permit)
            if not skip_odom:
                odom=Odometry();odom.header=grid.header;odom.child_frame_id=CONTROL_CHILD_FRAME
                odom.pose.pose.orientation.z=math.sin(yaw/2);odom.pose.pose.orientation.w=math.cos(yaw/2)
                odom.twist.twist.angular.z=last_wire_w
                node._odom_cb(odom);last_stamp=copy.deepcopy(stamp)
            plan.header.stamp=stamp
            plan.valid_until.sec=stamp.sec;plan.valid_until.nanosec=stamp.nanosec+250000000
            if plan.valid_until.nanosec>=1000000000:plan.valid_until.sec+=1;plan.valid_until.nanosec-=1000000000
            node._trajectory_cb(plan)
            return odom if not skip_odom else None
        def run(seconds,ready=False,skip_odom=False):
            begin=len(wire);deadline=time.monotonic()+seconds
            while time.monotonic()<deadline:
                feed(ready=ready,skip_odom=skip_odom)
                rclpy.spin_once(node,timeout_sec=.01)
                drain()
                time.sleep(.01)
            drain();return wire[begin:]
        try:
            for package,binary,ros_args in (
                ("gps_waypoint_dispatcher","corridor_cmd_vel_guard_node",["--params-file",str(args.repo/"src/bringup/config/master_params.yaml")]),
                ("serial_twistctl","serial_twistctl_node",["--params-file",str(args.repo/"src/bringup/config/master_params.yaml"),
                    "-p","port:="+slave_path,"-r","/cmd_vel:=/cmd_vel_guarded"])):
                log=args.output.with_name(args.output.stem+"-"+case+"-"+binary+".log").open("w");logs.append(log)
                children.append(subprocess.Popen([str(args.install/package/"lib"/package/binary),"--ros-args",*ros_args],
                    stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
            run(.5,ready=True)
            nominal=run(1.8)
            admitted=node._tracker._rotation_token is not None
            live_rotation=any(abs(float(line.split(",")[1][3:]))>.04 for line in nominal)
            if case=="authority":node._authority(Bool(data=False));node._authority(Bool(data=True))
            if case=="mode":node._authority_mode_cb(String(data="RTK_HOLD"));node._authority_mode_cb(String(data="RTK_AUTHORITATIVE"))
            if case=="health":node._health_cb(String(data="UNKNOWN:lost"));node._health_cb(String(data="OK:restored"))
            if case=="tf":node._tf_cb(Bool(data=False));node._tf_cb(Bool(data=True))
            if case=="stop":node._stop_override_cb(Bool(data=True));node._stop_override_cb(Bool(data=False))
            if case=="speed":node._speed_limit_cb(Float32(data=0.));node._speed_limit_cb(Float32(data=.85))
            if case=="map":node._map_version_cb(String(data="UNKNOWN"));node._map_version_cb(String(data="m1"))
            if case in ("duplicate_stamp","out_of_order_stamp"):
                odom=Odometry();odom.header.frame_id="odom";odom.header.stamp=last_stamp;odom.child_frame_id=CONTROL_CHILD_FRAME
                odom.pose.pose.orientation.z=math.sin(yaw/2);odom.pose.pose.orientation.w=math.cos(yaw/2)
                odom.twist.twist.angular.z=last_wire_w
                if case=="out_of_order_stamp":
                    ns=odom.header.stamp.sec*1000000000+odom.header.stamp.nanosec-1000000
                    odom.header.stamp.sec,odom.header.stamp.nanosec=divmod(ns,1000000000)
                node._odom_cb(odom)
                feed()
            if case=="acquisition_gap":
                run(.215,skip_odom=True)
                feed()
            revoked=node._tracker._rotation_token is None
            node._tick()
            tail=run(.35)[-5:]
            stopped=len(tail)==5 and all(line=="vcx=0.000,wc=0.000\n" for line in tail)
            results.append(dict(case=case,nominal_rotation_reached_serial=live_rotation,
                token_committed_before_fault=admitted,receipt_denial_revoked_before_next_tick=revoked,
                restored_within_original_one_second_window=True,final_serial_tail=tail,
                status="PASS" if live_rotation and admitted and revoked and stopped else "FAIL"))
        finally:
            node.destroy_node();rclpy.shutdown()
            for child in reversed(children):
                if child.poll() is None:os.killpg(child.pid,signal.SIGINT)
                try:child.wait(timeout=5)
                except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=5)
            for log in logs:log.close()
            os.close(master);os.close(slave)
    result=dict(status="PASS" if all(c["status"]=="PASS" for c in results) else "FAIL",
        source_commit=source,cases=results,domain=109,sink="owned PTY",
        scope="Actual installed ROS class receipt callbacks before tick; analytical state/grid/TF/consent; actual original guard and serial binary/final wire. No physical motion or emergency-stop claim")
    args.output.write_text(json.dumps(result,indent=2)+"\n");print(json.dumps(result,indent=2))
    return 0 if result["status"]=="PASS" else 1


if __name__=="__main__":raise SystemExit(main())
