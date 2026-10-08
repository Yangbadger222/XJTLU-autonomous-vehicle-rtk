#!/usr/bin/env python3
"""Real HTTP cockpit -> typed consent -> tracker -> original guard/PTY.

All sensor/health/ground values below are declared laboratory fixtures.
The task selection endpoint belongs to the actual observation node, with a
synthetic registered prior. No physical device is reachable from this test.
"""
import argparse
import http.cookiejar
import json
import math
import os
from pathlib import Path
import pty
import signal
import subprocess
import sys
import time
import urllib.error
import urllib.request
import uuid

import rclpy
from research_interfaces.msg import LocalEvidenceGrid2D
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Bool, Float32, String
from tf2_msgs.msg import TFMessage
from research_interfaces.msg import TimedTrajectory2D, TimedTrajectoryPoint2D
from validate_restricted_policy_ros import asset_fixtures, terminate


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--install",type=Path,required=True)
    parser.add_argument("--repo",type=Path,required=True)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--serve-seconds",type=float,default=0.)
    args=parser.parse_args()
    if os.environ.get("ROS_DOMAIN_ID")!="99" or os.environ.get("ROS_LOCALHOST_ONLY")!="1":
        raise SystemExit("requires domain 99 and localhost-only; mock serial only")
    args.output.parent.mkdir(parents=True,exist_ok=True)
    fixture=asset_fixtures(args.output.parent/"console-synthetic-prior")
    master,slave=pty.openpty();port=os.ttyname(slave);os.set_blocking(master,False)
    config=args.repo/"src/bringup/config"
    children=[];logs=[];wire=[];buffer=b""
    rclpy.init();node=rclpy.create_node("console_mock_acceptance_probe")
    topics={"authority":(Bool,"/localization_authority/motion_allowed"),"mode":(String,"/localization_authority/mode"),
        "speed":(Float32,"/localization_authority/max_linear_speed_mps"),"health":(String,"/lio/vehicle_health"),
        "version":(String,"/research/map_version"),"odom":(Odometry,"/research/odom_control"),
        "grid":(OccupancyGrid,"/research/local_obstacle_grid"),"permission":(OccupancyGrid,"/research/permission_grid"),
        "trajectory":(TimedTrajectory2D,"/research/ego_trajectory"),"tf":(TFMessage,"/tf")}
    pubs={k:node.create_publisher(kind,topic,10) for k,(kind,topic) in topics.items()}
    typed_grid=node.create_publisher(LocalEvidenceGrid2D,"/research/local_evidence_grid",10)
    base="http://127.0.0.1:8765"
    opener=urllib.request.build_opener(urllib.request.HTTPCookieProcessor(http.cookiejar.CookieJar()))
    csrf="";window=str(uuid.uuid4());cases=[]
    def request(action,payload=None,heartbeat=False):
        body=json.dumps({"action":action,"payload":payload or {},"request_id":str(uuid.uuid4())}).encode()
        req=urllib.request.Request(base+"/api/command",data=body,headers={"Content-Type":"application/json",
            "Origin":base,"X-Console-CSRF":csrf,"X-Console-Window":window})
        try:response=opener.open(req,timeout=.3)
        except urllib.error.HTTPError as exc:response=exc
        return json.loads(response.read())
    def snapshot():
        return json.loads(opener.open(urllib.request.Request(base+"/api/state",headers={"X-Console-Window":window}),timeout=.3).read())
    def drain():
        nonlocal buffer
        while True:
            try:data=os.read(master,65536)
            except BlockingIOError:break
            if not data:break
            buffer+=data
        while b"\n" in buffer:
            line,buffer=buffer.split(b"\n",1);wire.append(line.decode()+"\n")
    def tick(authority=True,heartbeat=True,lateral_speed=0.,vertical_speed=0.):
        stamp=node.get_clock().now().to_msg()
        pubs["authority"].publish(Bool(data=authority));pubs["mode"].publish(String(data="RTK_AUTHORITATIVE" if authority else "RTK_HOLD"))
        pubs["speed"].publish(Float32(data=.85));pubs["health"].publish(String(data="OK: SIMULATED HMI fixture"));pubs["version"].publish(String(data="console-simulation-m1"))
        odom=Odometry();odom.header.frame_id,odom.child_frame_id="odom","chassis_control_origin";odom.header.stamp=stamp
        odom.pose.pose.orientation.w=1.;odom.pose.covariance[0]=odom.pose.covariance[7]=.0009
        odom.twist.twist.linear.y=lateral_speed;odom.twist.twist.linear.z=vertical_speed
        pubs["odom"].publish(odom)
        for kind in ("grid","permission"):
            grid=OccupancyGrid();grid.header.stamp,grid.header.frame_id=stamp,"odom";grid.info.resolution=.3
            grid.info.width=grid.info.height=40;grid.info.origin.position.x=grid.info.origin.position.y=-6.
            grid.info.origin.orientation.w=1.;grid.data=[0]*1600;pubs[kind].publish(grid)
            if kind=="grid":typed_grid.publish(LocalEvidenceGrid2D(header=grid.header,grid=grid,
                map_version="console-simulation-m1",localization_session_id="mock-only",support_model="analytical_fixture_v1"))
        transforms=[]
        for parent,child in (("map","odom"),("odom","base_footprint")):
            tf=TransformStamped();tf.header.stamp,tf.header.frame_id,tf.child_frame_id=stamp,parent,child
            tf.transform.rotation.w=1.;transforms.append(tf)
        pubs["tf"].publish(TFMessage(transforms=transforms))
        trajectory=TimedTrajectory2D();trajectory.control_reference_contract="corridor_e54c6af_mid360_ground_control_origin_v1";trajectory.header.stamp,trajectory.header.frame_id=stamp,"odom"
        trajectory.generated_at=stamp;trajectory.valid_until.sec,trajectory.valid_until.nanosec=stamp.sec+2,stamp.nanosec
        trajectory.trajectory_id,trajectory.map_version="HMI-synthetic-track-fixture","console-simulation-m1";trajectory.status=trajectory.STATUS_OK
        for t,x in ((0.,0.),(2.,.4)):
            point=TimedTrajectoryPoint2D();point.t,point.x,point.v=t,x,.2;trajectory.points.append(point)
        pubs["trajectory"].publish(trajectory)
        rclpy.spin_once(node,timeout_sec=.005)
        if csrf and heartbeat:
            try:request("heartbeat",heartbeat=True)
            except (OSError,urllib.error.URLError):pass
        drain()
    def phase(seconds,authority=True,heartbeat=True,lateral_speed=0.,vertical_speed=0.):
        start=len(wire);deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:tick(authority,heartbeat,lateral_speed,vertical_speed);time.sleep(.03)
        return wire[start:]
    def spawn(package,executable,extra):
        log=(args.output.parent/("console-"+executable+".log")).open("w");logs.append(log)
        cmd=[str(args.install/package/"lib"/package/executable),"--ros-args"]+extra
        if package in ("research_runtime","active_road_mapping","gps_waypoint_dispatcher"):cmd.insert(0,sys.executable)
        child=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);children.append(child);return child
    def rearm():
        phase(1.2)
        assert request("reset")["accepted"],snapshot()["reason"]
        assert request("task",{"start_node":"start","goal_node":"goal"})["accepted"]
        phase(.7)
        result=request("start")
        assert result["accepted"],result["reason"]
        lines=phase(1.5)
        return any(float(line.split(",")[0][4:])>0 for line in lines)
    def check(name,nonzero,lines,state=None):
        tail=lines[-5:]
        ok=nonzero and len(tail)==5 and all(line=="vcx=0.000,wc=0.000\n" for line in tail)
        cases.append({"case":name,"nominal_nonzero_final_serial":nonzero,"final_serial_tail":tail,"console_state":state,"status":"PASS" if ok else "FAIL"})
    try:
        console=spawn("research_runtime","research_operator_console",["-p","execution_mode:=live","-p","actuator_enabled:=true","-p","mission_execution_enabled:=true"])
        spawn("research_runtime","research_safety_bridge",["--params-file",str(config/"research_safety_bridge.yaml"),"-p","mode:=live","-p","actuator_enabled:=true","-p","max_curvature_1pm:=1.0","-p","max_lateral_speed_mps:=0.05",
            "-p","localization_session_id:=mock-only","-p","allow_analytical_grid_fixture:=true"])
        spawn("gps_waypoint_dispatcher","corridor_cmd_vel_guard_node",["--params-file",str(config/"master_params.yaml")])
        spawn("serial_twistctl","serial_twistctl_node",["--params-file",str(config/"master_params.yaml"),"-p","port:="+port,"-r","/cmd_vel:=/cmd_vel_guarded"])
        spawn("ego_planner","research_tf_guard",[])
        spawn("active_road_mapping","active_observation",["--params-file",str(config/"active_observation.yaml"),"-p","execution_mode:=live","-p","mission_execution_enabled:=true",
            "-p","prior_manifest_path:="+str(fixture),"-p","task_start_node:=start","-p","task_goal_node:=goal","-p","sensor_range_m:=1.8","-p","sensor_fov_rad:="+str(2*math.pi),"-p","max_curvature_1pm:=1.0","-p","policy:=PASSIVE"])
        phase(3.,heartbeat=False)
        csrf=json.loads(opener.open(base+"/api/session").read())["csrf"]
        assert request("claim")["accepted"]
        if args.serve_seconds:
            # Release this test window so the human/browser can operate it.
            phase(.65,heartbeat=False)
            deadline=time.monotonic()+args.serve_seconds
            sampled_at=0.;samples=[]
            while time.monotonic()<deadline:
                tick(heartbeat=False);time.sleep(.03)
                if time.monotonic()-sampled_at>.2:
                    sampled_at=time.monotonic()
                    try:current=snapshot()
                    except OSError:current={}
                    samples.append({"time":time.time(),"state":current.get("state"),"reason":current.get("reason"),
                        "serial_tail":wire[-5:],"nonzero_count":sum(line!="vcx=0.000,wc=0.000\n" for line in wire)})
                    args.output.with_suffix('.samples.json').write_text(json.dumps(samples,indent=2))
            result={"status":"SERVE_COMPLETE","wire_tail":wire[-5:],"wire_nonzero_count":sum(line!="vcx=0.000,wc=0.000\n" for line in wire)}
        else:
            for kind in ('lateral','vertical'):
                phase(1.3,lateral_speed=.08 if kind=='lateral' else 0.,vertical_speed=.08 if kind=='vertical' else 0.)
                current=snapshot();reset=request('reset')
                cases.append(dict(case=kind+'_translation_cannot_confirm_stop',
                    status='PASS' if not current['stop_confirmed_from_odom'] and not reset['accepted'] else 'FAIL',
                    stop_confirmed=current['stop_confirmed_from_odom'],reset_response=reset))
            for kind in ('invalid','moving'):
                phase(1.2);assert request('reset')['accepted']
                assert request('task',{'start_node':'start','goal_node':'goal'})['accepted']
                phase(.7)
                phase(.12,lateral_speed=float('nan') if kind=='invalid' else .08)
                phase(.08)
                current=snapshot();start=request('start')
                cases.append(dict(case='ready_after_'+kind+'_short_recovery_cannot_start',
                    status='PASS' if not current['stop_confirmed_from_odom'] and not start['accepted'] else 'FAIL',
                    stop_confirmed=current['stop_confirmed_from_odom'],start_response=start))
            for action in ("pause","stop","takeover"):
                nonzero=rearm();assert request(action)["accepted"]
                check("http_"+action,nonzero,phase(.85),snapshot()["state"])
            nonzero=rearm();lines=phase(.9,heartbeat=False)
            check("browser_heartbeat_lost",nonzero,lines,snapshot()["state"])
            assert request("claim")["accepted"]
            nonzero=rearm();lines=phase(.85,authority=False)
            check("rtk_loss_with_healthy_lio",nonzero,lines,snapshot()["state"])
            lines=phase(.8)
            check("rtk_return_does_not_auto_resume",nonzero,lines,snapshot()["state"])
            nonzero=rearm();os.killpg(console.pid,signal.SIGKILL);console.wait(timeout=2)
            check("console_backend_crash",nonzero,phase(.9,heartbeat=False))
            result={"status":"PASS" if all(c["status"]=="PASS" for c in cases) else "FAIL","cases":cases,
                "scope":"Actual HTTP/typed consent/observer/task service/tracker/original guard/original serial PTY; synthetic odom, floor, RTK, health and trajectory only. Physical stop/KEY/joystick not validated.",
                "domain":99,"source_commit":subprocess.check_output(["git","-C",str(args.repo),"rev-parse","HEAD"],text=True).strip()}
    finally:
        for child in reversed(children):terminate(child)
        for log in logs:log.close()
        os.close(master);os.close(slave);node.destroy_node();rclpy.shutdown()
    args.output.write_text(json.dumps(result,indent=2)+"\n");print(json.dumps(result,indent=2))
    return 0 if result["status"] in ("PASS","SERVE_COMPLETE") else 1


if __name__=="__main__":raise SystemExit(main())
