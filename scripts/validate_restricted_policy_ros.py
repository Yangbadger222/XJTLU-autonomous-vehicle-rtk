#!/usr/bin/env python3
"""Same pinned Super-LIO/EGO/PT Y foundation, separate truth/perception/policy.

Synthetic GeoTIFF/MaGRoad-format fixtures are deliberately labelled. No real
satellite, MaGRoad segmentation or camera calibration is fabricated.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import pty
import signal
import subprocess
import sys
import time
import rclpy
from research_interfaces.msg import ResearchStatus, TimedTrajectory2D, ObservationGoal
from nav_msgs.msg import Odometry, OccupancyGrid, Path as RosPath
from geometry_msgs.msg import AccelStamped
import numpy as np
import rasterio
from rasterio.transform import from_origin
from research_runtime.active_road import EvidenceStore,GeoTransform


def asset_fixtures(directory):
    directory.mkdir(parents=True,exist_ok=True)
    road=directory/'synthetic-prior.geojson';tiff=directory/'synthetic-georeferenced-raster.tif'
    origin=(385000.,3430000.)
    features=[]
    for identity,start,end,a,b in (('a','start','left',0.,1.2),('b','right','goal',1.5,3.3)):
        features.append({'type':'Feature','properties':{'id':identity,'start_id':start,'end_id':end},
            'geometry':{'type':'LineString','coordinates':[[origin[0]+a,origin[1]],[origin[0]+b,origin[1]]]}})
    road.write_text(json.dumps({'type':'FeatureCollection','crs':{'properties':{'name':'EPSG:32651'}},'features':features}))
    with rasterio.open(tiff,'w',driver='GTiff',width=100,height=100,count=1,dtype='uint8',crs='EPSG:32651',
                       transform=from_origin(origin[0]-15,origin[1]+15,.3,.3)) as stream:stream.write(np.zeros((1,100,100),dtype=np.uint8))
    manifest={'schema':1,'map_registration_verified':True,'scope':'SIMULATION_ONLY synthetic metric registration; no vehicle RTK origin changed',
        'road_geojson':road.name,'road_geojson_sha256':hashlib.sha256(road.read_bytes()).hexdigest(),
        'geotiff':tiff.name,'geotiff_sha256':hashlib.sha256(tiff.read_bytes()).hexdigest(),
        'crs':'EPSG:32651','model_version':'synthetic-prior-fixture-v1','map_from_crs_xyyaw':[-origin[0],-origin[1],0.]}
    path=directory/'registration.json';path.write_text(json.dumps(manifest,indent=2)+'\n');return path


def terminate(child):
    if child.poll() is None:os.killpg(child.pid,signal.SIGINT)
    try:child.wait(timeout=5)
    except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=5)


def trial(args,mode,index,manifest):
    root=args.output.parent/mode/f'task-{index}';root.mkdir(parents=True,exist_ok=True)
    common=args.output.parent/mode;store=common/'evidence.json';snapshot=common/'policy.json'
    if not store.exists():EvidenceStore(GeoTransform('LOCAL_ODOM_SUBMAP','synthetic_local',0,0,.3,.3),hashlib.sha256(manifest.read_bytes()).hexdigest()).save(store)
    before=EvidenceStore.load(store)
    prior_ids={e.evidence_id for e in before.evidence()}
    before_graph_count=len(before.graph_updates)
    children=[];logs=[];master,slave=pty.openpty();port=os.ttyname(slave)
    def spawn(name,command,pass_fds=()):
        log=(root/f'{name}.log').open('w');logs.append(log)
        child=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True,pass_fds=pass_fds)
        children.append((name,child));return child
    def binary(package,executable,params=None,extra=()):
        command=[str(args.install/package/'lib'/package/executable),'--ros-args']
        if params:command+=['--params-file',str(params)]
        return command+list(extra)
    def python_node(package,executable,params=None,extra=()):
        return [sys.executable]+binary(package,executable,params,extra)
    config=args.repo/'src/bringup/config'
    rclpy.init();probe=rclpy.create_node("restricted_trial_evaluator")
    observations=[];reasons={};planning={}; inputs={}; failures=[]
    def odom_cb(msg):
        q=msg.pose.pose.orientation
        inputs['state']=[msg.pose.pose.position.x,msg.pose.pose.position.y,2*math.atan2(q.z,q.w),
                         msg.twist.twist.linear.x,msg.twist.twist.angular.z]
    probe.create_subscription(Odometry,'/lio/odom_vehicle',odom_cb,100)
    probe.create_subscription(AccelStamped,'/research/vehicle_acceleration',lambda msg:inputs.update(
        acceleration=[msg.accel.linear.x,msg.accel.linear.y]),100)
    probe.create_subscription(RosPath,'/research/road_reference',lambda msg:inputs.update(
        reference=[[p.pose.position.x,p.pose.position.y] for p in msg.poses]),10)
    probe.create_subscription(OccupancyGrid,'/research/local_obstacle_grid',lambda msg:inputs.update(
        grid={'resolution':msg.info.resolution,'origin':[msg.info.origin.position.x,msg.info.origin.position.y],
              'width':msg.info.width,'height':msg.info.height,'cells':list(msg.data)}),10)
    def status_cb(msg):reasons[msg.reason]=reasons.get(msg.reason,0)+1
    def plan_cb(msg):
        planning[msg.failure_reason or "OK"]=planning.get(msg.failure_reason or "OK",0)+1
        if msg.failure_reason=='ego_planner_vehicle_feasibility_rejected' and len(failures)<10 and len(inputs)==4:
            failures.append(json.loads(json.dumps(inputs)))
    probe.create_subscription(ResearchStatus,"/research/status",status_cb,100)
    probe.create_subscription(TimedTrajectory2D,"/research/ego_trajectory",plan_cb,100)
    probe.create_subscription(ObservationGoal,"/research/observation_goal",lambda msg:observations.append(
        {"id":msg.goal_id,"event":msg.event_id,"pose":[msg.x,msg.y,msg.yaw],"cost":msg.cost,"score":msg.score,"reason":msg.reason}),10)
    start=time.monotonic();truth=None
    try:
        spawn('super-lio',binary('super_lio','super_lio_node',config/'super_lio_vehicle.yaml'))
        spawn('map',python_node('active_road_mapping','active_road_map',config/'active_road_mapping.yaml',
            ['-p',f'evidence_store_path:={store}']))
        spawn('evidence',python_node('active_road_mapping','active_road_evidence',config/'active_road_evidence.yaml',
            ['-p',f'evidence_store_path:={store}','-p','global_anchor_uncertainty_verified:=true','-p','global_anchor_uncertainty_m:=0.03']))
        spawn('perception',[sys.executable,str(args.repo/'scripts/research_restricted_sim.py'),'perception','--output',str(root/'perception.json')]+(
            ['--seed-evidence',str(store)] if index>1 else []))
        spawn('local-grid',python_node('research_runtime','research_local_obstacle_grid',config/'research_local_grid.yaml'))
        spawn('ego',binary('ego_planner','motion_plan',config/'ego_vehicle_adapter.yaml',
            ['-p','use_measured_acceleration:=true','-p','max_curvature_1pm:=1.0','-p','max_lateral_speed_mps:=0.05','-p','max_jerk_mps3:=3.0',
             '-p','inflate_radius_m:='+str(math.hypot(.33,.305))]))
        spawn('tf-guard',binary('ego_planner','research_tf_guard'))
        spawn('safety',python_node('research_runtime','research_safety_bridge',config/'research_safety_bridge.yaml',
            ['-p','mode:=live','-p','actuator_enabled:=true','-p','max_curvature_1pm:=1.0','-p','max_lateral_speed_mps:=0.05']))
        spawn('guard',python_node('gps_waypoint_dispatcher','corridor_cmd_vel_guard_node',config/'master_params.yaml'))
        spawn('serial',binary('serial_twistctl','serial_twistctl_node',config/'master_params.yaml',
            ['-p','port:='+port,'-r','/cmd_vel:=/cmd_vel_guarded']))
        spawn('observer',python_node('active_road_mapping','active_observation',config/'active_observation.yaml',
            ['-p','execution_mode:=live','-p','mission_execution_enabled:=true','-p','policy:='+mode,'-p',f'prior_manifest_path:={manifest}','-p','task_start_node:=start','-p','task_goal_node:=goal',
             '-p',f'evidence_store_path:={store}',
             '-p',f'policy_snapshot_path:={snapshot}','-p','sensor_range_m:=1.8','-p','sensor_fov_rad:='+str(2*math.pi),
             '-p','max_curvature_1pm:=1.0','-p','observation_budget_s:='+str(args.budget_s),'--log-level','info']))
        truth=spawn('truth',[sys.executable,str(args.repo/'scripts/research_restricted_sim.py'),'truth',
            '--pty-fd',str(master),'--output',str(root/'truth-evaluation-only.json'),
            '--sensor-fault',args.sensor_fault,'--fault-after-s',str(args.fault_after_s)],pass_fds=(master,))
        while time.monotonic()-start<args.budget_s+5 and all(child.poll() is None for _,child in children):
            rclpy.spin_once(probe,timeout_sec=.02)
            # Evaluator-only termination reads policy status, never feeds truth back.
            if 'TASK_REACHED' in (root/'observer.log').read_text():break
        exits={name:child.poll() for name,child in children}
        terminate(truth);truth=None
        # Keep the original guard/serial alive while simulated RTK permission
        # falls; the truth role records actual final PTY zeros before exiting.
        for name,child in reversed(children):
            if name!='truth':terminate(child)
        final=json.loads((root/'truth-evaluation-only.json').read_text()) if (root/'truth-evaluation-only.json').exists() else {}
        measured=json.loads((root/'perception.json').read_text()) if (root/'perception.json').exists() else {}
        saved=json.loads(snapshot.read_text()) if snapshot.exists() else {}
        evidence=EvidenceStore.load(store)
        reached=bool(final) and math.dist(final['final_pose'][:2],(3.3,0.))<.25
        result={'strategy':mode,'task':index,'wall_s':time.monotonic()-start,'task_reached':reached,
            'final_truth_pose':final.get('final_pose'),'actual_superlio_odometry_count':measured.get('lio_odometry_count',0),
            'processed_limited_depth_frames':measured.get('processed_depth_count',0),
            'known_ground_cells':measured.get('known_ground_cell_count',0),'distance_m':final.get('distance_m'),
            'observation_attempts':saved.get('attempted_views',[]),'resolved_geometry_events':saved.get('resolved_geometry',[]),
            'tracking_reason_counts':reasons,'planning_result_counts':planning,'selected_views':observations,
            'observed_evidence_count':len(evidence.evidence()),
            'persisted_graph_increment_count':len(evidence.graph_updates),'anchored_graph_increment_count':len(evidence.graph_updates_in_map()),
            'previous_graph_increment_count':before_graph_count,
            'previous_evidence_uuid_count':len(prior_ids),'previous_evidence_uuids_preserved':prior_ids<={e.evidence_id for e in evidence.evidence()},
            'planned_observation_cost_s':sum(v['cost'] for v in observations),
            'resolved_event_correctness':{'correct':sum(str(e).startswith('gap:') for e in saved.get('resolved_geometry',[])),
                'incorrect':0,'unscored':sum(not str(e).startswith('gap:') for e in saved.get('resolved_geometry',[])),
                'scope':'evaluator-only synthetic prior gaps have support; other events are not assigned invented truth'},
            'sensor_fault':args.sensor_fault,'fault_phase_final_serial_tail':final.get('fault_phase_wire_tail_before_authority_cleanup',[]),
            'nonzero_before_fault':final.get('nonzero_before_fault',0),'map_version':evidence.map_version,
            'final_serial_tail':final.get('wire_tail',[]),'nonzero_serial_count':final.get('nonzero_wire_count',0),
            'actuation_exercised':final.get('nonzero_wire_count',0)>10,
            'child_exit_before_cleanup':exits,'child_cleanup_exits':{name:child.returncode for name,child in children},
            'status':'PASS' if all(exit is None for exit in exits.values()) and measured.get('lio_odometry_count',0)>20
                and measured.get('processed_depth_count',0)>20 and final.get('nonzero_wire_count',0)>10
                and final.get('wire_tail')==['vcx=0.000,wc=0.000\n']*5 else 'FAIL',
            'outcome':'TASK_REACHED' if reached else 'BUDGET_OR_NO_CONFIRMED_ROUTE'}
        if args.sensor_fault!='none':
            result['status']='PASS' if result['nonzero_before_fault']>10 and result['fault_phase_final_serial_tail']==['vcx=0.000,wc=0.000\n']*5 else 'FAIL'
        result['protocol_status']=result.pop('status')
    finally:
        for _,child in reversed(children):
            if child.poll() is None:terminate(child)
        for log in logs:log.close()
        os.close(master);os.close(slave)
        probe.destroy_node();rclpy.shutdown()
    (root/'failed-measured-inputs.json').write_text(json.dumps(failures)+'\n')
    (root/'result.json').write_text(json.dumps(result,indent=2)+'\n');return result


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--repo',type=Path,required=True);parser.add_argument('--install',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True);parser.add_argument('--budget-s',type=float,default=45.)
    parser.add_argument('--strategies',nargs='+',default=['PASSIVE','PERIODIC_LOOK','TASK_AWARE_LOOK']);parser.add_argument('--tasks',type=int,default=2)
    parser.add_argument('--sensor-fault',choices=['none','imu_lost','lidar_lost','depth_invalid'],default='none')
    parser.add_argument('--fault-after-s',type=float,default=12.)
    args=parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='94' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':raise SystemExit('requires isolated domain 94')
    args.output.parent.mkdir(parents=True,exist_ok=True);manifest=asset_fixtures(args.output.parent/'read-only-prior')
    prior_hashes={path.name:hashlib.sha256(path.read_bytes()).hexdigest() for path in manifest.parent.iterdir()}
    def foundation_hashes():
        files={}
        for package in ('super_lio','ego_planner','research_runtime','active_road_mapping','research_interfaces',
                        'gps_waypoint_dispatcher','serial_twistctl','super_lio_vehicle_adapter'):
            for path in sorted((args.install/package).rglob('*')):
                if path.is_file() and '__pycache__' not in path.parts and path.suffix!='.pyc':
                    files[str(path.relative_to(args.install))]=hashlib.sha256(path.read_bytes()).hexdigest()
        for path in sorted((args.repo/'src/bringup/config').glob('*.yaml')):
            files['config/'+path.name]=hashlib.sha256(path.read_bytes()).hexdigest()
        files['sensor_model']=hashlib.sha256((args.repo/'scripts/research_restricted_sim.py').read_bytes()).hexdigest()
        return files
    foundation=foundation_hashes()
    (args.output.parent/'foundation.json').write_text(json.dumps(foundation,indent=2)+'\n')
    trials=[];foundation_preserved=True
    for mode in args.strategies:
        for index in range(1,args.tasks+1):
            if foundation_hashes()!=foundation:raise RuntimeError('foundation changed before trial')
            result=trial(args,mode,index,manifest);trials.append(result)
            foundation_preserved &= foundation_hashes()==foundation
            print(json.dumps(result),flush=True)
            # Retain a failed trial and still execute its second task. Zero
            # motion is a reported protocol failure, never a reason to omit
            # the requested saved-map reuse comparison.
    preserved=prior_hashes=={path.name:hashlib.sha256(path.read_bytes()).hexdigest() for path in manifest.parent.iterdir()}
    result={'foundation_manifest_sha256':hashlib.sha256(json.dumps(foundation,sort_keys=True).encode()).hexdigest(),'foundation_file_count':len(foundation),'foundation_unchanged_each_trial':foundation_preserved,'scope':'finite synthetic sensor closed-loop comparison; actual pinned Super-LIO/EGO, original guard and serial PTY',
        'simulation_only_assumptions':['IMU=controlled-base extrinsic; measured simulated encoder body twist','synthetic .03m pose uncertainty bound','source health assumption; actual /lio/health remains UNKNOWN',
            'synthetic restricted depth model/support plane and prior registration; no physical camera acceptance'],
        'truth_isolation':'separate truth process publishes raw sensors only; policy/map processes have no truth-pose/full-map input or path',
        'prior_bytes_preserved':preserved,'trials':trials,'protocol_status':'PASS' if preserved and foundation_preserved and all(t['protocol_status']=='PASS' for t in trials) else 'FAIL',
        'benefit':'NOT_ESTABLISHED: finite scene measurements do not prove novelty or real-world policy benefit'}
    args.output.write_text(json.dumps(result,indent=2)+'\n');return 0 if result['protocol_status']=='PASS' else 1


if __name__=='__main__':raise SystemExit(main())
