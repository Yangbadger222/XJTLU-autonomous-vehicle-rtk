#!/usr/bin/env python3
"""Actual raw sensor replay through current Super/reference/cloud, no actuator."""
import argparse
import base64
from collections import Counter
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import statistics
import subprocess
import time

import rclpy
from rclpy.parameter import Parameter
from rclpy.serialization import serialize_message
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import String, Bool
from research_interfaces.msg import LocalEvidenceGrid2D, RoadEvidence2D
from tf2_msgs.msg import TFMessage
from livox_ros_driver2.msg import CustomMsg
from raw_replay_contract import replay_contract
from replay_acceptance import quaternion_distance, requested_prefix_completed, stream_continuity
from super_lio_vehicle_adapter.adapter_node import _covariance_is_known, _source_certificate


def main():
    parser=argparse.ArgumentParser()
    for name in ('repo','install','bag','output'):parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--duration-s',type=float,default=0.,help='0 means full EOF; positive is explicitly a prefix')
    parser.add_argument('--ground-pipeline',action='store_true',help='actual local TF/ground/evidence without any RTK or actuator fixture')
    args=parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='104' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise SystemExit('requires isolated domain104 and localhost-only')
    if not math.isfinite(args.duration_s) or args.duration_s<0:raise SystemExit('invalid duration')
    contract=replay_contract(args.bag)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    rclpy.init();node=rclpy.create_node('source_reference_raw_probe',parameter_overrides=[Parameter('use_sim_time',value=True)])
    stamp=lambda msg:msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
    native={};vehicle={};certificates={};health=Counter();reasons=Counter();vehicle_health=Counter()
    counts=Counter();covariance_failures=[];world_edges=[];bounds=[];resources=[];fault_capture=[]
    raw_lidar_stamps=[];raw_imu_stamps=[];last_source_health=None;last_vehicle_health=None
    children=[];logs=[];player=None
    local_frames=[];full_frames=[];ground_grids=[];combined_grids=[];road_evidence={};questions=[]
    ground_acquisitions=[]
    session='raw-ground-'+args.output.stem
    evidence_path=args.output.with_name(args.output.stem+'-evidence.json')
    def ground_grid(msg,collection,label):
        counts[label]+=1
        collection.append(dict(stamp_ns=stamp(msg),free=msg.grid.data.count(0),blocked=msg.grid.data.count(100),
            model=msg.support_model,version=msg.map_version,session=msg.localization_session_id,
            headers_match=msg.header==msg.grid.header))
    if args.ground_pipeline:
        from research_interfaces.msg import RoadEvent
        node.create_subscription(Bool,'/research/local_frame_integrity',lambda msg:local_frames.append(msg.data),100)
        node.create_subscription(Bool,'/research/tf_integrity',lambda msg:full_frames.append(msg.data),100)
        node.create_subscription(LocalEvidenceGrid2D,'/research/observed_ground_grid',
            lambda msg:ground_grid(msg,ground_grids,'ground'),100)
        node.create_subscription(LocalEvidenceGrid2D,'/research/local_evidence_grid',
            lambda msg:ground_grid(msg,combined_grids,'combined'),100)
        node.create_subscription(PointCloud2,'/research/ground_observation_cloud',lambda msg:ground_acquisitions.append(stamp(msg)),100)
        node.create_subscription(RoadEvidence2D,'/research/road_evidence',lambda msg:road_evidence.setdefault(msg.evidence_id,msg),1000)
        node.create_subscription(RoadEvent,'/research/measured_road_questions',lambda msg:questions.append(msg.event_id),1000)
    def odometry(msg,label,collection):
        counts[label]+=1;collection[stamp(msg)]=msg
        cov=tuple(msg.pose.covariance)+tuple(msg.twist.covariance)
        values=(msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z,
                msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w,
                msg.twist.twist.linear.x,msg.twist.twist.linear.y,msg.twist.twist.linear.z,
                msg.twist.twist.angular.x,msg.twist.twist.angular.y,msg.twist.twist.angular.z)
        if not all(map(math.isfinite,values)) or not _covariance_is_known(cov):covariance_failures.append([label,stamp(msg)])
    def source_health(msg):
        nonlocal last_source_health
        data=json.loads(msg.data);health[data['status']]+=1;reasons[data['reason']]+=1
        last_source_health=data
        certificate=_source_certificate(msg.data)
        if certificate:certificates[certificate['stamp_ns']]=certificate
        else:counts['invalid_native_certificate']+=1
        if data['iterations']>0:bounds.append(data['legacy_min_eig_lower_bound'])
    node.create_subscription(Odometry,'/lio/odom',lambda msg:odometry(msg,'native_odom',native),100)
    node.create_subscription(Odometry,'/lio/odom_vehicle',lambda msg:odometry(msg,'vehicle_odom',vehicle),100)
    node.create_subscription(String,'/lio/health',source_health,100)
    def adapter_health(msg):
        nonlocal last_vehicle_health
        last_vehicle_health=msg.data
        vehicle_health.update([msg.data])
        if 'restart with a new' in msg.data and not fault_capture and native and vehicle:
            last_vehicle=max(vehicle)
            selected=sorted(key for key in native if key>=last_vehicle)
            fault_capture.append({'reason':msg.data,'last_vehicle_stamp_ns':last_vehicle,
                'native_odometry_cdr_base64':[base64.b64encode(serialize_message(native[key])).decode() for key in selected],
                'certificates':[certificates.get(key) for key in selected]})
    node.create_subscription(String,'/lio/vehicle_health',adapter_health,100)
    def raw_input(msg,label,stamps):
        counts.update([label]);stamps.append(stamp(msg))
    node.create_subscription(Imu,'/livox/imu',lambda msg:raw_input(msg,'imu',raw_imu_stamps),QoSProfile(depth=1024,reliability=ReliabilityPolicy.BEST_EFFORT))
    node.create_subscription(CustomMsg,'/livox/lidar',lambda msg:raw_input(msg,'lidar',raw_lidar_stamps),QoSProfile(depth=100,reliability=ReliabilityPolicy.BEST_EFFORT))
    node.create_subscription(PointCloud2,'/lio/cloud_odom',lambda msg:counts.update(['cloud_odom' if msg.header.frame_id=='odom' else 'wrong_cloud_frame']),20)
    node.create_subscription(TFMessage,'/tf_static',lambda msg:world_edges.extend(t for t in msg.transforms if t.child_frame_id=='world'),
                             QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def spawn(package,binary,ros_args):
        executable=args.install/package/'lib'/package/binary
        log=args.output.with_name(args.output.stem+'-'+binary+'.log').open('w');logs.append(log)
        child=subprocess.Popen([str(executable),'--ros-args','-p','use_sim_time:=true',*ros_args],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        children.append(child);return child
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
    started=time.monotonic()
    try:
        source=spawn('super_lio','super_lio_node',['--params-file',str(args.repo/'src/bringup/config/super_lio_vehicle.yaml')])
        spawn('super_lio_vehicle_adapter','super_lio_vehicle_adapter',['--params-file',str(args.repo/'src/bringup/config/super_lio_reference.yaml')])
        spawn('super_lio_vehicle_adapter','super_lio_cloud_frame_adapter',['--params-file',str(args.repo/'src/bringup/config/super_lio_cloud_frame.yaml')])
        if args.ground_pipeline:
            spawn('ego_planner','research_tf_guard',['-p','protect_world_gauge:=true'])
            for package,binary,config in (
                ('active_road_mapping','active_road_map','active_road_mapping.yaml'),
                ('active_road_mapping','active_road_evidence','active_road_evidence.yaml'),
                ('research_runtime','research_observed_ground','research_observed_ground.yaml'),
                ('research_runtime','research_local_obstacle_grid','research_local_grid.yaml')):
                extra=['-p','evidence_store_path:='+str(evidence_path)] if package=='active_road_mapping' else []
                spawn(package,binary,['--params-file',str(args.repo/'src/bringup/config'/config),
                    '-p','localization_session_id:='+session,*extra])
        spin(2.)
        log=args.output.with_suffix('.play.log').open('w');logs.append(log)
        player=subprocess.Popen(['ros2','bag','play',str(args.bag),'--rate','1.0','--clock','100',
                                 '--topics','/livox/lidar','/livox/imu'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        play_started=time.monotonic();sample_at=play_started
        deadline=play_started+(args.duration_s if args.duration_s else contract['wall_budget_s'])
        while player.poll() is None and all(child.poll() is None for child in children) and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.01)
            if time.monotonic()>=sample_at:
                stat=Path(f'/proc/{source.pid}/stat').read_text().split()
                status=Path(f'/proc/{source.pid}/status').read_text().splitlines()
                resources.append({'elapsed_s':time.monotonic()-play_started,'cpu_s':(int(stat[13])+int(stat[14]))/os.sysconf('SC_CLK_TCK'),
                                  'rss_kb':int(next(line.split()[1] for line in status if line.startswith('VmRSS:')))})
                sample_at+=1.
        playback_exit=player.poll();child_exits=[child.poll() for child in children]
        prefix_completed=requested_prefix_completed(args.duration_s,deadline,time.monotonic(),playback_exit,child_exits)
        # Stop a requested prefix before draining; do not call it an EOF replay.
        if player.poll() is None:os.killpg(player.pid,signal.SIGINT);player.wait(timeout=5)
        spin(2.)
        matched=sorted(set(native)&set(vehicle)&set(certificates))
        mismatches=[]
        for key in matched:
            a,b=native[key],vehicle[key]
            av=(a.pose.pose.position.x,a.pose.pose.position.y,a.pose.pose.position.z,a.twist.twist.linear.x,a.twist.twist.linear.y,a.twist.twist.linear.z,
                a.twist.twist.angular.x,a.twist.twist.angular.y,a.twist.twist.angular.z,*a.pose.covariance,*a.twist.covariance)
            bv=(b.pose.pose.position.x,b.pose.pose.position.y,b.pose.pose.position.z,b.twist.twist.linear.x,b.twist.twist.linear.y,b.twist.twist.linear.z,
                b.twist.twist.angular.x,b.twist.twist.angular.y,b.twist.twist.angular.z,*b.pose.covariance,*b.twist.covariance)
            if max(abs(x-y) for x,y in zip(av,bv))>1e-8:mismatches.append(key)
            qa,qb=a.pose.pose.orientation,b.pose.pose.orientation
            quaternions=((qa.x,qa.y,qa.z,qa.w),(qb.x,qb.y,qb.z,qb.w))
            if quaternion_distance(*quaternions)>1e-6:mismatches.append(key)
        continuity=stream_continuity(raw_lidar_stamps,raw_imu_stamps,native,vehicle)
        frame_ok=all(msg.header.frame_id=='world' and msg.child_frame_id=='imu' for msg in native.values()) and all(
            msg.header.frame_id=='odom' and msg.child_frame_id=='base_footprint' for msg in vehicle.values())
        identity=lambda t:t.header.frame_id=='odom' and abs(t.transform.translation.x)+abs(t.transform.translation.y)+abs(t.transform.translation.z)<1e-9 and (
            abs(t.transform.rotation.x)+abs(t.transform.rotation.y)+abs(t.transform.rotation.z)<1e-9 and abs(abs(t.transform.rotation.w)-1)<1e-9)
        health_labels=Counter()
        for value,count in vehicle_health.items():health_labels[value.split(':',1)[0]]+=count
        checks={'all_children_alive':all(value is None for value in child_exits),
                'EOF_or_requested_prefix':playback_exit==0 or prefix_completed,
                'actual_raw_inputs':counts['imu']>100 and counts['lidar']>10,'source_observations':counts['native_odom']>50,
                'native_certificate_decode':counts['invalid_native_certificate']==0,'healthy_observations_exist':health['OK']>0,
                'native_vehicle_exact_stamp_pairs':len(matched)>50 and len(matched)==len(vehicle),
                'sustained_vehicle_odom_coverage':len(vehicle)>=.95*len(native),
                'terminal_vehicle_measurement_present':bool(vehicle) and max(vehicle)==max(native),
                'no_reference_fault_latched':not fault_capture,
                'identity_reference_mean_covariance':not mismatches,'finite_PSD_covariances':not covariance_failures,
                'frames':frame_ok,'owned_identity_world':bool(world_edges) and all(map(identity,world_edges)),
                'acquisition_transformed_cloud':counts['cloud_odom']>10 and counts['wrong_cloud_frame']==0,
                'vehicle_healthy_received':health_labels['OK']>0,
                'source_terminal_health_OK':bool(last_source_health) and last_source_health['status']=='OK',
                'vehicle_terminal_health_OK':bool(last_vehicle_health) and last_vehicle_health.split(':',1)[0]=='OK',
                **continuity['checks']}
        ground_result=None
        if args.ground_pipeline:
            from research_runtime.active_road import EvidenceStore
            store=EvidenceStore.load(evidence_path) if evidence_path.exists() else None
            records=store.evidence() if store else []
            support_keys={item['stamp_ns'] for item in ground_grids if item['free']>0}
            ground_checks=dict(local_perception_health_exists=any(local_frames),
                global_motion_TF_never_granted=bool(full_frames) and not any(full_frames),
                actual_ground_acquisitions=len(ground_acquisitions)>50,
                atomic_grid_identity=bool(ground_grids and combined_grids) and all(
                    item['headers_match'] and item['session']==session and item['version']!='UNKNOWN'
                    for item in [*ground_grids,*combined_grids]),
                ground_measurements_match_source_certificate=all(key in certificates and certificates[key]['eligible'] for key in ground_acquisitions),
                positive_support_measurements_match_acquisition=all(key in ground_acquisitions for key in support_keys),
                wall_EOF_expiry_retracts_ground=bool(ground_grids) and ground_grids[-1]['free']==0,
                local_store_initialized=store is not None and store.transform.crs=='LOCAL:odom',
                persistence_acknowledged=all(msg.evidence_id in {record.evidence_id for record in records} for msg in road_evidence.values()),
                persisted_geometry_stays_local=store is not None and not store.submap_anchors and
                    all(store.geometry_in_map(record.evidence_id) is None for record in records),
                bounded_geometric_evidence=all(record.state.value=='OBSERVED_GEOMETRY' and
                    record.local_submap_id.startswith(session+'/') and record.supported_width_m>0 and
                    record.valid_depth_m and record.observed_length_m>0 for record in records))
            ground_result=dict(status='PASS' if all(ground_checks.values()) else 'FAIL',checks=ground_checks,
                actual_ground_acquisitions=len(ground_acquisitions),ground_messages=len(ground_grids),combined_messages=len(combined_grids),
                acquisitions_with_positive_support=len(support_keys),maximum_supported_cells=max((item['free'] for item in ground_grids),default=0),
                positive_support_raw_status='PASS' if support_keys else 'FAIL',
                persisted_geometry_count=len(records),measured_question_count=len(set(questions)),
                road_evidence_source_status='PASS' if records else 'NOT_RUN',
                evidence_store=str(evidence_path),global_TF_true_count=sum(full_frames),local_TF_true_count=sum(local_frames),
                scope='Actual raw sensors and compiled local geometry/persistence, no injected odom/ground/RTK; geometry is not a semantic road or terrain/motion acceptance')
            checks['actual_ground_pipeline']=ground_result['status']=='PASS'
        executable=args.install/'super_lio/lib/super_lio/super_lio_node'
        result={'status':'PASS' if all(checks.values()) else 'FAIL','checks':checks,'counts':dict(counts),
                'source_health_counts':dict(health),'source_reasons':dict(reasons),'vehicle_health_counts':dict(health_labels),
                'vehicle_health_reasons':dict(vehicle_health),'reference_fault_capture':fault_capture,
                'measurement_continuity':continuity,'terminal_source_health':last_source_health,'terminal_vehicle_health':last_vehicle_health,
                'information_bound_summary':{'min':min(bounds),'median':statistics.median(bounds),'max':max(bounds)} if bounds else None,
                'paired_odom_count':len(matched),'covariance_failures':covariance_failures,'reference_mismatches':mismatches,
                'playback_exit_before_cleanup':playback_exit,'child_exits_before_cleanup':child_exits,'duration_s':args.duration_s,
                'requested_prefix_completed_before_cleanup':prefix_completed,
                'replay_scope':'PREFIX' if args.duration_s else 'FULL_EOF','rate':1.,'domain':104,'bag':str(args.bag),
                'topics':['/livox/lidar','/livox/imu'],'replay_contract':contract,'resources':resources,
                'runtime_source_commit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=args.repo,text=True).strip(),
                'source_binary_sha256':hashlib.sha256(executable.read_bytes()).hexdigest(),
                'input_hashes':{str(path.relative_to(args.repo)):hashlib.sha256(path.read_bytes()).hexdigest() for path in (
                    args.repo/'patches/super_lio/0002-certify-source-observations-and-covariance.patch',
                    args.repo/'src/bringup/config/super_lio_vehicle.yaml',args.repo/'src/bringup/config/super_lio_reference.yaml',
                    args.repo/'src/bringup/config/super_lio_cloud_frame.yaml')},
                'ground_pipeline':ground_result,
                'wall_elapsed_s':time.monotonic()-started,
                'scope':'Actual current Super/native health/body covariance/reference/cloud on raw sensors; no actuator, FAST ground truth, physical calibration or policy benefit claim'}
    finally:
        for child in [player,*reversed(children)]:
            if child is None:continue
            if child.poll() is None:os.killpg(child.pid,signal.SIGINT)
            try:child.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=5)
        for log in logs:log.close()
        node.destroy_node();rclpy.shutdown()
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({key:value for key,value in result.items() if key!='resources'},indent=2))
    return 0 if result['status']=='PASS' else 1


if __name__=='__main__':raise SystemExit(main())
