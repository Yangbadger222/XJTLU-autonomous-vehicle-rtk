#!/usr/bin/env python3
"""Actual observer/writer regression for historical snapshot plus new LIO epoch."""
import argparse,hashlib,json,math,os,signal,subprocess,time
from pathlib import Path
import rclpy
from geometry_msgs.msg import Point,TransformStamped
from nav_msgs.msg import Odometry,OccupancyGrid
from std_msgs.msg import Bool,String
from tf2_msgs.msg import TFMessage
from research_interfaces.msg import RoadEvidence2D
from research_runtime.active_road import EvidenceStore,GeoTransform,RoadEvidence,EvidenceState
from research_runtime.active_observation import FiniteObservationPolicy,save_snapshot
from research_runtime.prior_mission import registered_prior
from active_road_mapping.observation_node import ActiveObservationNode
from validate_restricted_policy_ros import asset_fixtures

def main():
    p=argparse.ArgumentParser();p.add_argument('--install',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='98' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':raise SystemExit('isolated domain 98 required')
    a.output.parent.mkdir(parents=True,exist_ok=True);manifest=asset_fixtures(a.output.parent/'observer-session-prior')
    prior,_,_,_=registered_prior(manifest);store_path=a.output.with_suffix('.store.json');snapshot=a.output.with_suffix('.policy.json')
    old=RoadEvidence('old-uuid',[(0.,0.),(1.,0.)],EvidenceState.OBSERVED_GEOMETRY,time.time(),'analytical','old/s1',.03,1.)
    store=EvidenceStore(GeoTransform('LOCAL_ODOM_SUBMAP','local',0,0,.3,.3),hashlib.sha256(manifest.read_bytes()).hexdigest())
    store.add(old);store.anchor_submap('old/s1',(0.,0.,0.),stamp=time.time(),uncertainty_m=.03,authority_valid=True);store.save(store_path)
    save_snapshot(snapshot,prior,[old],FiniteObservationPolicy('PASSIVE'))
    args=['--ros-args','-p','prior_manifest_path:='+str(manifest),'-p','task_start_node:=start','-p','task_goal_node:=goal',
          '-p','policy:=PASSIVE','-p','policy_snapshot_path:='+str(snapshot),'-p','evidence_store_path:='+str(store_path),
          '-p','sensor_range_m:=1.8','-p','sensor_fov_rad:='+str(2*math.pi),'-p','max_curvature_1pm:=1.0']
    rclpy.init(args=args);observer=ActiveObservationNode();fixture=rclpy.create_node('observer_session_fixture')
    pubs={key:fixture.create_publisher(typ,topic,100) for key,typ,topic in [('odom',Odometry,'/lio/odom_vehicle'),
        ('grid',OccupancyGrid,'/research/local_obstacle_grid'),('permission',OccupancyGrid,'/research/permission_grid'),
        ('health',String,'/lio/vehicle_health'),('version',String,'/research/map_version'),('mode',String,'/localization_authority/mode'),
        ('authority',Bool,'/localization_authority/motion_allowed'),('tf',TFMessage,'/tf'),('evidence',RoadEvidence2D,'/research/road_evidence')]}
    children=[];logs=[]
    def spawn(package,binary,args):
        log=a.output.with_name(a.output.stem+'-'+binary+'.log').open('w');logs.append(log)
        children.append(subprocess.Popen([str(a.install/package/'lib'/package/binary),'--ros-args']+args,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
    try:
        spawn('ego_planner','research_tf_guard',[])
        spawn('active_road_mapping','active_road_evidence',['-p','evidence_store_path:='+str(store_path),'-p','localization_session_id:=new',
            '-p','global_anchor_uncertainty_verified:=true','-p','global_anchor_uncertainty_m:=0.03'])
        until=time.monotonic()+10
        evidence_stamp=fixture.get_clock().now().to_msg()
        while time.monotonic()<until:
            now=fixture.get_clock().now();stamp=now.to_msg()
            odom=Odometry();odom.header.stamp=stamp;odom.header.frame_id='odom';odom.child_frame_id='base_footprint';odom.pose.pose.orientation.w=1.
            odom.pose.covariance=[.0009 if i%7==0 else 0. for i in range(36)];pubs['odom'].publish(odom)
            grid=OccupancyGrid();grid.header.stamp=stamp;grid.header.frame_id='odom';grid.info.resolution=.3
            grid.info.width=grid.info.height=100;grid.info.origin.position.x=grid.info.origin.position.y=-15.;grid.info.origin.orientation.w=1.;grid.data=[0]*10000
            pubs['grid'].publish(grid);pubs['permission'].publish(grid)
            pubs['health'].publish(String(data='OK: ANALYTICAL_FIXTURE'))
            pubs['version'].publish(String(data=EvidenceStore.load(store_path).map_version))
            pubs['mode'].publish(String(data='RTK_AUTHORITATIVE'));pubs['authority'].publish(Bool(data=True))
            transforms=[]
            for parent,child in [('map','odom'),('odom','base_footprint')]:
                t=TransformStamped();t.header.frame_id=parent;t.child_frame_id=child;t.header.stamp=(now+rclpy.duration.Duration(seconds=.1 if child=='odom' else 0)).to_msg();t.transform.rotation.w=1.;transforms.append(t)
            pubs['tf'].publish(TFMessage(transforms=transforms))
            e=RoadEvidence2D();e.header.stamp=evidence_stamp;e.header.frame_id='odom';e.evidence_id='new-uuid';e.source='analytical'
            e.local_submap_id='new/s1';e.state='OBSERVED_GEOMETRY';e.pose_uncertainty_m=.03;e.observed_length_m=3.3
            e.geometry=[Point(x=0.,y=0.),Point(x=3.3,y=0.)];pubs['evidence'].publish(e)
            for _ in range(6):rclpy.spin_once(observer,timeout_sec=.001);rclpy.spin_once(fixture,timeout_sec=.001)
            time.sleep(.02)
        final=EvidenceStore.load(store_path);updates=list(final.graph_updates.values())
        passed=len(observer.recorded_evidence)==2 and len(updates)>0 and all(
            u['local_submap_id']=='new/s1' and u['evidence_ids']==['new-uuid'] for u in updates)
        result={'status':'PASS' if passed else 'FAIL','historical_snapshot_loaded':any(e.evidence_id=='old-uuid' for e in observer.recorded_evidence),
            'current_session_received':observer.localization_session_id if hasattr(observer,'localization_session_id') else None,
            'persisted_graph_updates':updates,'scope':'actual installed ROS observer plus sole writer; analytical support/TF only; no physical actuation'}
    finally:
        for child in reversed(children):
            if child.poll() is None:os.killpg(child.pid,signal.SIGINT)
            child.wait(timeout=5)
        for log in logs:log.close()
        observer.destroy_node();fixture.destroy_node();rclpy.shutdown()
    a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result));return 0 if passed else 1
if __name__=='__main__':raise SystemExit(main())
