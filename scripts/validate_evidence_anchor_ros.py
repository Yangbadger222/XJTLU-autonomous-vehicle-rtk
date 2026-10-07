#!/usr/bin/env python3
"""Actual typed evidence persistence, native TF guard and RTK recovery probe.

Analytical transforms/support are test fixtures, never vehicle calibration.
"""
import argparse,json,math,os,signal,subprocess,time
from pathlib import Path
import rclpy
from std_msgs.msg import Bool,String
from geometry_msgs.msg import Point,TransformStamped
from tf2_msgs.msg import TFMessage
from research_interfaces.msg import RoadEvidence2D,RoadGraphUpdate2D
from research_runtime.active_road import EvidenceStore,GeoTransform


def main():
    p=argparse.ArgumentParser();p.add_argument('--install',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='96' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':raise SystemExit('isolated domain 96 required')
    a.output.parent.mkdir(parents=True,exist_ok=True);store_path=a.output.with_suffix('.store.json')
    EvidenceStore(GeoTransform('LOCAL_ODOM_SUBMAP','local',0,0,.3,.3),'analytical-prior').save(store_path)
    rclpy.init();node=rclpy.create_node('analytical_anchor_probe')
    pubs={'authority':node.create_publisher(Bool,'/localization_authority/motion_allowed',10),
          'mode':node.create_publisher(String,'/localization_authority/mode',10),
          'tf':node.create_publisher(TFMessage,'/tf',10),
          'evidence':node.create_publisher(RoadEvidence2D,'/research/road_evidence',10),
          'update':node.create_publisher(RoadGraphUpdate2D,'/research/road_graph_updates',10)}
    children=[];logs=[];checks=[]
    def spawn(package,binary,args=[]):
        log=a.output.with_name(binary+'.log').open('w');logs.append(log)
        child=subprocess.Popen([str(a.install/package/'lib'/package/binary),'--ros-args']+args,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        children.append(child);return child
    def phase(allowed,translation,seconds):
        until=time.monotonic()+seconds
        while time.monotonic()<until:
            stamp=node.get_clock().now();pubs['authority'].publish(Bool(data=allowed))
            pubs['mode'].publish(String(data='RTK_AUTHORITATIVE' if allowed else 'LIO_BRIDGE'))
            transforms=[]
            for parent,child in [('map','odom'),('odom','base_footprint')]:
                t=TransformStamped();t.header.frame_id=parent;t.child_frame_id=child
                t.header.stamp=(stamp+rclpy.duration.Duration(seconds=.10 if child=='odom' else 0)).to_msg();t.transform.rotation.w=1.
                if child=='odom':
                    t.transform.translation.x,t.transform.translation.y=translation
                    t.transform.rotation.z=t.transform.rotation.w=math.sqrt(.5)
                transforms.append(t)
            pubs['tf'].publish(TFMessage(transforms=transforms));rclpy.spin_once(node,timeout_sec=.02);time.sleep(.025)
    def checked(name,passed,detail=None):checks.append({'case':name,'status':'PASS' if passed else 'FAIL','detail':detail})
    try:
        spawn('ego_planner','research_tf_guard')
        args=['-p','evidence_store_path:='+str(store_path),'-p','global_anchor_uncertainty_verified:=true','-p','global_anchor_uncertainty_m:=0.03',
              '-p','localization_session_id:=session-1']
        writer=spawn('active_road_mapping','active_road_evidence',args)
        phase(True,(10.,20.),1.5)
        e=RoadEvidence2D();e.header.frame_id='odom';e.header.stamp=node.get_clock().now().to_msg()
        e.evidence_id='measured-fixture-uuid';e.source='analytical_depth_fixture';e.local_submap_id='session-1/s1';e.pose_uncertainty_m=.03
        e.state='OBSERVED_GEOMETRY';e.observed_length_m=1.;e.geometry=[Point(x=0.,y=0.),Point(x=1.,y=0.)]
        pubs['evidence'].publish(e);phase(True,(10.,20.),.7)
        u=RoadGraphUpdate2D();u.header=e.header;u.header.stamp=node.get_clock().now().to_msg()
        u.update_id='gap:a:b';u.prior_version='analytical-prior';u.start_node_id='a';u.end_node_id='b'
        u.geometry=e.geometry;u.supported_width_m=.61;u.local_submap_id='session-1/s1';u.pose_uncertainty_m=.03
        u.source='supported_ground_fixture';u.evidence_ids=[e.evidence_id]
        pubs['update'].publish(u);phase(True,(10.,20.),.6)
        store=EvidenceStore.load(store_path);geometry=store.graph_updates_in_map()
        checked('typed_increment_and_90_degree_nonzero_anchor',len(geometry)==1 and math.dist(geometry[0]['geometry_xy'][1],(10.,21.))<1e-8,geometry)
        version=store.map_version
        pubs['evidence'].publish(e);pubs['update'].publish(u);phase(True,(10.,20.),.2)
        checked('same_uuid_replay_idempotent',EvidenceStore.load(store_path).map_version==version)
        phase(False,(10.,20.),.7);store=EvidenceStore.load(store_path)
        checked('rtk_loss_local_evidence_retained_global_edges_withheld',len(store.evidence())==1 and len(store.graph_updates)==1 and not store.graph_updates_in_map())
        phase(True,(11.,20.),.8);store=EvidenceStore.load(store_path);geometry=store.graph_updates_in_map()
        checked('rtk_recovery_reanchors_without_duplicate',len(geometry)==1 and math.dist(geometry[0]['geometry_xy'][1],(11.,21.))<1e-8 and len(store.evidence())==1,geometry)
        os.killpg(writer.pid,signal.SIGINT);writer.wait(timeout=5)
        writer=spawn('active_road_mapping','active_road_evidence',args);phase(True,(11.,20.),2.)
        loaded=EvidenceStore.load(store_path)
        checked('second_process_reuses_persisted_uuid_and_increment',len(loaded.evidence())==1 and len(loaded.graph_updates_in_map())==1)
        historical=loaded.geometry_in_map(e.evidence_id)
        os.killpg(writer.pid,signal.SIGINT);writer.wait(timeout=5)
        args[-1]='localization_session_id:=session-2'
        spawn('active_road_mapping','active_road_evidence',args);phase(True,(100.,200.),2.)
        e.evidence_id='new-session-uuid';e.local_submap_id='session-2/s1';e.header.stamp=node.get_clock().now().to_msg()
        pubs['evidence'].publish(e);phase(True,(100.,200.),.7)
        loaded=EvidenceStore.load(store_path)
        checked('new_odom_origin_never_reanchors_historical_session',loaded.geometry_in_map('measured-fixture-uuid')==historical and
            math.dist(loaded.geometry_in_map('new-session-uuid')[0],(100.,200.))<1e-8,
            {'historical':loaded.geometry_in_map('measured-fixture-uuid'),'new':loaded.geometry_in_map('new-session-uuid')})
        phase(False,(100.,200.),.7);loaded=EvidenceStore.load(store_path)
        checked('authority_loss_marks_only_current_session_stale',loaded.geometry_in_map('measured-fixture-uuid')==historical and
            loaded.geometry_in_map('new-session-uuid') is None)
        result={'status':'PASS' if all(c['status']=='PASS' for c in checks) else 'FAIL','checks':checks,
            'scope':'actual Humble typed writer/native TF guard/persistence; analytical support and RTK registration fixtures only'}
    finally:
        for child in reversed(children):
            if child.poll() is None:os.killpg(child.pid,signal.SIGINT)
            child.wait(timeout=5)
        for log in logs:log.close()
        node.destroy_node();rclpy.shutdown()
    a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result));return 0 if result['status']=='PASS' else 1
if __name__=='__main__':raise SystemExit(main())
