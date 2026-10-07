#!/usr/bin/env python3
"""Actual EGO progress heartbeats on a captured, unchanged limited-view grid."""
import argparse,json,math,os,signal,subprocess,time
from pathlib import Path
import rclpy
from research_interfaces.msg import LocalEvidenceGrid2D
from nav_msgs.msg import Odometry,OccupancyGrid,Path as RosPath
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from research_interfaces.msg import TimedTrajectory2D


def main():
 p=argparse.ArgumentParser();p.add_argument('--repo',type=Path,required=True);p.add_argument('--install',type=Path,required=True)
 p.add_argument('--input',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
 if os.environ.get('ROS_DOMAIN_ID')!='91' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':raise SystemExit('isolated domain 91 required')
 raw=json.loads(a.input.read_text())[0];g=raw['grid'];s=raw['state'];samples=[];failures=[];fault_epoch=None
 a.output.parent.mkdir(parents=True,exist_ok=True);log=a.output.with_suffix('.node.log').open('w')
 rclpy.init();node=rclpy.create_node('actual_cache_progress_probe')
 def cb(m):
  if m.status==m.STATUS_OK:samples.append({'generated':m.generated_at.sec+m.generated_at.nanosec*1e-9,'stamp':m.header.stamp.sec+m.header.stamp.nanosec*1e-9})
  else:failures.append(m.header.stamp.sec+m.header.stamp.nanosec*1e-9)
 node.create_subscription(TimedTrajectory2D,'/research/ego_trajectory',cb,100)
 pubs=[node.create_publisher(typ,topic,10) for typ,topic in [(Odometry,'/lio/odom_vehicle'),(LocalEvidenceGrid2D,'/research/local_evidence_grid'),(String,'/research/map_version'),(RosPath,'/research/road_reference')]]
 child=subprocess.Popen([str(a.install/'ego_planner/lib/ego_planner/motion_plan'),'--ros-args','--params-file',str(a.repo/'src/bringup/config/ego_vehicle_adapter.yaml'),'-p','max_curvature_1pm:=1.0','-p','max_lateral_speed_mps:=0.05','-p','max_jerk_mps3:=3.0','-p','localization_session_id:=mock-only','-p','allow_analytical_grid_fixture:=true'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
 try:
  started=time.monotonic();until=started+6
  while time.monotonic()<until:
   stamp=node.get_clock().now().to_msg();o=Odometry();o.header.frame_id='odom';o.header.stamp=stamp;o.child_frame_id='base_footprint'
   o.pose.pose.position.x,o.pose.pose.position.y=s[:2];o.pose.pose.orientation.z=math.sin(s[2]/2);o.pose.pose.orientation.w=math.cos(s[2]/2)
   o.twist.twist.linear.x,o.twist.twist.angular.z=s[3:5]
   m=OccupancyGrid();m.header.frame_id='odom';m.header.stamp=stamp;m.info.resolution=g['resolution'];m.info.width=g['width'];m.info.height=g['height']
   m.info.origin.position.x,m.info.origin.position.y=g['origin'];m.info.origin.orientation.w=1.;m.data=g['cells']
   if time.monotonic()-started>5:
    fault_epoch=fault_epoch or node.get_clock().now().nanoseconds*1e-9
    m.data=[100]*len(g['cells'])
   ref=RosPath();ref.header.frame_id='odom';ref.header.stamp=stamp
   for x,y in raw['reference']:
    pose=PoseStamped();pose.pose.position.x,pose.pose.position.y=x,y;pose.pose.orientation.w=1.;ref.poses.append(pose)
   for pub,msg in zip(pubs,[o,LocalEvidenceGrid2D(header=m.header,grid=m,map_version='captured-cache',localization_session_id='mock-only',support_model='analytical_fixture_v1'),String(data='captured-cache'),ref]):pub.publish(msg)
   rclpy.spin_once(node,timeout_sec=.02);time.sleep(.01)
  normal=[m for m in samples if fault_epoch is None or m['stamp']<fault_epoch]
  ages=[m['stamp']-m['generated'] for m in normal]
  repeats=len(samples)-len({m['generated'] for m in samples})
  progress=bool(ages and max(ages)>.4 and repeats>5)
  obstacle=bool(fault_epoch and any(t>fault_epoch+.4 for t in failures) and not any(m['stamp']>fault_epoch+.4 for m in samples))
  result={'status':'PASS' if progress and obstacle else 'FAIL','checks':[{'case':'unchanged_partial_map_preserves_timed_execution_progress','status':'PASS' if progress else 'FAIL'},
    {'case':'new_occupied_grid_invalidates_cached_trajectory','status':'PASS' if obstacle else 'FAIL'}],
   'ok_count':len(samples),'unchanged_map_and_goal_before_fault':True,'max_execution_age_s':max(ages) if ages else None,'reused_generation_heartbeats':repeats,'samples':samples,
   'scope':'actual EGO with captured odom/grid/road; no actuator; checks time progress under unchanged partial-map geometry'}
 finally:
  if child.poll() is None:os.killpg(child.pid,signal.SIGINT)
  child.wait(timeout=5);log.close();node.destroy_node();rclpy.shutdown()
 a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k!='samples'}));return 0 if result['status']=='PASS' else 1
if __name__=='__main__':raise SystemExit(main())
