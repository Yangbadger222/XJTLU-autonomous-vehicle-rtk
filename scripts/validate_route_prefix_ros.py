#!/usr/bin/env python3
"""Replay measured failed state/grid against the unchanged actual EGO core.

Inputs are captured restricted-sensor measurements, not a truth map. The
reference fix only shortens to the existing footprint circle and never frees
unknown cells or changes limits.
"""
import argparse,json,math,os,signal,subprocess,time
from pathlib import Path
import rclpy
from nav_msgs.msg import Odometry,OccupancyGrid,Path as RosPath
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from research_interfaces.srv import PlanRoadReference
from research_runtime.active_observation import RoadGraph,GraphEdge
from research_runtime.prior_mission import confirmed_route_prefix
from research_runtime.grid_map import LocalObstacleGrid
from research_runtime.physical_parameter_lock import LOCKED_FOOTPRINT


def main():
 p=argparse.ArgumentParser();p.add_argument('--repo',type=Path,required=True);p.add_argument('--install',type=Path,required=True)
 p.add_argument('--input',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
 if os.environ.get('ROS_DOMAIN_ID')!='91' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':raise SystemExit('isolated domain 91 required')
 raw=json.loads(a.input.read_text())[0];g=raw['grid'];s=raw['state'];old=raw['reference']
 grid=LocalObstacleGrid('odom','captured',g['resolution'],*g['origin'],g['width'],g['height'],tuple(g['cells']))
 graph=RoadGraph([GraphEdge('measured-prefix','start','goal',(tuple(old[0]),tuple(old[-1])))])
 fixed,reason=confirmed_route_prefix(graph,'start','goal',s[:3],grid,LOCKED_FOOTPRINT)
 a.output.parent.mkdir(parents=True,exist_ok=True);log=a.output.with_suffix('.node.log').open('w')
 rclpy.init();node=rclpy.create_node('captured_prefix_regression');client=node.create_client(PlanRoadReference,'/research/ego_plan_query')
 pubs=[node.create_publisher(typ,topic,10) for typ,topic in [(Odometry,'/lio/odom_vehicle'),(OccupancyGrid,'/research/local_obstacle_grid'),(String,'/research/map_version')]]
 child=subprocess.Popen([str(a.install/'ego_planner/lib/ego_planner/motion_plan'),'--ros-args','--params-file',str(a.repo/'src/bringup/config/ego_vehicle_adapter.yaml'),'-p','max_curvature_1pm:=1.0','-p','max_lateral_speed_mps:=0.05','-p','max_jerk_mps3:=3.0'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
 def feed():
  stamp=node.get_clock().now().to_msg();o=Odometry();o.header.frame_id='odom';o.header.stamp=stamp;o.child_frame_id='base_footprint'
  o.pose.pose.position.x,o.pose.pose.position.y=s[:2];o.pose.pose.orientation.z=math.sin(s[2]/2);o.pose.pose.orientation.w=math.cos(s[2]/2)
  o.twist.twist.linear.x,o.twist.twist.angular.z=s[3:5]
  m=OccupancyGrid();m.header.frame_id='odom';m.header.stamp=stamp;m.info.resolution=g['resolution'];m.info.width=g['width'];m.info.height=g['height']
  m.info.origin.position.x,m.info.origin.position.y=g['origin'];m.info.origin.orientation.w=1.;m.data=g['cells']
  pubs[0].publish(o);pubs[1].publish(m);pubs[2].publish(String(data='captured'))
 def query(points):
  request=PlanRoadReference.Request();request.request_id='captured-prefix';request.map_version='captured';request.road_reference.header.frame_id='odom';request.road_reference.header.stamp=node.get_clock().now().to_msg()
  for x,y in points:
   pose=PoseStamped();pose.pose.position.x,pose.pose.position.y=x,y;pose.pose.orientation.w=1.;request.road_reference.poses.append(pose)
  future=client.call_async(request);deadline=time.monotonic()+3
  while not future.done() and time.monotonic()<deadline:feed();rclpy.spin_once(node,timeout_sec=.02)
  if not future.done():raise RuntimeError('planner query timeout')
  r=future.result().trajectory
  return {'feasible':r.status==r.STATUS_OK,'reason':r.failure_reason,'cost_s':r.points[-1].t if r.points else None}
 try:
  until=time.monotonic()+2
  while time.monotonic()<until:feed();rclpy.spin_once(node,timeout_sec=.025)
  assert client.wait_for_service(timeout_sec=2)
  before=query(old);after=query(fixed)
  result={'status':'PASS' if not before['feasible'] and after['feasible'] else 'FAIL','before':before,'after':after,'old_endpoint':old[-1],'fixed_endpoint':fixed[-1],
   'prefix_reason':reason,'grid_unchanged':True,'scope':'actual unchanged EGO query with captured measured odom/grid; conservative reference truncation only; no physical motion'}
 finally:
  if child.poll() is None:os.killpg(child.pid,signal.SIGINT)
  child.wait(timeout=5);log.close();node.destroy_node();rclpy.shutdown()
 a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result));return 0 if result['status']=='PASS' else 1
if __name__=='__main__':raise SystemExit(main())
