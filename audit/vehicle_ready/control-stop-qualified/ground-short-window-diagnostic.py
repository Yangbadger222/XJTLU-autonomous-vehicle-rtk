#!/usr/bin/env python3
import json, math, signal, time
from collections import OrderedDict, deque, Counter
from pathlib import Path
import numpy as np
import rclpy
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from research_runtime.observed_ground import project_observed_ground,expected_ground_z
from research_runtime.physical_parameter_lock import MID360_GROUND_REFERENCE
from research_runtime.local_road_evidence import supported_strips
from std_msgs.msg import String
rclpy.init()
node=rclpy.create_node('ground_density_window_diagnostic',parameter_overrides=[Parameter('use_sim_time',value=True)])
poses=OrderedDict(); history=deque(); rows=[]; count=0; fault=0
def stamp(m): return m.header.stamp.sec*1000000000+m.header.stamp.nanosec
def odom(m):
 poses[stamp(m)]=m
 while len(poses)>100: poses.popitem(last=False)
def cloud(m):
 global count,fault
 k=stamp(m)
 if k not in poses: fault+=1;return
 pose=poses[k].pose.pose;p,q=pose.position,pose.orientation
 try:
  floor=expected_ground_z((p.x,p.y,p.z),(q.x,q.y,q.z,q.w),MID360_GROUND_REFERENCE['lidar_in_imu_m'],MID360_GROUND_REFERENCE['lidar_height_m'])
  pts=np.asarray([tuple(float(v) for v in pt) for pt in point_cloud2.read_points(m,field_names=('x','y','z'),skip_nans=False)],dtype=float).reshape((-1,3))
  if history and k<=history[-1][0]:history.clear()
  history.append((k,floor,pts))
  while history and k-history[0][0]>400000000:history.popleft()
  count+=1
  if count%10:return
  result={'stamp_ns':k,'input_points':len(pts),'windows':[]}
  ox=(math.floor(p.x/.3)-50)*.3;oy=(math.floor(p.y/.3)-50)*.3
  for win in [0,200000000,400000000]:
   inputs=[x for x in history if k-x[0]<=win and abs(x[1]-floor)<=.025]
   merged=np.concatenate([x[2] for x in inputs],axis=0)
   grid,stats=project_observed_ground(merged,map_version='diagnostic-only',resolution_m=.3,origin_x_m=ox,origin_y_m=oy,width=100,height=100,floor_z_m=floor)
   result['windows'].append({'horizon_s':win/1e9,'clouds':len(inputs),'oldest_actual_stamp_ns':inputs[0][0],'stats':stats,'strips':len(supported_strips(grid,session='diagnostic-only',minimum_width_m=.61))})
  rows.append(result)
 except Exception as e:
  fault+=1; rows.append({'error':str(e),'stamp_ns':k})
node.create_subscription(Odometry,'/lio/odom_vehicle',odom,100)
node.create_subscription(PointCloud2,'/research/ground_observation_cloud',cloud,100)
done=False
def stop(*_):
 global done
 done=True
signal.signal(signal.SIGINT,stop)
signal.signal(signal.SIGTERM,stop)
end=time.monotonic()+50
while not done and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
payload={'scope':'Read-only diagnostic of actual compiled ground clouds and exact-acquisition odom. Existing geometry thresholds retained; temporal union NOT used for permission, evidence, control or acceptance. At most .4s history with .025m floor-reference agreement.','received_clouds':count,'matching_or_processing_faults':fault,'rows':rows}
Path('/home/badger/codex-research/superlio-ego-20261007/logs/stop-qualified/ground-short-window-diagnostic.json').write_text(json.dumps(payload,indent=2)+'\n')
node.destroy_node()
rclpy.shutdown()
