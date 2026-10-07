#!/usr/bin/env python3
"""Replay captured native CDR through the actual adapter callback, no ROS graph.

This isolates a coordinate/reference fault. Captured parsed certificate
decisions are reused, without publishing synthetic healthy or granting motion.
"""
import argparse
import base64
import json
import math
from pathlib import Path
import time
from types import SimpleNamespace

from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from super_lio_vehicle_adapter.adapter_node import SuperLioVehicleAdapter


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--capture',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    capture=json.loads(args.capture.read_text())['reference_fault_capture'][0]
    node=object.__new__(SuperLioVehicleAdapter)
    for key,value in dict(_legacy_reference=True,_verified=False,_own_gauge=True,_source_frame='world',
            _source_child_frame='imu',_target_frame='odom',_translation=(0.,0.,0.),_rotation=(0.,0.,0.,1.),
            _require_source_health_ok=True,_require_covariance=True,_publish_unhealthy=True,_reference_fault='',
            _last_source_stamp=None,_last_source_pose=None,_pending_odom=None).items():setattr(node,key,value)
    outputs=[];health=[];steps=[]
    node._odom=SimpleNamespace(publish=outputs.append)
    node._vehicle_tf=SimpleNamespace(sendTransform=lambda msg:None)
    node._publish_health=health.append
    node.get_parameter=lambda key:SimpleNamespace(value={'base_frame':'base_footprint'}[key])
    previous=None
    for encoded,certificate in zip(capture['native_odometry_cdr_base64'],capture['certificates']):
        if certificate is None:raise RuntimeError('capture missing an actual parsed native certificate')
        msg=deserialize_message(base64.b64decode(encoded),Odometry)
        node._source_certificate_stamp=certificate['stamp_ns'];node._source_health_ok=certificate['eligible']
        node._source_health_received=time.monotonic()
        node._callback(msg)
        if previous:
            a,b=previous.pose.pose,msg.pose.pose
            yaw=lambda p:math.atan2(2*(p.orientation.w*p.orientation.z+p.orientation.x*p.orientation.y),
                1-2*(p.orientation.y**2+p.orientation.z**2))
            steps.append({'dt_s':(certificate['stamp_ns']-(previous.header.stamp.sec*10**9+previous.header.stamp.nanosec))*1e-9,
                'translation_m':math.hypot(a.position.x-b.position.x,a.position.y-b.position.y),
                'yaw_degrees':math.degrees(abs(math.atan2(math.sin(yaw(b)-yaw(a)),math.cos(yaw(b)-yaw(a))))),
                'native_body_yaw_rates_rps':[previous.twist.twist.angular.z,msg.twist.twist.angular.z],
                'positions':[[a.position.x,a.position.y,a.position.z],[b.position.x,b.position.y,b.position.z]]})
        previous=msg
    result={'status':'FAIL' if node._reference_fault else 'PASS','callback_fault':node._reference_fault,
        'native_inputs':len(capture['native_odometry_cdr_base64']),'vehicle_outputs':len(outputs),
        'measured_native_steps':steps,'health':health,
        'scope':'Captured native odometry and parsed certificates; actual installed adapter callback, no graph/actuator or modified guard threshold'}
    args.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
    return 1 if node._reference_fault else 0


if __name__=='__main__':raise SystemExit(main())
