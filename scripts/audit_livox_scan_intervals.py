#!/usr/bin/env python3
"""Read-only MID360 packet/time audit using the exact protected filter profile."""
import argparse
import json
from pathlib import Path
import sqlite3
import statistics

from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu
from livox_ros_driver2.msg import CustomMsg
import yaml


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--bag',type=Path,required=True);parser.add_argument('--repo',type=Path,required=True)
    parser.add_argument('--fault-capture',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    config=yaml.safe_load((args.repo/'src/bringup/config/super_lio_vehicle.yaml').read_text())['/**']['ros__parameters']
    stride=config['lio.sensor.filter_rate'];blind2=config['lio.sensor.blind']**2;range2=config['lio.sensor.maxrange']**2
    fault=json.loads(args.fault_capture.read_text())['reference_fault_capture'][0]
    center=fault['last_vehicle_stamp_ns'];scans=[];gyro=[];offset_gaps=[];earlier_headers=0;previous=None
    info=yaml.safe_load((args.bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    for name in info['relative_file_paths']:
        with sqlite3.connect((args.bag/name).resolve().as_uri()+'?mode=ro',uri=True) as db:
            for topic,blob in db.execute("SELECT t.name,m.data FROM messages m JOIN topics t ON m.topic_id=t.id WHERE t.name IN ('/livox/lidar','/livox/imu') ORDER BY m.timestamp,m.id"):
                if topic=='/livox/imu':
                    msg=deserialize_message(blob,Imu);stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
                    if center-10**9<=stamp<=center+10**9:gyro.append({'stamp_ns':stamp,'gyro_z_rps':msg.angular_velocity.z})
                    continue
                msg=deserialize_message(blob,CustomMsg);stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
                offsets=[]
                for point in msg.points[::stride]:
                    distance=point.x**2+point.y**2+point.z**2
                    if point.tag&0x30 in (0,0x10) and blind2<distance<range2:offsets.append(point.offset_time)
                if not offsets:continue
                gap=max(offsets)-offsets[-1];offset_gaps.append(gap)
                before=previous is not None and stamp+min(offsets)<previous
                earlier_headers+=int(before);previous=stamp+max(offsets)
                if center-10**9<=stamp<=center+10**9:scans.append({'header_stamp_ns':stamp,'accepted_points':len(offsets),
                    'min_offset_ns':min(offsets),'last_offset_ns':offsets[-1],'max_offset_ns':max(offsets),
                    'max_minus_last_ns':gap,'begins_before_previous_max_end':before})
    result={'scope':'actual read-only sealed raw CDR; protected MID filter; distinguishes unordered packet end and old-bag turn dynamics, no clock rewrite or motion',
        'bag':str(args.bag),'scan_count':len(offset_gaps),'last_not_max_over_1us_count':sum(gap>1000 for gap in offset_gaps),
        'max_minus_last_ns':{'max':max(offset_gaps),'median':statistics.median(offset_gaps)},
        'minimum_point_before_previous_max_end_count':earlier_headers,'fault_window_scans':scans,
        'fault_window_gyro':{'count':len(gyro),'median_z_rps':statistics.median(g['gyro_z_rps'] for g in gyro),
                             'max_abs_z_rps':max(abs(g['gyro_z_rps']) for g in gyro)},
        'filter':{'stride':stride,'blind2':blind2,'range2':range2}}
    args.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2));return 0


if __name__=='__main__':raise SystemExit(main())
