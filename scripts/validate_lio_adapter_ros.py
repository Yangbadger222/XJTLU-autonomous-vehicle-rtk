#!/usr/bin/env python3
"""Analytical real-ROS rigid-body/cloud-health tests; fixtures are not calibration."""
import argparse,json,math,os,signal,subprocess,time
from pathlib import Path
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String,Header
from tf2_ros import StaticTransformBroadcaster


def main():
    p=argparse.ArgumentParser();p.add_argument('--install',type=Path,required=True);p.add_argument('--repo',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='97' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':raise SystemExit('isolated domain 97 required')
    rclpy.init();node=rclpy.create_node('analytical_lio_fixture');a.output.parent.mkdir(parents=True,exist_ok=True)
    odom=node.create_publisher(Odometry,'/lio/odom',100);health=node.create_publisher(String,'/lio/health',10);cloud=node.create_publisher(PointCloud2,'/lio/cloud_world',10)
    received=[];clouds=[];health_states=[];world_display=[];body_display=[]
    node.create_subscription(Odometry,'/lio/odom_vehicle',received.append,100)
    node.create_subscription(PointCloud2,'/lio/cloud_odom',clouds.append,100)
    node.create_subscription(PointCloud2,'/lio/cloud_filtered_world',world_display.append,100)
    node.create_subscription(PointCloud2,'/lio/cloud_filtered_body',body_display.append,100)
    node.create_subscription(String,'/lio/vehicle_health',lambda msg:health_states.append(msg.data),100)
    tf=StaticTransformBroadcaster(node);t=TransformStamped();t.header.frame_id='odom';t.child_frame_id='world';t.header.stamp=node.get_clock().now().to_msg()
    t.transform.translation.x,t.transform.translation.y,t.transform.translation.z=10.,20.,100.
    t.transform.rotation.z=t.transform.rotation.w=math.sqrt(.5);tf.sendTransform(t)
    children=[];logs=[];checks=[]
    def spawn(package,binary,args=[]):
        log=a.output.with_name(binary+'-analytical.log').open('w');logs.append(log)
        child=subprocess.Popen([str(a.install/package/'lib'/package/binary),'--ros-args']+args,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);children.append(child)
    def phase(seconds,send_health=True,send_cloud=False,tilt=False):
        until=time.monotonic()+seconds
        while time.monotonic()<until:
            stamp=node.get_clock().now().to_msg();msg=Odometry();msg.header=Header(stamp=stamp,frame_id='world');msg.child_frame_id='imu'
            msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z=1.,2.,100.
            msg.pose.pose.orientation.z=msg.pose.pose.orientation.w=math.sqrt(.5)
            if tilt:msg.pose.pose.orientation.x=math.sin(.25);msg.pose.pose.orientation.z=0.;msg.pose.pose.orientation.w=math.cos(.25)
            msg.twist.twist.angular.z=1.
            msg.pose.covariance=[.01 if i%7==0 else 0. for i in range(36)];msg.twist.covariance=list(msg.pose.covariance)
            odom.publish(msg)
            if send_health:health.publish(String(data='OK: ANALYTICAL_TEST_SOURCE'))
            if send_cloud:cloud.publish(point_cloud2.create_cloud_xyz32(msg.header,[(0.,0.,100.05),(1.,0.,100.2),(2.,0.,100.8),(3.,0.,101.3),(float('nan'),0.,0.)]))
            
            for _ in range(7):rclpy.spin_once(node,timeout_sec=.002)
            time.sleep(.025)
    def check(name,ok,detail=None):checks.append({'case':name,'status':'PASS' if ok else 'FAIL','detail':detail})
    try:
        spawn('super_lio_vehicle_adapter','super_lio_vehicle_adapter',['-p','imu_to_base_extrinsic_verified:=true','-p','imu_to_base_translation_m:=[1.0,0.0,0.0]',
            '-p','imu_to_base_quaternion_xyzw:=[0.0,0.0,0.7071067811865476,0.7071067811865476]'])
        spawn('super_lio_vehicle_adapter','super_lio_cloud_frame_adapter',['--params-file',str(a.repo/'src/bringup/config/super_lio_cloud_frame.yaml')])
        phase(2.)
        m=received[-1] if received else None
        pose=[m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z] if m else []
        check('nonzero_measured_lever_90_degree_frames_body_velocity',m is not None and math.dist(pose,(7.,21.,200.))<1e-8
              and abs(m.twist.twist.linear.x-1.)<1e-8 and abs(m.twist.twist.linear.y)<1e-8,{'pose':pose,'count':len(received)})
        phase(1.,send_cloud=True,tilt=True)
        pts=list(point_cloud2.read_points(clouds[-1],field_names=('x','y','z'))) if clouds else []
        check('gravity_height_window_tilted_body_nonzero_altitude_nan_rejection',len(pts)==2 and all(200.08<float(p[2])<201.2 for p in pts),[list(map(float,p)) for p in pts])
        wp=list(point_cloud2.read_points(world_display[-1],field_names=('x','y','z'))) if world_display else []
        bp=list(point_cloud2.read_points(body_display[-1],field_names=('x','y','z'))) if body_display else []
        expected=[(-1.,-2.*math.cos(.5)+.05*math.sin(.5),2.*math.sin(.5)+.05*math.cos(.5)),
                  (0.,-2.*math.cos(.5)+.2*math.sin(.5),2.*math.sin(.5)+.2*math.cos(.5))]
        check('regular_world_and_imu_body_cloud_same_gravity_filter',len(wp)==len(bp)==2 and
              world_display[-1].header.frame_id=='world' and body_display[-1].header.frame_id=='imu' and
              all(math.dist(tuple(map(float,p)),e)<1e-4 for p,e in zip(bp,expected)),
              {'world':[list(map(float,p)) for p in wp],'body':[list(map(float,p)) for p in bp]})
        phase(.7,send_health=False);count=len(received);phase(.3,send_health=False)
        check('source_health_receipt_expiry_with_continuing_odom',len(received)==count and health_states and health_states[-1].startswith('UNKNOWN'))
        result={'status':'PASS' if all(c['status']=='PASS' for c in checks) else 'FAIL','checks':checks,
            'scope':'actual ROS adapters, analytical rigid transforms/covariance/health only; real source health and physical extrinsics remain pending'}
    finally:
        for child in reversed(children):
            if child.poll() is None:os.killpg(child.pid,signal.SIGINT)
            child.wait(timeout=5)
        for log in logs:log.close()
        node.destroy_node();rclpy.shutdown()
    a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result));return 0 if result['status']=='PASS' else 1
if __name__=='__main__':raise SystemExit(main())
