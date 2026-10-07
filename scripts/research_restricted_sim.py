#!/usr/bin/env python3
"""Separated synthetic sensor/physics and measured-only perception processes.

Requires domain 94. Truth exists only in the `truth` role, which renders raw
LiDAR/IMU and local depth points and integrates the original serial PTY. The
`perception` role accepts measured Super-LIO odometry and depth points only.
All sensor geometry/noise here is an explicit simulation model, never vehicle
calibration. No ROS truth-pose or complete-world-map topic is published.
"""
import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped, TwistStamped, Point, AccelStamped
from research_interfaces.msg import RoadEvidence2D
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Float32, String, Header
from tf2_ros import TransformBroadcaster
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from research_runtime.physical_parameter_lock import PHYSICAL_LIMITS
from research_runtime.grid_map import LocalObstacleGrid


def stamp_seconds(stamp):return stamp.sec+stamp.nanosec*1e-9


def yaw_of(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


class TruthSensor(Node):
    def __init__(self, fd, output, sensor_fault="none", fault_after_s=12.):
        super().__init__('restricted_sensor_physics')
        self.fd,self.output=fd,Path(output)
        os.set_blocking(fd,False)
        self.buffer=b'';self.pose=np.zeros(3);self.velocity=np.zeros(2);self.command=np.zeros(2)
        self.last=time.monotonic();self.history=deque(maxlen=1000);self.trace=[];self.wire=[]
        self.started=time.monotonic();self.distance=0.;self.last_permission=0.
        self.mock_rtk_allowed=True
        self.sensor_fault,self.fault_after_s=sensor_fault,fault_after_s
        self.fault_wire_start=None
        self.authority=self.create_publisher(Bool,'/localization_authority/motion_allowed',10)
        self.authority_mode=self.create_publisher(String,'/localization_authority/mode',10)
        self.speed=self.create_publisher(Float32,'/localization_authority/max_linear_speed_mps',10)
        self.encoder=self.create_publisher(TwistStamped,"/simulation/encoder_twist",qos_profile_sensor_data)
        self.lidar=self.create_publisher(CustomMsg,'/livox/lidar',qos_profile_sensor_data)
        self.imu=self.create_publisher(Imu,'/livox/imu',qos_profile_sensor_data)
        self.depth=self.create_publisher(PointCloud2,'/simulation/depth_points_base',10)
        self.scene=self._world_surfaces()  # Never loaded or accepted by perception/policy.
        self.il_translation=np.array([-.011,-.02329,.04412]) # Exact locked lidar->IMU translation.
        self.create_timer(.005,self.physics)
        self.create_timer(.1,self.scan)

    @staticmethod
    def _world_surfaces():
        points=[]
        for x in (-8.,8.):
            for y in np.arange(-6,6.01,.35):
                for z in np.arange(-1.2,3.01,.35):points.append((x,y,z))
        for y in (-6.,6.):
            for x in np.arange(-8,8.01,.35):
                for z in np.arange(-1.2,3.01,.35):points.append((x,y,z))
        for z in (-1.2,3.):
            for x in np.arange(-7.8,7.81,.5):
                for y in np.arange(-5.8,5.81,.5):points.append((x,y,z))
        # Asymmetric off-road landmarks reduce the room's planar symmetry.
        for x,y in ((2.,2.),(-3.,1.),(4.,-3.)):
            for z in np.arange(-1.2,2.01,.2):
                for angle in np.arange(0,2*math.pi,.4):points.append((x+.15*math.cos(angle),y+.15*math.sin(angle),z))
        return np.asarray(points)[::2]

    def drain(self):
        while True:
            try:data=os.read(self.fd,65536)
            except BlockingIOError:break
            if not data:break
            self.buffer+=data
        while b'\n' in self.buffer:
            line,self.buffer=self.buffer.split(b'\n',1)
            fields=line.decode('ascii').strip().split(',')
            if len(fields)==2 and fields[0].startswith('vcx=') and fields[1].startswith('wc='):
                self.command[:]=float(fields[0][4:]),float(fields[1][3:]);self.wire.append(line.decode()+'\n')

    def physics(self):
        self.drain();now=time.monotonic();dt=min(now-self.last,.03);self.last=now
        if self.sensor_fault!='none' and now-self.started>=self.fault_after_s and self.fault_wire_start is None:
            self.fault_wire_start=len(self.wire)
        old=self.velocity.copy()
        for i,acc,dec in ((0,PHYSICAL_LIMITS['max_accel_mps2'],PHYSICAL_LIMITS['max_decel_mps2']),
                          (1,PHYSICAL_LIMITS['max_yaw_accel_rps2'],PHYSICAL_LIMITS['max_yaw_decel_rps2'])):
            self.velocity[i]+=max(-dec*dt,min(acc*dt,self.command[i]-self.velocity[i]))
        self.pose[:2]+=self.velocity[0]*np.array([math.cos(self.pose[2]),math.sin(self.pose[2])])*dt
        self.pose[2]+=self.velocity[1]*dt;self.distance+=abs(self.velocity[0])*dt
        stamp=self.get_clock().now().to_msg();epoch=stamp_seconds(stamp)
        self.history.append((epoch,self.pose.copy()))
        msg=Imu();msg.header=Header(stamp=stamp,frame_id='livox_frame')
        # Unit-g accelerometer matches the audited raw bag scale. Super-LIO
        # normalizes by its initialization mean. Gyro remains rad/s.
        msg.linear_acceleration.x=(self.velocity[0]-old[0])/max(dt,1e-6)/9.7946
        msg.linear_acceleration.y=self.velocity[0]*self.velocity[1]/9.7946
        msg.linear_acceleration.z=1.;msg.angular_velocity.z=float(self.velocity[1]);
        if not (self.sensor_fault=='imu_lost' and now-self.started>=self.fault_after_s):self.imu.publish(msg)
        if now-self.last_permission>=.05:
            encoder=TwistStamped();encoder.header=Header(stamp=stamp,frame_id='base_footprint')
            encoder.twist.linear.x,encoder.twist.angular.z=map(float,self.velocity);self.encoder.publish(encoder)
            self.authority.publish(Bool(data=self.mock_rtk_allowed and now-self.started>2.))
            self.authority_mode.publish(String(data='RTK_AUTHORITATIVE' if self.mock_rtk_allowed else 'RTK_DEGRADED'))
            self.speed.publish(Float32(data=.85));self.last_permission=now
        if not self.trace or epoch-self.trace[-1]['stamp']>=.1:
            self.trace.append({'stamp':epoch,'pose':self.pose.tolist(),'velocity':self.velocity.tolist()})

    def scan(self):
        if len(self.history)<30:return
        end=self.history[-1][0];start=end-.1
        times=np.array([entry[0] for entry in self.history]);poses=np.array([entry[1] for entry in self.history])
        offsets=np.linspace(0,.099999999,len(self.scene))
        acquisition=start+offsets
        x=np.interp(acquisition,times,poses[:,0]);y=np.interp(acquisition,times,poses[:,1]);yaw=np.interp(acquisition,times,poses[:,2])
        dx=self.scene[:,0]-x;dy=self.scene[:,1]-y
        local=np.column_stack((np.cos(yaw)*dx+np.sin(yaw)*dy,-np.sin(yaw)*dx+np.cos(yaw)*dy,self.scene[:,2]))-self.il_translation
        # The same opaque wall occludes raw LiDAR and synthetic depth rays.
        denominator=self.scene[:,0]-x
        ray_t=np.divide(2.2-x,denominator,out=np.full_like(x,-1.),where=np.abs(denominator)>1e-8)
        cross_y=y+ray_t*(self.scene[:,1]-y);cross_z=ray_t*self.scene[:,2]
        occluded=(ray_t>0)&(ray_t<1)&(cross_y>=1)&(cross_y<=3)&(cross_z>=-1.2)&(cross_z<=2)
        msg=CustomMsg();msg.header.frame_id='livox_frame';msg.header.stamp.sec=int(start)
        msg.header.stamp.nanosec=int((start-int(start))*1e9);msg.timebase=int(start*1e9);msg.lidar_id=0
        for index,point in enumerate(local):
            if occluded[index] or not .5<=np.linalg.norm(point)<=25:continue
            p=CustomPoint();p.x,p.y,p.z=map(float,point);p.offset_time=int(offsets[index]*1e9)
            p.reflectivity,p.tag,p.line=80,16,index%4;msg.points.append(p)
        msg.point_num=len(msg.points);
        if not (self.sensor_fault=='lidar_lost' and time.monotonic()-self.started>=self.fault_after_s):self.lidar.publish(msg)
        # A restricted positively measured synthetic depth view. It never
        # supplies the hidden whole floor/map. No return is supplied beyond it.
        depth=[]
        for xx in np.arange(-1.8,1.8001,.1):
            for yy in np.arange(-1.8,1.8001,.1):
                if math.hypot(xx,yy)>1.8:continue
                # Off-road occlusion: a opaque wall blocks the local viewing ray.
                wx=self.pose[0]+math.cos(self.pose[2])*xx-math.sin(self.pose[2])*yy
                wy=self.pose[1]+math.sin(self.pose[2])*xx+math.cos(self.pose[2])*yy
                if abs(wx-self.pose[0])>1e-8:
                    ray_t=(2.2-self.pose[0])/(wx-self.pose[0])
                    cross_y=self.pose[1]+ray_t*(wy-self.pose[1])
                    if 0<ray_t<1 and 1<=cross_y<=3:continue
                depth.append((float(xx),float(yy),-1.2))
        header=Header(stamp=self.get_clock().now().to_msg(),frame_id='base_footprint')
        if self.sensor_fault=='depth_invalid' and time.monotonic()-self.started>=self.fault_after_s:
            depth=[(float('nan'),0.,0.),(0.,0.,float('nan'))]
        self.depth.publish(point_cloud2.create_cloud_xyz32(header,depth))

    def save(self):
        self.drain();self.output.write_text(json.dumps({'model':'synthetic room; unit-g IMU; scan-per-point acquisition; restricted local depth',
            'final_pose':self.pose.tolist(),'distance_m':self.distance,'trace':self.trace,'wire_tail':self.wire[-5:],
            'sensor_fault':self.sensor_fault,'fault_after_s':self.fault_after_s,
            'fault_phase_wire_tail_before_authority_cleanup':getattr(self,'precleanup_fault_tail',[]),
            'nonzero_before_fault':sum(line!='vcx=0.000,wc=0.000\n' for line in self.wire[:self.fault_wire_start]),
            'wire_count':len(self.wire),'nonzero_wire_count':sum(line!='vcx=0.000,wc=0.000\n' for line in self.wire)},indent=2)+'\n')


class MeasuredPerception(Node):
    def __init__(self, output, seed_grid=None, seed_evidence=None,localization_session_id=""):
        super().__init__('restricted_measured_perception')
        self.output=Path(output);self.history=deque(maxlen=100);self.pending=deque(maxlen=20)
        if not localization_session_id:raise ValueError("explicit simulated LIO session identity required")
        self.localization_session_id=localization_session_id
        self.encoder_samples=deque(maxlen=100);self.imu_samples=deque(maxlen=200)
        self.unacked_evidence={};self.last_evidence_retry=0.
        self.create_subscription(String,'/research/road_evidence_ack',lambda msg:self.unacked_evidence.pop(msg.data,None),1000)
        self.version='UNKNOWN';self.received=0;self.depth_count=0;self.cells=[-1]*10000
        self.last_seen=np.zeros(10000);self.ledger_cells=[-1]*10000
        if seed_evidence and Path(seed_evidence).exists():
            from research_runtime.active_road import EvidenceStore
            for evidence in EvidenceStore.load(seed_evidence).evidence():
                if evidence.evidence_id.startswith(self.localization_session_id+':synthetic-depth-cell:'):
                    self.ledger_cells[int(evidence.evidence_id.split(':')[-1])]=0
        self.tf=TransformBroadcaster(self)
        self.acceleration=self.create_publisher(AccelStamped,"/research/vehicle_acceleration",10)
        self.odom_pub=self.create_publisher(Odometry,'/lio/odom_vehicle',10)
        self.health=self.create_publisher(String,'/lio/vehicle_health',10)
        self.evidence=self.create_publisher(RoadEvidence2D,"/research/road_evidence",1000)
        from research_interfaces.msg import LocalEvidenceGrid2D
        self.ground=self.create_publisher(LocalEvidenceGrid2D,'/research/observed_ground_grid',10)
        self.cloud=self.create_publisher(PointCloud2,'/lio/cloud_odom',10)
        self.permission=self.create_publisher(OccupancyGrid,'/research/permission_grid',10)
        self.create_subscription(Odometry,'/lio/odom',self.source_odom,100)
        self.create_subscription(Imu,'/livox/imu',self.imu_samples.append,qos_profile_sensor_data)
        self.create_subscription(TwistStamped,'/simulation/encoder_twist',self.encoder_samples.append,qos_profile_sensor_data)
        self.create_subscription(PointCloud2,'/simulation/depth_points_base',self.depth_input,10)
        self.create_subscription(String,'/research/map_version',lambda msg:setattr(self,'version',msg.data),10)
        self.create_timer(.05,self.tick)
        self.last_source=0.;self.source_pose=None

    def source_odom(self,msg):
        self.received+=1;self.last_source=time.monotonic()
        # This bridge is simulation-only: the model has IMU=controlled base
        # and map=initial odom. It supplies an explicit conservative synthetic
        # uncertainty for the experiment, not a measured source covariance or
        # an equivalent FAST-LIO degeneracy metric. /lio/health stays UNKNOWN.
        yaw=yaw_of(msg.pose.pose.orientation);stamp=stamp_seconds(msg.header.stamp)
        self.source_pose=(stamp,msg.pose.pose.position.x,msg.pose.pose.position.y,yaw)
        self.history.append(self.source_pose)
        output=Odometry();output.header.stamp=msg.header.stamp;output.header.frame_id='odom';output.child_frame_id='base_footprint'
        output.pose.pose=msg.pose.pose;output.twist=msg.twist
        vx,vy=msg.twist.twist.linear.x,msg.twist.twist.linear.y
        output.twist.twist.linear.x=math.cos(yaw)*vx+math.sin(yaw)*vy
        output.twist.twist.linear.y=-math.sin(yaw)*vx+math.cos(yaw)*vy
        output.pose.covariance[0]=output.pose.covariance[7]=.03**2;output.pose.covariance[35]=.01**2
        samples=[entry for entry in self.encoder_samples if abs(stamp_seconds(entry.header.stamp)-stamp)<=.1]
        if not samples:return
        encoder=min(samples,key=lambda entry:abs(stamp_seconds(entry.header.stamp)-stamp))
        # Actual simulated encoder measurement supplies body twist; no truth
        # pose/map reaches this process. Real wheel telemetry remains pending.
        output.twist.twist=encoder.twist
        measurements=[sample for sample in self.imu_samples if abs(stamp_seconds(sample.header.stamp)-stamp)<=.1]
        if not measurements:return
        imu=min(measurements,key=lambda sample:abs(stamp_seconds(sample.header.stamp)-stamp))
        a=AccelStamped();a.header=output.header
        ax,ay=imu.linear_acceleration.x*9.7946,imu.linear_acceleration.y*9.7946
        a.accel.linear.x=math.cos(yaw)*ax-math.sin(yaw)*ay
        a.accel.linear.y=math.sin(yaw)*ax+math.cos(yaw)*ay
        self.acceleration.publish(a)
        self.odom_pub.publish(output)
        transform=TransformStamped();transform.header=output.header;transform.child_frame_id='base_footprint'
        transform.transform.translation.x=output.pose.pose.position.x;transform.transform.translation.y=output.pose.pose.position.y
        transform.transform.translation.z=output.pose.pose.position.z;transform.transform.rotation=output.pose.pose.orientation
        self.tf.sendTransform(transform)
        while self.pending and stamp_seconds(self.pending[0].header.stamp)<=stamp:
            self.process_depth(self.pending.popleft())

    def depth_input(self,msg):self.pending.append(msg)

    def process_depth(self,msg):
        stamp=stamp_seconds(msg.header.stamp)
        samples=list(self.history)
        if len(samples)<2 or stamp<samples[0][0]:return
        pair=next(((a,b) for a,b in zip(samples,samples[1:]) if a[0]<=stamp<=b[0]),None)
        if pair is None or pair[1][0]-pair[0][0]>.20:return
        a,b=pair;t=(stamp-a[0])/max(b[0]-a[0],1e-8)
        x,y=a[1]+t*(b[1]-a[1]),a[2]+t*(b[2]-a[2]);yaw=a[3]+t*math.atan2(math.sin(b[3]-a[3]),math.cos(b[3]-a[3]))
        groups={}
        for point in point_cloud2.read_points(msg,field_names=('x','y','z'),skip_nans=True):
            px,py,pz=map(float,point)
            if abs(pz+1.2)>.03:continue # Explicit simulated support plane only.
            wx=x+math.cos(yaw)*px-math.sin(yaw)*py;wy=y+math.sin(yaw)*px+math.cos(yaw)*py
            col,row=int(math.floor((wx+15)/.3)),int(math.floor((wy+15)/.3))
            if 0<=col<100 and 0<=row<100:groups.setdefault(row*100+col,[]).append((wx,wy,math.sqrt(px*px+py*py+pz*pz)))
        for index,points in groups.items():
            # Multiple supported points with spatial span, never an empty ray.
            if len(points)>=4 and max(p[0] for p in points)-min(p[0] for p in points)>=.18 and max(p[1] for p in points)-min(p[1] for p in points)>=.18:
                if self.ledger_cells[index]!=0:
                    evidence=RoadEvidence2D();evidence.header=Header(stamp=msg.header.stamp,frame_id='odom')
                    evidence.evidence_id=self.localization_session_id+':synthetic-depth-cell:'+str(index)
                    evidence.source='measured_synthetic_depth_support';evidence.local_submap_id=self.localization_session_id+'/depth-submap'
                    evidence.pose_uncertainty_m=.03;evidence.state='OBSERVED_GEOMETRY'
                    evidence.valid_depth_min_m=min(p[2] for p in points);evidence.valid_depth_max_m=max(p[2] for p in points)
                    low,high=min(p[0] for p in points),max(p[0] for p in points);mean_y=sum(p[1] for p in points)/len(points)
                    evidence.geometry=[Point(x=low,y=mean_y,z=0.),Point(x=high,y=mean_y,z=0.)]
                    evidence.observed_length_m=high-low;self.unacked_evidence[evidence.evidence_id]=evidence
                    self.evidence.publish(evidence)
                self.cells[index]=self.ledger_cells[index]=0;self.last_seen[index]=time.monotonic()
        self.depth_count+=1

    def grid(self,cells):
        msg=OccupancyGrid();msg.header.frame_id='odom';msg.header.stamp=self.get_clock().now().to_msg()
        msg.info.resolution=.3;msg.info.width=msg.info.height=100;msg.info.origin.position.x=msg.info.origin.position.y=-15.
        msg.info.origin.orientation.w=1.;msg.data=cells;return msg

    def tick(self):
        now=time.monotonic()
        if now-self.last_evidence_retry>=1.:
            for evidence in list(self.unacked_evidence.values()):self.evidence.publish(evidence)
            self.last_evidence_retry=now
        fresh=now-self.last_source<.20 and self.received>15
        self.health.publish(String(data='OK: SIMULATION_SOURCE_HEALTH_AND_UNCERTAINTY_ASSUMPTION' if fresh else 'UNKNOWN: simulated sensor source stale'))
        if self.source_pose:
            transform=TransformStamped();transform.header.frame_id='map';transform.child_frame_id='odom'
            # Mirror the unchanged original RTK TF's 0.10 s horizon. This is
            # an explicitly registered constant model transform, never truth pose.
            transform.header.stamp=(self.get_clock().now()+rclpy.duration.Duration(seconds=.10)).to_msg()
            transform.transform.rotation.w=1.;self.tf.sendTransform(transform)
        self.permission.publish(self.grid([0]*10000)) # Explicit approved synthetic room, not inferred from free space.
        if self.version=='UNKNOWN' or not fresh:return
        current=[value if now-self.last_seen[i]<=2. else -1 for i,value in enumerate(self.cells)]
        from research_interfaces.msg import LocalEvidenceGrid2D
        grid=self.grid(current)
        self.ground.publish(LocalEvidenceGrid2D(header=grid.header,grid=grid,map_version=self.version,
            localization_session_id=self.localization_session_id,support_model='restricted_sensor_fixture_v1'))
        header=grid.header
        self.cloud.publish(point_cloud2.create_cloud_xyz32(header,[]))

    def save(self):
        self.output.write_text(json.dumps({'scope':'actual Super-LIO pose; simulation-only base extrinsic/uncertainty/health assumptions; restricted current depth',
            'lio_odometry_count':self.received,'processed_depth_count':self.depth_count,
            'last_source_pose':self.source_pose,'ledger_cells':self.ledger_cells,
            'known_ground_cell_count':sum(value==0 for value in self.ledger_cells)},indent=2)+'\n')


def main():
    parser=argparse.ArgumentParser();parser.add_argument('role',choices=('truth','perception'))
    parser.add_argument('--output',required=True);parser.add_argument('--pty-fd',type=int);parser.add_argument('--seed-grid');parser.add_argument('--seed-evidence')
    parser.add_argument('--sensor-fault',choices=['none','imu_lost','lidar_lost','depth_invalid'],default='none')
    parser.add_argument('--fault-after-s',type=float,default=12.)
    parser.add_argument('--localization-session-id',default='')
    args=parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='94' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':raise SystemExit('requires isolated domain 94')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=TruthSensor(args.pty_fd,args.output,args.sensor_fault,args.fault_after_s) if args.role=='truth' else MeasuredPerception(args.output,args.seed_grid,args.seed_evidence,args.localization_session_id)
    try:rclpy.spin(node)
    except KeyboardInterrupt:
        if args.role=='truth':
            node.drain()
            node.precleanup_fault_tail=node.wire[-5:] if node.fault_wire_start is not None else []
            node.mock_rtk_allowed=False
            until=time.monotonic()+.8
            while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.005)
    finally:
        node.save();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
