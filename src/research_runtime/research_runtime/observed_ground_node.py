"""Stamped MID360 support evidence from the unfiltered deskewed source cloud."""
import math
import time
from collections import OrderedDict

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.clock import Clock, ClockType
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener
from geometry_msgs.msg import Point
from research_interfaces.msg import LocalEvidenceGrid2D, RoadEvidence2D, RoadEvent

from super_lio_vehicle_adapter.adapter_node import _source_certificate, _covariance_is_known, _qrotate
from .physical_parameter_lock import MID360_GROUND_REFERENCE, LOCKED_FOOTPRINT
from .observed_ground import GroundPolicy, expected_ground_z, project_observed_ground
from .local_road_evidence import supported_strips,pose_points_xy_uncertainty


def _stamp(msg):
    return msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec


class ObservedGroundNode(Node):
    def __init__(self):
        super().__init__("research_observed_ground")
        self._resolution = float(self.declare_parameter("resolution_m",0.30).value)
        self._width = self.declare_parameter("width",100).value
        self._height = self.declare_parameter("height",100).value
        if (abs(self._resolution-0.30) > 1e-12 or type(self._width) is not int or
            type(self._height) is not int or not 1 <= self._width <= 200 or not 1 <= self._height <= 200):
            raise ValueError("ground grid must retain the research resolution and bounded dimensions")
        actual_height = self.declare_parameter("lidar_height_m",MID360_GROUND_REFERENCE["lidar_height_m"]).value
        actual_lever = self.declare_parameter("lidar_in_imu_m",list(MID360_GROUND_REFERENCE["lidar_in_imu_m"])).value
        if (actual_height != MID360_GROUND_REFERENCE["lidar_height_m"] or
            tuple(actual_lever) != MID360_GROUND_REFERENCE["lidar_in_imu_m"]):
            raise ValueError("recorded MID360 mounting/factory lever override rejected")
        self._policy = GroundPolicy(**{key:self.declare_parameter(key,getattr(GroundPolicy(),key)).value
            for key in GroundPolicy.__dataclass_fields__})
        self._version = "UNKNOWN"
        self._session = str(self.declare_parameter("localization_session_id","UNKNOWN").value)
        self._version_receipt = 0.
        self._last_published_stamp = 0
        self._last_clock = 0
        self._clock_fault = False
        self._poses, self._certificates, self._pending = OrderedDict(), OrderedDict(), OrderedDict()
        self._tf_ok, self._tf_receipt = False, 0.
        self._last_cloud_receipt = 0.
        self._last_grid = None
        self._invalidated = False
        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer,self)
        self._publisher = self.create_publisher(LocalEvidenceGrid2D,"/research/observed_ground_grid",10)
        self._cloud_publisher = self.create_publisher(PointCloud2,"/research/ground_observation_cloud",10)
        self._status = self.create_publisher(String,"/research/ground_status",10)
        self._road_pub = self.create_publisher(RoadEvidence2D,"/research/road_evidence",100)
        self._question_pub = self.create_publisher(RoadEvent,"/research/measured_road_questions",100)
        self._road_pending = OrderedDict()
        self._road_seen = set()
        self.create_subscription(String,"/research/road_evidence_ack",self._road_ack,100)
        self.create_subscription(String,"/research/map_version",self._version_callback,10)
        self.create_subscription(Odometry,"/lio/odom_vehicle",self._pose_callback,100)
        self.create_subscription(String,"/lio/health",self._certificate_callback,100)
        self.create_subscription(Bool,"/research/local_frame_integrity",self._tf_callback,10)
        self.create_subscription(PointCloud2,"/lio/cloud_world",self._cloud_callback,20)
        self.create_timer(0.05,self._drain,clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.create_timer(0.20,self._retry_roads,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def _road_ack(self,msg):
        self._road_pending.pop(str(msg.data),None)

    def _retry_roads(self):
        # Exactly the original acquisition object is resent. The persistence
        # acknowledgement suppresses retry, never counts it as new evidence.
        for msg in self._road_pending.values():self._road_pub.publish(msg)

    def _record_roads(self,grid,pose,points,header,floor):
        covariance=tuple(pose.pose.covariance)+tuple(pose.twist.covariance)
        if not _covariance_is_known(covariance):return
        p,q=pose.pose.pose.position,pose.pose.pose.orientation
        translation=_qrotate((q.x,q.y,q.z,q.w),MID360_GROUND_REFERENCE['lidar_in_imu_m'])
        sensor=(p.x+translation[0],p.y+translation[1],p.z+translation[2])
        width=max(y for _x,y in LOCKED_FOOTPRINT)-min(y for _x,y in LOCKED_FOOTPRINT)
        for strip in supported_strips(grid,session=self._session,minimum_width_m=width):
            a,b=strip.geometry_xy
            vertical=abs(a[0]-b[0])<1e-9
            corners=[(x+(sign*strip.width_m/2 if vertical else 0.),
                      y+(0. if vertical else sign*strip.width_m/2),z)
                for x,y in (a,b) for sign in (-1.,1.)
                for z in (floor-self._policy.floor_band_m,floor+self._policy.floor_band_m)]
            uncertainty=pose_points_xy_uncertainty(pose.pose.covariance,(p.x,p.y,p.z),corners)
            low_x,high_x=min(c[0] for c in corners),max(c[0] for c in corners)
            low_y,high_y=min(c[1] for c in corners),max(c[1] for c in corners)
            ranges=[math.dist((float(point[0]),float(point[1]),float(point[2])),sensor) for point in points
                if all(math.isfinite(float(value)) for value in (point[0],point[1],point[2])) and
                low_x<=point[0]<=high_x and low_y<=point[1]<=high_y and
                abs(point[2]-floor)<=self._policy.floor_band_m]
            ranges=[value for value in ranges if value>0]
            if not ranges:continue
            identity=self._session+':mid360-strip:'+strip.identity
            if identity not in self._road_seen and len(self._road_seen)<1024 and len(self._road_pending)<128:
                msg=RoadEvidence2D(header=header,evidence_id=identity,
                    source='mid360_single_scan_flat_geometry_v1',local_submap_id=self._session+'/native-odom',
                    state='OBSERVED_GEOMETRY',pose_uncertainty_m=uncertainty,
                    observed_length_m=strip.length_m,valid_depth_min_m=min(ranges),valid_depth_max_m=max(ranges),
                    supported_width_m=strip.width_m,
                    geometry=[Point(x=x,y=y,z=0.) for x,y in strip.geometry_xy])
                self._road_seen.add(identity);self._road_pending[identity]=msg
                self._road_pub.publish(msg)
            # Questions remain current observations, even when the strip was
            # already persisted. Unknown length is one tested cell, not a
            # fabricated remote edge or an assertion of semantic road identity.
            for index,(x,y) in enumerate(strip.frontier_xy):
                if uncertainty>.10:continue  # Same explicit research trust ceiling as the observer.
                self._question_pub.publish(RoadEvent(header=header,event_id=identity+':frontier:'+str(index),
                    kind='geometric_entry',x=x,y=y,observed_length_m=strip.length_m,
                    unknown_length_m=grid.resolution_m,state='UNCERTAIN',impact=0.,
                    source='mid360_single_scan_flat_geometry_v1'))

    @staticmethod
    def _remember(collection,key,value,limit):
        collection[key] = value
        while len(collection)>limit:
            collection.popitem(last=False)

    def _version_callback(self,msg):
        value = str(msg.data).strip()
        value = value if value and value.upper() != "UNKNOWN" else "UNKNOWN"
        if value != self._version:
            self._invalidate("map version changed")
            self._pending.clear()
        self._version = value
        self._version_receipt = time.monotonic()

    def _pose_callback(self,msg):
        if msg.header.frame_id == "odom" and msg.child_frame_id == "base_footprint" and _stamp(msg)>0:
            self._remember(self._poses,_stamp(msg),(msg,time.monotonic()),100)
            self._drain()

    def _certificate_callback(self,msg):
        certificate = _source_certificate(msg.data)
        if certificate:
            self._remember(self._certificates,certificate["stamp_ns"],(certificate,time.monotonic()),100)
            if not certificate["eligible"]:
                self._invalidate("source observation denied")
            self._drain()

    def _tf_callback(self,msg):
        self._tf_ok, self._tf_receipt = bool(msg.data),time.monotonic()
        if not self._tf_ok:
            self._invalidate("TF ownership denied")

    def _cloud_callback(self,msg):
        if msg.header.frame_id != "world" or _stamp(msg)<=0:
            self._invalidate("cloud frame/stamp invalid")
            return
        self._last_cloud_receipt = time.monotonic()
        self._remember(self._pending,_stamp(msg),(msg,time.monotonic()),20)
        self._drain()

    def _invalidate(self,reason):
        if self._last_grid is not None and not self._invalidated:
            output = self._last_grid
            output.grid.data = [-1]*len(output.grid.data)
            # Retain acquisition time; invalidation never freshens old evidence.
            self._publisher.publish(output)
            self._invalidated = True
        self._status.publish(String(data="UNKNOWN: "+reason))

    def _drain(self):
        now = time.monotonic()
        measurement_now = self.get_clock().now().nanoseconds
        if measurement_now+1000 < self._last_clock:
            self._clock_fault = True
            self._pending.clear()
        self._last_clock = measurement_now
        if (self._clock_fault or measurement_now<=0 or self._version == "UNKNOWN" or
            not self._session or self._session.upper()=="UNKNOWN" or
            now-self._version_receipt>0.50 or not self._tf_ok or now-self._tf_receipt>0.50 or
            (self._last_cloud_receipt and now-self._last_cloud_receipt>0.50)):
            self._invalidate("map/TF/cloud unavailable or expired")
            return
        if self._last_grid is not None and measurement_now-_stamp(self._last_grid)>500_000_000:
            self._invalidate("ground acquisition expired")
        for key,(cloud,receipt) in list(self._pending.items()):
            if key <= self._last_published_stamp:
                del self._pending[key]
                continue
            if now-receipt>0.50 or not -50_000_000 <= measurement_now-key <= 500_000_000:
                del self._pending[key]
                self._invalidate("acquisition inputs did not match before expiry")
                continue
            if key not in self._poses or key not in self._certificates:
                continue  # Bounded wait for reverse DDS callback ordering.
            pose,pose_receipt = self._poses[key]
            certificate,certificate_receipt = self._certificates[key]
            if not certificate["eligible"] or now-min(pose_receipt,certificate_receipt)>0.50:
                del self._pending[key]
                self._invalidate("matched source observation absent/expired")
                continue
            try:
                transform = self._buffer.lookup_transform("odom","world",cloud.header.stamp,
                    timeout=Duration(seconds=0.0))
                p,q = transform.transform.translation,transform.transform.rotation
                values = (p.x,p.y,p.z,q.x,q.y,q.z,q.w)
                if (not all(math.isfinite(v) for v in values) or
                    abs(p.x)+abs(p.y)+abs(p.z)>1e-9 or
                    abs(q.x)+abs(q.y)+abs(q.z)>1e-9 or abs(abs(q.w)-1)>1e-9):
                    raise ValueError("ground requires the owned identity odom/world gauge")
                p,q = pose.pose.pose.position,pose.pose.pose.orientation
                floor = expected_ground_z((p.x,p.y,p.z),(q.x,q.y,q.z,q.w),
                    MID360_GROUND_REFERENCE["lidar_in_imu_m"],MID360_GROUND_REFERENCE["lidar_height_m"])
                origin_x = (math.floor(p.x/self._resolution)-self._width//2)*self._resolution
                origin_y = (math.floor(p.y/self._resolution)-self._height//2)*self._resolution
                points = point_cloud2.read_points(cloud,field_names=("x","y","z"),skip_nans=False)
                grid,stats = project_observed_ground(points,map_version=self._version,
                    resolution_m=self._resolution,origin_x_m=origin_x,origin_y_m=origin_y,
                    width=self._width,height=self._height,floor_z_m=floor,policy=self._policy)
                output = OccupancyGrid()
                output.header.stamp = cloud.header.stamp
                output.header.frame_id = "odom"
                output.info.resolution = grid.resolution_m
                output.info.width,output.info.height = grid.width,grid.height
                output.info.origin.position.x,output.info.origin.position.y = grid.origin_x_m,grid.origin_y_m
                output.info.origin.orientation.w = 1.
                output.data = list(grid.cells)
                wrapped = LocalEvidenceGrid2D(header=output.header,map_version=self._version,
                    localization_session_id=self._session,support_model="single_scan_flat_dense_v1",grid=output)
                self._publisher.publish(wrapped)
                self._record_roads(grid,pose,points,output.header,floor)
                # This is an actual TF operation even for the owned identity;
                # the original safety/display filtered clouds remain separate.
                from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
                # Native PCL has a padded 32-byte XYZI point stride. Humble's
                # tf2 Python helper rebuilds an inferred packed dtype and
                # asserts on that input. This geometry-only research channel
                # packs every XYZ return (including invalid values) explicitly
                # before the stamped transform, without a height/range filter.
                xyz=point_cloud2.create_cloud_xyz32(cloud.header,
                    [tuple(float(point[index]) for index in range(3)) for point in points])
                transformed=do_transform_cloud(xyz,transform)
                # tf2_sensor_msgs copies the transform header. A static gauge
                # has stamp zero; retain the actual scan time in the odom frame.
                transformed.header=output.header
                self._cloud_publisher.publish(transformed)
                self._last_grid,self._invalidated = wrapped,False
                self._last_published_stamp = key
                self._status.publish(String(data=f"OBSERVED: {stats}; flat-ground research assumptions"))
                del self._pending[key]
            except TransformException:
                continue  # Retry only while the original acquisition is fresh.
            except (ValueError,TypeError,RuntimeError) as exc:
                del self._pending[key]
                self._invalidate(str(exc))


def main(args=None):
    rclpy.init(args=args)
    node = ObservedGroundNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
