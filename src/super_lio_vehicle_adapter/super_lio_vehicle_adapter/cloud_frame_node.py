"""Transform Super-LIO's source cloud into odom using stamped TF only.

Super-LIO publishes its deskewed cloud in its configured source/world frame.
This node never changes ``frame_id`` by assignment: a cloud is forwarded only
after a transform lookup at the cloud acquisition timestamp succeeds. Missing
or stale TF therefore withholds the cloud and leaves the unknown-space safety
policy intact.
"""
from __future__ import annotations
import math
from collections import deque
from .adapter_node import _qrotate, _normalize_quaternion

try:
    import rclpy
    from rclpy.duration import Duration
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Header
    from sensor_msgs_py import point_cloud2
    from tf2_ros import Buffer, TransformException, TransformListener
    from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
except ImportError:  # permits ROS-free source tests on the workstation
    rclpy = None
    Duration = PointCloud2 = Buffer = TransformException = TransformListener = do_transform_cloud = object
    Node = object


def _stamp_is_set(stamp) -> bool:
    """Require a real cloud acquisition stamp before querying TF."""
    try:
        sec = int(stamp.sec)
        nanosec = int(stamp.nanosec)
    except (AttributeError, TypeError, ValueError):
        return False
    return sec >= 0 and 0 <= nanosec < 1_000_000_000 and (sec > 0 or nanosec > 0)


class SuperLioCloudFrameNode(Node if rclpy else object):
    def __init__(self):
        super().__init__("super_lio_cloud_frame_adapter")
        self.declare_parameter("input_topic", "/lio/cloud_world")
        self.declare_parameter("output_topic", "/lio/cloud_odom")
        self.declare_parameter("source_frame", "world")
        self.declare_parameter("target_frame", "odom")
        self.declare_parameter("tf_timeout_s", 0.05)
        self._owned_identity_gauge = self.declare_parameter("require_owned_identity_gauge",False).value
        self.declare_parameter("height_odom_topic", "/lio/odom")
        self._source_poses=deque(maxlen=100)
        self.create_subscription(Odometry,str(self.get_parameter("height_odom_topic").value),self._source_poses.append,100)
        self.declare_parameter("obstacle_min_z_m", .08)
        self.declare_parameter("obstacle_max_z_m", 1.20)
        self.declare_parameter("publish_min_z_m", -.33)
        self.declare_parameter("publish_max_z_m", .30)
        self._input_frame = str(self.get_parameter("source_frame").value)
        self._target_frame = str(self.get_parameter("target_frame").value)
        self._tf_timeout_s = max(0.0, float(self.get_parameter("tf_timeout_s").value))
        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)
        self._publisher = self.create_publisher(
            PointCloud2, str(self.get_parameter("output_topic").value), 10)
        self._world_display = self.create_publisher(PointCloud2, "/lio/cloud_filtered_world", 10)
        self._body_display = self.create_publisher(PointCloud2, "/lio/cloud_filtered_body", 10)
        self.create_subscription(
            PointCloud2, str(self.get_parameter("input_topic").value),
            self._callback, 10)
        self._last_rejection = ""

    def _reject(self, reason: str) -> None:
        if reason != self._last_rejection:
            self.get_logger().warning(f"Super-LIO cloud withheld: {reason}")
            self._last_rejection = reason

    def _callback(self, msg: PointCloud2) -> None:
        if not _stamp_is_set(msg.header.stamp):
            self._reject("cloud has no acquisition timestamp; refusing latest-TF lookup")
            return
        if str(msg.header.frame_id) != self._input_frame:
            self._reject(f"source frame {msg.header.frame_id!r} != {self._input_frame!r}")
            return
        if self._input_frame == self._target_frame:
            self._reject("source and target frames must differ; refusing frame relabel")
            return
        try:
            transform = self._buffer.lookup_transform(
                self._target_frame, self._input_frame, msg.header.stamp,
                timeout=Duration(seconds=self._tf_timeout_s))
            if self._owned_identity_gauge:
                p, q = transform.transform.translation, transform.transform.rotation
                values = (p.x,p.y,p.z,q.x,q.y,q.z,q.w)
                if (not all(math.isfinite(value) for value in values) or
                    abs(p.x)+abs(p.y)+abs(p.z)>1e-9 or
                    abs(q.x)+abs(q.y)+abs(q.z)>1e-9 or abs(abs(q.w)-1)>1e-9):
                    raise ValueError("lookup TF conflicts with owned local-world identity gauge")
            stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            poses=[p for p in self._source_poses if p.header.frame_id==self._input_frame and p.child_frame_id=="imu" and
                   abs(p.header.stamp.sec+p.header.stamp.nanosec*1e-9-stamp)<=.05]
            if not poses:raise ValueError("no acquisition-matched source IMU height")
            pose=min(poses,key=lambda p:abs(p.header.stamp.sec+p.header.stamp.nanosec*1e-9-stamp))
            origin_z=pose.pose.pose.position.z
            if not math.isfinite(origin_z):raise ValueError("non-finite source IMU origin")
            low,high=(float(self.get_parameter(key).value) for key in ("obstacle_min_z_m","obstacle_max_z_m"))
            if not (math.isfinite(low) and math.isfinite(high) and low<=high):raise ValueError("invalid obstacle height contract")
            # Exact pinned FAST-LIO filter: world point z minus source IMU
            # origin z, along the gravity-aligned source world vertical. It is
            # not tilted body-frame z, nor absolute altitude in odom.
            kept=[tuple(map(float,p)) for p in point_cloud2.read_points(msg,field_names=("x","y","z"),skip_nans=True)
                  if low<=float(p[2])-origin_z<=high]
            filtered=point_cloud2.create_cloud_xyz32(msg.header,kept)
            output=do_transform_cloud(filtered,transform)
            display_low,display_high=(float(self.get_parameter(key).value) for key in ("publish_min_z_m","publish_max_z_m"))
            if not (math.isfinite(display_low) and math.isfinite(display_high) and display_low<=display_high):
                raise ValueError("invalid regular cloud height contract")
            display=[tuple(map(float,p)) for p in point_cloud2.read_points(msg,field_names=("x","y","z"),skip_nans=True)
                     if display_low<=float(p[2])-origin_z<=display_high]
            world_display=point_cloud2.create_cloud_xyz32(msg.header,display)
            q=pose.pose.pose.orientation
            normalized=_normalize_quaternion((q.x,q.y,q.z,q.w))
            if normalized is None:raise ValueError("invalid source IMU quaternion")
            inverse=(-normalized[0],-normalized[1],-normalized[2],normalized[3])
            origin=pose.pose.pose.position
            body=[_qrotate(inverse,(p[0]-origin.x,p[1]-origin.y,p[2]-origin.z)) for p in display]
            body_display=point_cloud2.create_cloud_xyz32(Header(stamp=msg.header.stamp,frame_id="imu"),body)
            # The upstream body cloud is relative to the source IMU, as in
            # pinned FAST-LIO. No unverified vehicle extrinsic is introduced.
        except (TransformException, TypeError, ValueError, RuntimeError) as exc:
            self._reject(f"stamped TF unavailable: {exc}")
            return
        if str(output.header.frame_id) != self._target_frame:
            self._reject("TF helper returned an unexpected target frame")
            return
        self._publisher.publish(output)
        self._world_display.publish(world_display)
        self._body_display.publish(body_display)
        self._last_rejection = ""


def main(args=None):
    if rclpy is None:
        raise RuntimeError("ROS 2 tf2_ros and tf2_sensor_msgs are required on the target")
    rclpy.init(args=args)
    node = SuperLioCloudFrameNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
