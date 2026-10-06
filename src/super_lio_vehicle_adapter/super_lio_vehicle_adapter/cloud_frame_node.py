"""Transform Super-LIO's source cloud into odom using stamped TF only.

Super-LIO publishes its deskewed cloud in its configured source/world frame.
This node never changes ``frame_id`` by assignment: a cloud is forwarded only
after a transform lookup at the cloud acquisition timestamp succeeds. Missing
or stale TF therefore withholds the cloud and leaves the unknown-space safety
policy intact.
"""
from __future__ import annotations

try:
    import rclpy
    from rclpy.duration import Duration
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2
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
        self._input_frame = str(self.get_parameter("source_frame").value)
        self._target_frame = str(self.get_parameter("target_frame").value)
        self._tf_timeout_s = max(0.0, float(self.get_parameter("tf_timeout_s").value))
        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)
        self._publisher = self.create_publisher(
            PointCloud2, str(self.get_parameter("output_topic").value), 10)
        self.create_subscription(
            PointCloud2, str(self.get_parameter("input_topic").value),
            self._callback, 10)
        self._last_rejection = ""

    def _reject(self, reason: str) -> None:
        if reason != self._last_rejection:
            self.get_logger().warning("Super-LIO cloud withheld: %s", reason)
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
            output = do_transform_cloud(msg, transform)
        except (TransformException, TypeError, ValueError, RuntimeError) as exc:
            self._reject(f"stamped TF unavailable: {exc}")
            return
        if str(output.header.frame_id) != self._target_frame:
            self._reject("TF helper returned an unexpected target frame")
            return
        self._publisher.publish(output)
        self._last_rejection = ""


def main(args=None):
    if rclpy is None:
        raise RuntimeError("ROS 2 tf2_ros and tf2_sensor_msgs are required on the target")
    rclpy.init(args=args)
    node = SuperLioCloudFrameNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
