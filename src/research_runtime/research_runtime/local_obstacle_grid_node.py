"""ROS transport for the fail-closed local obstacle-grid contract.

The node only marks measured obstacle cells.  It refuses clouds whose frame is
not already the declared odom frame and refuses to publish until a non-UNKNOWN
map version arrives from the active-road persistence layer.  It therefore
cannot turn a missing TF, missing map identity, or missing return into free
space.
"""
from __future__ import annotations
import struct

try:
    import rclpy
    from nav_msgs.msg import OccupancyGrid
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import String
except ImportError:  # permits ROS-free contract tests on the developer laptop
    rclpy = None
    OccupancyGrid = PointCloud2 = String = object
    point_cloud2 = None
    Node = object

from .grid_map import LocalObstacleGrid, project_obstacle_points


def _parameter_bool(value) -> bool:
    """Parse launch values by value; never treat the string ``false`` as true."""
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    normalized = str(value).strip().lower()
    if normalized in {"true", "1", "yes", "on"}:
        return True
    if normalized in {"false", "0", "no", "off", ""}:
        return False
    raise ValueError(f"invalid boolean parameter: {value!r}")


def _valid_map_version(value: str) -> bool:
    return bool(value and value.strip() and value.strip().upper() != "UNKNOWN")


class LocalObstacleGridNode(Node if rclpy else object):
    def __init__(self):
        super().__init__("research_local_obstacle_grid")
        self.declare_parameter("input_topic", "/lio/cloud_world")
        self.declare_parameter("output_topic", "/research/local_obstacle_grid")
        self.declare_parameter("map_version_topic", "/research/map_version")
        self.declare_parameter("target_frame", "odom")
        self.declare_parameter("resolution_m", 0.30)
        self.declare_parameter("origin_x_m", -15.0)
        self.declare_parameter("origin_y_m", -15.0)
        self.declare_parameter("width", 100)
        self.declare_parameter("height", 100)
        self.declare_parameter("obstacle_min_z_m", 0.08)
        self.declare_parameter("obstacle_max_z_m", 1.20)
        self.declare_parameter("input_height_filter_applied", False)
        self.declare_parameter("unknown_is_occupied", True)
        self.declare_parameter("observed_ground_topic", "/research/observed_ground_grid")
        self.declare_parameter("ground_timeout_s", 0.50)

        self._target_frame = str(self.get_parameter("target_frame").value)
        self._map_version = "UNKNOWN"
        self._ground = None
        self._ground_stamp = None
        if not _parameter_bool(self.get_parameter("unknown_is_occupied").value):
            raise ValueError("unknown_is_occupied must remain true")
        self._publisher = self.create_publisher(
            OccupancyGrid, str(self.get_parameter("output_topic").value), 10)
        self.create_subscription(
            String, str(self.get_parameter("map_version_topic").value),
            self._map_version_callback, 10)
        self.create_subscription(
            PointCloud2, str(self.get_parameter("input_topic").value),
            self._cloud_callback, 10)
        self.create_subscription(OccupancyGrid, str(self.get_parameter("observed_ground_topic").value),
                                 self._ground_callback, 10)
        self._last_rejection = ""

    def _map_version_callback(self, msg: String) -> None:
        value = str(msg.data).strip()
        self._map_version = value if _valid_map_version(value) else "UNKNOWN"

    def _reject(self, reason: str) -> None:
        if reason != self._last_rejection:
            self.get_logger().warning(f"local obstacle grid withheld: {reason}")
            self._last_rejection = reason

    def _ground_callback(self, msg):
        # This channel carries positively observed supported ground, never
        # absence-of-obstacle rays. A missing ground producer leaves cells unknown.
        q = msg.info.origin.orientation
        if (msg.header.frame_id != "odom" or not _valid_map_version(self._map_version) or
                abs(q.x)+abs(q.y)+abs(q.z) > 1e-9 or abs(q.w-1) > 1e-9):
            self._ground = None
            return
        try:
            self._ground = LocalObstacleGrid("odom", self._map_version, msg.info.resolution,
                msg.info.origin.position.x, msg.info.origin.position.y,
                msg.info.width, msg.info.height, tuple(msg.data), True)
            self._ground_stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        except (TypeError, ValueError):
            self._ground = None

    def _cloud_callback(self, msg: PointCloud2) -> None:
        if str(msg.header.frame_id) != self._target_frame:
            self._reject(f"cloud frame {msg.header.frame_id!r} != {self._target_frame!r}")
            return
        if not _valid_map_version(self._map_version):
            self._reject("map version is UNKNOWN or missing")
            return
        try:
            points = point_cloud2.read_points(
                msg, field_names=("x", "y", "z"), skip_nans=False)
            ground = None
            if self._ground is not None and self._ground_stamp is not None:
                age = (rclpy.time.Time.from_msg(msg.header.stamp)-self._ground_stamp).nanoseconds * 1e-9
                if 0 <= age <= float(self.get_parameter("ground_timeout_s").value):
                    ground = self._ground
            grid, _stats = project_obstacle_points(
                points,
                map_version=self._map_version,
                # OccupancyGrid resolution is float32. Compare the canonical
                # wire value rather than rejecting 0.3 vs its IEEE encoding.
                resolution_m=struct.unpack("f",struct.pack("f",float(self.get_parameter("resolution_m").value)))[0],
                origin_x_m=float(self.get_parameter("origin_x_m").value),
                origin_y_m=float(self.get_parameter("origin_y_m").value),
                width=int(self.get_parameter("width").value),
                height=int(self.get_parameter("height").value),
                obstacle_min_z_m=float(self.get_parameter("obstacle_min_z_m").value),
                obstacle_max_z_m=float(self.get_parameter("obstacle_max_z_m").value),
                unknown_is_occupied=True,
                observed_ground=ground,
                height_filter_applied=_parameter_bool(self.get_parameter("input_height_filter_applied").value),
            )
        except (TypeError, ValueError, RuntimeError) as exc:
            self._reject(str(exc))
            return

        output = OccupancyGrid()
        output.header = msg.header
        output.header.frame_id = self._target_frame
        output.info.resolution = float(grid.resolution_m)
        output.info.width = int(grid.width)
        output.info.height = int(grid.height)
        output.info.origin.position.x = float(grid.origin_x_m)
        output.info.origin.position.y = float(grid.origin_y_m)
        output.info.origin.orientation.w = 1.0
        output.data = list(grid.cells)
        self._publisher.publish(output)
        self._last_rejection = ""


def main(args=None):
    if rclpy is None:
        raise RuntimeError("ROS 2 rclpy and sensor_msgs_py are required on the Humble target")
    rclpy.init(args=args)
    node = LocalObstacleGridNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
