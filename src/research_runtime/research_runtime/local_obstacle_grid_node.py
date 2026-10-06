"""ROS transport for the fail-closed local obstacle-grid contract.

The node only marks measured obstacle cells.  It refuses clouds whose frame is
not already the declared odom frame and refuses to publish until a non-UNKNOWN
map version arrives from the active-road persistence layer.  It therefore
cannot turn a missing TF, missing map identity, or missing return into free
space.
"""
from __future__ import annotations

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

from .grid_map import project_obstacle_points


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
        self.declare_parameter("unknown_is_occupied", True)

        self._target_frame = str(self.get_parameter("target_frame").value)
        self._map_version = "UNKNOWN"
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
        self._last_rejection = ""

    def _map_version_callback(self, msg: String) -> None:
        value = str(msg.data).strip()
        self._map_version = value if _valid_map_version(value) else "UNKNOWN"

    def _reject(self, reason: str) -> None:
        if reason != self._last_rejection:
            self.get_logger().warning("local obstacle grid withheld: %s", reason)
            self._last_rejection = reason

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
            grid, _stats = project_obstacle_points(
                points,
                map_version=self._map_version,
                resolution_m=float(self.get_parameter("resolution_m").value),
                origin_x_m=float(self.get_parameter("origin_x_m").value),
                origin_y_m=float(self.get_parameter("origin_y_m").value),
                width=int(self.get_parameter("width").value),
                height=int(self.get_parameter("height").value),
                obstacle_min_z_m=float(self.get_parameter("obstacle_min_z_m").value),
                obstacle_max_z_m=float(self.get_parameter("obstacle_max_z_m").value),
                unknown_is_occupied=True,
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
    finally:
        node.destroy_node()
        rclpy.shutdown()
