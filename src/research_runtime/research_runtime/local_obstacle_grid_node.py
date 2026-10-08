"""ROS transport for the fail-closed local obstacle-grid contract.

The node only marks measured obstacle cells.  It refuses clouds whose frame is
not already the declared odom frame and refuses to publish until a non-UNKNOWN
map version arrives from the active-road persistence layer.  It therefore
cannot turn a missing TF, missing map identity, or missing return into free
space.
"""
from __future__ import annotations
import struct
import time
import signal
from collections import OrderedDict

try:
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from nav_msgs.msg import OccupancyGrid
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import String
    from research_interfaces.msg import LocalEvidenceGrid2D
except ImportError:  # permits ROS-free contract tests on the developer laptop
    rclpy = None
    OccupancyGrid = PointCloud2 = String = LocalEvidenceGrid2D = object
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
        self._session = str(self.declare_parameter("localization_session_id","UNKNOWN").value)
        self._allow_fixture_ground = _parameter_bool(self.declare_parameter("allow_analytical_grid_fixture",False).value)

        self._target_frame = str(self.get_parameter("target_frame").value)
        self._map_version = "UNKNOWN"
        self._ground = None
        self._ground_stamp = None
        self._ground_support_model = ""
        self._pending_clouds = OrderedDict()
        if not _parameter_bool(self.get_parameter("unknown_is_occupied").value):
            raise ValueError("unknown_is_occupied must remain true")
        self._publisher = self.create_publisher(
            OccupancyGrid, str(self.get_parameter("output_topic").value), 10)
        self._evidence_publisher = self.create_publisher(LocalEvidenceGrid2D,"/research/local_evidence_grid",10)
        self.create_subscription(
            String, str(self.get_parameter("map_version_topic").value),
            self._map_version_callback, 10)
        self.create_subscription(
            PointCloud2, str(self.get_parameter("input_topic").value),
            self._cloud_callback, 10)
        self.create_subscription(LocalEvidenceGrid2D, str(self.get_parameter("observed_ground_topic").value),
                                 self._ground_callback, 10)
        self._last_rejection = ""

    def _map_version_callback(self, msg: String) -> None:
        value = str(msg.data).strip()
        if value != self._map_version:
            self._ground = None
            self._ground_stamp = None
            self._ground_support_model = ""
            self._pending_clouds.clear()
        self._map_version = value if _valid_map_version(value) else "UNKNOWN"

    def _reject(self, reason: str) -> None:
        if reason != self._last_rejection:
            self.get_logger().warning(f"local obstacle grid withheld: {reason}")
            self._last_rejection = reason

    def _ground_callback(self, wrapped):
        # This channel carries positively observed supported ground, never
        # absence-of-obstacle rays. A missing ground producer leaves cells unknown.
        msg = wrapped.grid
        q = msg.info.origin.orientation
        if (msg.header.frame_id != "odom" or not _valid_map_version(self._map_version) or
                wrapped.header != msg.header or wrapped.map_version != self._map_version or
                not self._session or self._session == "UNKNOWN" or
                wrapped.localization_session_id != self._session or
                (wrapped.support_model != "single_scan_flat_dense_v1" and not
                 (self._allow_fixture_ground and wrapped.support_model == "restricted_sensor_fixture_v1")) or
                abs(q.x)+abs(q.y)+abs(q.z) > 1e-9 or abs(q.w-1) > 1e-9):
            self._ground = None
            self._ground_support_model = ""
            return
        try:
            self._ground = LocalObstacleGrid("odom", self._map_version, msg.info.resolution,
                msg.info.origin.position.x, msg.info.origin.position.y,
                msg.info.width, msg.info.height, tuple(msg.data), True)
            self._ground_stamp = rclpy.time.Time.from_msg(msg.header.stamp)
            self._ground_support_model = str(wrapped.support_model)
        except (TypeError, ValueError):
            self._ground = None
            self._ground_support_model = ""
        key = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        pending = self._pending_clouds.get(key)
        if pending is not None and (time.monotonic()-pending[1] <= float(self.get_parameter("ground_timeout_s").value) or
                                    (self._ground is not None and 0 not in self._ground.cells)):
            # Recompute the SAME acquisition when ground arrives after the
            # obstacle cloud, including immediate retraction to UNKNOWN.
            self._cloud_callback(pending[0])

    def _cloud_callback(self, msg: PointCloud2) -> None:
        key = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        if key not in self._pending_clouds:
            self._pending_clouds[key] = (msg,time.monotonic())
        while len(self._pending_clouds)>20:
            self._pending_clouds.popitem(last=False)
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
                # Never refresh old support using a newer cloud's header.
                if age == 0:
                    ground = self._ground
            grid, _stats = project_obstacle_points(
                points,
                map_version=self._map_version,
                # OccupancyGrid resolution is float32. Compare the canonical
                # wire value rather than rejecting 0.3 vs its IEEE encoding.
                resolution_m=struct.unpack("f",struct.pack("f",float(self.get_parameter("resolution_m").value)))[0],
                origin_x_m=ground.origin_x_m if ground else float(self.get_parameter("origin_x_m").value),
                origin_y_m=ground.origin_y_m if ground else float(self.get_parameter("origin_y_m").value),
                width=ground.width if ground else int(self.get_parameter("width").value),
                height=ground.height if ground else int(self.get_parameter("height").value),
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
        self._evidence_publisher.publish(LocalEvidenceGrid2D(header=output.header,
            map_version=self._map_version,localization_session_id=self._session,
            support_model=("analytical_fixture_v1" if ground is not None and
                self._ground_support_model == "restricted_sensor_fixture_v1" else
                "single_scan_flat_dense_with_locked_obstacles_v1"),grid=output))
        self._last_rejection = ""


def main(args=None):
    if rclpy is None:
        raise RuntimeError("ROS 2 rclpy is required on the Humble target")
    # Finish pending application work before destroying the ROS context.
    stopping = False
    previous = {kind: signal.getsignal(kind) for kind in (signal.SIGINT, signal.SIGTERM)}

    def request_stop(_kind, _frame):
        nonlocal stopping
        stopping = True

    node = None
    for kind in previous:
        signal.signal(kind, request_stop)
    try:
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
        if not stopping:
            node = LocalObstacleGridNode()
        while node is not None and rclpy.ok() and not stopping:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node is not None:
                node.destroy_node()
        finally:
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            finally:
                for kind, handler in previous.items():
                    signal.signal(kind, handler)
