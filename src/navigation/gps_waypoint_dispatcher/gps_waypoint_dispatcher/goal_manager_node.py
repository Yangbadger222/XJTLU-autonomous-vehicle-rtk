#!/usr/bin/env python3
from __future__ import annotations

import math
import time

from action_msgs.msg import GoalStatus
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Odometry, Path as NavPath
from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from std_msgs.msg import Bool, Empty, String
from tf2_ros import Buffer, TransformException, TransformListener

from gps_waypoint_dispatcher.route_graph_planner import (
    RouteGraphPlanner,
    RoutePlan,
    RoutePlanningError,
    densify_polyline,
)
from gps_waypoint_dispatcher.route_safety import (
    BlockedRetryDecision,
    BlockedRetryState,
    ContinuousReadiness,
    LocalOdomWatchdog,
    WatchdogDecision,
)
from gps_waypoint_dispatcher.road_rejoin import (
    RoadKeepoutError,
    RoadKeepoutMap,
    RoadRejoinTarget,
    make_road_rejoin_target,
)
from gps_waypoint_dispatcher.scene_runtime import (
    FixedENUProjector,
    default_scene_points_file,
    load_scene_points,
    quaternion_to_yaw,
    yaw_to_quaternion,
)


class GPSGoalManager(Node):
    def __init__(self) -> None:
        super().__init__("gps_waypoint_dispatcher")

        self.declare_parameter("scene_points_file", str(default_scene_points_file()))
        self.declare_parameter("route_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("require_nav_ready", False)
        self.declare_parameter("controller_id", "FollowPath")
        self.declare_parameter("goal_checker_id", "general_goal_checker")
        self.declare_parameter("max_route_snap_distance_m", 8.0)
        self.declare_parameter("route_snap_candidate_count", 8)
        self.declare_parameter("route_snap_candidate_distance_slack_m", 1.0)
        self.declare_parameter("path_density_m", 0.20)
        self.declare_parameter("goal_success_tolerance_m", 1.0)
        self.declare_parameter("goal_pose_topic", "/goal_pose")
        self.declare_parameter("geo_goal_topic", "/gps_goal")
        self.declare_parameter("stop_override_topic", "/gps_nav/stop_override")
        self.declare_parameter(
            "motion_allowed_topic", "/localization_authority/motion_allowed"
        )
        self.declare_parameter(
            "authority_status_topic", "/localization_authority/status"
        )
        self.declare_parameter("lio_odom_topic", "/fastlio2/lio_odom")
        self.declare_parameter("stop_override_publish_hz", 20.0)
        self.declare_parameter("motion_authority_max_age_s", 0.50)
        self.declare_parameter("authority_ready_confirmation_s", 1.0)
        self.declare_parameter("authority_loss_replan_delay_s", 2.0)
        self.declare_parameter("global_hold_timeout_s", 15.0)
        self.declare_parameter("blocked_retry_delay_s", 2.0)
        self.declare_parameter("blocked_wait_timeout_s", 60.0)
        self.declare_parameter("blocked_recovery_confirmation_s", 3.0)
        self.declare_parameter("local_rate_abort_mps", 3.0)
        self.declare_parameter("local_yaw_rate_abort_radps", 3.0)
        self.declare_parameter("local_rate_abort_count", 3)
        self.declare_parameter("local_catastrophic_rate_mps", 10.0)
        self.declare_parameter("local_catastrophic_yaw_rate_radps", 10.0)
        self.declare_parameter("road_keepout_yaml", "")
        self.declare_parameter("road_rejoin_active_topic", "/gps_nav/road_rejoin_active")
        self.declare_parameter("road_rejoin_max_outside_distance_m", 1.0)
        self.declare_parameter("road_rejoin_max_graph_distance_m", 1.25)
        self.declare_parameter("road_rejoin_timeout_s", 12.0)
        self.declare_parameter("road_rejoin_parameter_timeout_s", 2.0)
        self.declare_parameter("local_costmap_node", "/local_costmap/local_costmap")
        self.declare_parameter(
            "local_keepout_enabled_parameter", "road_keepout_filter.enabled"
        )

        self.scene_points_file = str(self.get_parameter("scene_points_file").value)
        self.route_frame = str(self.get_parameter("route_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.require_nav_ready = bool(self.get_parameter("require_nav_ready").value)
        self.controller_id = str(self.get_parameter("controller_id").value)
        self.goal_checker_id = str(self.get_parameter("goal_checker_id").value)
        self.max_route_snap_distance_m = float(
            self.get_parameter("max_route_snap_distance_m").value
        )
        self.route_snap_candidate_count = int(
            self.get_parameter("route_snap_candidate_count").value
        )
        self.route_snap_candidate_distance_slack_m = float(
            self.get_parameter("route_snap_candidate_distance_slack_m").value
        )
        self.path_density_m = float(self.get_parameter("path_density_m").value)
        self.goal_success_tolerance_m = float(
            self.get_parameter("goal_success_tolerance_m").value
        )
        self.motion_authority_max_age_s = float(
            self.get_parameter("motion_authority_max_age_s").value
        )
        self.authority_ready_confirmation_s = float(
            self.get_parameter("authority_ready_confirmation_s").value
        )
        self.authority_loss_replan_delay_s = max(
            0.0,
            float(self.get_parameter("authority_loss_replan_delay_s").value),
        )
        self.global_hold_timeout_s = float(
            self.get_parameter("global_hold_timeout_s").value
        )
        self.blocked_retry = BlockedRetryState(
            retry_delay_s=float(
                self.get_parameter("blocked_retry_delay_s").value
            ),
            timeout_s=float(self.get_parameter("blocked_wait_timeout_s").value),
            recovery_confirmation_s=float(
                self.get_parameter("blocked_recovery_confirmation_s").value
            ),
        )
        self.road_rejoin_max_outside_distance_m = float(
            self.get_parameter("road_rejoin_max_outside_distance_m").value
        )
        self.road_rejoin_max_graph_distance_m = float(
            self.get_parameter("road_rejoin_max_graph_distance_m").value
        )
        self.road_rejoin_timeout_s = float(
            self.get_parameter("road_rejoin_timeout_s").value
        )
        self.road_rejoin_parameter_timeout_s = float(
            self.get_parameter("road_rejoin_parameter_timeout_s").value
        )
        self.local_keepout_enabled_parameter = str(
            self.get_parameter("local_keepout_enabled_parameter").value
        )

        scene = load_scene_points(self.scene_points_file)
        self.scene_name = scene["scene_name"]
        self.nodes = scene["nodes"]
        self.destination_names = scene["destination_names"]
        self.route_planner = RouteGraphPlanner(self.nodes, scene["edges"])
        origin = scene.get("fixed_origin", {})
        self.projector = FixedENUProjector(
            float(origin["lat"]),
            float(origin["lon"]),
            float(origin.get("alt", 0.0)),
        )
        self.road_keepout_map = self._load_road_keepout_map(
            str(self.get_parameter("road_keepout_yaml").value)
        )

        self.follow_path_client = ActionClient(self, FollowPath, "follow_path")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.status_pub = self.create_publisher(String, "/gps_goal_manager/status", 10)
        self.goal_pub = self.create_publisher(
            PoseStamped, "/gps_waypoint_dispatcher/goal_map", 10
        )
        self.path_pub = self.create_publisher(
            NavPath, "/gps_waypoint_dispatcher/path_map", 10
        )
        self.stop_override_pub = self.create_publisher(
            Bool, str(self.get_parameter("stop_override_topic").value), 10
        )
        self.road_rejoin_pub = self.create_publisher(
            Bool, str(self.get_parameter("road_rejoin_active_topic").value), 10
        )
        local_costmap_node = str(self.get_parameter("local_costmap_node").value)
        self.local_costmap_params = self.create_client(
            SetParameters,
            f"{local_costmap_node.rstrip('/')}/set_parameters",
        )

        self.create_subscription(
            String, "/gps_system/status", self._system_status_callback, 10
        )
        self.create_subscription(
            String, "/gps_waypoint_dispatcher/goto_name", self._goto_name_callback, 10
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("goal_pose_topic").value),
            self._goal_pose_callback,
            10,
        )
        self.create_subscription(
            GeoPoint,
            str(self.get_parameter("geo_goal_topic").value),
            self._geo_goal_callback,
            10,
        )
        self.create_subscription(
            Empty, "/gps_waypoint_dispatcher/stop", self._stop_callback, 10
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("motion_allowed_topic").value),
            self._motion_allowed_callback,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("authority_status_topic").value),
            self._authority_status_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("lio_odom_topic").value),
            self._lio_odom_callback,
            50,
        )

        self.local_watchdog = self._make_local_watchdog()
        self.local_watchdog_result = None
        self.system_status = "NO_FIX"
        self.motion_allowed = False
        self.motion_allowed_mono: float | None = None
        self.authority_status = "STARTUP"
        self.authority_readiness = ContinuousReadiness(
            self.authority_ready_confirmation_s
        )

        self.busy = False
        self.stop_override = True
        self.current_target_label: str | None = None
        self.requested_goal_xy: tuple[float, float] | None = None
        self.snapped_goal_xy: tuple[float, float] | None = None
        self.pending_path: NavPath | None = None
        self.follow_path_goal_handle = None
        self.goal_send_pending = False
        self.cancel_reason: str | None = None
        self.authority_loss_started_mono: float | None = None
        self.hold_started_mono: float | None = None
        self.generation = 0
        self.road_rejoin_phase = "IDLE"
        self.road_rejoin_target: RoadRejoinTarget | None = None
        self.road_rejoin_path: NavPath | None = None
        self.road_rejoin_deadline_mono: float | None = None
        self.road_rejoin_parameter_deadline_mono: float | None = None
        self.road_rejoin_parameter_future = None
        self.road_rejoin_parameter_operation: str | None = None
        self.road_rejoin_restore_outcome: str | None = None

        publish_hz = max(
            1.0, float(self.get_parameter("stop_override_publish_hz").value)
        )
        self.supervision_timer = self.create_timer(
            1.0 / publish_hz, self._supervision_timer_callback
        )
        self._publish_stop_override(True)
        self._publish_road_rejoin_active(False)
        destinations = ", ".join(sorted(self.destination_names)) or "(none)"
        self.get_logger().info(
            "GPS A* goal manager ready: scene=%s destinations=%s"
            % (self.scene_name, destinations)
        )
        self._publish_status("IDLE", "waiting_for_goal")

    def _make_local_watchdog(self) -> LocalOdomWatchdog:
        return LocalOdomWatchdog(
            ordinary_linear_rate_mps=float(
                self.get_parameter("local_rate_abort_mps").value
            ),
            ordinary_yaw_rate_radps=float(
                self.get_parameter("local_yaw_rate_abort_radps").value
            ),
            ordinary_abort_count=int(
                self.get_parameter("local_rate_abort_count").value
            ),
            catastrophic_linear_rate_mps=float(
                self.get_parameter("local_catastrophic_rate_mps").value
            ),
            catastrophic_yaw_rate_radps=float(
                self.get_parameter("local_catastrophic_yaw_rate_radps").value
            ),
        )

    def _publish_status(self, state: str, detail: str = "") -> None:
        message = state if not detail else f"{state}; {detail}"
        self.status_pub.publish(String(data=message))
        self.get_logger().info(message)

    def _publish_stop_override(self, stop: bool) -> None:
        self.stop_override = bool(stop)
        self.stop_override_pub.publish(Bool(data=self.stop_override))

    def _publish_road_rejoin_active(self, active: bool) -> None:
        self.road_rejoin_pub.publish(Bool(data=bool(active)))

    def _load_road_keepout_map(self, yaml_path: str) -> RoadKeepoutMap | None:
        if not yaml_path:
            self.get_logger().warning(
                "ROAD_REJOIN_DISABLED: road_keepout_yaml is not configured"
            )
            return None
        try:
            road_map = RoadKeepoutMap.load(yaml_path)
        except RoadKeepoutError as exc:
            self.get_logger().error(
                f"ROAD_REJOIN_DISABLED: cannot load road keepout map: {exc}"
            )
            return None
        self.get_logger().info(
            "ROAD_REJOIN_READY: %dx%d at %.3fm/cell"
            % (road_map.width, road_map.height, road_map.resolution)
        )
        return road_map

    def _system_status_callback(self, msg: String) -> None:
        self.system_status = msg.data.strip() or "NO_FIX"

    def _motion_allowed_callback(self, msg: Bool) -> None:
        self.motion_allowed = bool(msg.data)
        self.motion_allowed_mono = time.monotonic()

    def _authority_status_callback(self, msg: String) -> None:
        self.authority_status = msg.data.strip() or "UNKNOWN"

    def _lio_odom_callback(self, msg: Odometry) -> None:
        stamp_s = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        yaw = quaternion_to_yaw(
            orientation.x, orientation.y, orientation.z, orientation.w
        )
        self.local_watchdog_result = self.local_watchdog.update(
            stamp_s, float(position.x), float(position.y), yaw
        )

    def _authority_faulted(self) -> bool:
        return "FAULT_HOLD" in self.authority_status or "FAULT_LATCHED" in self.authority_status

    def _authority_ready(self) -> bool:
        if self.motion_allowed_mono is None:
            return False
        age_s = time.monotonic() - self.motion_allowed_mono
        return (
            self.motion_allowed
            and 0.0 <= age_s <= self.motion_authority_max_age_s
            and not self._authority_faulted()
        )

    def _lookup_current_pose(self) -> PoseStamped | None:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.route_frame, self.base_frame, Time()
            )
        except TransformException:
            return None
        pose = PoseStamped()
        pose.header.frame_id = self.route_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(transform.transform.translation.x)
        pose.pose.position.y = float(transform.transform.translation.y)
        pose.pose.position.z = float(transform.transform.translation.z)
        pose.pose.orientation = transform.transform.rotation
        return pose

    def _goal_xy_in_route_frame(self, msg: PoseStamped) -> tuple[float, float] | None:
        source_frame = msg.header.frame_id.strip() or self.route_frame
        x = float(msg.pose.position.x)
        y = float(msg.pose.position.y)
        if source_frame == self.route_frame:
            return x, y
        try:
            transform = self.tf_buffer.lookup_transform(
                self.route_frame, source_frame, Time()
            )
        except TransformException as exc:
            self._publish_status("REJECTED", f"goal_transform_failed={exc}")
            return None
        rotation = transform.transform.rotation
        yaw = quaternion_to_yaw(rotation.x, rotation.y, rotation.z, rotation.w)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        return (
            float(transform.transform.translation.x) + cos_yaw * x - sin_yaw * y,
            float(transform.transform.translation.y) + sin_yaw * x + cos_yaw * y,
        )

    def _goto_name_callback(self, msg: String) -> None:
        target_name = msg.data.strip()
        if target_name not in self.destination_names:
            available = ", ".join(sorted(self.destination_names)) or "(none)"
            self._publish_status(
                "REJECTED", f"unknown_destination={target_name}; available={available}"
            )
            return
        node = self.nodes[int(self.destination_names[target_name])]
        self._accept_request(target_name, (float(node["x"]), float(node["y"])))

    def _goal_pose_callback(self, msg: PoseStamped) -> None:
        goal_xy = self._goal_xy_in_route_frame(msg)
        if goal_xy is not None:
            self._accept_request("map_goal", goal_xy)

    def _geo_goal_callback(self, msg: GeoPoint) -> None:
        if not all(math.isfinite(value) for value in (msg.latitude, msg.longitude)):
            self._publish_status("REJECTED", "nonfinite_geo_goal")
            return
        goal_xy = self.projector.forward(float(msg.latitude), float(msg.longitude))
        self._accept_request("geo_goal", goal_xy)

    def _accept_request(self, label: str, goal_xy: tuple[float, float]) -> None:
        if self.busy:
            same_request = (
                self.current_target_label == label
                and self.requested_goal_xy is not None
                and math.hypot(
                    goal_xy[0] - self.requested_goal_xy[0],
                    goal_xy[1] - self.requested_goal_xy[1],
                )
                <= 1e-3
            )
            if same_request:
                self.get_logger().debug(f"Ignoring duplicate goal request: {label}")
                return
            self._publish_status("REJECTED", "manager_busy")
            return
        if self.require_nav_ready and self.system_status != "NAV_READY":
            self._publish_status("REJECTED", f"system_status={self.system_status}")
            return
        if not self.follow_path_client.wait_for_server(timeout_sec=2.0):
            self._publish_status("FAILED", "missing_action_server=follow_path")
            return
        if self._lookup_current_pose() is None:
            self._publish_status("FAILED", "missing_current_map_pose")
            return

        self.busy = True
        self.current_target_label = label
        self.requested_goal_xy = goal_xy
        self.snapped_goal_xy = None
        self.pending_path = None
        self.cancel_reason = None
        self.authority_loss_started_mono = None
        self.hold_started_mono = time.monotonic()
        self.blocked_retry.clear()
        self.local_watchdog = self._make_local_watchdog()
        self.local_watchdog_result = None
        self.authority_readiness = ContinuousReadiness(
            self.authority_ready_confirmation_s
        )
        self._publish_stop_override(True)
        self._publish_status("WAITING_FOR_AUTHORITY", f"target={label}")

    def _build_path(self, plan: RoutePlan) -> NavPath:
        return self._build_path_from_points(plan.points)

    def _build_path_from_points(
        self, source_points: tuple[tuple[float, float], ...]
    ) -> NavPath:
        points = densify_polyline(source_points, self.path_density_m)
        path = NavPath()
        path.header.frame_id = self.route_frame
        path.header.stamp = self.get_clock().now().to_msg()
        for index, point in enumerate(points):
            if len(points) == 1:
                yaw = 0.0
            elif index + 1 < len(points):
                nxt = points[index + 1]
                yaw = math.atan2(nxt[1] - point[1], nxt[0] - point[0])
            else:
                previous = points[index - 1]
                yaw = math.atan2(point[1] - previous[1], point[0] - previous[0])
            qx, qy, qz, qw = yaw_to_quaternion(yaw)
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            path.poses.append(pose)
        return path

    def _plan_from_current_pose(self) -> bool:
        current_pose = self._lookup_current_pose()
        if current_pose is None or self.requested_goal_xy is None:
            self._finish_failure("missing_pose_or_goal_for_replan")
            return False
        start_xy = (
            float(current_pose.pose.position.x),
            float(current_pose.pose.position.y),
        )
        if self._start_road_rejoin_if_needed(start_xy):
            return False
        try:
            plan = self.route_planner.plan(
                start_xy,
                self.requested_goal_xy,
                max_snap_distance_m=self.max_route_snap_distance_m,
                snap_candidate_count=self.route_snap_candidate_count,
                snap_candidate_distance_slack_m=(
                    self.route_snap_candidate_distance_slack_m
                ),
            )
        except RoutePlanningError as exc:
            self._finish_failure(f"astar_failed={exc}")
            return False
        self.pending_path = self._build_path(plan)
        self.snapped_goal_xy = plan.points[-1]
        self.path_pub.publish(self.pending_path)
        if self.pending_path.poses:
            self.goal_pub.publish(self.pending_path.poses[-1])
        self._publish_status(
            "ROUTE_PLANNED",
            "target=%s; cost=%.2f; start_snap=%.2f; goal_snap=%.2f; poses=%d"
            % (
                self.current_target_label,
                plan.graph_cost_m,
                plan.start_snap_distance_m,
                plan.goal_snap_distance_m,
                len(self.pending_path.poses),
            ),
        )
        return True

    def _start_road_rejoin_if_needed(self, start_xy: tuple[float, float]) -> bool:
        """Start a bounded re-entry when the current pose is just off-road.

        ``True`` means this call handled planning, either by beginning the
        rejoin state machine or by failing closed.  Normal route planning can
        continue only when the current pose is already in the drivable mask or
        no compiled mask is configured.
        """

        if self.road_keepout_map is None or self.road_keepout_map.is_drivable(*start_xy):
            return False
        try:
            projection = self.route_planner.nearest_edge(start_xy)
            decision = make_road_rejoin_target(
                self.road_keepout_map,
                start_xy,
                (projection.x, projection.y),
                self.road_rejoin_max_outside_distance_m,
                self.road_rejoin_max_graph_distance_m,
            )
        except (RoutePlanningError, ValueError) as exc:
            self._finish_failure(f"road_rejoin_projection_failed={exc}")
            return True
        if not decision.approved:
            self._finish_failure("road_rejoin_unsafe; %s" % decision.status_fields())
            return True

        target = decision.target
        assert target is not None
        self.road_rejoin_target = target
        self.road_rejoin_path = self._build_path_from_points(
            (start_xy, (target.x, target.y))
        )
        self.path_pub.publish(self.road_rejoin_path)
        if self.road_rejoin_path.poses:
            self.goal_pub.publish(self.road_rejoin_path.poses[-1])
        now_mono = time.monotonic()
        self.road_rejoin_phase = "DISABLING_LOCAL_KEEPOUT"
        self.road_rejoin_deadline_mono = now_mono + self.road_rejoin_timeout_s
        self.road_rejoin_parameter_deadline_mono = (
            now_mono + self.road_rejoin_parameter_timeout_s
        )
        self.road_rejoin_parameter_future = None
        self.road_rejoin_restore_outcome = None
        self._publish_stop_override(True)
        self._publish_road_rejoin_active(True)
        self._publish_status(
            "ROAD_REJOIN_PREPARE",
            "target=%s; outside_m=%.2f; graph_m=%.2f; x=%.2f; y=%.2f; %s"
            % (
                self.current_target_label,
                target.outside_distance_m,
                target.graph_distance_m,
                target.x,
                target.y,
                decision.status_fields(),
            ),
        )
        return True

    def _send_follow_path(self, *, road_rejoin: bool = False) -> None:
        path = self.road_rejoin_path if road_rejoin else self.pending_path
        if path is None:
            self._finish_failure("missing_pending_path")
            return
        self.generation += 1
        generation = self.generation
        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = self.controller_id
        goal.goal_checker_id = self.goal_checker_id
        self.goal_send_pending = True
        self._publish_stop_override(True)
        if road_rejoin:
            self._publish_status(
                "ROAD_REJOIN_FOLLOWING", f"target={self.current_target_label}"
            )
        else:
            self._publish_status("FOLLOWING_ROUTE", f"target={self.current_target_label}")
        future = self.follow_path_client.send_goal_async(goal)
        future.add_done_callback(
            lambda completed, token=generation: self._on_goal_response(completed, token)
        )

    def _road_rejoin_active(self) -> bool:
        return self.road_rejoin_phase != "IDLE"

    @staticmethod
    def _parameter_results_successful(results) -> bool:
        return bool(results) and all(
            bool(getattr(result, "successful", False)) for result in results
        )

    def _request_keepout_parameter(self, enabled: bool) -> bool:
        if self.road_rejoin_parameter_future is not None:
            return True
        if not self.local_costmap_params.wait_for_service(timeout_sec=0.0):
            return False
        try:
            request = SetParameters.Request()
            request.parameters = [
                Parameter(
                    name=self.local_keepout_enabled_parameter,
                    value=bool(enabled),
                ).to_parameter_msg()
            ]
            self.road_rejoin_parameter_future = self.local_costmap_params.call_async(
                request
            )
            self.road_rejoin_parameter_operation = "ENABLE" if enabled else "DISABLE"
        except Exception as exc:
            self.get_logger().warning(
                "ROAD_REJOIN parameter request failed to start: %s" % exc
            )
            return False
        return True

    def _consume_keepout_parameter_result(self) -> tuple[str, bool, str] | None:
        future = self.road_rejoin_parameter_future
        if future is None or not future.done():
            return None
        operation = getattr(self, "road_rejoin_parameter_operation", "UNKNOWN")
        self.road_rejoin_parameter_future = None
        self.road_rejoin_parameter_operation = None
        try:
            response = future.result()
        except Exception as exc:
            return operation, False, str(exc)
        results = response.results if response is not None else []
        if self._parameter_results_successful(results):
            return operation, True, ""
        reasons = [str(getattr(result, "reason", "")) for result in results or []]
        return operation, False, "; ".join(reason for reason in reasons if reason)

    def _begin_road_rejoin_restore(self, outcome: str) -> None:
        if not self._road_rejoin_active():
            return
        self._publish_stop_override(True)
        self.road_rejoin_phase = "RESTORING_LOCAL_KEEPOUT"
        self.road_rejoin_restore_outcome = outcome
        self.road_rejoin_parameter_deadline_mono = (
            time.monotonic() + self.road_rejoin_parameter_timeout_s
        )
        self._publish_status(
            "ROAD_REJOIN_RESTORE",
            f"target={self.current_target_label}; outcome={outcome}",
        )

    def _complete_road_rejoin_restore(self) -> None:
        outcome = self.road_rejoin_restore_outcome or "FAILED"
        self.road_rejoin_phase = "IDLE"
        self.road_rejoin_target = None
        self.road_rejoin_path = None
        self.road_rejoin_deadline_mono = None
        self.road_rejoin_parameter_deadline_mono = None
        self.road_rejoin_parameter_future = None
        self.road_rejoin_parameter_operation = None
        self.road_rejoin_restore_outcome = None
        self._publish_road_rejoin_active(False)

        if not self.busy:
            return
        if outcome == "RESUME":
            self._publish_status(
                "ROAD_REJOIN_COMPLETE", f"target={self.current_target_label}"
            )
            if self._authority_ready() and self._plan_from_current_pose():
                self._send_follow_path()
            return
        if outcome == "AUTHORITY_HOLD":
            self.authority_readiness = ContinuousReadiness(
                self.authority_ready_confirmation_s
            )
            self._publish_status(
                "GLOBAL_CORRECTION_HOLD", f"target={self.current_target_label}"
            )
            return
        if outcome == "USER":
            self._finish_cancelled("user_stop")
            return
        self._finish_failure(outcome.lower())

    def _poll_road_rejoin(self, now_mono: float) -> bool:
        """Advance the fail-closed local-keepout handoff.

        Returns ``True`` while the ordinary route state machine must remain
        paused.  Every exit either restores the local road filter or leaves the
        guard publishing an explicit stop.
        """

        if not self._road_rejoin_active():
            return False

        result = self._consume_keepout_parameter_result()
        if self.road_rejoin_phase == "DISABLING_LOCAL_KEEPOUT":
            if result is not None:
                operation, successful, reason = result
                if operation != "DISABLE" or not successful:
                    self._publish_status(
                        "ROAD_REJOIN_ABORT",
                        "local_keepout_disable_failed=%s" % (reason or operation),
                    )
                    self._begin_road_rejoin_restore("ROAD_REJOIN_KEEP_OUT_DISABLE")
                    return True
                self.road_rejoin_phase = "FOLLOWING"
                self._publish_status(
                    "ROAD_REJOIN_LOCAL_KEEPOUT_DISABLED",
                    f"target={self.current_target_label}",
                )
                self._send_follow_path(road_rejoin=True)
                return True
            if now_mono > (self.road_rejoin_parameter_deadline_mono or now_mono):
                self._publish_status(
                    "ROAD_REJOIN_ABORT", "local_keepout_disable_timeout"
                )
                self._begin_road_rejoin_restore("ROAD_REJOIN_KEEP_OUT_TIMEOUT")
                return True
            self._request_keepout_parameter(False)
            return True

        if self.road_rejoin_phase == "FOLLOWING":
            if now_mono > (self.road_rejoin_deadline_mono or now_mono):
                if self.cancel_reason is None:
                    self.cancel_reason = "ROAD_REJOIN_TIMEOUT"
                    self._publish_status(
                        "ROAD_REJOIN_ABORT",
                        f"target={self.current_target_label}; timeout",
                    )
                    self._request_cancel()
            return True

        if self.road_rejoin_phase != "RESTORING_LOCAL_KEEPOUT":
            self._begin_road_rejoin_restore("ROAD_REJOIN_UNKNOWN_STATE")
            return True

        if result is not None:
            operation, successful, reason = result
            if operation == "DISABLE" and successful:
                # A user/authority stop can arrive while the disable call is
                # in flight.  Restore only after its result is known.
                self.road_rejoin_parameter_deadline_mono = (
                    now_mono + self.road_rejoin_parameter_timeout_s
                )
            elif operation == "ENABLE" and successful:
                self._complete_road_rejoin_restore()
                return True
            elif operation == "DISABLE":
                self._complete_road_rejoin_restore()
                return True
            else:
                self.get_logger().warning(
                    "ROAD_REJOIN restore parameter failed: %s" % (reason or operation)
                )

        if self.road_rejoin_parameter_future is None:
            self._request_keepout_parameter(True)
        if now_mono > (self.road_rejoin_parameter_deadline_mono or now_mono):
            # Keep the vehicle stopped and continue retrying.  Forgetting the
            # request here would risk returning to normal driving with the
            # local road boundary still disabled.
            self.road_rejoin_parameter_deadline_mono = (
                now_mono + self.road_rejoin_parameter_timeout_s
            )
            self._publish_status(
                "ROAD_REJOIN_RESTORE_WAIT",
                f"target={self.current_target_label}; local_keepout_unconfirmed",
            )
        return True

    def _on_goal_response(self, future, generation: int) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            if generation != self.generation or not self.busy:
                return
            self.goal_send_pending = False
            if self.road_rejoin_phase == "FOLLOWING":
                self._begin_road_rejoin_restore("ROAD_REJOIN_FOLLOW_PATH_SEND")
                return
            self._finish_failure(f"follow_path_send_failed={exc}")
            return
        if generation != self.generation or not self.busy:
            if goal_handle is not None and goal_handle.accepted:
                try:
                    goal_handle.cancel_goal_async()
                except Exception as exc:
                    self.get_logger().warning(
                        f"Failed to cancel stale FollowPath goal: {exc}"
                    )
            return
        self.goal_send_pending = False
        if goal_handle is None or not goal_handle.accepted:
            if self.road_rejoin_phase == "FOLLOWING":
                self._begin_road_rejoin_restore("ROAD_REJOIN_FOLLOW_PATH_REJECTED")
                return
            self._finish_failure("follow_path_rejected")
            return
        self.follow_path_goal_handle = goal_handle
        if self.cancel_reason is not None:
            self._request_cancel()
        elif not self._authority_ready():
            self.authority_loss_started_mono = (
                self.authority_loss_started_mono or time.monotonic()
            )
            self._publish_stop_override(True)
        else:
            self.authority_loss_started_mono = None
            self._publish_stop_override(False)
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda completed, token=generation: self._on_follow_path_result(
                completed, token
            )
        )

    def _request_cancel(self) -> None:
        self._publish_stop_override(True)
        if self.follow_path_goal_handle is None:
            return
        future = self.follow_path_goal_handle.cancel_goal_async()
        generation = self.generation
        future.add_done_callback(
            lambda completed, token=generation: self._on_cancel_response(
                completed, token
            )
        )

    def _on_cancel_response(self, future, generation: int) -> None:
        if generation != self.generation or not self.busy:
            return
        try:
            response = future.result()
        except Exception as exc:
            self._finish_failure(f"cancel_failed={exc}")
            return
        if response is None or not response.goals_canceling:
            self._publish_status("CANCEL_PENDING", "cancel_not_acknowledged")

    def _on_follow_path_result(self, future, generation: int) -> None:
        if generation != self.generation or not self.busy:
            return
        self._publish_stop_override(True)
        self.follow_path_goal_handle = None
        try:
            wrapped = future.result()
        except Exception as exc:
            if self.road_rejoin_phase == "FOLLOWING":
                self._begin_road_rejoin_restore("ROAD_REJOIN_FOLLOW_PATH_RESULT")
                return
            self._finish_failure(f"follow_path_result_failed={exc}")
            return

        if self.road_rejoin_phase == "FOLLOWING":
            if wrapped.status == GoalStatus.STATUS_SUCCEEDED:
                current_pose = self._lookup_current_pose()
                returned_to_road = bool(
                    current_pose is not None
                    and self.road_keepout_map is not None
                    and self.road_keepout_map.is_drivable(
                        float(current_pose.pose.position.x),
                        float(current_pose.pose.position.y),
                    )
                )
                self._begin_road_rejoin_restore(
                    "RESUME" if returned_to_road else "ROAD_REJOIN_NO_ROAD_ENTRY"
                )
                return
            if wrapped.status == GoalStatus.STATUS_CANCELED:
                reason = self.cancel_reason or "ROAD_REJOIN_CANCELLED"
                self.cancel_reason = None
                outcome = {
                    "AUTHORITY_HOLD": "AUTHORITY_HOLD",
                    "USER": "USER",
                }.get(reason, reason)
                self._begin_road_rejoin_restore(outcome)
                return
            self._begin_road_rejoin_restore(
                f"ROAD_REJOIN_FOLLOW_PATH_STATUS_{wrapped.status}"
            )
            return

        if wrapped.status == GoalStatus.STATUS_CANCELED:
            reason = self.cancel_reason or "CANCELLED"
            self.cancel_reason = None
            if reason == "AUTHORITY_HOLD":
                self.authority_readiness = ContinuousReadiness(
                    self.authority_ready_confirmation_s
                )
                self._publish_status(
                    "GLOBAL_CORRECTION_HOLD", f"target={self.current_target_label}"
                )
                return
            if reason == "USER":
                self._finish_cancelled("user_stop")
                return
            self._finish_failure(reason.lower())
            return

        if self.cancel_reason == "AUTHORITY_HOLD":
            self.cancel_reason = None
            self.blocked_retry.clear()
            self.authority_readiness = ContinuousReadiness(
                self.authority_ready_confirmation_s
            )
            self._publish_status(
                "GLOBAL_CORRECTION_HOLD",
                f"target={self.current_target_label}; result={wrapped.status}",
            )
            return
        if self.cancel_reason == "BLOCKED_TIMEOUT":
            self.cancel_reason = None
            self._finish_failure("blocked_timeout")
            return

        local_odom_healthy = not (
            self.local_watchdog_result is not None
            and self.local_watchdog_result.decision is WatchdogDecision.LOCAL_ABORT
        )
        if (
            wrapped.status == GoalStatus.STATUS_ABORTED
            and self.cancel_reason is None
            and local_odom_healthy
            and not self._authority_faulted()
        ):
            blocked = self.blocked_retry.enter(time.monotonic())
            self.authority_loss_started_mono = None
            self.hold_started_mono = None
            self._publish_status(
                "BLOCKED_WAIT",
                "target=%s; result=%d; %s; retry_in=%.2fs; timeout=%.1fs"
                % (
                    self.current_target_label,
                    wrapped.status,
                    self._follow_path_abort_context(wrapped),
                    self.blocked_retry.retry_delay_s,
                    self.blocked_retry.timeout_s - blocked.elapsed_s,
                ),
            )
            return
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            self._finish_failure(f"follow_path_status={wrapped.status}")
            return
        current_pose = self._lookup_current_pose()
        if current_pose is None or self.snapped_goal_xy is None:
            self._finish_failure("success_without_current_pose")
            return
        remaining_m = math.hypot(
            float(current_pose.pose.position.x) - self.snapped_goal_xy[0],
            float(current_pose.pose.position.y) - self.snapped_goal_xy[1],
        )
        if remaining_m > self.goal_success_tolerance_m:
            self._finish_failure(
                "nav2_false_success_remaining=%.2f" % remaining_m
            )
            return
        self._finish_success()

    def _follow_path_abort_context(self, wrapped) -> str:
        result = getattr(wrapped, "result", None)
        error_code = getattr(result, "error_code", "unknown")
        error_msg = str(getattr(result, "error_msg", "")).replace(";", ",")
        pose = self._lookup_current_pose()
        if pose is None:
            return "nav2_error_code=%s; nav2_error_msg=%s; current_xy=unavailable" % (
                error_code,
                error_msg or "none",
            )
        x = float(pose.pose.position.x)
        y = float(pose.pose.position.y)
        keepout_drivable = (
            self.road_keepout_map.is_drivable(x, y)
            if self.road_keepout_map is not None
            else None
        )
        return (
            "nav2_error_code=%s; nav2_error_msg=%s; current_xy=(%.2f,%.2f); "
            "keepout_drivable=%s"
            % (error_code, error_msg or "none", x, y, keepout_drivable)
        )

    def _stop_callback(self, _: Empty) -> None:
        if not self.busy:
            self._publish_stop_override(True)
            self._publish_status("IDLE", "no_active_goal")
            return
        self.cancel_reason = "USER"
        if self._road_rejoin_active():
            self._publish_status("CANCEL_REQUESTED", f"target={self.current_target_label}")
            if self.follow_path_goal_handle is not None or self.goal_send_pending:
                self._request_cancel()
            else:
                self._begin_road_rejoin_restore("USER")
            return
        if self.follow_path_goal_handle is None and not self.goal_send_pending:
            self._finish_cancelled("user_stop")
            return
        self._publish_status("CANCEL_REQUESTED", f"target={self.current_target_label}")
        self._request_cancel()

    def _supervision_timer_callback(self) -> None:
        self.stop_override_pub.publish(Bool(data=self.stop_override))
        if not self.busy:
            return
        now_mono = time.monotonic()
        if self._road_rejoin_active():
            local_abort = bool(
                self.local_watchdog_result is not None
                and self.local_watchdog_result.decision is WatchdogDecision.LOCAL_ABORT
            )
            authority_problem = self._authority_faulted() or not self._authority_ready()
            if local_abort:
                if self.road_rejoin_phase == "FOLLOWING" and (
                    self.follow_path_goal_handle is not None or self.goal_send_pending
                ):
                    self.cancel_reason = "ROAD_REJOIN_LOCAL_ODOM_INVALID"
                    self._request_cancel()
                elif self.road_rejoin_phase != "RESTORING_LOCAL_KEEPOUT":
                    self._begin_road_rejoin_restore("ROAD_REJOIN_LOCAL_ODOM_INVALID")
                return
            if authority_problem and self.road_rejoin_phase != "RESTORING_LOCAL_KEEPOUT":
                self._publish_stop_override(True)
                if self.road_rejoin_phase == "FOLLOWING" and (
                    self.follow_path_goal_handle is not None or self.goal_send_pending
                ):
                    self.cancel_reason = "AUTHORITY_HOLD"
                    self._request_cancel()
                elif self.road_rejoin_phase != "FOLLOWING":
                    self._begin_road_rejoin_restore("AUTHORITY_HOLD")
                return
            self._poll_road_rejoin(now_mono)
            return
        if (
            self.local_watchdog_result is not None
            and self.local_watchdog_result.decision is WatchdogDecision.LOCAL_ABORT
        ):
            if self.cancel_reason != "LOCAL_ODOM_INVALID":
                self.cancel_reason = "LOCAL_ODOM_INVALID"
                self._publish_status(
                    "ODOM_DIVERGENCE_ABORT",
                    self.local_watchdog_result.reason or "LOCAL_ODOM_INVALID",
                )
                if self.follow_path_goal_handle is None and not self.goal_send_pending:
                    self._finish_failure("local_odom_invalid")
                else:
                    self._request_cancel()
            return
        if self._authority_faulted():
            if self.cancel_reason != "AUTHORITY_FAULT":
                self.cancel_reason = "AUTHORITY_FAULT"
                self._publish_status("GLOBAL_CORRECTION_ABORT", self.authority_status)
                if self.follow_path_goal_handle is None and not self.goal_send_pending:
                    self._finish_failure("authority_fault")
                else:
                    self._request_cancel()
            return

        ready = self._authority_ready()
        action_active = (
            self.follow_path_goal_handle is not None or self.goal_send_pending
        )
        if action_active and self.blocked_retry.active:
            blocked = self.blocked_retry.poll(
                now_mono,
                retry_allowed=False,
            )
            if blocked.decision is BlockedRetryDecision.TIMEOUT:
                if self.cancel_reason != "BLOCKED_TIMEOUT":
                    self.cancel_reason = "BLOCKED_TIMEOUT"
                    self._publish_status(
                        "BLOCKED_TIMEOUT",
                        "blocked_for=%.1fs; retries=%d"
                        % (blocked.elapsed_s, blocked.retry_count),
                    )
                    self._request_cancel()
                return
        local_motion_observed = bool(
            self.local_watchdog_result is not None
            and (
                self.local_watchdog_result.linear_rate_mps >= 0.05
                or self.local_watchdog_result.yaw_rate_radps >= 0.05
            )
        )
        if (
            self.follow_path_goal_handle is not None
            and ready
            and self.blocked_retry.confirm_action_running(
                now_mono,
                moving=local_motion_observed,
            )
        ):
            self._publish_status(
                "BLOCKED_RECOVERED",
                f"target={self.current_target_label}; controller_stable",
            )

        if (
            self.cancel_reason == "AUTHORITY_HOLD"
            and self.hold_started_mono is not None
            and now_mono - self.hold_started_mono > self.global_hold_timeout_s
        ):
            self._finish_failure("authority_hold_timeout")
            return

        if action_active and not ready:
            self._publish_stop_override(True)
            if self.authority_loss_started_mono is None:
                self.authority_loss_started_mono = now_mono
                self._publish_status(
                    "AUTHORITY_GRACE",
                    "motion_authority_not_ready; replan_after=%.2fs"
                    % self.authority_loss_replan_delay_s,
                )
            authority_loss_s = now_mono - self.authority_loss_started_mono
            if (
                self.cancel_reason is None
                and authority_loss_s >= self.authority_loss_replan_delay_s
            ):
                self.cancel_reason = "AUTHORITY_HOLD"
                self.hold_started_mono = now_mono
                self._publish_status(
                    "GLOBAL_CORRECTION_HOLD",
                    "motion_authority_not_ready_for=%.2fs" % authority_loss_s,
                )
                self._request_cancel()
            return
        if action_active:
            if self.cancel_reason is not None:
                self._publish_stop_override(True)
                return
            if self.authority_loss_started_mono is not None:
                self.authority_loss_started_mono = None
                self._publish_status(
                    "FOLLOWING_ROUTE", "motion_authority_recovered"
                )
            self._publish_stop_override(False)
            return
        if self.blocked_retry.active:
            self._publish_stop_override(True)
            blocked = self.blocked_retry.poll(
                now_mono,
                retry_allowed=ready,
            )
            if blocked.decision is BlockedRetryDecision.TIMEOUT:
                self._finish_failure(
                    "blocked_timeout=%.1fs; retries=%d"
                    % (blocked.elapsed_s, blocked.retry_count)
                )
                return
            if blocked.decision is BlockedRetryDecision.RETRY:
                self._publish_status(
                    "BLOCKED_RETRY",
                    "target=%s; attempt=%d; blocked_for=%.1fs"
                    % (
                        self.current_target_label,
                        blocked.retry_count,
                        blocked.elapsed_s,
                    ),
                )
                if self._plan_from_current_pose():
                    self._send_follow_path()
            return
        if self.hold_started_mono is None:
            self.hold_started_mono = now_mono
        if now_mono - self.hold_started_mono > self.global_hold_timeout_s:
            self._finish_failure("authority_hold_timeout")
            return
        if not self.authority_readiness.update(ready=ready, now_s=now_mono):
            return
        self.hold_started_mono = None
        if self._plan_from_current_pose():
            self._send_follow_path()

    def _reset_request(self) -> None:
        active_goal_handle = self.follow_path_goal_handle
        if active_goal_handle is not None:
            try:
                active_goal_handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warning(
                    f"Best-effort FollowPath cancellation failed during reset: {exc}"
                )
        self.generation += 1
        self.busy = False
        self.current_target_label = None
        self.requested_goal_xy = None
        self.snapped_goal_xy = None
        self.pending_path = None
        self.follow_path_goal_handle = None
        self.goal_send_pending = False
        self.cancel_reason = None
        self.authority_loss_started_mono = None
        self.hold_started_mono = None
        self.blocked_retry.clear()
        self._publish_road_rejoin_active(False)
        self._publish_stop_override(True)

    def _finish_success(self) -> None:
        target = self.current_target_label
        self._publish_status("SUCCEEDED", f"target={target}")
        self._reset_request()

    def _finish_cancelled(self, detail: str) -> None:
        self._publish_status("CANCELLED", detail)
        self._reset_request()

    def _finish_failure(self, detail: str) -> None:
        self._publish_status("FAILED", detail)
        self._reset_request()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GPSGoalManager()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node._publish_stop_override(True)
            node._publish_road_rejoin_active(False)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
