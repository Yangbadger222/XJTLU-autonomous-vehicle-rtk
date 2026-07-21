#!/usr/bin/env python3
from __future__ import annotations

import math
import time

from action_msgs.msg import GoalStatus
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Odometry, Path as NavPath
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
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
    ContinuousReadiness,
    LocalOdomWatchdog,
    WatchdogDecision,
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
        self.declare_parameter("local_rate_abort_mps", 3.0)
        self.declare_parameter("local_yaw_rate_abort_radps", 3.0)
        self.declare_parameter("local_rate_abort_count", 3)
        self.declare_parameter("local_catastrophic_rate_mps", 10.0)
        self.declare_parameter("local_catastrophic_yaw_rate_radps", 10.0)

        self.scene_points_file = str(self.get_parameter("scene_points_file").value)
        self.route_frame = str(self.get_parameter("route_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.require_nav_ready = bool(self.get_parameter("require_nav_ready").value)
        self.controller_id = str(self.get_parameter("controller_id").value)
        self.goal_checker_id = str(self.get_parameter("goal_checker_id").value)
        self.max_route_snap_distance_m = float(
            self.get_parameter("max_route_snap_distance_m").value
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

        publish_hz = max(
            1.0, float(self.get_parameter("stop_override_publish_hz").value)
        )
        self.supervision_timer = self.create_timer(
            1.0 / publish_hz, self._supervision_timer_callback
        )
        self._publish_stop_override(True)
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
        self.local_watchdog = self._make_local_watchdog()
        self.local_watchdog_result = None
        self.authority_readiness = ContinuousReadiness(
            self.authority_ready_confirmation_s
        )
        self._publish_stop_override(True)
        self._publish_status("WAITING_FOR_AUTHORITY", f"target={label}")

    def _build_path(self, plan: RoutePlan) -> NavPath:
        points = densify_polyline(plan.points, self.path_density_m)
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
        try:
            plan = self.route_planner.plan(
                start_xy,
                self.requested_goal_xy,
                max_snap_distance_m=self.max_route_snap_distance_m,
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

    def _send_follow_path(self) -> None:
        if self.pending_path is None:
            self._finish_failure("missing_pending_path")
            return
        self.generation += 1
        generation = self.generation
        goal = FollowPath.Goal()
        goal.path = self.pending_path
        goal.controller_id = self.controller_id
        goal.goal_checker_id = self.goal_checker_id
        self.goal_send_pending = True
        self._publish_stop_override(True)
        self._publish_status("FOLLOWING_ROUTE", f"target={self.current_target_label}")
        future = self.follow_path_client.send_goal_async(goal)
        future.add_done_callback(
            lambda completed, token=generation: self._on_goal_response(completed, token)
        )

    def _on_goal_response(self, future, generation: int) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            if generation != self.generation or not self.busy:
                return
            self.goal_send_pending = False
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
            self._finish_failure(f"follow_path_result_failed={exc}")
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
            self.authority_readiness = ContinuousReadiness(
                self.authority_ready_confirmation_s
            )
            self._publish_status(
                "GLOBAL_CORRECTION_HOLD",
                f"target={self.current_target_label}; result={wrapped.status}",
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

    def _stop_callback(self, _: Empty) -> None:
        if not self.busy:
            self._publish_stop_override(True)
            self._publish_status("IDLE", "no_active_goal")
            return
        self.cancel_reason = "USER"
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

        if (
            self.cancel_reason == "AUTHORITY_HOLD"
            and self.hold_started_mono is not None
            and now_mono - self.hold_started_mono > self.global_hold_timeout_s
        ):
            self._finish_failure("authority_hold_timeout")
            return

        ready = self._authority_ready()
        if (self.follow_path_goal_handle is not None or self.goal_send_pending) and not ready:
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
        if self.follow_path_goal_handle is not None or self.goal_send_pending:
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
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
