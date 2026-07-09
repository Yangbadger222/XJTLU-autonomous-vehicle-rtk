#!/usr/bin/env python3

from __future__ import annotations

import math

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputeRoute, FollowPath, NavigateToPose
from nav_msgs.msg import Path as NavPath
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Empty, Int32, String
from tf2_ros import Buffer, TransformException, TransformListener

from gps_waypoint_dispatcher.scene_runtime import default_scene_points_file, load_scene_points, yaw_to_quaternion


class GPSGoalManager(Node):
    def __init__(self) -> None:
        super().__init__("gps_waypoint_dispatcher")

        self.declare_parameter("scene_points_file", str(default_scene_points_file()))
        self.declare_parameter("route_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("navigate_to_anchor_tolerance_m", 2.5)
        self.declare_parameter("use_route_pose_start", False)
        self.declare_parameter("require_nav_ready", True)
        self.declare_parameter("controller_id", "FollowPath")
        self.declare_parameter("goal_checker_id", "general_goal_checker")

        self.scene_points_file = str(self.get_parameter("scene_points_file").value)
        self.route_frame = str(self.get_parameter("route_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.navigate_to_anchor_tolerance_m = float(
            self.get_parameter("navigate_to_anchor_tolerance_m").value
        )
        self.use_route_pose_start = bool(self.get_parameter("use_route_pose_start").value)
        self.require_nav_ready = bool(self.get_parameter("require_nav_ready").value)
        self.controller_id = str(self.get_parameter("controller_id").value)
        self.goal_checker_id = str(self.get_parameter("goal_checker_id").value)

        scene = load_scene_points(self.scene_points_file)
        self.scene_name = scene["scene_name"]
        self.nodes = scene["nodes"]
        self.destination_names = scene["destination_names"]

        self.compute_route_client = ActionClient(self, ComputeRoute, "compute_route")
        self.navigate_to_pose_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.follow_path_client = ActionClient(self, FollowPath, "follow_path")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.status_pub = self.create_publisher(String, "/gps_goal_manager/status", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/gps_waypoint_dispatcher/goal_map", 10)
        self.path_pub = self.create_publisher(NavPath, "/gps_waypoint_dispatcher/path_map", 10)

        self.create_subscription(String, "/gps_system/status", self._system_status_callback, 10)
        self.create_subscription(Int32, "/gps_system/nearest_anchor_id", self._nearest_anchor_id_callback, 10)
        self.create_subscription(String, "/gps_waypoint_dispatcher/goto_name", self._goto_name_callback, 10)
        self.create_subscription(Empty, "/gps_waypoint_dispatcher/stop", self._stop_callback, 10)

        self.system_status = "NO_FIX"
        self.nearest_anchor_id: int | None = None
        self.busy = False
        self.cancel_requested = False
        self.current_target_name: str | None = None
        self.current_dest_id: int | None = None
        self.current_anchor_id: int | None = None
        self.pending_path: NavPath | None = None

        self.compute_route_goal_handle = None
        self.navigate_goal_handle = None
        self.follow_path_goal_handle = None

        destinations = ", ".join(sorted(self.destination_names)) or "(none)"
        self.get_logger().info(
            "GPS goal manager ready: scene=%s destinations=%s" % (self.scene_name, destinations)
        )
        self._publish_status("IDLE", "waiting_for_nav_ready")

    def _publish_status(self, state: str, detail: str = "") -> None:
        message = state if not detail else f"{state}; {detail}"
        self.status_pub.publish(String(data=message))
        self.get_logger().info(message)

    def _system_status_callback(self, msg: String) -> None:
        self.system_status = msg.data.strip() or "NO_FIX"

    def _nearest_anchor_id_callback(self, msg: Int32) -> None:
        if msg.data < 0 or msg.data not in self.nodes:
            self.nearest_anchor_id = None
            return
        self.nearest_anchor_id = int(msg.data)

    def _lookup_current_pose(self) -> PoseStamped | None:
        try:
            transform = self.tf_buffer.lookup_transform(self.route_frame, self.base_frame, Time())
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

    def _lookup_current_xy(self) -> tuple[float, float] | None:
        pose = self._lookup_current_pose()
        if pose is None:
            return None
        return (
            float(pose.pose.position.x),
            float(pose.pose.position.y),
        )

    def _node_pose(self, node: dict) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.route_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(node["x"])
        pose.pose.position.y = float(node["y"])
        pose.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quaternion(0.0)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def _action_servers_ready(self) -> bool:
        checks = [
            (self.compute_route_client, "compute_route"),
            (self.follow_path_client, "follow_path"),
        ]
        if not self.use_route_pose_start:
            checks.append((self.navigate_to_pose_client, "navigate_to_pose"))
        for client, name in checks:
            if not client.wait_for_server(timeout_sec=2.0):
                self._publish_status("FAILED", f"missing_action_server={name}")
                return False
        return True

    def _goto_name_callback(self, msg: String) -> None:
        target_name = msg.data.strip()
        if not target_name:
            self.get_logger().warn("Ignoring empty goto_name request")
            return

        if self.busy:
            if target_name == self.current_target_name:
                self.get_logger().info("Ignoring duplicate goto_name while current target is active")
                return
            self._publish_status("REJECTED", "manager_busy")
            return

        if self.require_nav_ready and self.system_status != "NAV_READY":
            self._publish_status("REJECTED", f"system_status={self.system_status}")
            return

        if not self.use_route_pose_start and self.nearest_anchor_id is None:
            self._publish_status("REJECTED", "no_anchor_matched")
            return

        if target_name not in self.destination_names:
            available = ", ".join(sorted(self.destination_names)) or "(none)"
            self._publish_status("REJECTED", f"unknown_destination={target_name}; available={available}")
            return

        if not self._action_servers_ready():
            return

        self.busy = True
        self.cancel_requested = False
        self.current_target_name = target_name
        self.current_dest_id = int(self.destination_names[target_name])
        self.current_anchor_id = int(self.nearest_anchor_id) if self.nearest_anchor_id is not None else None
        self.pending_path = None

        route_goal = ComputeRoute.Goal()
        if self.use_route_pose_start:
            start_pose = self._lookup_current_pose()
            if start_pose is None:
                self._finish_failure("missing_current_pose_for_pose_start")
                return
            dest_pose = self._node_pose(self.nodes[self.current_dest_id])
            self.goal_pub.publish(dest_pose)
            route_goal.use_poses = True
            route_goal.use_start = True
            route_goal.start = start_pose
            route_goal.goal = dest_pose
            self._publish_status(
                "COMPUTING_ROUTE",
                f"target={target_name}; start=pose_start; dest={self.current_dest_id}",
            )
        else:
            route_goal.use_poses = False
            route_goal.start_id = int(self.current_anchor_id)
            route_goal.goal_id = int(self.current_dest_id)
            self._publish_status(
                "COMPUTING_ROUTE",
                f"target={target_name}; anchor={self.current_anchor_id}; dest={self.current_dest_id}",
            )

        future = self.compute_route_client.send_goal_async(route_goal)
        future.add_done_callback(self._on_compute_route_goal_response)

    def _on_compute_route_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._finish_failure(f"compute_route_send_failed={exc}")
            return

        if goal_handle is None or not goal_handle.accepted:
            self._finish_failure("compute_route_rejected")
            return

        self.compute_route_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_compute_route_result)

    def _on_compute_route_result(self, future) -> None:
        try:
            wrapped = future.result()
        except Exception as exc:
            self._finish_failure(f"compute_route_result_failed={exc}")
            return

        if wrapped.status == GoalStatus.STATUS_CANCELED:
            self._finish_cancelled("compute_route_cancelled")
            return

        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            self._finish_failure(f"compute_route_status={wrapped.status}")
            return

        result = wrapped.result
        if result is None or not result.path.poses:
            self._finish_failure("compute_route_empty_path")
            return

        self.pending_path = result.path
        self.path_pub.publish(result.path)

        if self.use_route_pose_start:
            self._send_follow_path()
            return

        anchor_node = self.nodes[int(self.current_anchor_id)]
        current_xy = self._lookup_current_xy()
        if current_xy is None:
            self.get_logger().warn("Current map pose unavailable; forcing Stage A navigate_to_pose to anchor")
            need_anchor_stage = True
        else:
            distance_to_anchor = math.hypot(current_xy[0] - anchor_node["x"], current_xy[1] - anchor_node["y"])
            need_anchor_stage = distance_to_anchor > self.navigate_to_anchor_tolerance_m

        if need_anchor_stage:
            self._send_navigate_to_anchor(anchor_node, result.path)
        else:
            self._send_follow_path()

    def _send_navigate_to_anchor(self, anchor_node: dict, route_path: NavPath) -> None:
        anchor_pose = PoseStamped()
        anchor_pose.header.frame_id = self.route_frame
        anchor_pose.pose.position.x = float(anchor_node["x"])
        anchor_pose.pose.position.y = float(anchor_node["y"])
        anchor_pose.pose.position.z = 0.0

        if len(route_path.poses) >= 2:
            start = route_path.poses[0].pose.position
            nxt = route_path.poses[1].pose.position
            yaw = math.atan2(float(nxt.y) - float(start.y), float(nxt.x) - float(start.x))
        else:
            yaw = 0.0
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        anchor_pose.pose.orientation.x = qx
        anchor_pose.pose.orientation.y = qy
        anchor_pose.pose.orientation.z = qz
        anchor_pose.pose.orientation.w = qw

        self.goal_pub.publish(anchor_pose)
        self._publish_status(
            "NAVIGATING_TO_ANCHOR",
            f"anchor={anchor_node['name']}; target={self.current_target_name}",
        )

        navigate_goal = NavigateToPose.Goal()
        navigate_goal.pose = anchor_pose
        future = self.navigate_to_pose_client.send_goal_async(navigate_goal)
        future.add_done_callback(self._on_navigate_goal_response)

    def _on_navigate_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._finish_failure(f"navigate_to_anchor_send_failed={exc}")
            return

        if goal_handle is None or not goal_handle.accepted:
            self._finish_failure("navigate_to_anchor_rejected")
            return

        self.navigate_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_navigate_result)

    def _on_navigate_result(self, future) -> None:
        try:
            wrapped = future.result()
        except Exception as exc:
            self._finish_failure(f"navigate_to_anchor_result_failed={exc}")
            return

        if wrapped.status == GoalStatus.STATUS_CANCELED:
            self._finish_cancelled("navigate_to_anchor_cancelled")
            return

        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            self._finish_failure(f"navigate_to_anchor_status={wrapped.status}")
            return

        self._send_follow_path()

    def _send_follow_path(self) -> None:
        if self.pending_path is None:
            self._finish_failure("missing_pending_path")
            return

        self._publish_status(
            "FOLLOWING_ROUTE",
            f"target={self.current_target_name}; anchor={self.current_anchor_id}; dest={self.current_dest_id}",
        )

        follow_goal = FollowPath.Goal()
        follow_goal.path = self.pending_path
        follow_goal.controller_id = self.controller_id
        follow_goal.goal_checker_id = self.goal_checker_id

        future = self.follow_path_client.send_goal_async(follow_goal)
        future.add_done_callback(self._on_follow_path_goal_response)

    def _on_follow_path_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._finish_failure(f"follow_path_send_failed={exc}")
            return

        if goal_handle is None or not goal_handle.accepted:
            self._finish_failure("follow_path_rejected")
            return

        self.follow_path_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_follow_path_result)

    def _on_follow_path_result(self, future) -> None:
        try:
            wrapped = future.result()
        except Exception as exc:
            self._finish_failure(f"follow_path_result_failed={exc}")
            return

        if wrapped.status == GoalStatus.STATUS_CANCELED:
            self._finish_cancelled("follow_path_cancelled")
            return

        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            self._finish_failure(f"follow_path_status={wrapped.status}")
            return

        self._finish_success()

    def _cancel_goal_handle(self, goal_handle, label: str) -> None:
        if goal_handle is None:
            return
        try:
            goal_handle.cancel_goal_async()
            self.get_logger().info(f"cancel requested: {label}")
        except Exception as exc:
            self.get_logger().warn(f"cancel failed for {label}: {exc}")

    def _stop_callback(self, _: Empty) -> None:
        if not self.busy:
            self._publish_status("IDLE", "no_active_goal")
            return

        self.cancel_requested = True
        self._publish_status("CANCEL_REQUESTED", f"target={self.current_target_name}")
        self._cancel_goal_handle(self.compute_route_goal_handle, "compute_route")
        self._cancel_goal_handle(self.navigate_goal_handle, "navigate_to_anchor")
        self._cancel_goal_handle(self.follow_path_goal_handle, "follow_path")

    def _reset_request(self) -> None:
        self.busy = False
        self.cancel_requested = False
        self.current_target_name = None
        self.current_dest_id = None
        self.current_anchor_id = None
        self.pending_path = None
        self.compute_route_goal_handle = None
        self.navigate_goal_handle = None
        self.follow_path_goal_handle = None

    def _finish_success(self) -> None:
        target = self.current_target_name
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
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
