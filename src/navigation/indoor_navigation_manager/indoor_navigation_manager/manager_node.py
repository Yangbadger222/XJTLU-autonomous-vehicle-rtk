import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from interface.action import NavigateNamedDestination
from interface.msg import LocalizationStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node

from indoor_navigation_manager.destinations import load_destinations, resolve_destination


def quaternion_from_yaw(yaw):
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


class IndoorNavigationManager(Node):
    def __init__(self):
        super().__init__("indoor_navigation_manager")
        self.declare_parameter("destinations_file", "")
        self.declare_parameter("map_id", "")
        self.declare_parameter("localization_topic", "/localizer/status")
        self.declare_parameter("navigate_to_pose_action", "/navigate_to_pose")

        destinations_file = str(self.get_parameter("destinations_file").value)
        if not destinations_file:
            raise RuntimeError("destinations_file is required")
        self.catalog = load_destinations(destinations_file)
        configured_map_id = str(self.get_parameter("map_id").value).strip()
        self.map_id = configured_map_id or self.catalog["map_id"]
        if self.catalog["map_id"] and self.catalog["map_id"] != self.map_id:
            raise RuntimeError("destination map_id does not match configured map_id")

        localization_topic = str(self.get_parameter("localization_topic").value)
        nav_action = str(self.get_parameter("navigate_to_pose_action").value)
        self.localization_state = "UNINITIALIZED"
        self.localized = False
        self.nav_goal_handle = None
        self.named_goal_handle = None
        self.goal_reserved = False
        self.cancel_requested = False
        self.resolved_name = ""
        self.remaining_distance = float("nan")
        self.create_subscription(LocalizationStatus, localization_topic, self.on_localization, 10)
        self.nav_client = ActionClient(self, NavigateToPose, nav_action)
        self.server = ActionServer(
            self,
            NavigateNamedDestination,
            "/navigate_named_destination",
            execute_callback=self.execute,
            goal_callback=self.on_goal,
            cancel_callback=self.on_cancel,
        )

    def on_localization(self, msg):
        self.localization_state = msg.state_label
        self.localized = bool(
            msg.localized
            and msg.sensors_ready
            and msg.state
            in (LocalizationStatus.LOCALIZED, LocalizationStatus.DEGRADED)
        )
        if (
            not self.localized
            and self.nav_goal_handle is not None
            and not self.cancel_requested
        ):
            self.cancel_requested = True
            self.nav_goal_handle.cancel_goal_async()

    def on_goal(self, request):
        if self.goal_reserved or self.nav_goal_handle is not None or self.named_goal_handle is not None:
            return GoalResponse.REJECT
        if request.backend and request.backend != "indoor":
            return GoalResponse.REJECT
        if request.map_id and request.map_id != self.map_id:
            return GoalResponse.REJECT
        if resolve_destination(self.catalog, request.destination_name) is None:
            return GoalResponse.REJECT
        if not self.localized:
            return GoalResponse.REJECT
        self.goal_reserved = True
        self.cancel_requested = False
        return GoalResponse.ACCEPT

    def on_cancel(self, _goal_handle):
        self.cancel_requested = True
        if self.nav_goal_handle is not None:
            self.nav_goal_handle.cancel_goal_async()
        return CancelResponse.ACCEPT

    def on_nav_feedback(self, feedback_msg):
        self.remaining_distance = float(feedback_msg.feedback.distance_remaining)
        if self.named_goal_handle is not None:
            feedback = NavigateNamedDestination.Feedback()
            feedback.state = "CANCELING" if self.cancel_requested else "NAVIGATING"
            feedback.resolved_name = self.resolved_name
            feedback.remaining_distance = self.remaining_distance
            feedback.localization_state = self.localization_state
            self.named_goal_handle.publish_feedback(feedback)

    async def execute(self, goal_handle):
        result = NavigateNamedDestination.Result()
        self.named_goal_handle = goal_handle
        try:
            destination = resolve_destination(
                self.catalog, goal_handle.request.destination_name
            )
            if destination is None or not self.localized:
                result.success = False
                result.error_code = 1
                result.message = "destination unavailable or localization not ready"
                goal_handle.abort()
                return result

            if self.cancel_requested or goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.error_code = 4
                result.message = "navigation canceled before dispatch"
                return result

            if not self.nav_client.server_is_ready():
                result.success = False
                result.error_code = 2
                result.message = "navigate_to_pose action unavailable"
                goal_handle.abort()
                return result

            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = "map"
            pose.pose.position.x = destination["x"]
            pose.pose.position.y = destination["y"]
            qx, qy, qz, qw = quaternion_from_yaw(destination["yaw"])
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw

            nav_goal = NavigateToPose.Goal()
            nav_goal.pose = pose
            self.remaining_distance = float("nan")
            self.resolved_name = destination["display_name"]
            send_future = self.nav_client.send_goal_async(
                nav_goal, feedback_callback=self.on_nav_feedback
            )
            self.nav_goal_handle = await send_future
            if self.nav_goal_handle is None or not self.nav_goal_handle.accepted:
                result.success = False
                result.error_code = 3
                result.message = "navigate_to_pose goal rejected"
                goal_handle.abort()
                return result

            if (
                self.cancel_requested
                or goal_handle.is_cancel_requested
                or not self.localized
            ):
                self.cancel_requested = True
                await self.nav_goal_handle.cancel_goal_async()

            feedback = NavigateNamedDestination.Feedback()
            feedback.state = "CANCELING" if self.cancel_requested else "NAVIGATING"
            feedback.resolved_name = destination["display_name"]
            feedback.remaining_distance = self.remaining_distance
            feedback.localization_state = self.localization_state
            goal_handle.publish_feedback(feedback)

            wrapped = await self.nav_goal_handle.get_result_async()
            if (
                self.cancel_requested
                or goal_handle.is_cancel_requested
                or wrapped.status == GoalStatus.STATUS_CANCELED
            ):
                goal_handle.canceled()
                result.success = False
                result.error_code = 4
                result.message = "navigation canceled"
                return result
            if wrapped.status != GoalStatus.STATUS_SUCCEEDED or not self.localized:
                goal_handle.abort()
                result.success = False
                result.error_code = 5
                result.message = "navigation failed or localization degraded"
                return result
            goal_handle.succeed()
            result.success = True
            result.error_code = 0
            result.message = f"arrived at {destination['display_name']}"
            return result
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Named navigation failed: {exc}")
            if goal_handle.is_active:
                goal_handle.abort()
            result.success = False
            result.error_code = 6
            result.message = f"navigation exception: {exc}"
            return result
        finally:
            self.nav_goal_handle = None
            self.named_goal_handle = None
            self.goal_reserved = False
            self.cancel_requested = False


def main(args=None):
    rclpy.init(args=args)
    node = IndoorNavigationManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
