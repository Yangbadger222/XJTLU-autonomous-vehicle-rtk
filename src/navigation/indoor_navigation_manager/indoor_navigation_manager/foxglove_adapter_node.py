import json
import math

import rclpy
from action_msgs.msg import GoalStatus
from frc_msgs.msg import ChassisStatus
from geometry_msgs.msg import PoseStamped
from interface.action import NavigateNamedDestination
from interface.msg import LocalizationStatus, NavigationStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from indoor_navigation_manager.destinations import load_destinations


def pose_is_finite(pose):
    values = (
        pose.position.x,
        pose.position.y,
        pose.position.z,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    )
    quaternion_norm = sum(value * value for value in values[3:])
    return all(math.isfinite(value) for value in values) and quaternion_norm > 1.0e-6


def normalize_pose_quaternion(pose):
    orientation = pose.orientation
    norm = math.sqrt(
        orientation.x * orientation.x
        + orientation.y * orientation.y
        + orientation.z * orientation.z
        + orientation.w * orientation.w
    )
    orientation.x /= norm
    orientation.y /= norm
    orientation.z /= norm
    orientation.w /= norm


class FoxgloveNavigationAdapter(Node):
    def __init__(self):
        super().__init__("foxglove_navigation_adapter")
        self.declare_parameter("destinations_file", "")
        self.declare_parameter("map_id", "")
        self.declare_parameter("goal_pose_topic", "/foxglove/goal_pose")
        self.declare_parameter(
            "compat_goal_pose_topic", "/move_base_simple/goal"
        )
        self.declare_parameter("named_destination_topic", "/foxglove/named_destination")
        self.declare_parameter("localization_topic", "/localizer/status")
        self.declare_parameter("chassis_status_topic", "/chassis/status")
        self.declare_parameter("navigate_to_pose_action", "/navigate_to_pose")
        self.declare_parameter("named_navigation_action", "/navigate_named_destination")
        self.declare_parameter("localization_cancel_grace_s", 5.0)
        self.declare_parameter("chassis_cancel_grace_s", 1.0)

        destinations_file = str(self.get_parameter("destinations_file").value)
        if not destinations_file:
            raise RuntimeError("destinations_file is required")
        self.catalog = load_destinations(destinations_file)
        configured_map_id = str(self.get_parameter("map_id").value).strip()
        self.map_id = configured_map_id or self.catalog["map_id"]
        if self.catalog["map_id"] != self.map_id:
            raise RuntimeError("destination map_id does not match configured map_id")

        transient_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.status_pub = self.create_publisher(
            NavigationStatus, "/foxglove/navigation/status", transient_qos
        )
        self.catalog_pub = self.create_publisher(
            String, "/foxglove/navigation/destinations", transient_qos
        )
        self.markers_pub = self.create_publisher(
            MarkerArray, "/foxglove/navigation/destination_markers", transient_qos
        )

        goal_pose_topic = str(self.get_parameter("goal_pose_topic").value)
        compat_goal_pose_topic = str(
            self.get_parameter("compat_goal_pose_topic").value
        ).strip()
        named_topic = str(self.get_parameter("named_destination_topic").value)
        localization_topic = str(self.get_parameter("localization_topic").value)
        chassis_status_topic = str(
            self.get_parameter("chassis_status_topic").value
        )
        self.localization_cancel_grace_s = max(
            0.0,
            float(self.get_parameter("localization_cancel_grace_s").value),
        )
        self.chassis_cancel_grace_s = max(
            0.0,
            float(self.get_parameter("chassis_cancel_grace_s").value),
        )
        self.goal_pose_subscriptions = [
            self.create_subscription(
                PoseStamped, goal_pose_topic, self.on_pose_goal, 10
            )
        ]
        if compat_goal_pose_topic and compat_goal_pose_topic != goal_pose_topic:
            self.goal_pose_subscriptions.append(
                self.create_subscription(
                    PoseStamped, compat_goal_pose_topic, self.on_pose_goal, 10
                )
            )
        self.create_subscription(String, named_topic, self.on_named_goal, 10)
        self.create_subscription(
            LocalizationStatus, localization_topic, self.on_localization, 10
        )
        self.create_subscription(
            ChassisStatus, chassis_status_topic, self.on_chassis_status, 10
        )
        self.cancel_service = self.create_service(
            Trigger, "/foxglove/cancel_navigation", self.on_cancel
        )

        pose_action = str(self.get_parameter("navigate_to_pose_action").value)
        named_action = str(self.get_parameter("named_navigation_action").value)
        self.pose_client = ActionClient(self, NavigateToPose, pose_action)
        self.named_client = ActionClient(
            self, NavigateNamedDestination, named_action
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.localized = False
        self.chassis_mode = None
        self.chassis_ready = False
        self.chassis_not_ready_since = None
        self.last_localization_ready = None
        self.localization_not_ready_since = None
        self.localization_state = "UNINITIALIZED"
        self.pending_goal = False
        self.cancel_when_accepted = False
        self.active_goal_handle = None
        self.cancel_requested = False
        self.active_source = ""
        self.active_target = ""
        self.remaining_distance = float("nan")
        self.create_timer(0.1, self.enforce_localization_loss_grace)
        self.create_timer(0.1, self.enforce_chassis_mode_grace)
        self.publish_catalog()
        self.publish_destination_markers()
        self.publish_status("IDLE", "waiting for localization")

    def publish_status(self, state, message):
        state_codes = {
            "IDLE": NavigationStatus.IDLE,
            "SENDING": NavigationStatus.SENDING,
            "NAVIGATING": NavigationStatus.NAVIGATING,
            "CANCELING": NavigationStatus.CANCELING,
            "SUCCEEDED": NavigationStatus.SUCCEEDED,
            "CANCELED": NavigationStatus.CANCELED,
            "REJECTED": NavigationStatus.REJECTED,
            "FAILED": NavigationStatus.FAILED,
        }
        msg = NavigationStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.state = state_codes[state]
        msg.state_label = state
        msg.message = message
        msg.map_id = self.map_id
        msg.source = self.active_source
        msg.target = self.active_target
        msg.localization_state = self.localization_state
        msg.localization_ready = self.localized
        msg.has_remaining_distance = math.isfinite(self.remaining_distance)
        msg.remaining_distance_m = (
            self.remaining_distance if msg.has_remaining_distance else 0.0
        )
        self.status_pub.publish(msg)
        if state in ("REJECTED", "FAILED"):
            self.get_logger().warning(f"{state}: {message}")
        elif state in ("SENDING", "CANCELING", "SUCCEEDED", "CANCELED"):
            self.get_logger().info(f"{state}: {message}")

    def publish_catalog(self):
        destinations = []
        for entry in self.catalog["canonical"].values():
            destinations.append(
                {
                    "id": entry["id"],
                    "display_name": entry["display_name"],
                    "x": entry["x"],
                    "y": entry["y"],
                    "yaw": entry["yaw"],
                }
            )
        msg = String()
        msg.data = json.dumps(
            {"map_id": self.map_id, "destinations": destinations},
            ensure_ascii=False,
            sort_keys=True,
        )
        self.catalog_pub.publish(msg)

    def publish_destination_markers(self):
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for index, entry in enumerate(self.catalog["canonical"].values()):
            arrow = Marker()
            arrow.header.frame_id = "map"
            arrow.header.stamp = stamp
            arrow.ns = "indoor_destinations"
            arrow.id = index * 2
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose.position.x = entry["x"]
            arrow.pose.position.y = entry["y"]
            arrow.pose.position.z = 0.05
            arrow.pose.orientation.z = math.sin(entry["yaw"] * 0.5)
            arrow.pose.orientation.w = math.cos(entry["yaw"] * 0.5)
            arrow.scale.x = 0.45
            arrow.scale.y = 0.10
            arrow.scale.z = 0.10
            arrow.color.r = 0.10
            arrow.color.g = 0.85
            arrow.color.b = 0.35
            arrow.color.a = 0.95
            markers.markers.append(arrow)

            label = Marker()
            label.header.frame_id = "map"
            label.header.stamp = stamp
            label.ns = "indoor_destination_labels"
            label.id = index * 2 + 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = entry["x"]
            label.pose.position.y = entry["y"]
            label.pose.position.z = 0.45
            label.pose.orientation.w = 1.0
            label.scale.z = 0.24
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = entry["display_name"]
            markers.markers.append(label)
        self.markers_pub.publish(markers)

    def on_localization(self, msg):
        self.localization_state = msg.state_label
        self.localized = bool(
            msg.localized
            and msg.sensors_ready
            and msg.state
            in (LocalizationStatus.LOCALIZED, LocalizationStatus.DEGRADED)
        )
        localization_changed = self.localized != self.last_localization_ready
        self.last_localization_ready = self.localized
        if self.localized:
            self.localization_not_ready_since = None
            if localization_changed and self.active_goal_handle is not None:
                self.publish_status(
                    "NAVIGATING", "localization recovered; resuming navigation"
                )
        elif self.localization_not_ready_since is None:
            self.localization_not_ready_since = self.get_clock().now()
            if self.active_goal_handle is not None and not self.cancel_requested:
                self.publish_status(
                    "NAVIGATING",
                    "localization temporarily unavailable; holding position",
                )
        if (
            localization_changed
            and not self.pending_goal
            and self.active_goal_handle is None
        ):
            if not self.localized:
                idle_message = "localization is not ready"
            elif not self.chassis_ready:
                idle_message = self.chassis_unavailable_message()
            else:
                idle_message = "ready for navigation"
            self.publish_status(
                "IDLE",
                idle_message,
            )

    def enforce_localization_loss_grace(self):
        if (
            self.localized
            or self.localization_not_ready_since is None
            or self.active_goal_handle is None
            or self.cancel_requested
        ):
            return
        elapsed_s = (
            self.get_clock().now() - self.localization_not_ready_since
        ).nanoseconds * 1.0e-9
        if elapsed_s >= self.localization_cancel_grace_s:
            self.cancel_requested = True
            self.active_goal_handle.cancel_goal_async()
            self.publish_status(
                "CANCELING",
                "localization unavailable beyond grace period; canceling active goal",
            )

    def on_chassis_status(self, msg):
        was_ready = self.chassis_ready
        self.chassis_mode = int(msg.ctrl_mode)
        raw_ready = self.chassis_mode == ChassisStatus.CTRL_MODE_HOST
        if raw_ready:
            self.chassis_not_ready_since = None
            self.chassis_ready = True
        elif self.chassis_not_ready_since is None:
            self.chassis_not_ready_since = self.get_clock().now()
        if was_ready != self.chassis_ready and self.active_goal_handle is None:
            self.publish_status(
                "IDLE",
                "ready for navigation"
                if self.chassis_ready and self.localized
                else self.chassis_unavailable_message(),
            )

    def enforce_chassis_mode_grace(self):
        if self.chassis_not_ready_since is None or self.chassis_ready is False:
            return
        elapsed_s = (
            self.get_clock().now() - self.chassis_not_ready_since
        ).nanoseconds * 1.0e-9
        if elapsed_s < self.chassis_cancel_grace_s:
            return
        self.chassis_ready = False
        if self.active_goal_handle is not None and not self.cancel_requested:
            self.cancel_requested = True
            self.active_goal_handle.cancel_goal_async()
            self.publish_status(
                "CANCELING", "chassis left host serial mode; canceling active goal"
            )
        elif self.pending_goal:
            self.cancel_when_accepted = True
            self.publish_status(
                "CANCELING", "chassis left host serial mode before goal acceptance"
            )
        else:
            self.publish_status("IDLE", self.chassis_unavailable_message())

    def chassis_unavailable_message(self):
        if self.chassis_mode == ChassisStatus.CTRL_MODE_DISABLED:
            return "chassis motors are disabled; enable motors, then select host serial mode"
        if self.chassis_mode == ChassisStatus.CTRL_MODE_GAMEPAD:
            return "chassis is in gamepad mode; select host serial mode"
        return "chassis status is unavailable"

    def goal_is_available(self):
        if not self.localized:
            self.publish_status("REJECTED", "localization is not ready")
            return False
        if not self.chassis_ready:
            self.publish_status("REJECTED", self.chassis_unavailable_message())
            return False
        if self.pending_goal or self.active_goal_handle is not None:
            self.publish_status("REJECTED", "another navigation goal is active")
            return False
        return True

    def on_pose_goal(self, msg):
        source_frame = (msg.header.frame_id or "map").lstrip("/")
        self.get_logger().info(
            "Received pose goal: "
            f"frame={source_frame} "
            f"x={msg.pose.position.x:.3f} y={msg.pose.position.y:.3f}"
        )
        if not self.goal_is_available():
            return
        if not pose_is_finite(msg.pose):
            self.publish_status("REJECTED", "pose goal is invalid")
            return
        if not self.pose_client.server_is_ready():
            self.publish_status("REJECTED", "navigate_to_pose action is unavailable")
            return

        pose_goal = msg
        if source_frame != "map":
            try:
                transform = self.tf_buffer.lookup_transform(
                    "map", source_frame, Time()
                )
                pose_goal = do_transform_pose_stamped(msg, transform)
            except TransformException as exc:
                self.publish_status(
                    "REJECTED",
                    f"cannot transform pose goal from {source_frame} to map: {exc}",
                )
                return
            self.get_logger().info(
                f"Transformed pose goal from {source_frame} to map: "
                f"x={pose_goal.pose.position.x:.3f} "
                f"y={pose_goal.pose.position.y:.3f}"
            )

        goal = NavigateToPose.Goal()
        goal.pose = pose_goal
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.z = 0.0
        normalize_pose_quaternion(goal.pose.pose)
        self.active_source = "pose"
        self.active_target = (
            f"{pose_goal.pose.position.x:.2f}, {pose_goal.pose.position.y:.2f}"
        )
        self.send_goal(
            self.pose_client,
            goal,
            self.on_pose_feedback,
        )

    def on_named_goal(self, msg):
        destination_name = msg.data.strip()
        if not destination_name:
            self.publish_status("REJECTED", "destination name is empty")
            return
        if not self.goal_is_available():
            return
        if not self.named_client.server_is_ready():
            self.publish_status(
                "REJECTED", "named destination action is unavailable"
            )
            return
        goal = NavigateNamedDestination.Goal()
        goal.map_id = self.map_id
        goal.destination_name = destination_name
        goal.backend = "indoor"
        self.active_source = "named_destination"
        self.active_target = destination_name
        self.send_goal(self.named_client, goal, self.on_named_feedback)

    def send_goal(self, client, goal, feedback_callback):
        self.pending_goal = True
        self.remaining_distance = float("nan")
        self.publish_status("SENDING", "sending navigation goal")
        future = client.send_goal_async(goal, feedback_callback=feedback_callback)
        future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        self.pending_goal = False
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.finish_goal("FAILED", f"goal request failed: {exc}")
            return
        if goal_handle is None or not goal_handle.accepted:
            self.finish_goal("REJECTED", "navigation action rejected the goal")
            return
        self.active_goal_handle = goal_handle
        self.cancel_requested = False
        if self.cancel_when_accepted:
            self.cancel_requested = True
            self.cancel_when_accepted = False
            goal_handle.cancel_goal_async()
            self.publish_status(
                "CANCELING",
                "canceling goal accepted after the request",
            )
        else:
            if not self.localized and self.localization_not_ready_since is None:
                self.localization_not_ready_since = self.get_clock().now()
            self.publish_status(
                "NAVIGATING",
                "navigation goal accepted"
                if self.localized
                else "goal accepted; waiting for localization recovery",
            )
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_result)

    def on_pose_feedback(self, feedback_msg):
        self.remaining_distance = float(feedback_msg.feedback.distance_remaining)
        if not self.cancel_requested:
            self.publish_status("NAVIGATING", "following pose goal")

    def on_named_feedback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.remaining_distance = float(feedback.remaining_distance)
        if feedback.resolved_name:
            self.active_target = feedback.resolved_name
        if not self.cancel_requested:
            self.publish_status("NAVIGATING", feedback.state or "following named goal")

    def on_result(self, future):
        try:
            wrapped = future.result()
        except Exception as exc:  # noqa: BLE001
            self.finish_goal("FAILED", f"navigation result failed: {exc}")
            return
        if wrapped.status == GoalStatus.STATUS_SUCCEEDED:
            self.finish_goal("SUCCEEDED", "navigation completed")
        elif wrapped.status == GoalStatus.STATUS_CANCELED:
            self.finish_goal("CANCELED", "navigation canceled")
        else:
            self.finish_goal("FAILED", f"navigation ended with status {wrapped.status}")

    def finish_goal(self, state, message):
        self.pending_goal = False
        self.cancel_when_accepted = False
        self.active_goal_handle = None
        self.cancel_requested = False
        self.localization_not_ready_since = (
            None if self.localized else self.get_clock().now()
        )
        self.remaining_distance = float("nan")
        self.publish_status(state, message)

    def on_cancel(self, _request, response):
        if self.pending_goal:
            self.cancel_when_accepted = True
            self.publish_status("CANCELING", "cancel requested before goal acceptance")
            response.success = True
            response.message = "goal will be canceled when accepted"
            return response
        if self.active_goal_handle is None:
            response.success = False
            response.message = "no active navigation goal"
            return response
        if self.cancel_requested:
            response.success = True
            response.message = "cancel request already pending"
            return response
        self.cancel_requested = True
        self.active_goal_handle.cancel_goal_async()
        self.publish_status("CANCELING", "cancel requested from Foxglove")
        response.success = True
        response.message = "cancel request sent"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = FoxgloveNavigationAdapter()
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
