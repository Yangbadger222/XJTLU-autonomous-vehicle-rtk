"""锚记忆管理节点：锚建立/再验证/衰减/跨 session 重挂 + 状态发布。

- 唯一的 anchors.db 写者；risk_pipeline 经 /frc/anchor_states 消费（单写者原则）。
- 锚挂载到最近 PGO keyframe + 相对偏移：每次 /pgo/keyframes 刷新后重算锚的
  map 位姿，回环修正后锚自动跟随位姿图移动（RViz 中可见整体平移）。
- 平稳通过判定：机器人进入锚半径 -> 离开，期间无事件且在移动，即记一次
  smooth traversal（P_usable *= beta）。
- 跨 session（F2）：首批 keyframe 就位后，把历史 session 的锚用 map 位姿快照
  重挂到最近 keyframe，降级 candidate 注入，首次再验证后按状态机恢复。
"""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from frc_bev import ros_adapter
from frc_msgs.msg import AnchorState, AnchorStateArray, EventMarker, KeyframeArray
from frc_nodes import session
from frc_nodes.anchor_store import TIER_SEVERITY, AnchorStore

STATE_COLORS = {
    "candidate": (1.0, 0.85, 0.2),
    "confirmed": (0.9, 0.15, 0.15),
    "stale": (0.6, 0.6, 0.6),
    "retired": (0.25, 0.25, 0.25),
}


class MemoryManagerNode(Node):
    def __init__(self):
        super().__init__("frc_memory_manager")
        p = self.declare_parameter
        p("scene_id", "S0")
        p("anchor_db_path", str(session.frc_dir() / "anchors.db"))
        p("beta", 0.8)
        p("stale_thresh", 0.3)
        p("retired_thresh", 0.1)
        p("smooth_pass_radius_m", 2.0)
        p("anchor_merge_radius_m", 1.5)
        p("min_moving_speed", 0.1)
        p("remount_min_keyframes", 5)

        self._session = session.session_id()
        scene = str(self.get_parameter("scene_id").value)
        jsonl = session.frc_dir("anchor_log") / f"{self._session}.jsonl"
        self.store = AnchorStore(
            str(self.get_parameter("anchor_db_path").value),
            session_id=self._session, scene_id=scene, transitions_jsonl=jsonl)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._keyframes = {}            # id -> (x, y, yaw) map 系
        self._remounted = False
        self._pose_map = None           # 机器人 map 位姿
        self._speed = 0.0
        self._inside = {}               # anchor_id -> 进入时刻
        self._last_event_t = {}         # anchor_id -> 最近事件时刻

        self.create_subscription(EventMarker, "/frc/event_marker",
                                 self._on_event, 20)
        self.create_subscription(KeyframeArray, "/pgo/keyframes",
                                 self._on_keyframes, 5)
        self.create_subscription(Odometry, "/fastlio2/lio_odom",
                                 self._on_odom, 20)

        self._states_pub = self.create_publisher(AnchorStateArray,
                                                 "/frc/anchor_states", 10)
        self._marker_pub = self.create_publisher(MarkerArray,
                                                 "/frc/anchor_markers", 5)
        self.create_timer(1.0, self._publish_states)
        self.create_timer(0.2, self._check_traversals)
        self.get_logger().info(
            f"frc_memory_manager started: scene={scene} session={self._session} "
            f"anchors={len(self.store.all_anchors())}")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ---------- 输入 ----------

    def _on_keyframes(self, msg):
        self._keyframes = {}
        for kid, pose in zip(msg.ids, msg.poses):
            q = pose.orientation
            yaw = ros_adapter.quat_to_yaw(q.x, q.y, q.z, q.w)
            self._keyframes[int(kid)] = (pose.position.x, pose.position.y, yaw)

        # 跨 session 重挂：首批 keyframe 就位后做一次
        if not self._remounted and len(self._keyframes) >= int(
                self.get_parameter("remount_min_keyframes").value):
            remounted = self.store.remount_stale_sessions(self._keyframes)
            self._remounted = True
            if remounted:
                self.get_logger().warn(
                    f"remounted {len(remounted)} anchors from previous "
                    f"sessions as candidates: {remounted}")

        # 锚跟随位姿图：map 位姿 = keyframe 位姿 ⊕ 偏移
        for a in self.store.active_anchors():
            kf = self._keyframes.get(a.keyframe_id)
            if kf is None:
                continue
            kx, ky, kyaw = kf
            cos_y, sin_y = math.cos(kyaw), math.sin(kyaw)
            mx = kx + a.dx * cos_y - a.dy * sin_y
            my = ky + a.dx * sin_y + a.dy * cos_y
            if math.hypot(mx - a.map_x, my - a.map_y) > 1e-4:
                self.store.update_map_pose(a.id, mx, my, kyaw + a.dyaw)

    def _on_odom(self, msg):
        x, y, yaw = ros_adapter.odometry_to_se2(msg)
        self._speed = math.hypot(msg.twist.twist.linear.x,
                                 msg.twist.twist.linear.y)
        try:
            tf = self._tf_buffer.lookup_transform("map", "odom",
                                                  rclpy.time.Time())
            q = tf.transform.rotation
            tyaw = ros_adapter.quat_to_yaw(q.x, q.y, q.z, q.w)
            tx, ty = tf.transform.translation.x, tf.transform.translation.y
            self._pose_map = (tx + x * math.cos(tyaw) - y * math.sin(tyaw),
                              ty + x * math.sin(tyaw) + y * math.cos(tyaw))
        except Exception:
            self._pose_map = (x, y)   # TF 未就绪时近似（session 初期 map≈odom）

    def _on_event(self, msg):
        severity = TIER_SEVERITY.get(msg.severity, 0.3)
        ex, ey = msg.pose.position.x, msg.pose.position.y
        q = msg.pose.orientation
        eyaw = ros_adapter.quat_to_yaw(q.x, q.y, q.z, q.w)

        merge_r = float(self.get_parameter("anchor_merge_radius_m").value)
        nearby = self.store.find_nearby(ex, ey, merge_r)
        if nearby is not None:
            a = self.store.on_event(nearby.id, severity)
            self._last_event_t[a.id] = self._now()
            self.get_logger().info(
                f"anchor {a.id} reinforced by {msg.type}: state={a.state} "
                f"hits={a.hit_count}")
            self._publish_states()
            return

        if not self._keyframes:
            self.get_logger().warn(
                "event before any keyframe, anchor creation skipped")
            return
        kf_id, kf = min(self._keyframes.items(),
                        key=lambda kv: math.hypot(kv[1][0] - ex,
                                                  kv[1][1] - ey))
        kx, ky, kyaw = kf
        cos_y, sin_y = math.cos(-kyaw), math.sin(-kyaw)
        rx, ry = ex - kx, ey - ky
        anchor_id = self.store.create_anchor(
            kind="map", severity=severity, keyframe_id=kf_id,
            dx=rx * cos_y - ry * sin_y, dy=rx * sin_y + ry * cos_y,
            dyaw=eyaw - kyaw, map_x=ex, map_y=ey, map_yaw=eyaw,
            route_id=msg.route_id, layout_id=msg.layout_id)
        self._last_event_t[anchor_id] = self._now()
        self.get_logger().info(
            f"anchor {anchor_id} created from {msg.severity}/{msg.type} "
            f"at ({ex:.2f},{ey:.2f}) kf={kf_id}")
        self._publish_states()

    # ---------- 平稳通过检测 ----------

    def _check_traversals(self):
        if self._pose_map is None:
            return
        px, py = self._pose_map
        radius = float(self.get_parameter("smooth_pass_radius_m").value)
        moving = self._speed > float(
            self.get_parameter("min_moving_speed").value)
        now = self._now()

        for a in self.store.active_anchors():
            d = math.hypot(a.map_x - px, a.map_y - py)
            inside_since = self._inside.get(a.id)
            if d <= radius:
                if inside_since is None and moving:
                    self._inside[a.id] = now
            elif inside_since is not None:
                del self._inside[a.id]
                # 通过期间（含 1s 余量）无事件 -> 平稳通过一次
                if self._last_event_t.get(a.id, -1e9) < inside_since - 1.0:
                    updated = self.store.on_smooth_traversal(
                        a.id,
                        beta=float(self.get_parameter("beta").value),
                        stale_thresh=float(
                            self.get_parameter("stale_thresh").value),
                        retired_thresh=float(
                            self.get_parameter("retired_thresh").value))
                    self.get_logger().info(
                        f"anchor {a.id} smooth pass: p_usable="
                        f"{updated.p_usable:.3f} state={updated.state}")
                    self._publish_states()

    # ---------- 输出 ----------

    def _publish_states(self):
        msg = AnchorStateArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        markers = MarkerArray()
        wipe = Marker()
        wipe.action = Marker.DELETEALL
        markers.markers.append(wipe)

        for a in self.store.all_anchors():
            s = AnchorState()
            s.anchor_id = a.id
            s.kind = a.kind
            s.state = a.state
            s.p_usable = a.p_usable
            s.severity = a.severity
            s.hit_count = a.hit_count
            s.keyframe_id = a.keyframe_id
            s.dx, s.dy, s.dyaw = a.dx, a.dy, a.dyaw
            s.map_x, s.map_y, s.map_yaw = a.map_x, a.map_y, a.map_yaw
            s.scene_id = a.scene_id
            s.session_id = a.session_id
            msg.anchors.append(s)

            color = STATE_COLORS.get(a.state, (1.0, 1.0, 1.0))
            sphere = Marker()
            sphere.header.frame_id = "map"
            sphere.header.stamp = msg.header.stamp
            sphere.ns = "frc_anchors"
            sphere.id = a.id
            sphere.type = Marker.SPHERE
            sphere.pose.position.x = a.map_x
            sphere.pose.position.y = a.map_y
            sphere.pose.position.z = 0.2
            sphere.pose.orientation.w = 1.0
            size = 0.3 + 0.5 * a.severity * a.p_usable
            sphere.scale.x = sphere.scale.y = sphere.scale.z = size
            sphere.color.r, sphere.color.g, sphere.color.b = color
            sphere.color.a = 0.35 if a.state == "retired" else 0.85
            markers.markers.append(sphere)

            text = Marker()
            text.header.frame_id = "map"
            text.header.stamp = msg.header.stamp
            text.ns = "frc_anchor_labels"
            text.id = a.id
            text.type = Marker.TEXT_VIEW_FACING
            text.pose.position.x = a.map_x
            text.pose.position.y = a.map_y
            text.pose.position.z = 0.8
            text.scale.z = 0.25
            text.color.r = text.color.g = text.color.b = 1.0
            text.color.a = 1.0
            text.text = f"#{a.id} {a.state} p={a.p_usable:.2f}"
            markers.markers.append(text)

        self._states_pub.publish(msg)
        self._marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = MemoryManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.store.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
