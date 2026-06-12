"""事件标记节点：失败/异常事件的在线自动挖掘 + 手柄人工标记。

在线规则（Phase 0.6 子集，阈值集中在参数里，与离线 failure_miner 同语义）：
- takeover        gold    /chassis/status ctrl_mode 0->1/2 边沿（上位机被人接管/急停）
- manual          gold    PS2 SELECT 按键边沿
- stuck           silver  ||v_odom|| < 阈值 持续 > 时长 且 |cmd_vel| > 阈值
- near_collision  silver  costmap 最近致命格距离 < 阈值 持续 > 时长
- recovery        silver  Nav2 行为树 recovery 节点（Spin/BackUp/Wait/DriveOnHeading）进入 RUNNING
- jerk            bronze  cmd_vel 加加速度超滚动基线 3 sigma
- plan_instability bronze 相邻 /plan 前向 5m 平均横向偏差滑窗 RMS 超基线 3 sigma
  （基线 BT 为周期重规划，"重规划频率"指标失效，故用几何不稳定度替代——F6）

输出 /frc/event_marker（map 系位姿），并镜像 jsonl 到
runtime-data/frc/events/<session>.jsonl（bag 之外的纯文本兜底）。
"""

import json
import math
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path as PathMsg
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

from frc_bev import ros_adapter
from frc_msgs.msg import ChassisStatus, EventMarker
from frc_nodes import session

try:
    from nav2_msgs.msg import Costmap, BehaviorTreeLog
    HAVE_NAV2_MSGS = True
except ImportError:  # nav2 未构建时仍可启动（少两类事件源）
    HAVE_NAV2_MSGS = False

RECOVERY_NODES = {"Spin", "BackUp", "Wait", "DriveOnHeading"}
PSB_SELECT = 1


class RunningStats:
    """Welford 滚动均值/方差，用于 jerk 与 plan 偏差的 3 sigma 基线。"""

    def __init__(self):
        self.n, self.mean, self.m2 = 0, 0.0, 0.0

    def push(self, x: float):
        self.n += 1
        d = x - self.mean
        self.mean += d / self.n
        self.m2 += d * (x - self.mean)

    @property
    def std(self) -> float:
        return math.sqrt(self.m2 / self.n) if self.n > 1 else 0.0

    def exceeds_3sigma(self, x: float, min_samples: int = 50) -> bool:
        return self.n >= min_samples and self.std > 1e-9 and \
            x > self.mean + 3.0 * self.std


class EventMarkerNode(Node):
    def __init__(self):
        super().__init__("frc_event_marker")
        p = self.declare_parameter
        p("route_id", "")
        p("layout_id", "")
        p("stuck_odom_speed", 0.05)
        p("stuck_cmd_speed", 0.2)
        p("stuck_duration_s", 3.0)
        p("nearcol_clearance_m", 0.35)
        p("nearcol_duration_s", 0.5)
        p("jerk_min_samples", 200)
        p("plan_lookahead_m", 5.0)
        p("plan_window", 10)
        p("cooldown_s", 8.0)

        self._session = session.session_id()
        self._jsonl = session.frc_dir("events") / f"{self._session}.jsonl"

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # 状态
        self._last_ctrl_mode = None
        self._last_ps2_key = 0
        self._odom_speed = 0.0          # 底盘回读速度
        self._cmd = (0.0, 0.0)          # 最新 cmd_vel
        self._pose_odom = None          # (x, y, yaw) 来自 lio_odom
        self._stuck_since = None
        self._nearcol_since = None
        self._last_cmd = None           # (t, vx)
        self._last_accel = None         # (t, ax)
        self._jerk_stats = RunningStats()
        self._plan_stats = RunningStats()
        self._last_plan = None          # (N,2) odom 系
        self._plan_devs = []
        self._last_emit = {}            # type -> t

        self.create_subscription(ChassisStatus, "/chassis/status",
                                 self._on_chassis, 20)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 20)
        self.create_subscription(Odometry, "/odom_CBoar", self._on_chassis_odom, 20)
        self.create_subscription(Odometry, "/fastlio2/lio_odom", self._on_lio, 20)
        self.create_subscription(PathMsg, "/plan", self._on_plan, 10)
        if HAVE_NAV2_MSGS:
            self.create_subscription(Costmap, "/local_costmap/costmap_raw",
                                     self._on_costmap, 2)
            self.create_subscription(BehaviorTreeLog, "/behavior_tree_log",
                                     self._on_bt_log, 10)
        else:
            self.get_logger().warn(
                "nav2_msgs unavailable: near_collision/recovery rules disabled")

        self._pub = self.create_publisher(EventMarker, "/frc/event_marker", 10)
        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f"frc_event_marker started, session={self._session}")

    # ---------- 输入回调 ----------

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_chassis(self, msg):
        mode = int(msg.ctrl_mode)
        if self._last_ctrl_mode == ChassisStatus.CTRL_MODE_HOST and \
                mode in (ChassisStatus.CTRL_MODE_GAMEPAD,
                         ChassisStatus.CTRL_MODE_DISABLED):
            self._emit("takeover", "gold",
                       note=f"ctrl_mode {self._last_ctrl_mode}->{mode}")
        self._last_ctrl_mode = mode

        key = int(msg.ps2_key)
        if key == PSB_SELECT and self._last_ps2_key != PSB_SELECT:
            self._emit("manual", "gold", note="ps2 SELECT", bypass_cooldown=True)
        self._last_ps2_key = key

    def _on_cmd(self, msg):
        t = self._now()
        vx = float(msg.linear.x)
        self._cmd = (vx, float(msg.angular.z))
        if self._last_cmd is not None:
            dt = t - self._last_cmd[0]
            if 1e-3 < dt < 1.0:
                ax = (vx - self._last_cmd[1]) / dt
                if self._last_accel is not None:
                    dt2 = t - self._last_accel[0]
                    if 1e-3 < dt2 < 1.0:
                        jerk = abs((ax - self._last_accel[1]) / dt2)
                        min_n = int(self.get_parameter("jerk_min_samples").value)
                        if self._jerk_stats.exceeds_3sigma(jerk, min_n):
                            self._emit("jerk", "bronze", note=f"jerk={jerk:.1f}")
                        self._jerk_stats.push(jerk)
                self._last_accel = (t, ax)
        self._last_cmd = (t, vx)

    def _on_chassis_odom(self, msg):
        self._odom_speed = math.hypot(msg.twist.twist.linear.x,
                                      msg.twist.twist.linear.y)

    def _on_lio(self, msg):
        x, y, yaw = ros_adapter.odometry_to_se2(msg)
        self._pose_odom = (x, y, yaw)

    def _on_costmap(self, msg):
        if self._pose_odom is None:
            return
        grid, meta = ros_adapter.costmap_to_numpy(msg)
        lethal = np.argwhere(grid >= 253.0)
        if not len(lethal):
            self._nearcol_since = None
            return
        res = meta["resolution"]
        ox, oy = meta["origin_xy"]
        rx, ry = self._pose_odom[0], self._pose_odom[1]
        cell_xy = np.stack([lethal[:, 1] * res + ox, lethal[:, 0] * res + oy],
                           axis=1)
        clearance = float(np.min(np.hypot(cell_xy[:, 0] - rx,
                                          cell_xy[:, 1] - ry)))
        thresh = float(self.get_parameter("nearcol_clearance_m").value)
        t = self._now()
        if clearance < thresh:
            if self._nearcol_since is None:
                self._nearcol_since = t
            elif t - self._nearcol_since >= float(
                    self.get_parameter("nearcol_duration_s").value):
                self._emit("near_collision", "silver",
                           note=f"clearance={clearance:.2f}m")
                self._nearcol_since = None
        else:
            self._nearcol_since = None

    def _on_bt_log(self, msg):
        for ev in msg.event_log:
            if ev.node_name in RECOVERY_NODES and ev.current_status == "RUNNING":
                self._emit("recovery", "silver", note=ev.node_name)
                return

    def _on_plan(self, msg):
        if self._pose_odom is None or len(msg.poses) < 2:
            return
        pts = np.array([[p.pose.position.x, p.pose.position.y]
                        for p in msg.poses], dtype=np.float64)
        lookahead = float(self.get_parameter("plan_lookahead_m").value)
        rx, ry = self._pose_odom[0], self._pose_odom[1]
        dist_along = np.hypot(pts[:, 0] - rx, pts[:, 1] - ry)
        ahead = pts[dist_along <= lookahead]
        if self._last_plan is not None and len(ahead) >= 2 and \
                len(self._last_plan) >= 2:
            # 新 plan 各点到旧 plan 折线（点集近似）的平均横向偏差
            d = np.hypot(ahead[:, None, 0] - self._last_plan[None, :, 0],
                         ahead[:, None, 1] - self._last_plan[None, :, 1])
            dev = float(d.min(axis=1).mean())
            self._plan_devs.append(dev)
            window = int(self.get_parameter("plan_window").value)
            if len(self._plan_devs) > window:
                self._plan_devs.pop(0)
            rms = float(np.sqrt(np.mean(np.square(self._plan_devs))))
            if self._plan_stats.exceeds_3sigma(rms, min_samples=30):
                self._emit("plan_instability", "bronze", note=f"rms={rms:.2f}m")
            self._plan_stats.push(rms)
        self._last_plan = ahead if len(ahead) >= 2 else pts

    # ---------- 周期评估（stuck）----------

    def _tick(self):
        t = self._now()
        stuck_v = float(self.get_parameter("stuck_odom_speed").value)
        stuck_cmd = float(self.get_parameter("stuck_cmd_speed").value)
        if self._odom_speed < stuck_v and abs(self._cmd[0]) > stuck_cmd:
            if self._stuck_since is None:
                self._stuck_since = t
            elif t - self._stuck_since >= float(
                    self.get_parameter("stuck_duration_s").value):
                self._emit("stuck", "silver",
                           note=f"v={self._odom_speed:.3f} cmd={self._cmd[0]:.2f}")
                self._stuck_since = None
        else:
            self._stuck_since = None

    # ---------- 发布 ----------

    def _pose_in_map(self):
        """lio_odom 位姿 -> map 系（TF map->odom 不可用时退化为 odom 位姿）。"""
        if self._pose_odom is None:
            return 0.0, 0.0, 0.0
        x, y, yaw = self._pose_odom
        try:
            tf = self._tf_buffer.lookup_transform("map", "odom",
                                                  rclpy.time.Time())
            q = tf.transform.rotation
            tyaw = ros_adapter.quat_to_yaw(q.x, q.y, q.z, q.w)
            tx, ty = tf.transform.translation.x, tf.transform.translation.y
            mx = tx + x * math.cos(tyaw) - y * math.sin(tyaw)
            my = ty + x * math.sin(tyaw) + y * math.cos(tyaw)
            return mx, my, yaw + tyaw
        except Exception:
            return x, y, yaw

    def _emit(self, etype: str, severity: str, note: str = "",
              bypass_cooldown: bool = False):
        t = self._now()
        cooldown = float(self.get_parameter("cooldown_s").value)
        if not bypass_cooldown and t - self._last_emit.get(etype, -1e9) < cooldown:
            return
        self._last_emit[etype] = t

        mx, my, myaw = self._pose_in_map()
        msg = EventMarker()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.type = etype
        msg.severity = severity
        msg.pose.position.x = mx
        msg.pose.position.y = my
        qx, qy, qz, qw = ros_adapter.yaw_to_quat(myaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        msg.route_id = str(self.get_parameter("route_id").value)
        msg.layout_id = str(self.get_parameter("layout_id").value)
        msg.note = note
        msg.session_id = self._session
        self._pub.publish(msg)
        self.get_logger().warn(f"EVENT {severity}/{etype} at "
                               f"({mx:.2f},{my:.2f}) {note}")

        with open(self._jsonl, "a") as f:
            f.write(json.dumps({
                "stamp": t, "type": etype, "severity": severity,
                "map_x": mx, "map_y": my, "map_yaw": myaw, "note": note,
                "route_id": msg.route_id, "layout_id": msg.layout_id,
                "session_id": self._session}) + "\n")


def main(args=None):
    rclpy.init(args=args)
    node = EventMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
