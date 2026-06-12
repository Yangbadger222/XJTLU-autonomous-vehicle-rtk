"""趟次执行节点：YAML 路线 -> FollowWaypoints -> 趟级元数据落盘（E4/E5 承载）。

- 路线文件：runtime-data/frc/routes/R{n}.yaml
  {route_id, scene_id, layout_id, frame: map, launch_note, waypoints: [{x,y,yaw}]}
- A/B 交替由本节点强制执行（--ab-schedule A,B,A,B 不靠人记）：
  A = baseline（调 /frc/disable），B = +FRC（调 /frc/enable），
  B'（地图锚 only 消融）需手工配置 risk_pipeline，此处仅记录条件标签。
- 每趟结束写 runtime-data/frc/trials/<date>/<trial_id>.yaml，
  趟级指标由 frc_offline/eval/trial_metrics.py 离线从 bag+events 计算。
"""

import os
import time
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger

from frc_bev import ros_adapter
from frc_nodes import session


class TrialRunnerNode(Node):
    def __init__(self):
        super().__init__("frc_trial_runner")
        p = self.declare_parameter
        p("route_file", "")
        p("ab_schedule", "A,B,A,B")
        p("inter_trial_pause_s", 10.0)
        p("server_wait_s", 30.0)

        route_file = str(self.get_parameter("route_file").value)
        if not route_file:
            raise RuntimeError(
                "route_file 未设置。用法: ros2 run frc_nodes frc_trial_runner "
                "--ros-args -p route_file:=runtime-data/frc/routes/R1.yaml "
                "-p ab_schedule:=A,B,A,B")
        self.route = yaml.safe_load(Path(route_file).read_text())
        self.schedule = [c.strip().upper() for c in
                         str(self.get_parameter("ab_schedule").value).split(",")
                         if c.strip()]

        self._session = session.session_id()
        self._trials_dir = session.frc_dir(
            "trials", time.strftime("%Y-%m-%d"))
        self._client = ActionClient(self, FollowWaypoints, "follow_waypoints")
        self._enable_cli = self.create_client(Trigger, "/frc/enable")
        self._disable_cli = self.create_client(Trigger, "/frc/disable")

        self._idx = 0
        self._goal_handle = None
        self._trial_start = None
        self._current_trial = None
        self.get_logger().info(
            f"trial_runner: route={self.route.get('route_id')} "
            f"schedule={self.schedule}")
        self.create_timer(1.0, self._kick)
        self._kicked = False

    # ---------- 调度 ----------

    def _kick(self):
        if self._kicked:
            return
        self._kicked = True
        if not self._client.wait_for_server(
                timeout_sec=float(self.get_parameter("server_wait_s").value)):
            self.get_logger().fatal("follow_waypoints action server 不可用")
            rclpy.shutdown()
            return
        self._start_next_trial()

    def _start_next_trial(self):
        if self._idx >= len(self.schedule):
            self.get_logger().info("schedule 执行完毕")
            rclpy.shutdown()
            return
        condition = self.schedule[self._idx]
        self._apply_condition(condition)

        goal = FollowWaypoints.Goal()
        for wp in self.route["waypoints"]:
            ps = PoseStamped()
            ps.header.frame_id = self.route.get("frame", "map")
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.pose.position.x = float(wp["x"])
            ps.pose.position.y = float(wp["y"])
            qx, qy, qz, qw = ros_adapter.yaw_to_quat(float(wp.get("yaw", 0.0)))
            ps.pose.orientation.x = qx
            ps.pose.orientation.y = qy
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            goal.poses.append(ps)

        self._trial_start = time.time()
        self._current_trial = {
            "trial_id": f"{self._session}_{self._idx:02d}",
            "session_id": self._session,
            "route_id": self.route.get("route_id", ""),
            "scene_id": self.route.get("scene_id", ""),
            "layout_id": self.route.get("layout_id", ""),
            "condition": condition,
            "frc_mode": os.environ.get("FRC_MODE", "off"),
            "schedule_index": self._idx,
            "num_waypoints": len(goal.poses),
            "start_time": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "battery_note": "",
        }
        self.get_logger().info(
            f"trial {self._idx + 1}/{len(self.schedule)} "
            f"condition={condition} waypoints={len(goal.poses)}")
        future = self._client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

    def _apply_condition(self, condition: str):
        """A = frc_layer 旁路；B/B' = frc_layer 注入。服务不可用则记录并继续。"""
        cli = self._enable_cli if condition.startswith("B") else self._disable_cli
        name = "/frc/enable" if condition.startswith("B") else "/frc/disable"
        if cli.wait_for_service(timeout_sec=3.0):
            cli.call_async(Trigger.Request())
            self.get_logger().info(f"condition {condition}: called {name}")
        else:
            self.get_logger().warn(
                f"{name} 不可用（frc_mode=off 或插件未加载），按原样继续")

    # ---------- 回调 ----------

    def _on_goal_response(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self._finish_trial("REJECTED", [])
            return
        result_future = self._goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future):
        result = future.result()
        status_map = {4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}
        status = status_map.get(result.status, f"STATUS_{result.status}")
        missed = list(result.result.missed_waypoints)
        self._finish_trial(status, missed)

    def _finish_trial(self, status: str, missed_waypoints: list):
        trial = self._current_trial
        trial["end_time"] = time.strftime("%Y-%m-%dT%H:%M:%S")
        trial["duration_s"] = round(time.time() - self._trial_start, 1)
        trial["result"] = status
        trial["missed_waypoints"] = [int(i) for i in missed_waypoints]
        out = self._trials_dir / f"{trial['trial_id']}.yaml"
        out.write_text(yaml.safe_dump(trial, allow_unicode=True,
                                      sort_keys=False))
        self.get_logger().info(
            f"trial {trial['trial_id']} -> {status} "
            f"({trial['duration_s']}s) saved {out}")

        self._idx += 1
        pause = float(self.get_parameter("inter_trial_pause_s").value)
        self.get_logger().info(f"复位窗口 {pause:.0f}s 后开始下一趟…")
        self.create_timer(pause, self._next_once)

    def _next_once(self):
        # one-shot：rclpy 定时器无 once 选项，用标志位防重入
        if self._current_trial is not None and \
                self._current_trial.get("schedule_index") == self._idx - 1:
            self._current_trial = None
            self._start_next_trial()


def main(args=None):
    rclpy.init(args=args)
    node = TrialRunnerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
