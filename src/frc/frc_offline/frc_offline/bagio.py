"""rosbags 纯 Python 读包层：工作站与 CI 无需 ROS 环境。

- frc_msgs 自定义消息从仓库源码 `src/frc/frc_msgs/msg/*.msg` 注册；
- nav2_msgs 的 Costmap / BehaviorTreeLog 不在 rosbags 内置 store，
  以内嵌文本注册（与 Humble 定义逐字一致）。
"""

from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_types_from_msg, get_typestore

# Humble nav2_msgs 中 FRC 用到的三个类型（rosbags 未内置）
NAV2_MSG_DEFS = {
    "nav2_msgs/msg/CostmapMetaData": """
builtin_interfaces/Time map_load_time
builtin_interfaces/Time update_time
string layer
float32 resolution
uint32 size_x
uint32 size_y
geometry_msgs/Pose origin
""",
    "nav2_msgs/msg/Costmap": """
std_msgs/Header header
nav2_msgs/CostmapMetaData metadata
uint8[] data
""",
    "nav2_msgs/msg/BehaviorTreeStatusChange": """
builtin_interfaces/Time timestamp
string node_name
string previous_status
string current_status
""",
    "nav2_msgs/msg/BehaviorTreeLog": """
builtin_interfaces/Time timestamp
nav2_msgs/BehaviorTreeStatusChange[] event_log
""",
}


def _find_frc_msgs_dir() -> Path | None:
    """从本文件向上找仓库内的 frc_msgs/msg（工作站 checkout 布局固定）。"""
    here = Path(__file__).resolve()
    for parent in here.parents:
        candidate = parent / "frc_msgs" / "msg"
        if candidate.is_dir():
            return candidate
    return None


def make_typestore():
    store = get_typestore(Stores.ROS2_HUMBLE)
    defs = {}
    for name, text in NAV2_MSG_DEFS.items():
        defs.update(get_types_from_msg(text, name))
    msg_dir = _find_frc_msgs_dir()
    if msg_dir is not None:
        for msg_file in sorted(msg_dir.glob("*.msg")):
            name = f"frc_msgs/msg/{msg_file.stem}"
            defs.update(get_types_from_msg(msg_file.read_text(), name))
    store.register(defs)
    return store


class BagReader:
    """单个 rosbag2 目录的便捷读取器。"""

    def __init__(self, bag_path):
        self.path = Path(bag_path)
        self.store = make_typestore()
        self._reader = AnyReader([self.path], default_typestore=self.store)
        self._reader.open()

    def close(self):
        self._reader.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def topics(self) -> dict:
        return {c.topic: c.msgtype for c in self._reader.connections}

    def messages(self, topics):
        """按时间序产出 (topic, t_s, msg)。topics 为话题名列表。"""
        conns = [c for c in self._reader.connections if c.topic in set(topics)]
        for conn, t_ns, raw in self._reader.messages(connections=conns):
            yield conn.topic, t_ns * 1e-9, self._reader.deserialize(
                raw, conn.msgtype)

    # ---------- 常用轨迹/标量序列 ----------

    def read_odometry(self, topic: str) -> np.ndarray:
        """-> (N,6) [t, x, y, yaw, v, w]。"""
        rows = []
        for _, t, m in self.messages([topic]):
            q = m.pose.pose.orientation
            yaw = float(np.arctan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
            rows.append((t, m.pose.pose.position.x, m.pose.pose.position.y,
                         yaw, np.hypot(m.twist.twist.linear.x,
                                       m.twist.twist.linear.y),
                         m.twist.twist.angular.z))
        return np.asarray(rows, dtype=np.float64) if rows \
            else np.zeros((0, 6))

    def read_float_array(self, topic: str) -> list:
        """Float32MultiArray 序列 -> [(t, [..]), ...]（degeneracy / correction）。"""
        return [(t, list(m.data)) for _, t, m in self.messages([topic])]

    def read_twist(self, topic: str) -> np.ndarray:
        """cmd_vel -> (N,3) [t, vx, wz]。"""
        rows = [(t, m.linear.x, m.angular.z)
                for _, t, m in self.messages([topic])]
        return np.asarray(rows, dtype=np.float64) if rows else np.zeros((0, 3))


def nearest_row(arr: np.ndarray, t: float, max_dt: float = 0.5):
    """按第 0 列时间取最近行；超出 max_dt 返回 None。"""
    if not len(arr):
        return None
    idx = int(np.argmin(np.abs(arr[:, 0] - t)))
    if abs(arr[idx, 0] - t) > max_dt:
        return None
    return arr[idx]
