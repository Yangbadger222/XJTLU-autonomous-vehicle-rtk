"""ROS 消息 <-> numpy 的唯一触点。core 模块（config/builder/channels/patches/labels）
禁止 import 本文件；只有在线节点与 bagio 经由这里转换。

本模块刻意只依赖消息的字段布局（duck-typing），同时兼容 rclpy 消息对象
与 rosbags 反序列化对象——离线工作站无需安装 ROS。
"""

import numpy as np

_POINT_STEP_CACHE = {}


def pointcloud2_to_xyz(msg) -> np.ndarray:
    """sensor_msgs/PointCloud2 -> (N,3) float32。只取 x/y/z 字段。"""
    offsets = {}
    for f in msg.fields:
        if f.name in ("x", "y", "z"):
            offsets[f.name] = f.offset
    if len(offsets) != 3:
        return np.zeros((0, 3), dtype=np.float32)

    data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    n = msg.width * msg.height
    if n == 0 or msg.point_step == 0:
        return np.zeros((0, 3), dtype=np.float32)
    data = data[: n * msg.point_step].reshape(n, msg.point_step)

    xyz = np.empty((n, 3), dtype=np.float32)
    for i, name in enumerate(("x", "y", "z")):
        off = offsets[name]
        xyz[:, i] = data[:, off:off + 4].copy().view(np.float32)[:, 0]
    finite = np.isfinite(xyz).all(axis=1)
    return xyz[finite]


def occupancy_grid_to_numpy(msg):
    """nav_msgs/OccupancyGrid -> ((H,W) float32 0-254, meta dict)。-1 未知 -> 0。"""
    h, w = msg.info.height, msg.info.width
    grid = np.asarray(msg.data, dtype=np.int16).reshape(h, w)
    out = np.where(grid < 0, 0, grid).astype(np.float32) * 254.0 / 100.0
    meta = {
        "resolution": float(msg.info.resolution),
        "origin_xy": (float(msg.info.origin.position.x),
                      float(msg.info.origin.position.y)),
    }
    return out, meta


def costmap_to_numpy(msg):
    """nav2_msgs/Costmap（/local_costmap/costmap_raw）-> ((H,W) float32 0-254, meta)。"""
    h, w = msg.metadata.size_y, msg.metadata.size_x
    grid = np.asarray(msg.data, dtype=np.uint8).reshape(h, w).astype(np.float32)
    meta = {
        "resolution": float(msg.metadata.resolution),
        "origin_xy": (float(msg.metadata.origin.position.x),
                      float(msg.metadata.origin.position.y)),
    }
    return grid, meta


def odometry_to_se2(msg):
    """nav_msgs/Odometry -> (x, y, yaw)。"""
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
    return float(p.x), float(p.y), yaw


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return float(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def yaw_to_quat(yaw: float):
    """yaw -> (x, y, z, w)。"""
    return 0.0, 0.0, float(np.sin(yaw / 2.0)), float(np.cos(yaw / 2.0))


def numpy_to_occupancy_grid_data(risk01: np.ndarray) -> np.ndarray:
    """[0,1] float 风险/置信度 -> OccupancyGrid data int8（0-100）。"""
    return np.clip(np.round(risk01 * 100.0), 0, 100).astype(np.int8)
