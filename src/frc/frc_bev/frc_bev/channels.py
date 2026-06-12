"""BEV 通道栅格化实现。全部 numpy 向量化，无 Python 像素循环。

约定：输入点云已变换到"机器人居中、航向对齐"的 BEV 局部系
（x 向前、y 向左），高度为相对车体原点的 z。
"""

import numpy as np


def rasterize_geometry(points_xyz: np.ndarray, cfg) -> dict:
    """点云 -> 几何通道字典。

    points_xyz: (N,3) BEV 局部系点。z 已按 cfg.z_min/z_max 裁剪在外层完成。
    返回 {name: float32 (H,W)}，未观测格 max_z/min_z/mean_z/var_z 为 0、unknown 为 1。
    """
    n = cfg.grid_size
    half = cfg.size_m / 2.0

    max_z = np.zeros((n, n), dtype=np.float32)
    min_z = np.zeros((n, n), dtype=np.float32)
    mean_z = np.zeros((n, n), dtype=np.float32)
    var_z = np.zeros((n, n), dtype=np.float32)
    density = np.zeros((n, n), dtype=np.float32)
    unknown = np.ones((n, n), dtype=np.float32)

    if points_xyz.size:
        x = points_xyz[:, 0]
        y = points_xyz[:, 1]
        z = points_xyz[:, 2].astype(np.float64)

        in_range = (np.abs(x) < half) & (np.abs(y) < half)
        x, y, z = x[in_range], y[in_range], z[in_range]

        if x.size:
            # 行 = x（向前），列 = y（向左）；原点在栅格中心
            row = np.clip(((x + half) / cfg.resolution).astype(np.int64), 0, n - 1)
            col = np.clip(((y + half) / cfg.resolution).astype(np.int64), 0, n - 1)
            flat = row * n + col

            count = np.bincount(flat, minlength=n * n).astype(np.float64)
            sum_z = np.bincount(flat, weights=z, minlength=n * n)
            sum_z2 = np.bincount(flat, weights=z * z, minlength=n * n)

            occupied = count > 0
            mean_flat = np.zeros(n * n)
            mean_flat[occupied] = sum_z[occupied] / count[occupied]
            var_flat = np.zeros(n * n)
            var_flat[occupied] = np.maximum(
                sum_z2[occupied] / count[occupied] - mean_flat[occupied] ** 2, 0.0
            )

            max_flat = np.full(n * n, -np.inf)
            np.maximum.at(max_flat, flat, z)
            min_flat = np.full(n * n, np.inf)
            np.minimum.at(min_flat, flat, z)
            max_flat[~occupied] = 0.0
            min_flat[~occupied] = 0.0

            max_z = max_flat.reshape(n, n).astype(np.float32)
            min_z = min_flat.reshape(n, n).astype(np.float32)
            mean_z = mean_flat.reshape(n, n).astype(np.float32)
            var_z = var_flat.reshape(n, n).astype(np.float32)
            # 点密度对数压缩到 [0,1] 量级，减小近处过密的动态范围
            density = np.log1p(count.reshape(n, n)).astype(np.float32) / 5.0
            unknown = (~occupied.reshape(n, n)).astype(np.float32)

    return {
        "max_z": max_z,
        "min_z": min_z,
        "mean_z": mean_z,
        "var_z": var_z,
        "density": density,
        "unknown": unknown,
    }


def resample_costmap(costmap: np.ndarray, costmap_resolution: float,
                     costmap_origin_xy: tuple, robot_xy: tuple, robot_yaw: float,
                     cfg) -> np.ndarray:
    """costmap（其源坐标系，如 odom）-> BEV 局部系 costmap 通道。

    costmap: (H,W) float32，0-254（lethal 253/254），未知格应预先填 0。
    采用逆向映射：对 BEV 每格中心反算 costmap 索引（最近邻采样，等效
    粗到细情形下的 max-pool 语义损失可忽略，0.05->0.1 时取 2x2 邻域 max）。
    """
    n = cfg.grid_size
    half = cfg.size_m / 2.0

    # BEV 各格中心在局部系的坐标
    centers = (np.arange(n) + 0.5) * cfg.resolution - half
    bev_x, bev_y = np.meshgrid(centers, centers, indexing="ij")

    # 局部系 -> costmap 源坐标系
    cos_y, sin_y = np.cos(robot_yaw), np.sin(robot_yaw)
    wx = robot_xy[0] + bev_x * cos_y - bev_y * sin_y
    wy = robot_xy[1] + bev_x * sin_y + bev_y * cos_y

    ci = np.floor((wy - costmap_origin_xy[1]) / costmap_resolution).astype(np.int64)
    cj = np.floor((wx - costmap_origin_xy[0]) / costmap_resolution).astype(np.int64)

    out = np.zeros((n, n), dtype=np.float32)
    h, w = costmap.shape
    # 2x2 邻域 max（覆盖 0.05m -> 0.1m 的重采样）
    for di in (0, 1):
        for dj in (0, 1):
            ii = np.clip(ci + di - 1, 0, h - 1)
            jj = np.clip(cj + dj - 1, 0, w - 1)
            valid = (ci + di - 1 >= 0) & (ci + di - 1 < h) & \
                    (cj + dj - 1 >= 0) & (cj + dj - 1 < w)
            sampled = np.where(valid, costmap[ii, jj], 0.0)
            out = np.maximum(out, sampled.astype(np.float32))
    return out / 254.0
