"""swath 标注：失败事件 -> BEV 像素级 label/weight/ignore 掩码。

论文方法节核心算法（设计文档 §4.3）：
- 正区域 = 事件前 lookback 轨迹足迹扫掠带 + 沿运动方向外推 d_ahead；
- 像素权重 = 层级权重 x 距事件点高斯衰减；
- ignore 区 = lookback 窗口内、swath 外的邻域（归因不确定，不回传梯度）；
- 归因过滤闸门：纯定位退化失败（LIO 退化超阈值且 swath 内无几何异常）
  -> 空间标签整体转 ignore，样本只保留健康标量。
"""

from dataclasses import dataclass

import numpy as np


@dataclass
class LabelConfig:
    lookback_s: float = 4.0
    d_ahead_m: float = 1.5
    footprint_radius_m: float = 0.45     # robot_radius 0.38625 + 余量
    sigma_m: float = 1.0                 # 距事件点高斯衰减
    ignore_margin_m: float = 2.0         # swath 外 ignore 邻域
    tier_weights: dict = None            # gold/silver/bronze -> 权重
    # 归因过滤
    lio_min_eig_thresh: float = 75.0     # 等于 IESKF 正则化阈值
    geometry_max_z_thresh: float = 0.08  # swath 内 max_z 凸起判定（米）
    geometry_costmap_thresh: float = 0.5 # swath 内 costmap 通道高 cost 判定

    def __post_init__(self):
        if self.tier_weights is None:
            self.tier_weights = {"gold": 1.0, "silver": 0.6, "bronze": 0.3}


IGNORE = 255


def _rasterize_disc(mask: np.ndarray, cx: float, cy: float, radius: float, cfg_bev):
    """在 BEV 局部系掩码上画圆盘（向量化）。"""
    n = cfg_bev.grid_size
    half = cfg_bev.size_m / 2.0
    centers = (np.arange(n) + 0.5) * cfg_bev.resolution - half
    gx, gy = np.meshgrid(centers, centers, indexing="ij")
    mask |= (gx - cx) ** 2 + (gy - cy) ** 2 <= radius ** 2


def label_event(event_xy_local: tuple, traj_xy_local: np.ndarray,
                severity: str, bev_tensor: np.ndarray, channel_names: tuple,
                lio_min_eig: float, cfg_bev, cfg: LabelConfig | None = None):
    """单事件 swath 标注。

    event_xy_local: 事件位置（当前帧 BEV 局部系，米）
    traj_xy_local: (T,2) lookback 轨迹在 BEV 局部系的足迹点（含事件时刻）
    severity: gold/silver/bronze
    bev_tensor: (C,H,W) 本帧 BEV（归因过滤用）
    返回 (label uint8 HxW {0,1,255}, weight float16 HxW)
    """
    cfg = cfg or LabelConfig()
    n = cfg_bev.grid_size

    swath = np.zeros((n, n), dtype=bool)
    for x, y in traj_xy_local:
        _rasterize_disc(swath, x, y, cfg.footprint_radius_m, cfg_bev)

    # 沿事件时刻运动方向外推 d_ahead
    if len(traj_xy_local) >= 2:
        direction = traj_xy_local[-1] - traj_xy_local[-2]
        norm = np.linalg.norm(direction)
        if norm > 1e-6:
            direction = direction / norm
            steps = np.arange(0.0, cfg.d_ahead_m + 1e-9, cfg_bev.resolution)
            for step in steps:
                px, py = traj_xy_local[-1] + direction * step
                _rasterize_disc(swath, px, py, cfg.footprint_radius_m, cfg_bev)

    # ignore 邻域 = swath 膨胀 ignore_margin 后去掉 swath 本体
    margin_cells = int(round(cfg.ignore_margin_m / cfg_bev.resolution))
    dilated = swath.copy()
    for _ in range(margin_cells):
        d = dilated
        dilated = d.copy()
        dilated[1:, :] |= d[:-1, :]
        dilated[:-1, :] |= d[1:, :]
        dilated[:, 1:] |= d[:, :-1]
        dilated[:, :-1] |= d[:, 1:]
    ignore_ring = dilated & ~swath

    # 归因过滤闸门：纯定位失败 -> 空间标签整体 ignore
    if lio_min_eig < cfg.lio_min_eig_thresh and not _has_geometry_anomaly(
            bev_tensor, channel_names, swath, cfg):
        label = np.full((n, n), IGNORE, dtype=np.uint8)
        weight = np.zeros((n, n), dtype=np.float16)
        return label, weight

    # 像素权重：层级权重 x 距事件点高斯衰减
    half = cfg_bev.size_m / 2.0
    centers = (np.arange(n) + 0.5) * cfg_bev.resolution - half
    gx, gy = np.meshgrid(centers, centers, indexing="ij")
    dist2 = (gx - event_xy_local[0]) ** 2 + (gy - event_xy_local[1]) ** 2
    gauss = np.exp(-dist2 / (2.0 * cfg.sigma_m ** 2))
    tier_w = cfg.tier_weights.get(severity, 0.3)

    label = np.zeros((n, n), dtype=np.uint8)
    label[ignore_ring] = IGNORE
    label[swath] = 1
    weight = np.zeros((n, n), dtype=np.float32)
    weight[swath] = tier_w * gauss[swath]
    return label, weight.astype(np.float16)


def _has_geometry_anomaly(bev_tensor: np.ndarray, channel_names: tuple,
                          swath: np.ndarray, cfg: LabelConfig) -> bool:
    """swath 内是否存在几何异常（max_z 凸起或 costmap 高 cost）。"""
    if not swath.any():
        return False
    if "max_z" in channel_names:
        zi = channel_names.index("max_z")
        if float(np.asarray(bev_tensor[zi], dtype=np.float32)[swath].max()) \
                > cfg.geometry_max_z_thresh:
            return True
    if "costmap" in channel_names:
        ci = channel_names.index("costmap")
        if float(np.asarray(bev_tensor[ci], dtype=np.float32)[swath].max()) \
                > cfg.geometry_costmap_thresh:
            return True
    return False


def label_negative(traj_xy_local: np.ndarray, cfg_bev,
                   cfg: LabelConfig | None = None, hard: bool = False):
    """成功通过帧的负标注：swath 标 0、权重 1.0；hard negative 权重 1.5。"""
    cfg = cfg or LabelConfig()
    n = cfg_bev.grid_size
    swath = np.zeros((n, n), dtype=bool)
    for x, y in traj_xy_local:
        _rasterize_disc(swath, x, y, cfg.footprint_radius_m, cfg_bev)
    label = np.full((n, n), IGNORE, dtype=np.uint8)
    label[swath] = 0
    weight = np.zeros((n, n), dtype=np.float32)
    weight[swath] = 1.5 if hard else 1.0
    return label, weight.astype(np.float16)
