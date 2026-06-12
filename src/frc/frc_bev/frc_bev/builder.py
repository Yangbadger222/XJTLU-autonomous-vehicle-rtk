"""BevBuilder：点云环形缓冲 + 多通道栅格化。

在线被 risk_pipeline_node 调用，离线被 bag_to_bev_dataset.py 调用——同一份代码。
坐标约定：push_cloud 接收 odom 系点云（cloud 已配对 lio_odom 位姿变换后），
build() 时以当前位姿居中、航向对齐切出 BEV 局部系。
"""

from collections import deque
from dataclasses import dataclass, field

import numpy as np

from frc_bev.channels import rasterize_geometry, resample_costmap
from frc_bev.config import BevConfig


@dataclass
class SE2:
    x: float
    y: float
    yaw: float


@dataclass
class BevFrame:
    tensor: np.ndarray              # float16 [C, H, W]
    health: np.ndarray              # float32 [K]
    stamp: float
    pose: SE2
    channel_names: tuple
    fingerprint: str
    meta: dict = field(default_factory=dict)


class BevBuilder:
    def __init__(self, cfg: BevConfig | None = None):
        self.cfg = cfg or BevConfig()
        self._clouds: deque = deque()   # (stamp, (N,3) odom 系点)

    def push_cloud(self, xyz_odom: np.ndarray, stamp: float) -> None:
        """缓存一帧 odom 系点云，并裁掉聚合窗口之外的旧帧。"""
        if xyz_odom.ndim != 2 or xyz_odom.shape[1] != 3:
            raise ValueError(f"expect (N,3) cloud, got {xyz_odom.shape}")
        self._clouds.append((float(stamp), np.asarray(xyz_odom, dtype=np.float32)))
        horizon = stamp - self.cfg.agg_window_s
        while self._clouds and self._clouds[0][0] < horizon:
            self._clouds.popleft()

    def reset(self) -> None:
        self._clouds.clear()

    def build(self, pose: SE2, costmap: np.ndarray | None,
              health: np.ndarray, stamp: float,
              costmap_meta: dict | None = None) -> BevFrame:
        """聚合窗口内点云 -> BEV 局部系 -> 多通道张量。

        costmap: odom 系 (H,W) float32 0-254 或 None；
        costmap_meta: {"resolution": float, "origin_xy": (x, y)}。
        """
        cfg = self.cfg

        if self._clouds:
            merged = np.concatenate([c for _, c in self._clouds], axis=0)
        else:
            merged = np.zeros((0, 3), dtype=np.float32)

        # odom 系 -> BEV 局部系（机器人居中、航向对齐）
        cos_y, sin_y = np.cos(pose.yaw), np.sin(pose.yaw)
        dx = merged[:, 0] - pose.x
        dy = merged[:, 1] - pose.y
        local = np.empty_like(merged)
        local[:, 0] = dx * cos_y + dy * sin_y
        local[:, 1] = -dx * sin_y + dy * cos_y
        local[:, 2] = merged[:, 2]

        # 高度窗口裁剪（与上游裁剪取交，F5）
        keep = (local[:, 2] >= cfg.z_min) & (local[:, 2] <= cfg.z_max)
        local = local[keep]

        geom = rasterize_geometry(local, cfg)

        layers = []
        for name in cfg.channels:
            if name == "costmap":
                if costmap is not None and costmap_meta is not None:
                    layers.append(resample_costmap(
                        costmap, costmap_meta["resolution"],
                        costmap_meta["origin_xy"], (pose.x, pose.y), pose.yaw, cfg))
                else:
                    layers.append(np.zeros((cfg.grid_size, cfg.grid_size),
                                           dtype=np.float32))
            else:
                layers.append(geom[name])

        tensor = np.stack(layers, axis=0).astype(np.float16)
        health_vec = np.asarray(health, dtype=np.float32)
        if health_vec.shape != (len(cfg.health_keys),):
            raise ValueError(
                f"health vector must have {len(cfg.health_keys)} entries "
                f"({cfg.health_keys}), got {health_vec.shape}")

        return BevFrame(
            tensor=tensor,
            health=health_vec,
            stamp=float(stamp),
            pose=pose,
            channel_names=cfg.channels,
            fingerprint=cfg.fingerprint(),
        )
