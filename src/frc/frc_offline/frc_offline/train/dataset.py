"""数据集加载 + BEV 合法增广集（设计文档 §5.3）。

增广：随机旋转 ±180°（label/weight 同步、最近邻）、左右翻转、
density 乘性噪声、costmap 通道 p=0.2 整体置零（防抄 costmap）、健康标量高斯噪。
不做尺度增广——物理分辨率固定。
"""

import json
from pathlib import Path

import numpy as np

from frc_bev.config import BevConfig
from frc_bev.labels import IGNORE


def rotate_nn(img: np.ndarray, angle_rad: float, fill=0):
    """绕中心最近邻旋转（label/weight/通道共用，无插值伪标签）。"""
    h, w = img.shape[-2:]
    cy, cx = (h - 1) / 2.0, (w - 1) / 2.0
    yy, xx = np.meshgrid(np.arange(h), np.arange(w), indexing="ij")
    cos_a, sin_a = np.cos(-angle_rad), np.sin(-angle_rad)
    sy = cy + (yy - cy) * cos_a - (xx - cx) * sin_a
    sx = cx + (yy - cy) * sin_a + (xx - cx) * cos_a
    syi = np.round(sy).astype(np.int64)
    sxi = np.round(sx).astype(np.int64)
    valid = (syi >= 0) & (syi < h) & (sxi >= 0) & (sxi < w)
    out = np.full_like(img, fill)
    if img.ndim == 3:
        out[:, valid] = img[:, syi[valid], sxi[valid]]
    else:
        out[valid] = img[syi[valid], sxi[valid]]
    return out


def augment(bev, label, weight, health, cfg: BevConfig, rng):
    angle = rng.uniform(-np.pi, np.pi)
    bev = rotate_nn(bev, angle)
    label = rotate_nn(label, angle, fill=IGNORE)
    weight = rotate_nn(weight, angle)

    if rng.random() < 0.5:                      # 左右翻转（y 轴=列）
        bev = bev[:, :, ::-1].copy()
        label = label[:, ::-1].copy()
        weight = weight[:, ::-1].copy()

    di = cfg.channels.index("density")
    bev[di] *= rng.uniform(0.7, 1.3)
    if "costmap" in cfg.channels and rng.random() < 0.2:
        bev[cfg.channels.index("costmap")] = 0.0
    health = health + rng.normal(0, 0.05, size=health.shape).astype(np.float32)
    return bev, label, weight, health


class FrcBevDataset:
    """torch.utils.data.Dataset 兼容（不强制继承，纯 numpy 返回）。"""

    def __init__(self, dataset_dir, split="train", augment_enabled=None,
                 cfg=None, seed=0):
        self.dir = Path(dataset_dir)
        self.cfg = cfg or BevConfig()
        manifest = self.dir / "manifest.yaml"
        if manifest.exists():
            import yaml
            fp = yaml.safe_load(manifest.read_text())["bev_fingerprint"]
            if fp != self.cfg.fingerprint():
                raise RuntimeError(
                    f"dataset fingerprint {fp} != BevConfig "
                    f"{self.cfg.fingerprint()}")
        split_file = self.dir / "splits" / f"{split}.txt"
        self.names = [n for n in split_file.read_text().splitlines()
                      if n.strip()]
        self.augment_enabled = (split == "train") \
            if augment_enabled is None else augment_enabled
        self.rng = np.random.default_rng(seed)

    def __len__(self):
        return len(self.names)

    def __getitem__(self, idx):
        data = np.load(self.dir / "samples" / self.names[idx],
                       allow_pickle=False)
        bev = data["bev"].astype(np.float32)
        label = data["label"].astype(np.uint8)
        weight = data["weight"].astype(np.float32)
        health = data["health"].astype(np.float32)
        if self.augment_enabled:
            bev, label, weight, health = augment(
                bev, label, weight, health, self.cfg, self.rng)
        return {
            "bev": bev, "label": label, "weight": weight, "health": health,
            "meta": json.loads(str(data["meta"])),
        }


def collate(batch):
    """numpy -> torch 张量批（在 train 脚本里 import torch 后调用）。"""
    import torch
    return {
        "bev": torch.from_numpy(np.stack([b["bev"] for b in batch])),
        "label": torch.from_numpy(np.stack([b["label"] for b in batch])),
        "weight": torch.from_numpy(np.stack([b["weight"] for b in batch])),
        "health": torch.from_numpy(np.stack([b["health"] for b in batch])),
    }
