"""BEV patch 切分与编码（特征锚共用，v1 阶段 0：手工特征 + PCA 白化）。

阶段 0 编码免训练即可用：各通道 mean/var + max_z 梯度方向直方图，约 64 维。
阶段 1 的自监督 conv 编码器在 frc_offline/train 中实现，接口与本模块对齐
（encode(patches) -> (M, D)），上线时替换 PatchEncoder 即可。
"""

import numpy as np


def split_patches(tensor: np.ndarray, cfg) -> tuple:
    """BEV 张量 -> (patches (M,C,P,P), centers_xy (M,2) BEV 局部系米)。"""
    c, h, w = tensor.shape
    p = int(round(cfg.patch_size_m / cfg.resolution))
    s = int(round(cfg.patch_stride_m / cfg.resolution))
    half = cfg.size_m / 2.0

    patches, centers = [], []
    for i0 in range(0, h - p + 1, s):
        for j0 in range(0, w - p + 1, s):
            patches.append(tensor[:, i0:i0 + p, j0:j0 + p])
            centers.append((
                (i0 + p / 2.0) * cfg.resolution - half,
                (j0 + p / 2.0) * cfg.resolution - half,
            ))
    return np.asarray(patches, dtype=np.float32), np.asarray(centers, dtype=np.float32)


def handcrafted_features(patches: np.ndarray, channel_names: tuple) -> np.ndarray:
    """阶段 0 手工特征：每通道 mean/var + max_z 梯度方向直方图（8 bin）。

    patches: (M,C,P,P) -> (M, 2C+16)
    """
    m, c, _, _ = patches.shape
    feats = [patches.mean(axis=(2, 3)), patches.var(axis=(2, 3))]  # (M,C) x2

    if "max_z" in channel_names:
        zi = channel_names.index("max_z")
        gx = np.diff(patches[:, zi], axis=1)[:, :, :-1]   # (M,P-1,P-1)
        gy = np.diff(patches[:, zi], axis=2)[:, :-1, :]
        mag = np.hypot(gx, gy)
        ang = np.arctan2(gy, gx)                          # [-pi, pi]
        hist = np.zeros((m, 8), dtype=np.float32)
        bin_idx = np.clip(((ang + np.pi) / (2 * np.pi) * 8).astype(np.int64), 0, 7)
        for b in range(8):
            hist[:, b] = (mag * (bin_idx == b)).sum(axis=(1, 2))
        norm = hist.sum(axis=1, keepdims=True)
        hist = np.divide(hist, norm, out=np.zeros_like(hist), where=norm > 1e-9)
        feats.append(hist)
        # 梯度能量分箱（粗糙度 8 维）：区分平地/路沿/台阶
        energy = np.log1p(mag.sum(axis=(1, 2)))
        feats.append(np.stack([energy] + [np.zeros(m, dtype=np.float32)] * 7, axis=1))

    return np.concatenate([f.astype(np.float32) for f in feats], axis=1)


class PcaWhitener:
    """PCA 白化（在 prototype 库构建时拟合，在线只做 transform）。"""

    def __init__(self, dim: int = 32):
        self.dim = dim
        self.mean_ = None
        self.components_ = None
        self.scale_ = None

    def fit(self, feats: np.ndarray) -> "PcaWhitener":
        self.mean_ = feats.mean(axis=0)
        centered = feats - self.mean_
        # SVD 比协方差特征分解数值更稳
        _, s, vt = np.linalg.svd(centered, full_matrices=False)
        k = min(self.dim, vt.shape[0])
        self.components_ = vt[:k]
        self.scale_ = s[:k] / np.sqrt(max(len(feats) - 1, 1)) + 1e-8
        return self

    def transform(self, feats: np.ndarray) -> np.ndarray:
        z = (feats - self.mean_) @ self.components_.T / self.scale_
        norm = np.linalg.norm(z, axis=1, keepdims=True)
        return z / np.maximum(norm, 1e-9)

    def to_dict(self) -> dict:
        return {"dim": self.dim, "mean": self.mean_.tolist(),
                "components": self.components_.tolist(),
                "scale": self.scale_.tolist()}

    @classmethod
    def from_dict(cls, d: dict) -> "PcaWhitener":
        obj = cls(dim=d["dim"])
        obj.mean_ = np.asarray(d["mean"], dtype=np.float64)
        obj.components_ = np.asarray(d["components"], dtype=np.float64)
        obj.scale_ = np.asarray(d["scale"], dtype=np.float64)
        return obj


class PrototypeRetriever:
    """prototype 库余弦 kNN 检索。库规模 ~300，numpy 即可（< 1 ms）。"""

    def __init__(self, embeddings: np.ndarray, severities: np.ndarray,
                 k: int = 3, temperature: float = 0.07,
                 similarity_floor: float = 0.5):
        assert len(embeddings) == len(severities)
        norms = np.linalg.norm(embeddings, axis=1, keepdims=True)
        self.embeddings = embeddings / np.maximum(norms, 1e-9)
        self.severities = severities
        self.k = k
        self.temperature = temperature
        self.similarity_floor = similarity_floor

    def score(self, queries: np.ndarray) -> np.ndarray:
        """queries (M,D) 单位化嵌入 -> 风险分 (M,) in [0,1]。"""
        if not len(self.embeddings):
            return np.zeros(len(queries), dtype=np.float32)
        sim = queries @ self.embeddings.T                      # (M, N)
        k = min(self.k, sim.shape[1])
        topk_idx = np.argpartition(-sim, k - 1, axis=1)[:, :k]
        rows = np.arange(len(queries))[:, None]
        topk_sim = sim[rows, topk_idx]
        topk_sev = self.severities[topk_idx]
        # 低于相似度地板的近邻不贡献风险（防"到处加 cost"）
        gate = np.maximum(topk_sim - self.similarity_floor, 0.0) \
            / max(1.0 - self.similarity_floor, 1e-9)
        weights = np.exp(topk_sim / self.temperature)
        weights /= weights.sum(axis=1, keepdims=True)
        return np.clip((weights * gate * topk_sev).sum(axis=1), 0.0, 1.0) \
            .astype(np.float32)
