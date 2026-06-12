"""E2 离线风险图质量评测：AUROC / AP，四方法对比（设计文档 §3.3）。

方法：
- costmap     ：几何 costmap 按 cost 当风险分（内置 baseline）
- map_anchor  ：anchors.db 地图锚高斯渲染（无泛化能力，S4 上必然失效）
- feature     ：prototypes.npz patch 检索
- dual        ：max(map_anchor, feature) —— 双锚融合
- model       ：可选，TinyUNet checkpoint（v2）

正负像素来自数据集 swath 标注（label 0/1，ignore 排除）。
按 split 输出（test_s4 即 E3a 留出场景；test_layout 即 E3b 摆放改变）。

用法：
  python -m frc_offline.eval.offline_auroc --dataset datasets/frc-v1-... \
      --split test_s4 [--anchors-db runtime-data/frc/anchors.db] \
      [--prototypes runtime-data/frc/models/prototypes.npz] [--ckpt ...]
"""

import argparse
import json
import math
from pathlib import Path

import numpy as np

from frc_bev.config import BevConfig
from frc_bev.labels import IGNORE
from frc_bev.patches import (PcaWhitener, PrototypeRetriever,
                             handcrafted_features, split_patches)


# ---------- 指标（纯 numpy，无 sklearn 依赖）----------

def auroc(labels: np.ndarray, scores: np.ndarray) -> float:
    """rank 法 AUROC。labels {0,1}。"""
    pos = scores[labels == 1]
    neg = scores[labels == 0]
    if not len(pos) or not len(neg):
        return float("nan")
    order = np.argsort(np.concatenate([pos, neg]), kind="mergesort")
    ranks = np.empty(len(order), dtype=np.float64)
    ranks[order] = np.arange(1, len(order) + 1)
    # 并列分数取平均秩
    all_scores = np.concatenate([pos, neg])
    sorted_scores = all_scores[order]
    i = 0
    while i < len(sorted_scores):
        j = i
        while j + 1 < len(sorted_scores) and \
                sorted_scores[j + 1] == sorted_scores[i]:
            j += 1
        if j > i:
            ranks[order[i:j + 1]] = (i + 1 + j + 1) / 2.0
        i = j + 1
    r_pos = ranks[: len(pos)].sum()
    n_pos, n_neg = len(pos), len(neg)
    return float((r_pos - n_pos * (n_pos + 1) / 2.0) / (n_pos * n_neg))


def average_precision(labels: np.ndarray, scores: np.ndarray) -> float:
    if labels.sum() == 0:
        return float("nan")
    order = np.argsort(-scores, kind="mergesort")
    lab = labels[order]
    tp = np.cumsum(lab)
    precision = tp / np.arange(1, len(lab) + 1)
    return float((precision * lab).sum() / lab.sum())


# ---------- 评分器 ----------

def gaussian_local(n, res, half_m, cx, cy, weight, sigma, out):
    centers = (np.arange(n) + 0.5) * res - half_m
    gx, gy = np.meshgrid(centers, centers, indexing="ij")
    g = weight * np.exp(-((gx - cx) ** 2 + (gy - cy) ** 2)
                        / (2.0 * sigma ** 2))
    np.maximum(out, g, out=out)


class CostmapScorer:
    name = "costmap"

    def __init__(self, cfg):
        self.ci = cfg.channels.index("costmap")

    def score(self, bev, health, meta):
        return np.asarray(bev[self.ci], dtype=np.float32)


class MapAnchorScorer:
    name = "map_anchor"

    def __init__(self, cfg, db_path, scene_id, sigma=0.75):
        import sqlite3
        self.cfg, self.sigma = cfg, sigma
        con = sqlite3.connect(db_path)
        self.anchors = con.execute(
            "SELECT map_x,map_y,severity,p_usable,state FROM anchors "
            "WHERE state != 'retired' AND scene_id = ?", (scene_id,)) \
            .fetchall()
        con.close()

    def score(self, bev, health, meta):
        n = self.cfg.grid_size
        out = np.zeros((n, n), dtype=np.float32)
        pose = meta.get("pose_map") or meta.get("pose_odom")
        if pose is None:
            return out
        px, py, pyaw = pose
        cos_y, sin_y = math.cos(-pyaw), math.sin(-pyaw)
        for ax, ay, sev, p, state in self.anchors:
            w = sev * p * (0.5 if state == "stale" else 1.0)
            rx, ry = ax - px, ay - py
            lx = rx * cos_y - ry * sin_y
            ly = rx * sin_y + ry * cos_y
            if abs(lx) > self.cfg.size_m or abs(ly) > self.cfg.size_m:
                continue
            gaussian_local(n, self.cfg.resolution, self.cfg.size_m / 2.0,
                           lx, ly, w, self.sigma, out)
        return out


class FeatureScorer:
    name = "feature"

    def __init__(self, cfg, prototypes_path, sigma=1.0):
        self.cfg, self.sigma = cfg, sigma
        data = np.load(prototypes_path, allow_pickle=False)
        if str(data["bev_fingerprint"]) != cfg.fingerprint():
            raise RuntimeError("prototypes 指纹与 BevConfig 不一致")
        self.whitener = PcaWhitener.from_dict(
            json.loads(str(data["pca_json"])))
        self.retriever = PrototypeRetriever(
            data["embeddings"].astype(np.float32),
            data["severities"].astype(np.float32))

    def score(self, bev, health, meta):
        n = self.cfg.grid_size
        patches, centers = split_patches(bev.astype(np.float32), self.cfg)
        emb = self.whitener.transform(
            handcrafted_features(patches, self.cfg.channels))
        scores = self.retriever.score(emb)
        out = np.zeros((n, n), dtype=np.float32)
        for (lx, ly), s in zip(centers, scores):
            if s > 0.01:
                gaussian_local(n, self.cfg.resolution, self.cfg.size_m / 2.0,
                               lx, ly, float(s), self.sigma, out)
        return out


class DualScorer:
    name = "dual"

    def __init__(self, map_scorer, feature_scorer):
        self.m, self.f = map_scorer, feature_scorer

    def score(self, bev, health, meta):
        return np.maximum(self.m.score(bev, health, meta),
                          self.f.score(bev, health, meta))


class ModelScorer:
    name = "model"

    def __init__(self, cfg, ckpt_path, device="cpu"):
        import torch

        from frc_offline.train.model import TinyUNet
        ckpt = torch.load(ckpt_path, map_location=device)
        if ckpt["bev_fingerprint"] != cfg.fingerprint():
            raise RuntimeError("checkpoint 指纹与 BevConfig 不一致")
        self.model = TinyUNet(in_channels=ckpt["in_channels"],
                              health_dim=ckpt["health_dim"],
                              use_health_film=ckpt["use_health_film"])
        self.model.load_state_dict(ckpt["state_dict"])
        self.model.eval()
        self.device = device

    def score(self, bev, health, meta):
        import torch
        with torch.no_grad():
            out = self.model(
                torch.from_numpy(bev[None].astype(np.float32)),
                torch.from_numpy(health[None].astype(np.float32)))
            return torch.sigmoid(out[0, 0]).numpy()


# ---------- 主流程 ----------

def evaluate(dataset_dir, split, scorers, cfg=None):
    cfg = cfg or BevConfig()
    names = (Path(dataset_dir) / "splits" / f"{split}.txt") \
        .read_text().splitlines()
    per_method = {s.name: {"scores": [], "labels": []} for s in scorers}

    for name in names:
        if not name.strip():
            continue
        data = np.load(Path(dataset_dir) / "samples" / name,
                       allow_pickle=False)
        bev = data["bev"].astype(np.float32)
        label = data["label"]
        health = data["health"]
        meta = json.loads(str(data["meta"]))
        valid = label != IGNORE
        if not valid.any():
            continue
        lab = (label[valid] == 1).astype(np.int8)
        for s in scorers:
            risk = s.score(bev, health, meta)
            per_method[s.name]["scores"].append(risk[valid])
            per_method[s.name]["labels"].append(lab)

    results = {}
    for mname, d in per_method.items():
        if not d["scores"]:
            results[mname] = {"auroc": float("nan"), "ap": float("nan")}
            continue
        scores = np.concatenate(d["scores"])
        labels = np.concatenate(d["labels"])
        results[mname] = {
            "auroc": round(auroc(labels, scores), 4),
            "ap": round(average_precision(labels, scores), 4),
            "n_pixels": int(len(labels)),
            "pos_rate": round(float(labels.mean()), 4),
        }
    return results


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--dataset", required=True)
    ap.add_argument("--split", default="test_s4")
    ap.add_argument("--anchors-db", default="")
    ap.add_argument("--scene-id", default="")
    ap.add_argument("--prototypes", default="")
    ap.add_argument("--ckpt", default="")
    args = ap.parse_args()

    cfg = BevConfig()
    scorers = [CostmapScorer(cfg)]
    map_s = feat_s = None
    if args.anchors_db:
        map_s = MapAnchorScorer(cfg, args.anchors_db, args.scene_id)
        scorers.append(map_s)
    if args.prototypes:
        feat_s = FeatureScorer(cfg, args.prototypes)
        scorers.append(feat_s)
    if map_s and feat_s:
        scorers.append(DualScorer(map_s, feat_s))
    if args.ckpt:
        scorers.append(ModelScorer(cfg, args.ckpt))

    results = evaluate(args.dataset, args.split, scorers, cfg)
    print(json.dumps({"split": args.split, "results": results},
                     indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
