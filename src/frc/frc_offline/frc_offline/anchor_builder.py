"""anchor_builder：离线把 cleaned 事件写入 anchors.db（地图锚）并构建特征锚
prototype 库（prototypes.npz，供在线 risk_pipeline 检索）。

- 地图锚：事件位置挂最近 PGO keyframe（bag 内 /pgo/keyframes 最后一帧）；
  与 memory_manager 在线建锚逻辑一致，用于离线 bootstrap/重建。
- 特征锚：正样本 npz 中与 swath 重叠的 patch -> 手工特征 -> PCA 白化拟合 ->
  每个事件按帧间多样性选 top-k 代表嵌入，库规模 ~300（设计文档 §5.5）。

用法：
  python -m frc_offline.anchor_builder map --bag <rosbag2> \
      --events cleaned_events.jsonl --db runtime-data/frc/anchors.db \
      --scene-id S1 --session-id <ts>
  python -m frc_offline.anchor_builder prototypes --dataset datasets/frc-v1-... \
      --out runtime-data/frc/models/prototypes.npz
"""

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

from frc_bev import BevConfig
from frc_bev.patches import PcaWhitener, handcrafted_features, split_patches


def _import_anchor_store():
    """frc_nodes.anchor_store 零 rclpy 依赖；工作站直接从源码树 import。"""
    try:
        from frc_nodes.anchor_store import TIER_SEVERITY, AnchorStore
    except ImportError:
        here = Path(__file__).resolve()
        for parent in here.parents:
            candidate = parent / "frc_nodes"
            if (candidate / "frc_nodes" / "anchor_store.py").exists():
                sys.path.insert(0, str(candidate))
                break
        from frc_nodes.anchor_store import TIER_SEVERITY, AnchorStore
    return AnchorStore, TIER_SEVERITY


def build_map_anchors(bag_path, events_path, db_path, scene_id, session_id):
    from frc_offline.bagio import BagReader
    AnchorStore, TIER_SEVERITY = _import_anchor_store()

    keyframes = {}
    with BagReader(bag_path) as reader:
        if "/pgo/keyframes" in reader.topics():
            for _, _t, m in reader.messages(["/pgo/keyframes"]):
                keyframes = {}
                for kid, pose in zip(m.ids, m.poses):
                    q = pose.orientation
                    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                    keyframes[int(kid)] = (pose.position.x, pose.position.y,
                                           yaw)
    if not keyframes:
        raise RuntimeError("bag 缺少 /pgo/keyframes，地图锚需要 P2 补丁在线录包")

    store = AnchorStore(db_path, session_id=session_id, scene_id=scene_id)
    created = merged = 0
    with open(events_path) as f:
        for line in f:
            if not line.strip():
                continue
            e = json.loads(line)
            if e.get("map_x") is None:
                continue
            severity = TIER_SEVERITY.get(e["severity"], 0.3)
            ex, ey = e["map_x"], e["map_y"]
            nearby = store.find_nearby(ex, ey, radius_m=1.5)
            if nearby is not None:
                store.on_event(nearby.id, severity)
                merged += 1
                continue
            kf_id, kf = min(keyframes.items(),
                            key=lambda kv: math.hypot(kv[1][0] - ex,
                                                      kv[1][1] - ey))
            kx, ky, kyaw = kf
            cos_y, sin_y = math.cos(-kyaw), math.sin(-kyaw)
            rx, ry = ex - kx, ey - ky
            store.create_anchor(
                kind="map", severity=severity, keyframe_id=kf_id,
                dx=rx * cos_y - ry * sin_y, dy=rx * sin_y + ry * cos_y,
                dyaw=0.0, map_x=ex, map_y=ey, map_yaw=0.0,
                route_id=e.get("route_id", ""),
                layout_id=e.get("layout_id", ""))
            created += 1
    store.close()
    return {"created": created, "merged": merged}


def build_prototypes(dataset_dir, out_path, per_event_k=3, pca_dim=32,
                     cfg=None):
    cfg = cfg or BevConfig()
    samples = sorted(Path(dataset_dir).glob("samples/*.npz"))
    if not samples:
        raise RuntimeError(f"no samples under {dataset_dir}")

    all_feats = []                # 全部 patch（白化拟合用，含负样本）
    event_patches = {}            # event_key -> [(feat, severity), ...]
    tier_w = {"gold": 1.0, "silver": 0.6, "bronze": 0.3}

    for npz_path in samples:
        data = np.load(npz_path, allow_pickle=False)
        meta = json.loads(str(data["meta"]))
        bev = data["bev"].astype(np.float32)
        patches, centers = split_patches(bev, cfg)
        feats = handcrafted_features(patches, cfg.channels)
        all_feats.append(feats)

        if meta["kind"] != "pos":
            continue
        label = data["label"]
        half_patch = int(round(cfg.patch_size_m / cfg.resolution)) // 2
        n = cfg.grid_size
        half_m = cfg.size_m / 2.0
        key = f"{meta['day']}_{meta['event_type']}_{meta['route_id']}_" \
              f"{round(meta['stamp'])}"
        for feat, (cx, cy) in zip(feats, centers):
            ri = int((cx + half_m) / cfg.resolution)
            ci = int((cy + half_m) / cfg.resolution)
            r0, r1 = max(ri - half_patch, 0), min(ri + half_patch, n)
            c0, c1 = max(ci - half_patch, 0), min(ci + half_patch, n)
            window = label[r0:r1, c0:c1]
            if (window == 1).mean() > 0.05:   # patch 与正 swath 重叠
                event_patches.setdefault(key, []).append(
                    (feat, tier_w.get(meta["tier"], 0.3)))

    feats_mat = np.concatenate(all_feats, axis=0)
    whitener = PcaWhitener(dim=pca_dim).fit(feats_mat)

    embeddings, severities = [], []
    for key, items in event_patches.items():
        feats = np.stack([f for f, _ in items])
        sevs = np.array([s for _, s in items])
        emb = whitener.transform(feats)
        # 帧间多样性：贪心最远点采样选 top-k 代表
        chosen = [0]
        while len(chosen) < min(per_event_k, len(emb)):
            d = 1.0 - emb @ emb[chosen].T          # 余弦距离
            next_idx = int(np.argmax(d.min(axis=1)))
            if next_idx in chosen:
                break
            chosen.append(next_idx)
        embeddings.append(emb[chosen])
        severities.append(sevs[chosen])

    if not embeddings:
        raise RuntimeError("没有与 swath 重叠的正 patch，检查标注/事件质量")
    embeddings = np.concatenate(embeddings, axis=0).astype(np.float32)
    severities = np.concatenate(severities, axis=0).astype(np.float32)

    out = Path(out_path)
    out.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        out, embeddings=embeddings, severities=severities,
        pca_json=json.dumps(whitener.to_dict()),
        bev_fingerprint=cfg.fingerprint())
    return {"prototypes": len(embeddings), "events": len(event_patches),
            "fingerprint": cfg.fingerprint()}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    sub = ap.add_subparsers(dest="cmd", required=True)

    m = sub.add_parser("map", help="事件 -> anchors.db 地图锚")
    m.add_argument("--bag", required=True)
    m.add_argument("--events", required=True)
    m.add_argument("--db", required=True)
    m.add_argument("--scene-id", required=True)
    m.add_argument("--session-id", required=True)

    p = sub.add_parser("prototypes", help="数据集 -> prototypes.npz 特征锚库")
    p.add_argument("--dataset", required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--per-event-k", type=int, default=3)
    p.add_argument("--pca-dim", type=int, default=32)

    args = ap.parse_args()
    if args.cmd == "map":
        print(build_map_anchors(args.bag, args.events, args.db,
                                args.scene_id, args.session_id))
    else:
        print(build_prototypes(args.dataset, args.out,
                               args.per_event_k, args.pca_dim))


if __name__ == "__main__":
    main()
