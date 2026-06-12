"""bag -> BEV 数据集导出（与在线共用 frc_bev，train/runtime 唯一真源）。

抽帧策略（设计文档 §5.1）：
- 正帧：每个事件 lookback 窗口内按 pos_hz 抽帧（同一风险区多视角，天然增广）；
- 负帧：无事件区段按 neg_hz 抽帧，swath 标 0 权重 1.0；
- hard negative：与正帧同位置（同路线同里程近似：距正帧位置 < pair_radius 米）
  的成功通过帧，权重 1.5（专治"到处加 cost"）；
- 归因过滤：LIO 退化且 swath 无几何异常的事件帧空间标签整体转 ignore
  （frc_bev.labels 内实现），过滤计数写入 manifest——E1 表格直接用。

用法：
  python -m frc_offline.bag_to_bev_dataset --bag <rosbag2> --events events.jsonl \
      --out datasets/frc-v0-2026wXX --scene-id S1 --route-id R1 --layout-id L0 \
      --day 2026-06-15
"""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml

from frc_bev import BevBuilder, BevConfig
from frc_bev.builder import SE2
from frc_bev.labels import IGNORE, LabelConfig, label_event, label_negative
from frc_bev.ros_adapter import pointcloud2_to_xyz
from frc_offline.bagio import BagReader, nearest_row

TIERS = {"gold", "silver", "bronze"}


def assemble_health(cfg: BevConfig, t, degeneracy, correction, fix_rows, odom):
    """与在线 health_aggregator 同字段顺序组装健康向量。"""
    vec = np.zeros(len(cfg.health_keys), dtype=np.float32)
    d = _nearest_list(degeneracy, t)
    if d is not None and len(d) >= 3:
        vec[0] = d[0]                                   # lio_min_eig
        vec[1] = 1.0 if (d[2] > 0.5 or d[0] < 75.0) else 0.0
    c = _nearest_list(correction, t)
    if c is not None and len(c) >= 1:
        vec[2] = 1.0 if c[0] > 0.5 else 0.0             # pgo_correcting
    f = nearest_row(fix_rows, t, max_dt=2.0) if len(fix_rows) else None
    vec[3] = f[1] if f is not None else -1.0            # rtk_status
    o = nearest_row(odom, t, max_dt=0.5)
    if o is not None:
        vec[4], vec[5] = o[4], o[5]                     # v, w
    return vec


def _nearest_list(seq, t, max_dt=0.5):
    if not seq:
        return None
    best = min(seq, key=lambda x: abs(x[0] - t))
    return best[1] if abs(best[0] - t) <= max_dt else None


def to_local(traj_xy, pose: SE2):
    """odom 系点集 -> 帧局部系（机器人居中、航向对齐）。"""
    cos_y, sin_y = np.cos(pose.yaw), np.sin(pose.yaw)
    dx = traj_xy[:, 0] - pose.x
    dy = traj_xy[:, 1] - pose.y
    return np.stack([dx * cos_y + dy * sin_y,
                     -dx * sin_y + dy * cos_y], axis=1)


def export_dataset(bag_path, events_path, out_dir, scene_id, route_id,
                   layout_id, day, pos_hz=1.0, neg_hz=0.5,
                   pair_radius_m=2.0, cfg=None, label_cfg=None):
    cfg = cfg or BevConfig()
    label_cfg = label_cfg or LabelConfig()
    out = Path(out_dir)
    (out / "samples").mkdir(parents=True, exist_ok=True)

    events = []
    if Path(events_path).exists():
        with open(events_path) as f:
            events = [json.loads(line) for line in f if line.strip()]
    events = [e for e in events if e["severity"] in TIERS]

    with BagReader(bag_path) as reader:
        odom = reader.read_odometry("/fastlio2/lio_odom")
        if not len(odom):
            raise RuntimeError("bag 缺少 /fastlio2/lio_odom，无法重建 BEV")
        map_odom = reader.read_odometry("/pgo/optimized_odom") \
            if "/pgo/optimized_odom" in reader.topics() else odom
        degeneracy = reader.read_float_array("/fastlio2/degeneracy") \
            if "/fastlio2/degeneracy" in reader.topics() else []
        correction = reader.read_float_array("/pgo/correction_status") \
            if "/pgo/correction_status" in reader.topics() else []
        fix_rows = _read_fix(reader)

        # ---- 抽样时间表 ----
        t0, t1 = odom[0, 0], odom[-1, 0]
        pos_samples = []        # (t, event)
        for e in events:
            te = e["stamp"]
            ts = np.arange(max(te - label_cfg.lookback_s, t0), te,
                           1.0 / pos_hz)
            pos_samples.extend((float(t), e) for t in ts)

        def near_event(t, margin):
            return any(abs(t - e["stamp"]) < margin for e in events)

        neg_samples = [float(t) for t in np.arange(t0 + 2.0, t1, 1.0 / neg_hz)
                       if not near_event(t, label_cfg.lookback_s + 2.0)]

        # hard negative：与正帧同位置的成功通过帧
        hard_flags = {}
        pos_xy = []
        for t, _e in pos_samples:
            row = nearest_row(odom, t, max_dt=0.5)
            if row is not None:
                pos_xy.append((row[1], row[2]))
        for t in neg_samples:
            row = nearest_row(odom, t, max_dt=0.5)
            if row is None:
                continue
            hard_flags[t] = any(
                np.hypot(row[1] - px, row[2] - py) < pair_radius_m
                for px, py in pos_xy)

        all_samples = sorted(
            [(t, "pos", e) for t, e in pos_samples] +
            [(t, "neg", None) for t in neg_samples])

        # ---- 单趟扫描：点云流 + 到点构建 ----
        builder = BevBuilder(cfg)
        costmap_buf = [None]
        stats = {"pos": 0, "neg": 0, "hard_neg": 0, "gated": 0}
        idx = 0
        written = []

        stream_topics = ["/fastlio2/body_cloud", "/local_costmap/costmap_raw"]
        for topic, t, msg in reader.messages(stream_topics):
            if topic == "/local_costmap/costmap_raw":
                h, w = msg.metadata.size_y, msg.metadata.size_x
                grid = np.asarray(msg.data, dtype=np.uint8) \
                    .reshape(h, w).astype(np.float32)
                costmap_buf[0] = (grid, {
                    "resolution": float(msg.metadata.resolution),
                    "origin_xy": (float(msg.metadata.origin.position.x),
                                  float(msg.metadata.origin.position.y))})
                continue

            row = nearest_row(odom, t, max_dt=0.5)
            if row is None:
                continue
            xyz = pointcloud2_to_xyz(msg)
            if len(xyz):
                cos_y, sin_y = np.cos(row[3]), np.sin(row[3])
                pts = np.empty_like(xyz)
                pts[:, 0] = row[1] + xyz[:, 0] * cos_y - xyz[:, 1] * sin_y
                pts[:, 1] = row[2] + xyz[:, 0] * sin_y + xyz[:, 1] * cos_y
                pts[:, 2] = xyz[:, 2]      # body 相对高度（F5 语义）
                builder.push_cloud(pts, t)

            while idx < len(all_samples) and all_samples[idx][0] <= t:
                st, kind, event = all_samples[idx]
                idx += 1
                srow = nearest_row(odom, st, max_dt=0.3)
                if srow is None:
                    continue
                pose = SE2(srow[1], srow[2], srow[3])
                health = assemble_health(cfg, st, degeneracy, correction,
                                         fix_rows, odom)
                cm = costmap_buf[0]
                frame = builder.build(pose, cm[0] if cm else None, health, st,
                                      cm[1] if cm else None)
                label, weight = _make_label(
                    kind, event, st, pose, odom, frame, cfg, label_cfg,
                    hard=hard_flags.get(st, False))
                if kind == "pos" and (label == IGNORE).all():
                    stats["gated"] += 1
                name = f"{Path(bag_path).name}_{int(st * 1e3)}.npz"
                mrow = nearest_row(map_odom, st, max_dt=1.0)
                np.savez_compressed(
                    out / "samples" / name,
                    bev=frame.tensor, label=label, weight=weight,
                    health=frame.health,
                    meta=json.dumps({
                        "kind": kind, "scene_id": scene_id,
                        "route_id": route_id, "layout_id": layout_id,
                        "day": day, "stamp": st,
                        "tier": event["severity"] if event else "",
                        "event_type": event["type"] if event else "",
                        "hard_negative": bool(hard_flags.get(st, False)),
                        "pose_odom": [pose.x, pose.y, pose.yaw],
                        "pose_map": [float(mrow[1]), float(mrow[2]),
                                     float(mrow[3])] if mrow is not None
                        else None,
                    }))
                written.append(name)
                if kind == "pos":
                    stats["pos"] += 1
                elif hard_flags.get(st, False):
                    stats["hard_neg"] += 1
                else:
                    stats["neg"] += 1

    _update_manifest(out, cfg, bag_path, scene_id, route_id, layout_id, day,
                     stats, written, len(events))
    return stats


def _read_fix(reader):
    if "/fix" not in reader.topics():
        return np.zeros((0, 2))
    rows = [(t, float(m.status.status))
            for _, t, m in reader.messages(["/fix"])]
    return np.asarray(rows) if rows else np.zeros((0, 2))


def _make_label(kind, event, st, pose, odom, frame, cfg, label_cfg, hard):
    if kind == "pos":
        te = event["stamp"]
        seg = odom[(odom[:, 0] >= st) & (odom[:, 0] <= te)]
        traj_local = to_local(seg[:, 1:3], pose) if len(seg) \
            else np.zeros((0, 2))
        erow = nearest_row(odom, te, max_dt=1.0)
        event_xy = to_local(np.array([[erow[1], erow[2]]]), pose)[0] \
            if erow is not None else (0.0, 0.0)
        lio_min_eig = float(frame.health[0]) if frame.health[0] > 0 else 1e6
        return label_event(tuple(event_xy), traj_local, event["severity"],
                           frame.tensor.astype(np.float32),
                           cfg.channels, lio_min_eig, cfg, label_cfg)
    seg = odom[(odom[:, 0] >= st) & (odom[:, 0] <= st + label_cfg.lookback_s)]
    traj_local = to_local(seg[:, 1:3], pose) if len(seg) else np.zeros((0, 2))
    return label_negative(traj_local, cfg, label_cfg, hard=hard)


def _update_manifest(out, cfg, bag_path, scene_id, route_id, layout_id, day,
                     stats, written, n_events):
    manifest_path = out / "manifest.yaml"
    manifest = yaml.safe_load(manifest_path.read_text()) \
        if manifest_path.exists() else {
            "bev_fingerprint": cfg.fingerprint(), "sources": [],
            "counts": {"pos": 0, "neg": 0, "hard_neg": 0, "gated": 0}}
    if manifest["bev_fingerprint"] != cfg.fingerprint():
        raise RuntimeError("manifest 指纹与当前 BevConfig 不一致，禁止混入")
    manifest["sources"].append({
        "bag": str(bag_path), "scene_id": scene_id, "route_id": route_id,
        "layout_id": layout_id, "day": day, "events": n_events,
        "samples": len(written), **stats})
    for k, v in stats.items():
        manifest["counts"][k] = manifest["counts"].get(k, 0) + v
    sample_hash = hashlib.sha1(
        "".join(sorted(written)).encode()).hexdigest()[:12]
    manifest["last_batch_hash"] = sample_hash
    manifest_path.write_text(yaml.safe_dump(manifest, sort_keys=False,
                                            allow_unicode=True))


def make_splits(dataset_dir, holdout_scene="S4", val_day=""):
    """group split：(scene, route, day) 整组划分，杜绝相邻帧跨集泄漏。"""
    out = Path(dataset_dir)
    splits = {"train": [], "val": [], "test_s4": [], "test_layout": []}
    for npz in sorted((out / "samples").glob("*.npz")):
        meta = json.loads(str(np.load(npz, allow_pickle=False)["meta"]))
        if meta["scene_id"] == holdout_scene:
            splits["test_s4"].append(npz.name)
        elif meta["layout_id"].endswith("_alt"):
            splits["test_layout"].append(npz.name)
        elif val_day and meta["day"] == val_day:
            splits["val"].append(npz.name)
        else:
            splits["train"].append(npz.name)
    (out / "splits").mkdir(exist_ok=True)
    for name, items in splits.items():
        (out / "splits" / f"{name}.txt").write_text("\n".join(items) + "\n")
    return {k: len(v) for k, v in splits.items()}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--bag", required=True)
    ap.add_argument("--events", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--scene-id", required=True)
    ap.add_argument("--route-id", default="")
    ap.add_argument("--layout-id", default="L0")
    ap.add_argument("--day", required=True)
    ap.add_argument("--pos-hz", type=float, default=1.0)
    ap.add_argument("--neg-hz", type=float, default=0.5)
    ap.add_argument("--make-splits", action="store_true")
    ap.add_argument("--val-day", default="")
    args = ap.parse_args()

    stats = export_dataset(args.bag, args.events, args.out, args.scene_id,
                           args.route_id, args.layout_id, args.day,
                           args.pos_hz, args.neg_hz)
    print(f"exported: {stats}")
    if args.make_splits:
        print(f"splits: {make_splits(args.out, val_day=args.val_day)}")


if __name__ == "__main__":
    main()
