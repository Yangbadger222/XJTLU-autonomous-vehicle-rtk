"""failure_miner：从 bag + 事件 jsonl 离线挖掘失败/近失败事件（E1 数据治理）。

与在线 event_marker_node 同语义但更完整（离线可双向扫描、阈值可重扫）。
阈值全部来自 config/miner_thresholds.yaml；输出 events.jsonl（含来源标注），
供 contact_sheet 人工复核与 auto_label_from_events 消费。

用法：
  python -m frc_offline.failure_miner --bag <rosbag2目录> \
      [--thresholds config/miner_thresholds.yaml] [--out events.jsonl] \
      [--route-id R1] [--layout-id L0]
"""

import argparse
import json
from pathlib import Path

import numpy as np
import yaml

from frc_offline.bagio import BagReader, nearest_row


def _emit(events, t, etype, severity, note, pose_map):
    events.append({
        "stamp": float(t), "type": etype, "severity": severity,
        "note": note, "source": "miner",
        "map_x": float(pose_map[0]) if pose_map is not None else None,
        "map_y": float(pose_map[1]) if pose_map is not None else None,
    })


def _pose_at(map_odom: np.ndarray, t: float):
    row = nearest_row(map_odom, t, max_dt=1.0)
    return (row[1], row[2]) if row is not None else None


def mine_takeover(reader, cfg, events, pose_lut):
    last = None
    for _, t, m in reader.messages(["/chassis/status"]):
        mode = int(m.ctrl_mode)
        if last == 0 and mode in (1, 2):
            _emit(events, t, "takeover", "gold", f"ctrl 0->{mode}",
                  _pose_at(pose_lut, t))
        key = int(m.ps2_key)
        if key == 1 and getattr(mine_takeover, "_lastkey", 0) != 1:
            _emit(events, t, "manual", "gold", "SELECT", _pose_at(pose_lut, t))
        mine_takeover._lastkey = key
        last = mode


def mine_manual_markers(reader, events, pose_lut):
    """在线 event_marker / frc_mark.py 已发布的事件直接并入（保留 gold 语义）。"""
    for _, t, m in reader.messages(["/frc/event_marker"]):
        events.append({
            "stamp": float(t), "type": str(m.type), "severity": str(m.severity),
            "note": str(m.note), "source": "online",
            "map_x": float(m.pose.position.x), "map_y": float(m.pose.position.y),
        })


def mine_recovery(reader, cfg, events, pose_lut):
    nodes = set(cfg["recovery"]["nodes"])
    for _, t, m in reader.messages(["/behavior_tree_log"]):
        for ev in m.event_log:
            if ev.node_name in nodes and ev.current_status == "RUNNING":
                _emit(events, t, "recovery", "silver", ev.node_name,
                      _pose_at(pose_lut, t))
                break


def mine_stuck(reader, cfg, events, pose_lut, chassis_odom, cmd):
    c = cfg["stuck"]
    if not len(chassis_odom) or not len(cmd):
        return
    since = None
    for t, v, _w in chassis_odom[:, [0, 4, 5]]:
        cmd_row = nearest_row(cmd, t, max_dt=0.5)
        cmd_v = abs(cmd_row[1]) if cmd_row is not None else 0.0
        if v < c["odom_speed_max"] and cmd_v > c["cmd_speed_min"]:
            if since is None:
                since = t
            elif t - since >= c["duration_s"]:
                _emit(events, t, "stuck", "silver",
                      f"v={v:.3f} cmd={cmd_v:.2f}", _pose_at(pose_lut, t))
                since = None
        else:
            since = None


def mine_near_collision(reader, cfg, events, pose_lut, lio_odom):
    c = cfg["near_collision"]
    since = None
    for _, t, m in reader.messages(["/local_costmap/costmap_raw"]):
        row = nearest_row(lio_odom, t, max_dt=0.5)
        if row is None:
            continue
        h, w = m.metadata.size_y, m.metadata.size_x
        grid = np.asarray(m.data, dtype=np.uint8).reshape(h, w)
        lethal = np.argwhere(grid >= 253)
        if not len(lethal):
            since = None
            continue
        res = float(m.metadata.resolution)
        ox = float(m.metadata.origin.position.x)
        oy = float(m.metadata.origin.position.y)
        cell_xy = np.stack([lethal[:, 1] * res + ox,
                            lethal[:, 0] * res + oy], axis=1)
        clearance = float(np.min(np.hypot(cell_xy[:, 0] - row[1],
                                          cell_xy[:, 1] - row[2])))
        if clearance < c["clearance_m"]:
            if since is None:
                since = t
            elif t - since >= c["duration_s"]:
                _emit(events, t, "near_collision", "silver",
                      f"clearance={clearance:.2f}", _pose_at(pose_lut, t))
                since = None
        else:
            since = None


def mine_jerk(cfg, events, pose_lut, cmd):
    c = cfg["jerk"]
    if len(cmd) < 3:
        return
    t = cmd[:, 0]
    dt = np.diff(t)
    ok = dt > 1e-3
    accel = np.diff(cmd[:, 1]) / np.maximum(dt, 1e-3)
    jerk = np.abs(np.diff(accel) / np.maximum(dt[1:], 1e-3))
    jt = t[2:]
    valid = ok[:-1] & ok[1:]
    jerk, jt = jerk[valid], jt[valid]
    n = len(jerk)
    if n <= c["min_samples"]:
        return
    # 滚动基线（前缀统计），避免事件自身抬高阈值
    mean = np.cumsum(jerk) / np.arange(1, n + 1)
    sq = np.cumsum(jerk ** 2) / np.arange(1, n + 1)
    std = np.sqrt(np.maximum(sq - mean ** 2, 0.0))
    for i in range(c["min_samples"], n):
        if std[i - 1] > 1e-9 and \
                jerk[i] > mean[i - 1] + c["sigma_k"] * std[i - 1]:
            _emit(events, jt[i], "jerk", "bronze", f"jerk={jerk[i]:.1f}",
                  _pose_at(pose_lut, jt[i]))


def mine_plan_instability(reader, cfg, events, pose_lut, lio_odom):
    c = cfg["plan_instability"]
    last_plan, devs, hist = None, [], []
    for _, t, m in reader.messages(["/plan"]):
        if len(m.poses) < 2:
            continue
        pts = np.array([[p.pose.position.x, p.pose.position.y]
                        for p in m.poses])
        row = nearest_row(lio_odom, t, max_dt=0.5)
        if row is None:
            continue
        d_along = np.hypot(pts[:, 0] - row[1], pts[:, 1] - row[2])
        ahead = pts[d_along <= c["lookahead_m"]]
        if last_plan is not None and len(ahead) >= 2 and len(last_plan) >= 2:
            d = np.hypot(ahead[:, None, 0] - last_plan[None, :, 0],
                         ahead[:, None, 1] - last_plan[None, :, 1])
            devs.append(float(d.min(axis=1).mean()))
            if len(devs) > c["window"]:
                devs.pop(0)
            rms = float(np.sqrt(np.mean(np.square(devs))))
            if len(hist) >= c["min_samples"]:
                mean, std = float(np.mean(hist)), float(np.std(hist))
                if std > 1e-9 and rms > mean + c["sigma_k"] * std:
                    _emit(events, t, "plan_instability", "bronze",
                          f"rms={rms:.2f}", _pose_at(pose_lut, t))
            hist.append(rms)
        last_plan = ahead if len(ahead) >= 2 else pts


def dedup(events, merge_window_s):
    """同类型事件在窗口内合并（保留最高层级；gold 优先）。"""
    order = {"gold": 0, "silver": 1, "bronze": 2}
    events.sort(key=lambda e: e["stamp"])
    out = []
    for e in events:
        if out and out[-1]["type"] == e["type"] and \
                e["stamp"] - out[-1]["stamp"] < merge_window_s:
            if order.get(e["severity"], 9) < order.get(out[-1]["severity"], 9):
                out[-1] = e
            continue
        out.append(e)
    return out


def mine_bag(bag_path, thresholds, route_id="", layout_id=""):
    cfg = thresholds
    events = []
    with BagReader(bag_path) as reader:
        topics = reader.topics()
        lio_odom = reader.read_odometry("/fastlio2/lio_odom") \
            if "/fastlio2/lio_odom" in topics else np.zeros((0, 6))
        map_odom = reader.read_odometry("/pgo/optimized_odom") \
            if "/pgo/optimized_odom" in topics else lio_odom
        chassis_odom = reader.read_odometry("/odom_CBoar") \
            if "/odom_CBoar" in topics else np.zeros((0, 6))
        cmd = reader.read_twist("/cmd_vel") \
            if "/cmd_vel" in topics else np.zeros((0, 3))

        mine_manual_markers(reader, events, map_odom)
        if cfg["takeover"]["enabled"] and "/chassis/status" in topics:
            mine_takeover(reader, cfg, events, map_odom)
        if cfg["recovery"]["enabled"] and "/behavior_tree_log" in topics:
            mine_recovery(reader, cfg, events, map_odom)
        if cfg["stuck"]["enabled"]:
            mine_stuck(reader, cfg, events, map_odom, chassis_odom, cmd)
        if cfg["near_collision"]["enabled"] and \
                "/local_costmap/costmap_raw" in topics:
            mine_near_collision(reader, cfg, events, map_odom, lio_odom)
        if cfg["jerk"]["enabled"]:
            mine_jerk(cfg, events, map_odom, cmd)
        if cfg["plan_instability"]["enabled"] and "/plan" in topics:
            mine_plan_instability(reader, cfg, events, map_odom, lio_odom)

    events = dedup(events, cfg.get("merge_window_s", 5.0))
    for e in events:
        e["route_id"] = route_id
        e["layout_id"] = layout_id
        e["bag"] = str(bag_path)
    return events


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--bag", required=True)
    ap.add_argument("--thresholds",
                    default=str(Path(__file__).resolve().parents[1]
                                / "config" / "miner_thresholds.yaml"))
    ap.add_argument("--out", default="")
    ap.add_argument("--route-id", default="")
    ap.add_argument("--layout-id", default="")
    args = ap.parse_args()

    thresholds = yaml.safe_load(Path(args.thresholds).read_text())
    events = mine_bag(args.bag, thresholds, args.route_id, args.layout_id)

    out = Path(args.out) if args.out else Path(args.bag) / "events.jsonl"
    with open(out, "w") as f:
        for e in events:
            f.write(json.dumps(e) + "\n")

    by_tier = {}
    for e in events:
        by_tier[e["severity"]] = by_tier.get(e["severity"], 0) + 1
    print(f"mined {len(events)} events -> {out}")
    print(f"  by tier: {by_tier}")


if __name__ == "__main__":
    main()
