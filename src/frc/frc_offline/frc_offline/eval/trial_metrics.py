"""E4/E5 趟级指标：从 trials yaml + 事件 jsonl +（可选）bag 自动计算。

指标定义在正式采集前冻结（预注册），主指标 = 接管+recovery+near-collision
合计事件率。统计：成对 Wilcoxon signed-rank（scipy 可用时）+ bootstrap 95% CI。

用法：
  python -m frc_offline.eval.trial_metrics --trials runtime-data/frc/trials/<date> \
      --events runtime-data/frc/events/<session>.jsonl [--bag <rosbag2>] \
      [--out metrics.csv]
"""

import argparse
import csv
import json
import time
from pathlib import Path

import numpy as np
import yaml

PRIMARY_EVENT_TYPES = ("takeover", "recovery", "near_collision")


def _iso_to_epoch(s: str) -> float:
    return time.mktime(time.strptime(s, "%Y-%m-%dT%H:%M:%S"))


def load_events(events_path):
    events = []
    p = Path(events_path)
    if p.exists():
        with open(p) as f:
            events = [json.loads(line) for line in f if line.strip()]
    return events


def trial_window_events(events, t0, t1):
    return [e for e in events if t0 <= e["stamp"] <= t1]


def compute_trial(trial, events, bag_odom=None, bag_cmd=None):
    t0 = _iso_to_epoch(trial["start_time"])
    t1 = _iso_to_epoch(trial["end_time"])
    evs = trial_window_events(events, t0, t1)
    counts = {t: sum(1 for e in evs if e["type"] == t)
              for t in ("takeover", "recovery", "near_collision",
                        "stuck", "jerk", "plan_instability")}
    primary = sum(counts[t] for t in PRIMARY_EVENT_TYPES)
    row = {
        "trial_id": trial["trial_id"],
        "route_id": trial.get("route_id", ""),
        "condition": trial.get("condition", ""),
        "result": trial.get("result", ""),
        # 成功 = 到达且无接管（设计文档 §3.3 E4 定义）
        "success": int(trial.get("result") == "SUCCEEDED"
                       and counts["takeover"] == 0),
        "duration_s": trial.get("duration_s", 0.0),
        "primary_events": primary,
        **counts,
        "path_length_m": "", "jerk_rms": "",
    }
    if bag_odom is not None and len(bag_odom):
        seg = bag_odom[(bag_odom[:, 0] >= t0) & (bag_odom[:, 0] <= t1)]
        if len(seg) > 1:
            row["path_length_m"] = round(float(np.sum(np.hypot(
                np.diff(seg[:, 1]), np.diff(seg[:, 2])))), 2)
    if bag_cmd is not None and len(bag_cmd) > 3:
        seg = bag_cmd[(bag_cmd[:, 0] >= t0) & (bag_cmd[:, 0] <= t1)]
        if len(seg) > 3:
            dt = np.maximum(np.diff(seg[:, 0]), 1e-3)
            accel = np.diff(seg[:, 1]) / dt
            jerk = np.diff(accel) / dt[1:]
            row["jerk_rms"] = round(float(np.sqrt(np.mean(jerk ** 2))), 3)
    return row


def ab_summary(rows, metric="primary_events"):
    """A/B 成对比较：按 schedule 配对，bootstrap CI + Wilcoxon（可选）。"""
    a = [r for r in rows if r["condition"] == "A"]
    b = [r for r in rows if r["condition"].startswith("B")]
    n = min(len(a), len(b))
    if n == 0:
        return {}
    da = np.array([float(r[metric]) for r in a[:n]])
    db = np.array([float(r[metric]) for r in b[:n]])
    diff = db - da
    rng = np.random.default_rng(0)
    boots = [float(np.mean(rng.choice(diff, size=n, replace=True)))
             for _ in range(2000)]
    summary = {
        "metric": metric, "n_pairs": n,
        "A_mean": round(float(da.mean()), 3),
        "B_mean": round(float(db.mean()), 3),
        "diff_mean": round(float(diff.mean()), 3),
        "diff_ci95": [round(float(np.percentile(boots, 2.5)), 3),
                      round(float(np.percentile(boots, 97.5)), 3)],
    }
    try:
        from scipy.stats import wilcoxon
        if np.any(diff != 0):
            stat, p = wilcoxon(da, db)
            summary["wilcoxon_p"] = round(float(p), 4)
    except ImportError:
        summary["wilcoxon_p"] = "scipy unavailable"
    return summary


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--trials", required=True, help="trials yaml 目录")
    ap.add_argument("--events", required=True)
    ap.add_argument("--bag", default="")
    ap.add_argument("--out", default="")
    args = ap.parse_args()

    trials = sorted(Path(args.trials).glob("*.yaml"))
    events = load_events(args.events)
    bag_odom = bag_cmd = None
    if args.bag:
        from frc_offline.bagio import BagReader
        with BagReader(args.bag) as reader:
            bag_odom = reader.read_odometry("/fastlio2/lio_odom")
            bag_cmd = reader.read_twist("/cmd_vel")

    rows = []
    for t in trials:
        trial = yaml.safe_load(t.read_text())
        rows.append(compute_trial(trial, events, bag_odom, bag_cmd))

    out = Path(args.out) if args.out else Path(args.trials) / "metrics.csv"
    if rows:
        with open(out, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)
    print(f"{len(rows)} trials -> {out}")
    for metric in ("primary_events", "success", "duration_s"):
        s = ab_summary(rows, metric)
        if s:
            print(json.dumps(s, ensure_ascii=False))


if __name__ == "__main__":
    main()
