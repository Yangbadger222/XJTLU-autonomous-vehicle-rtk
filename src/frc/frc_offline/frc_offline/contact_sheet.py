"""contact_sheet：每个事件生成快照拼图 PNG + review.csv 模板（人工复核 <=5 min/天）。

拼图内容：BEV max_z / density / costmap 三通道 + lookback 轨迹叠加。
复核界面就是 PNG + CSV 勾选表，不做 GUI（设计文档 §7.5）。

用法：
  python -m frc_offline.contact_sheet --bag <rosbag2> --events events.jsonl \
      --out review/
"""

import argparse
import json
from pathlib import Path

import numpy as np

from frc_bev import BevBuilder, BevConfig
from frc_bev.builder import SE2
from frc_bev.labels import LabelConfig
from frc_bev.ros_adapter import pointcloud2_to_xyz
from frc_offline.bag_to_bev_dataset import to_local
from frc_offline.bagio import BagReader, nearest_row


def render_event_sheets(bag_path, events_path, out_dir, cfg=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    cfg = cfg or BevConfig()
    label_cfg = LabelConfig()
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)

    with open(events_path) as f:
        events = [json.loads(line) for line in f if line.strip()]
    events.sort(key=lambda e: e["stamp"])

    with BagReader(bag_path) as reader:
        odom = reader.read_odometry("/fastlio2/lio_odom")
        builder = BevBuilder(cfg)
        ev_idx = 0
        health = np.zeros(len(cfg.health_keys), dtype=np.float32)

        for _, t, msg in reader.messages(["/fastlio2/body_cloud"]):
            row = nearest_row(odom, t, max_dt=0.5)
            if row is None:
                continue
            xyz = pointcloud2_to_xyz(msg)
            if len(xyz):
                cos_y, sin_y = np.cos(row[3]), np.sin(row[3])
                pts = np.empty_like(xyz)
                pts[:, 0] = row[1] + xyz[:, 0] * cos_y - xyz[:, 1] * sin_y
                pts[:, 1] = row[2] + xyz[:, 0] * sin_y + xyz[:, 1] * cos_y
                pts[:, 2] = xyz[:, 2]
                builder.push_cloud(pts, t)

            while ev_idx < len(events) and events[ev_idx]["stamp"] <= t:
                e = events[ev_idx]
                erow = nearest_row(odom, e["stamp"], max_dt=1.0)
                if erow is not None:
                    pose = SE2(erow[1], erow[2], erow[3])
                    frame = builder.build(pose, None, health, e["stamp"])
                    seg = odom[(odom[:, 0] >= e["stamp"] - label_cfg.lookback_s)
                               & (odom[:, 0] <= e["stamp"])]
                    traj = to_local(seg[:, 1:3], pose) if len(seg) \
                        else np.zeros((0, 2))
                    _plot(plt, frame, traj, e, ev_idx, cfg,
                          out / f"event_{ev_idx:03d}_{e['type']}.png")
                ev_idx += 1

    # review.csv 模板
    review = out / "review.csv"
    if not review.exists():
        lines = ["event_index,keep,comment,stamp,type,severity"]
        lines += [f"{i},y,,,," for i in range(len(events))]
        review.write_text("\n".join(lines) + "\n")
    return len(events)


def _plot(plt, frame, traj_local, event, idx, cfg, path):
    half = cfg.size_m / 2.0
    names = ["max_z", "density", "costmap"]
    fig, axes = plt.subplots(1, 3, figsize=(12, 4.2))
    for ax, name in zip(axes, names):
        img = np.asarray(frame.tensor[cfg.channels.index(name)],
                         dtype=np.float32)
        # 行=x 前、列=y 左 -> 显示为 x 向上、y 向左
        ax.imshow(img, origin="lower", extent=[-half, half, -half, half],
                  cmap="viridis")
        if len(traj_local):
            ax.plot(traj_local[:, 1], traj_local[:, 0], "r.-", lw=1, ms=2)
        ax.plot(0, 0, "w^", ms=8)
        ax.set_title(name)
        ax.set_xlabel("y left (m)")
    axes[0].set_ylabel("x fwd (m)")
    fig.suptitle(f"#{idx} {event['severity']}/{event['type']} "
                 f"t={event['stamp']:.1f} {event.get('note', '')}")
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--bag", required=True)
    ap.add_argument("--events", required=True)
    ap.add_argument("--out", default="review")
    args = ap.parse_args()
    n = render_event_sheets(args.bag, args.events, args.out)
    print(f"{n} event sheets -> {args.out}/ (review.csv 模板已生成)")


if __name__ == "__main__":
    main()
