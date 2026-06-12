"""风险图叠加可视化（论文图 + shadow 期人工核查）。

用法：
  python -m frc_offline.eval.risk_overlay_viz --dataset datasets/... \
      --split val --out viz/ [--prototypes ...] [--anchors-db ... --scene-id S1]
"""

import argparse
import json
from pathlib import Path

import numpy as np

from frc_bev.config import BevConfig
from frc_bev.labels import IGNORE
from frc_offline.eval.offline_auroc import (CostmapScorer, DualScorer,
                                            FeatureScorer, MapAnchorScorer)


def render(dataset_dir, split, out_dir, scorers, cfg=None, limit=50):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    cfg = cfg or BevConfig()
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    half = cfg.size_m / 2.0
    extent = [-half, half, -half, half]

    names = (Path(dataset_dir) / "splits" / f"{split}.txt") \
        .read_text().splitlines()
    count = 0
    for name in names:
        if not name.strip() or count >= limit:
            break
        data = np.load(Path(dataset_dir) / "samples" / name,
                       allow_pickle=False)
        bev = data["bev"].astype(np.float32)
        label = data["label"]
        meta = json.loads(str(data["meta"]))
        if meta["kind"] != "pos":
            continue
        count += 1

        ncols = 2 + len(scorers)
        fig, axes = plt.subplots(1, ncols, figsize=(3.6 * ncols, 3.8))
        axes[0].imshow(bev[cfg.channels.index("max_z")], origin="lower",
                       extent=extent, cmap="viridis")
        axes[0].set_title("max_z")
        lab_img = np.ma.masked_where(label == IGNORE, label)
        axes[1].imshow(lab_img, origin="lower", extent=extent,
                       cmap="coolwarm", vmin=0, vmax=1)
        axes[1].set_title("label (swath)")
        for ax, s in zip(axes[2:], scorers):
            risk = s.score(bev, data["health"], meta)
            ax.imshow(risk, origin="lower", extent=extent, cmap="hot",
                      vmin=0, vmax=1)
            ax.set_title(f"risk: {s.name}")
        for ax in axes:
            ax.plot(0, 0, "w^", ms=7)
        fig.suptitle(f"{name} {meta['tier']}/{meta['event_type']}")
        fig.tight_layout()
        fig.savefig(out / f"{Path(name).stem}.png", dpi=100)
        plt.close(fig)
    return count


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--dataset", required=True)
    ap.add_argument("--split", default="val")
    ap.add_argument("--out", default="viz")
    ap.add_argument("--anchors-db", default="")
    ap.add_argument("--scene-id", default="")
    ap.add_argument("--prototypes", default="")
    ap.add_argument("--limit", type=int, default=50)
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

    n = render(args.dataset, args.split, args.out, scorers, cfg, args.limit)
    print(f"{n} overlays -> {args.out}/")


if __name__ == "__main__":
    main()
