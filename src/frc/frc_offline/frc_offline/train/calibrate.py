"""温度缩放校准 + 部署阈值反推（设计文档 §5.4）。

- 在 val split 上拟合温度 T（最小化 NLL）；
- 部署置信度阈值 tau 不拍脑袋：按"预期注入面积约束"（风险像素占比 < area_budget）
  在 val 上反推，或按 FPR@95%TPR。
结果写入 <ckpt>.calib.json，export_onnx 读取并内嵌温度。

用法：
  python -m frc_offline.train.calibrate --ckpt runs/frc-v2/tinyunet_seed0.pt \
      --dataset datasets/frc-v2-... [--area-budget 0.05]
"""

import argparse
import json
from pathlib import Path

import numpy as np

from frc_bev.labels import IGNORE
from frc_offline.train.dataset import FrcBevDataset, collate


def collect_logits(ckpt_path, dataset_dir, device="cpu"):
    import torch
    from torch.utils.data import DataLoader

    from frc_offline.train.model import TinyUNet

    ckpt = torch.load(ckpt_path, map_location=device)
    model = TinyUNet(in_channels=ckpt["in_channels"],
                     health_dim=ckpt["health_dim"],
                     use_health_film=ckpt["use_health_film"]).to(device)
    model.load_state_dict(ckpt["state_dict"])
    model.eval()

    val = FrcBevDataset(dataset_dir, "val", augment_enabled=False)
    loader = DataLoader(val, batch_size=16, collate_fn=collate)
    logits, labels = [], []
    with torch.no_grad():
        for batch in loader:
            out = model(batch["bev"].to(device), batch["health"].to(device))
            lab = batch["label"].numpy()
            valid = lab != IGNORE
            logits.append(out[:, 0].cpu().numpy()[valid])
            labels.append((lab[valid] == 1).astype(np.float64))
    return np.concatenate(logits), np.concatenate(labels), ckpt


def fit_temperature(logits, labels, t_grid=None):
    """网格搜索 NLL 最优温度（无 scipy 依赖，1D 问题网格即可）。"""
    t_grid = t_grid if t_grid is not None else np.geomspace(0.25, 8.0, 161)
    best_t, best_nll = 1.0, np.inf
    for t in t_grid:
        p = 1.0 / (1.0 + np.exp(-logits / t))
        p = np.clip(p, 1e-7, 1.0 - 1e-7)
        nll = float(-(labels * np.log(p)
                      + (1 - labels) * np.log(1 - p)).mean())
        if nll < best_nll:
            best_t, best_nll = float(t), nll
    return best_t, best_nll


def threshold_by_area(logits, temperature, area_budget):
    """注入面积约束：使 p>tau 的像素占比 = area_budget 的分位点。"""
    p = 1.0 / (1.0 + np.exp(-logits / temperature))
    return float(np.quantile(p, 1.0 - area_budget))


def threshold_fpr95(logits, labels, temperature):
    p = 1.0 / (1.0 + np.exp(-logits / temperature))
    pos = np.sort(p[labels > 0.5])
    if not len(pos):
        return 0.5
    tpr95_thresh = float(pos[int(0.05 * len(pos))])   # 95% 正例仍超过该阈值
    return tpr95_thresh


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--ckpt", required=True)
    ap.add_argument("--dataset", required=True)
    ap.add_argument("--area-budget", type=float, default=0.05)
    ap.add_argument("--device", default="cpu")
    args = ap.parse_args()

    logits, labels, ckpt = collect_logits(args.ckpt, args.dataset, args.device)
    t, nll = fit_temperature(logits, labels)
    result = {
        "temperature": t,
        "val_nll": nll,
        "tau_area_budget": threshold_by_area(logits, t, args.area_budget),
        "tau_fpr95": threshold_fpr95(logits, labels, t),
        "area_budget": args.area_budget,
        "bev_fingerprint": ckpt["bev_fingerprint"],
        "dataset_hash": ckpt["dataset_hash"],
    }
    out = Path(args.ckpt).with_suffix(".calib.json")
    out.write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))
    print(f"-> {out}")


if __name__ == "__main__":
    main()
