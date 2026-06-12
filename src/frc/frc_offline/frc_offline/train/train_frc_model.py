"""TinyUNet 训练（focal BCE x weight，ignore 不回传；早停 val AP）。

用法：
  python -m frc_offline.train.train_frc_model --dataset datasets/frc-v2-... \
      --out runs/frc-v2 --seeds 5 [--no-health-film]
checkpoint 内嵌 bev_fingerprint + dataset_hash，部署侧三方校验。
"""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from frc_bev.config import BevConfig
from frc_bev.labels import IGNORE
from frc_offline.eval.offline_auroc import average_precision
from frc_offline.train.dataset import FrcBevDataset, collate


def focal_bce(logits, label, weight, gamma=2.0, alpha=0.75):
    import torch
    import torch.nn.functional as F
    valid = (label != IGNORE)
    if valid.sum() == 0:
        return logits.sum() * 0.0
    target = (label == 1).float()
    bce = F.binary_cross_entropy_with_logits(logits, target, reduction="none")
    p = torch.sigmoid(logits)
    pt = torch.where(target > 0.5, p, 1.0 - p)
    a = torch.where(target > 0.5, torch.full_like(p, alpha),
                    torch.full_like(p, 1.0 - alpha))
    loss = a * (1.0 - pt) ** gamma * bce * weight
    return loss[valid].mean()


def conf_loss(out, label):
    """confidence 头：回归 1-|p-y|（自估正确度）。"""
    import torch
    import torch.nn.functional as F
    valid = (label != IGNORE)
    if valid.sum() == 0:
        return out.sum() * 0.0
    with torch.no_grad():
        p = torch.sigmoid(out[:, 0])
        target = 1.0 - (p - (label == 1).float()).abs()
    return F.mse_loss(torch.sigmoid(out[:, 1])[valid], target[valid])


def evaluate_ap(model, loader, device):
    import torch
    model.eval()
    scores, labels = [], []
    with torch.no_grad():
        for batch in loader:
            out = model(batch["bev"].to(device), batch["health"].to(device))
            p = torch.sigmoid(out[:, 0]).cpu().numpy()
            lab = batch["label"].numpy()
            valid = lab != IGNORE
            scores.append(p[valid])
            labels.append((lab[valid] == 1).astype(np.int8))
    if not scores:
        return 0.0
    return average_precision(np.concatenate(labels), np.concatenate(scores))


def dataset_hash(dataset_dir) -> str:
    names = sorted(p.name for p in
                   (Path(dataset_dir) / "samples").glob("*.npz"))
    return hashlib.sha1("".join(names).encode()).hexdigest()[:12]


def train_one(dataset_dir, out_dir, seed, epochs, batch_size, lr,
              use_health_film, device):
    import torch
    from torch.utils.data import DataLoader

    from frc_offline.train.model import TinyUNet, count_params

    torch.manual_seed(seed)
    np.random.seed(seed)
    cfg = BevConfig()

    train_ds = FrcBevDataset(dataset_dir, "train", seed=seed)
    val_ds = FrcBevDataset(dataset_dir, "val", augment_enabled=False)
    train_loader = DataLoader(train_ds, batch_size=batch_size, shuffle=True,
                              collate_fn=collate, num_workers=2)
    val_loader = DataLoader(val_ds, batch_size=batch_size,
                            collate_fn=collate, num_workers=2)

    model = TinyUNet(in_channels=cfg.num_channels,
                     health_dim=len(cfg.health_keys),
                     use_health_film=use_health_film).to(device)
    print(f"seed {seed}: {count_params(model) / 1e6:.2f}M params, "
          f"train={len(train_ds)} val={len(val_ds)} film={use_health_film}")

    opt = torch.optim.AdamW(model.parameters(), lr=lr, weight_decay=1e-4)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=epochs)

    best_ap, best_state, patience = -1.0, None, 0
    for epoch in range(epochs):
        model.train()
        total = 0.0
        for batch in train_loader:
            out = model(batch["bev"].to(device), batch["health"].to(device))
            label = batch["label"].to(device)
            weight = batch["weight"].to(device)
            loss = focal_bce(out[:, 0], label, weight) + \
                0.2 * conf_loss(out, label)
            opt.zero_grad()
            loss.backward()
            opt.step()
            total += float(loss)
        sched.step()

        ap = evaluate_ap(model, val_loader, device)
        print(f"  epoch {epoch:02d} loss={total / max(len(train_loader), 1):.4f} "
              f"val_AP={ap:.4f}")
        if ap > best_ap:
            best_ap, patience = ap, 0
            best_state = {k: v.cpu().clone()
                          for k, v in model.state_dict().items()}
        else:
            patience += 1
            if patience >= 10:
                print("  early stop")
                break

    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    ckpt_path = out / f"tinyunet_seed{seed}.pt"
    torch.save({
        "state_dict": best_state,
        "val_ap": best_ap,
        "bev_fingerprint": cfg.fingerprint(),
        "dataset_hash": dataset_hash(dataset_dir),
        "in_channels": cfg.num_channels,
        "health_dim": len(cfg.health_keys),
        "grid": cfg.grid_size,
        "use_health_film": use_health_film,
        "seed": seed,
    }, ckpt_path)
    print(f"seed {seed}: best val_AP={best_ap:.4f} -> {ckpt_path}")
    return best_ap


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--dataset", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--seeds", type=int, default=5)
    ap.add_argument("--epochs", type=int, default=80)
    ap.add_argument("--batch-size", type=int, default=16)
    ap.add_argument("--lr", type=float, default=3e-4)
    ap.add_argument("--no-health-film", action="store_true",
                    help="E6 消融：去掉健康条件通道")
    ap.add_argument("--device", default="cuda")
    args = ap.parse_args()

    import torch
    device = args.device if torch.cuda.is_available() else "cpu"
    results = {}
    for seed in range(args.seeds):
        results[seed] = train_one(args.dataset, args.out, seed, args.epochs,
                                  args.batch_size, args.lr,
                                  not args.no_health_film, device)
    (Path(args.out) / "summary.json").write_text(json.dumps({
        "val_ap_by_seed": results,
        "mean_val_ap": float(np.mean(list(results.values()))),
        "health_film": not args.no_health_film}, indent=2))
    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
