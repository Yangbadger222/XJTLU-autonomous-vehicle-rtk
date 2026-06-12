"""ONNX 导出（opset 17）。engine 转换在 Jetson 上做（trtexec，与硬件/TRT 版本
绑定，不要在工作站转）：

  # Jetson:
  trtexec --onnx=frc_tinyunet.onnx --fp16 \
      --saveEngine=runtime-data/frc/models/frc-v2.engine
  cp frc_tinyunet.onnx.meta.json runtime-data/frc/models/frc-v2.engine.meta.json

用法：
  python -m frc_offline.train.export_onnx --ckpt runs/frc-v2/tinyunet_seed0.pt \
      --out frc_tinyunet.onnx
"""

import argparse
import json
from pathlib import Path


def export(ckpt_path, out_path):
    import torch

    from frc_offline.train.model import FlatTinyUNet, TinyUNet

    ckpt = torch.load(ckpt_path, map_location="cpu")
    model = TinyUNet(in_channels=ckpt["in_channels"],
                     health_dim=ckpt["health_dim"],
                     use_health_film=ckpt["use_health_film"])
    model.load_state_dict(ckpt["state_dict"])
    model.eval()

    calib_path = Path(ckpt_path).with_suffix(".calib.json")
    temperature = 1.0
    if calib_path.exists():
        temperature = json.loads(calib_path.read_text())["temperature"]

    c, n, k = ckpt["in_channels"], ckpt["grid"], ckpt["health_dim"]
    flat = FlatTinyUNet(model, c, n, k, temperature=temperature)
    flat.eval()
    dummy = torch.zeros(c * n * n + k)
    torch.onnx.export(flat, dummy, out_path, opset_version=17,
                      input_names=["flat_input"], output_names=["flat_output"])

    meta = {
        "bev_fingerprint": ckpt["bev_fingerprint"],
        "dataset_hash": ckpt["dataset_hash"],
        "temperature": temperature,
        "in_channels": c, "grid": n, "health_dim": k,
        "use_health_film": ckpt["use_health_film"],
        "val_ap": ckpt.get("val_ap"),
        "seed": ckpt.get("seed"),
    }
    meta_path = Path(str(out_path) + ".meta.json")
    meta_path.write_text(json.dumps(meta, indent=2))
    print(f"onnx -> {out_path}\nmeta -> {meta_path}\n"
          f"temperature={temperature} fingerprint={ckpt['bev_fingerprint']}")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--ckpt", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    export(args.ckpt, args.out)


if __name__ == "__main__":
    main()
