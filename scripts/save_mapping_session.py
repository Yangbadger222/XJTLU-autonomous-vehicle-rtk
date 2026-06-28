#!/usr/bin/env python3
"""Save a mapping session's 2D map, 3D PGO map, and manifest."""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml

from mapping_session_utils import (
    evaluate_frame_snapshot,
    estimate_planar_alignment,
    load_occupied_points_from_map,
    patch_pose_integrity,
    sample_cloud_xy_from_pcd,
    validate_map_name,
)


def runtime_root_from_env() -> Path:
    if os.environ.get("FYP_RUNTIME_ROOT"):
        return Path(os.environ["FYP_RUNTIME_ROOT"]).expanduser()
    return Path.home() / "XJTLU-autonomous-vehicle" / "runtime-data"


def run_command(command: list[str], *, timeout_s: float | None = None) -> subprocess.CompletedProcess[str]:
    print("+ " + " ".join(command), flush=True)
    return subprocess.run(
        command,
        check=False,
        timeout=timeout_s,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )


def require_success(result: subprocess.CompletedProcess[str], action: str) -> str:
    if result.returncode != 0:
        raise RuntimeError(f"{action} failed with exit code {result.returncode}\n{result.stdout}")
    return result.stdout


def capture_optional(command: list[str], *, timeout_s: float = 4.0) -> dict[str, Any]:
    try:
        result = run_command(command, timeout_s=timeout_s)
        return {
            "ok": result.returncode == 0,
            "returncode": result.returncode,
            "output": result.stdout.strip(),
        }
    except subprocess.TimeoutExpired as exc:
        return {"ok": False, "returncode": None, "output": f"timeout after {exc.timeout}s"}
    except OSError as exc:
        return {"ok": False, "returncode": None, "output": str(exc)}


def file_info(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {"exists": False}
    return {"exists": True, "bytes": path.stat().st_size}


def save_2d_map(map2d_dir: Path, timeout_s: float) -> str:
    map2d_dir.mkdir(parents=True, exist_ok=True)
    result = run_command(
        [
            "ros2",
            "run",
            "nav2_map_server",
            "map_saver_cli",
            "-f",
            str(map2d_dir / "map"),
            "--ros-args",
            "-p",
            f"save_map_timeout:={timeout_s}",
            "-p",
            "map_subscribe_transient_local:=true",
        ],
        timeout_s=timeout_s + 10.0,
    )
    return require_success(result, "2D map save")


def save_3d_map(map3d_dir: Path) -> str:
    map3d_dir.mkdir(parents=True, exist_ok=True)
    request = f"{{file_path: '{map3d_dir}', save_patches: true}}"
    result = run_command(
        [
            "ros2",
            "service",
            "call",
            "/pgo/save_maps",
            "interface/srv/SaveMaps",
            request,
        ],
        timeout_s=120.0,
    )
    output = require_success(result, "3D PGO map save")
    if "success=True" not in output:
        raise RuntimeError(f"3D PGO map save returned unsuccessful response\n{output}")
    return output


def compute_consistency(map2d_dir: Path, map3d_dir: Path) -> dict[str, Any]:
    map_yaml = map2d_dir / "map.yaml"
    map_pcd = map3d_dir / "map.pcd"
    if not map_yaml.exists() or not map_pcd.exists():
        return {"ok": False, "reason": "missing_2d_or_3d_artifact"}
    occupied_2d = load_occupied_points_from_map(map_yaml)
    cloud_xy = sample_cloud_xy_from_pcd(map_pcd)
    return estimate_planar_alignment(occupied_2d, cloud_xy)


def build_manifest(
    *,
    map_name: str,
    runtime_root: Path,
    map_root: Path,
    map2d_dir: Path,
    map3d_dir: Path,
    save_outputs: dict[str, str],
    errors: list[str],
    expected_base_frame: str,
) -> dict[str, Any]:
    patch_integrity = patch_pose_integrity(map3d_dir)
    consistency = compute_consistency(map2d_dir, map3d_dir)
    frame_snapshot = {
        "scan_header": capture_optional(
            ["ros2", "topic", "echo", "/scan", "sensor_msgs/msg/LaserScan", "--field", "header", "--once"],
            timeout_s=5.0,
        ),
        "fastlio_odom_child_frame": capture_optional(
            [
                "ros2",
                "topic",
                "echo",
                "/fastlio2/lio_odom",
                "nav_msgs/msg/Odometry",
                "--field",
                "child_frame_id",
                "--once",
            ],
            timeout_s=5.0,
        ),
    }
    frame_check = evaluate_frame_snapshot(frame_snapshot, expected_base_frame=expected_base_frame)
    rtk_snapshot = {
        "status": capture_optional(["ros2", "topic", "echo", "/rtk/status", "std_msgs/msg/String", "--once"]),
        "heading": capture_optional(["ros2", "topic", "echo", "/heading", "--once"]),
        "note": "Geo-registration must use RTK Fixed samples plus heading; indoor invalid/float samples are records only.",
    }
    geo_registration_readiness = {
        "ready": bool(rtk_snapshot["status"].get("ok")) and bool(rtk_snapshot["heading"].get("ok")),
        "heading_required": True,
        "fixed_rtk_required": True,
        "note": "This snapshot only records topic availability. Accept geo-registration only from a moved RTK Fixed trajectory or dual-antenna heading, not from a single RTK point.",
    }
    consistency_ok = (
        bool(consistency.get("ok"))
        and bool(patch_integrity.get("ok"))
        and bool(frame_check.get("ok"))
        and not errors
    )
    return {
        "map_name": map_name,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "runtime_root": str(runtime_root),
        "artifacts": {
            "map_root": str(map_root),
            "map2d": {
                "dir": str(map2d_dir),
                "yaml": file_info(map2d_dir / "map.yaml"),
                "pgm": file_info(map2d_dir / "map.pgm"),
            },
            "map3d": {
                "dir": str(map3d_dir),
                "pcd": file_info(map3d_dir / "map.pcd"),
                "poses": file_info(map3d_dir / "poses.txt"),
                "patches_dir": str(map3d_dir / "patches"),
            },
        },
        "save_outputs": save_outputs,
        "patch_pose_integrity": patch_integrity,
        "alignment_diagnostic": consistency,
        "consistency_ok": consistency_ok,
        "frame_snapshot": frame_snapshot,
        "frame_check": frame_check,
        "rtk_snapshot": rtk_snapshot,
        "geo_registration_readiness": geo_registration_readiness,
        "errors": errors,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("map_name", help="Map name, e.g. indoor_0628")
    parser.add_argument(
        "--runtime-root",
        type=Path,
        default=runtime_root_from_env(),
        help="Runtime data root. Defaults to FYP_RUNTIME_ROOT or ~/XJTLU-autonomous-vehicle/runtime-data.",
    )
    parser.add_argument("--map-saver-timeout-s", type=float, default=30.0)
    parser.add_argument(
        "--expected-base-frame",
        default="base_link",
        help="Frame expected in /scan.header.frame_id and /fastlio2/lio_odom.child_frame_id.",
    )
    parser.add_argument("--skip-2d", action="store_true", help="Do not call map_saver_cli.")
    parser.add_argument("--skip-3d", action="store_true", help="Do not call /pgo/save_maps.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        map_name = validate_map_name(args.map_name)
    except ValueError as exc:
        print(f"Invalid map name: {exc}", file=sys.stderr)
        return 2

    runtime_root = args.runtime_root.expanduser().resolve()
    map_root = runtime_root / "maps" / map_name
    map2d_dir = runtime_root / "maps" / "2d" / map_name
    map3d_dir = runtime_root / "maps" / "3d" / map_name
    map_root.mkdir(parents=True, exist_ok=True)

    save_outputs: dict[str, str] = {}
    errors: list[str] = []
    if not args.skip_2d:
        try:
            save_outputs["2d"] = save_2d_map(map2d_dir, args.map_saver_timeout_s)
        except Exception as exc:  # noqa: BLE001 - manifest records runtime failures for field debugging.
            errors.append(str(exc))
    if not args.skip_3d:
        try:
            save_outputs["3d"] = save_3d_map(map3d_dir)
        except Exception as exc:  # noqa: BLE001
            errors.append(str(exc))

    manifest = build_manifest(
        map_name=map_name,
        runtime_root=runtime_root,
        map_root=map_root,
        map2d_dir=map2d_dir,
        map3d_dir=map3d_dir,
        save_outputs=save_outputs,
        errors=errors,
        expected_base_frame=args.expected_base_frame,
    )
    manifest_path = map_root / "manifest.yaml"
    manifest_path.write_text(yaml.safe_dump(manifest, sort_keys=False, allow_unicode=True), encoding="utf-8")
    print(f"Wrote manifest: {manifest_path}")
    if errors:
        print("Mapping session save completed with errors:", file=sys.stderr)
        for error in errors:
            print(f"- {error}", file=sys.stderr)
        return 1
    if not manifest["consistency_ok"]:
        print("Mapping session saved, but consistency_ok=false. Check manifest before using the map.", file=sys.stderr)
        return 3
    print("Mapping session saved successfully.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
