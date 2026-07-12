#!/usr/bin/env python3
"""Save a mapping session's 2D map, 3D PGO map, and manifest."""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml

from mapping_session_utils import (
    build_scan_context_index,
    downsample_pcd,
    evaluate_frame_snapshot,
    extract_vertical_structure_xy_from_pcd,
    load_occupied_points_from_map,
    load_regions,
    patch_pose_integrity,
    register_planar_maps,
    save_alignment_overlay,
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


def _twist_motion(capture: dict[str, Any], *, odometry: bool) -> tuple[float, float] | None:
    if not capture.get("ok"):
        return None
    try:
        message = yaml.safe_load(str(capture.get("output", "")).split("---", maxsplit=1)[0])
        twist = message["twist"]
        if odometry:
            twist = twist["twist"]
        linear = twist["linear"]
        angular = twist["angular"]
        linear_speed = (
            float(linear.get("x", 0.0)) ** 2
            + float(linear.get("y", 0.0)) ** 2
            + float(linear.get("z", 0.0)) ** 2
        ) ** 0.5
        angular_speed = abs(float(angular.get("z", 0.0)))
        return linear_speed, angular_speed
    except (KeyError, TypeError, ValueError, yaml.YAMLError):
        return None


def verify_stationary(
    *,
    hold_s: float = 2.0,
    max_linear_mps: float = 0.03,
    max_angular_rps: float = 0.05,
) -> dict[str, Any]:
    samples = []
    for index in range(2):
        lio_capture = capture_optional(
            ["ros2", "topic", "echo", "/fastlio2/lio_odom", "nav_msgs/msg/Odometry", "--once"],
            timeout_s=5.0,
        )
        chassis_capture = capture_optional(
            ["ros2", "topic", "echo", "/odom_CBoar", "nav_msgs/msg/Odometry", "--once"],
            timeout_s=5.0,
        )
        command_capture = capture_optional(
            ["ros2", "topic", "echo", "/cmd_vel", "geometry_msgs/msg/Twist", "--once"],
            timeout_s=1.0,
        )
        samples.append(
            {
                "lio_capture": lio_capture,
                "chassis_capture": chassis_capture,
                "command_capture": command_capture,
                "lio_motion": _twist_motion(lio_capture, odometry=True),
                "chassis_motion": _twist_motion(chassis_capture, odometry=True),
                "command_motion": _twist_motion(command_capture, odometry=False),
            }
        )
        if index == 0 and hold_s > 0.0:
            time.sleep(hold_s)

    def is_still(motion: tuple[float, float] | None, *, required: bool) -> bool:
        if motion is None:
            return not required
        return motion[0] <= max_linear_mps and motion[1] <= max_angular_rps

    ok = all(
        is_still(sample["lio_motion"], required=True)
        and is_still(sample["chassis_motion"], required=True)
        and is_still(sample["command_motion"], required=False)
        for sample in samples
    )
    return {
        "ok": ok,
        "hold_s": hold_s,
        "max_linear_mps": max_linear_mps,
        "max_angular_rps": max_angular_rps,
        "samples": samples,
        "note": "A missing /cmd_vel sample means no live command was observed; LIO and chassis odometry are mandatory.",
    }


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


def save_pose_graph(map2d_dir: Path) -> str:
    target = map2d_dir / "slam_toolbox"
    result = run_command(
        [
            "ros2",
            "service",
            "call",
            "/slam_toolbox/serialize_map",
            "slam_toolbox/srv/SerializePoseGraph",
            f"{{filename: '{target}'}}",
        ],
        timeout_s=60.0,
    )
    output = require_success(result, "Slam Toolbox pose-graph save")
    if "result=0" not in output and "result: 0" not in output:
        raise RuntimeError(f"Slam Toolbox pose-graph save returned unsuccessful response\n{output}")
    return output


def compute_consistency(
    map2d_dir: Path, localization_pcd: Path, regions_path: Path
) -> dict[str, Any]:
    map_yaml = map2d_dir / "map.yaml"
    map_pcd = localization_pcd
    if not map_yaml.exists() or not map_pcd.exists():
        return {"ok": False, "reason": "missing_2d_or_3d_artifact"}
    occupied_2d = load_occupied_points_from_map(map_yaml)
    cloud_xy, extraction = extract_vertical_structure_xy_from_pcd(map_pcd)
    result = register_planar_maps(
        occupied_2d, cloud_xy, regions=load_regions(regions_path)
    )
    result["source_extraction"] = extraction
    return result


def write_alignment_artifacts(
    map2d_dir: Path,
    localization_pcd: Path,
    consistency: dict[str, Any],
    calibration_dir: Path,
) -> None:
    calibration_dir.mkdir(parents=True, exist_ok=True)
    transform = consistency.get(
        "transform_map_2d_from_map_3d", {"x": 0.0, "y": 0.0, "yaw": 0.0}
    )
    alignment_payload = {
        "schema_version": 1,
        "parent_frame": "map_2d",
        "child_frame": "map_3d",
        "transform": transform,
        "quality": consistency,
        "accepted": bool(consistency.get("ok")),
    }
    (calibration_dir / "map_3d_to_map_2d.yaml").write_text(
        yaml.safe_dump(alignment_payload, sort_keys=False), encoding="utf-8"
    )
    (calibration_dir / "alignment_report.yaml").write_text(
        yaml.safe_dump(consistency, sort_keys=False), encoding="utf-8"
    )
    if consistency.get("transform_map_2d_from_map_3d") and localization_pcd.exists():
        save_alignment_overlay(
            load_occupied_points_from_map(map2d_dir / "map.yaml"),
            extract_vertical_structure_xy_from_pcd(localization_pcd)[0],
            consistency["transform_map_2d_from_map_3d"],
            calibration_dir / "alignment_overlay.png",
        )


def recalibrate_existing_bundle(map_root: Path) -> int:
    manifest_path = map_root / "manifest.yaml"
    if not manifest_path.exists():
        raise RuntimeError(f"existing map manifest not found: {manifest_path}")
    manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8")) or {}
    if manifest.get("schema_version") != 1:
        raise RuntimeError("recalibration requires indoor map bundle schema_version 1")
    if manifest.get("map_id") != map_root.name:
        raise RuntimeError("manifest map_id does not match bundle directory")

    map2d_dir = map_root / "navigation"
    map3d_dir = map_root / "localization"
    localization_pcd = map3d_dir / "map_localization.pcd"
    regions_path = map_root / "regions.yaml"
    consistency = compute_consistency(map2d_dir, localization_pcd, regions_path)
    write_alignment_artifacts(
        map2d_dir, localization_pcd, consistency, map_root / "calibration"
    )

    descriptor_path = map3d_dir / "descriptor_index" / "scan_context.yaml"
    if descriptor_path.exists() and consistency.get("transform_map_2d_from_map_3d"):
        descriptor_config = yaml.safe_load(
            descriptor_path.read_text(encoding="utf-8")
        ) or {}
        descriptor_result = build_scan_context_index(
            map3d_dir,
            descriptor_path,
            keyframe_stride=int(descriptor_config.get("keyframe_stride", 5)),
            rings=int(descriptor_config.get("rings", 20)),
            sectors=int(descriptor_config.get("sectors", 60)),
            max_radius_m=float(descriptor_config.get("max_radius_m", 20.0)),
            regions_file=regions_path,
            map2d_from_map3d=consistency["transform_map_2d_from_map_3d"],
        )
        manifest.setdefault("processing_outputs", {})[
            "descriptor_index"
        ] = descriptor_result

    errors = [
        error
        for error in manifest.get("errors", [])
        if not str(error).startswith("2D/3D alignment failed:")
    ]
    manifest["errors"] = errors
    manifest["alignment_diagnostic"] = consistency
    manifest["calibration"] = {
        "file": "calibration/map_3d_to_map_2d.yaml",
        "accepted": bool(consistency.get("ok")),
    }
    manifest.setdefault("processing_outputs", {})["alignment_recalibration"] = {
        "at": datetime.now(timezone.utc).isoformat(),
        "source_extraction": consistency.get("source_extraction", {}),
    }
    manifest["consistency_ok"] = bool(
        consistency.get("ok")
        and manifest.get("artifact_integrity", {}).get("ok")
        and manifest.get("stationary_check", {}).get("ok")
        and manifest.get("patch_pose_integrity", {}).get("ok")
        and manifest.get("frame_check", {}).get("ok")
        and not errors
    )
    temporary_path = manifest_path.with_suffix(".yaml.tmp")
    temporary_path.write_text(
        yaml.safe_dump(manifest, sort_keys=False, allow_unicode=True), encoding="utf-8"
    )
    temporary_path.replace(manifest_path)
    print(f"Recalibrated manifest: {manifest_path}")
    if not manifest["consistency_ok"]:
        print(
            "Recalibration completed, but consistency_ok=false. Check alignment_report.yaml.",
            file=sys.stderr,
        )
        return 3
    print("Mapping session recalibrated successfully.")
    return 0


def build_manifest(
    *,
    map_name: str,
    runtime_root: Path,
    map_root: Path,
    map2d_dir: Path,
    map3d_dir: Path,
    localization_pcd: Path,
    save_outputs: dict[str, str],
    processing_outputs: dict[str, Any],
    stationary_check: dict[str, Any],
    consistency: dict[str, Any],
    errors: list[str],
    expected_base_frame: str,
) -> dict[str, Any]:
    patch_integrity = patch_pose_integrity(map3d_dir)
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
        and bool(stationary_check.get("ok"))
        and not errors
    )
    calibration_dir = map_root / "calibration"
    alignment_file = calibration_dir / "map_3d_to_map_2d.yaml"
    alignment_report = calibration_dir / "alignment_report.yaml"
    try:
        write_alignment_artifacts(
            map2d_dir, localization_pcd, consistency, calibration_dir
        )
    except Exception as exc:  # noqa: BLE001
        errors.append(f"alignment artifact generation failed: {exc}")

    git_commit = capture_optional(["git", "rev-parse", "HEAD"], timeout_s=2.0)
    git_branch = capture_optional(["git", "branch", "--show-current"], timeout_s=2.0)
    required_artifacts = {
        "navigation_map": map2d_dir / "map.yaml",
        "navigation_image": map2d_dir / "map.pgm",
        "pose_graph": map2d_dir / "slam_toolbox.posegraph",
        "pose_graph_data": map2d_dir / "slam_toolbox.data",
        "localization_map": localization_pcd,
        "raw_map": map3d_dir / "map_raw.pcd",
        "poses": map3d_dir / "poses.txt",
        "patches": map3d_dir / "patches",
        "descriptor_index": map3d_dir / "descriptor_index" / "scan_context.yaml",
        "alignment": alignment_file,
        "alignment_report": alignment_report,
        "alignment_overlay": calibration_dir / "alignment_overlay.png",
        "destinations": map_root / "destinations.yaml",
        "regions": map_root / "regions.yaml",
    }
    missing_artifacts = [
        name for name, path in required_artifacts.items() if not path.exists()
    ]
    artifact_integrity = {
        "ok": not missing_artifacts,
        "missing": missing_artifacts,
    }
    consistency_ok = consistency_ok and artifact_integrity["ok"] and not errors
    return {
        "schema_version": 1,
        "map_id": map_name,
        "map_name": map_name,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "runtime_root": str(runtime_root),
        "git": {
            "branch": git_branch.get("output", ""),
            "commit": git_commit.get("output", ""),
        },
        "frames": {"navigation": "map_2d", "localization": "map_3d", "runtime": "map"},
        "artifacts": {
            "navigation_map": "navigation/map.yaml",
            "navigation_image": "navigation/map.pgm",
            "pose_graph": "navigation/slam_toolbox.posegraph",
            "pose_graph_data": "navigation/slam_toolbox.data",
            "localization_map": "localization/map_localization.pcd",
            "raw_map": "localization/map_raw.pcd",
            "poses": "localization/poses.txt",
            "patches": "localization/patches",
            "descriptor_index": "localization/descriptor_index/scan_context.yaml",
            "alignment": "calibration/map_3d_to_map_2d.yaml",
            "alignment_report": "calibration/alignment_report.yaml",
            "alignment_overlay": "calibration/alignment_overlay.png",
            "destinations": "destinations.yaml",
            "regions": "regions.yaml",
        },
        "artifact_info": {
            "navigation_yaml": file_info(map2d_dir / "map.yaml"),
            "navigation_pgm": file_info(map2d_dir / "map.pgm"),
            "localization_pcd": file_info(localization_pcd),
            "raw_pcd": file_info(map3d_dir / "map_raw.pcd"),
            "poses": file_info(map3d_dir / "poses.txt"),
        },
        "artifact_integrity": artifact_integrity,
        "save_outputs": save_outputs,
        "processing_outputs": processing_outputs,
        "stationary_check": stationary_check,
        "patch_pose_integrity": patch_integrity,
        "alignment_diagnostic": consistency,
        "calibration": {"file": "calibration/map_3d_to_map_2d.yaml", "accepted": bool(consistency.get("ok"))},
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
    parser.add_argument("--localization-voxel-m", type=float, default=0.10)
    parser.add_argument("--descriptor-stride", type=int, default=5)
    parser.add_argument("--regions-file", type=Path, default=None)
    parser.add_argument("--stationary-hold-s", type=float, default=2.0)
    parser.add_argument(
        "--expected-base-frame",
        default="base_footprint",
        help="Frame expected in /scan.header.frame_id and /fastlio2/lio_odom.child_frame_id.",
    )
    parser.add_argument("--skip-2d", action="store_true", help="Do not call map_saver_cli.")
    parser.add_argument("--skip-3d", action="store_true", help="Do not call /pgo/save_maps.")
    parser.add_argument("--skip-pose-graph", action="store_true", help="Do not serialize the Slam Toolbox pose graph.")
    parser.add_argument("--skip-stationary-check", action="store_true")
    parser.add_argument(
        "--recalibrate-only",
        action="store_true",
        help="Recompute calibration and manifest from an existing bundle without ROS saves.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        map_name = validate_map_name(args.map_name)
    except ValueError as exc:
        print(f"Invalid map name: {exc}", file=sys.stderr)
        return 2

    runtime_root = args.runtime_root.expanduser().resolve()
    map_root = runtime_root / "maps" / "indoor" / map_name
    map2d_dir = map_root / "navigation"
    map3d_dir = map_root / "localization"
    if args.recalibrate_only:
        try:
            return recalibrate_existing_bundle(map_root)
        except Exception as exc:  # noqa: BLE001
            print(f"Recalibration failed: {exc}", file=sys.stderr)
            return 1
    map_root.mkdir(parents=True, exist_ok=True)

    save_outputs: dict[str, str] = {}
    processing_outputs: dict[str, Any] = {}
    errors: list[str] = []
    regions_path = map_root / "regions.yaml"
    if args.regions_file is not None:
        source_regions = args.regions_file.expanduser().resolve()
        regions_path.write_text(source_regions.read_text(encoding="utf-8"), encoding="utf-8")
    elif not regions_path.exists():
        regions_path.write_text(
            yaml.safe_dump({"schema_version": 1, "map_id": map_name, "regions": {}}, sort_keys=False),
            encoding="utf-8",
        )
    stationary_check = (
        {"ok": True, "skipped": True}
        if args.skip_stationary_check
        else verify_stationary(hold_s=args.stationary_hold_s)
    )
    if not stationary_check.get("ok"):
        errors.append("vehicle stationary gate failed")
    if not args.skip_2d:
        try:
            save_outputs["2d"] = save_2d_map(map2d_dir, args.map_saver_timeout_s)
        except Exception as exc:  # noqa: BLE001 - manifest records runtime failures for field debugging.
            errors.append(str(exc))
    if not args.skip_pose_graph:
        try:
            save_outputs["pose_graph"] = save_pose_graph(map2d_dir)
        except Exception as exc:  # noqa: BLE001
            errors.append(str(exc))
    if not args.skip_3d:
        try:
            save_outputs["3d"] = save_3d_map(map3d_dir)
        except Exception as exc:  # noqa: BLE001
            errors.append(str(exc))

    raw_pcd = map3d_dir / "map_raw.pcd"
    pgo_pcd = map3d_dir / "map.pcd"
    localization_pcd = map3d_dir / "map_localization.pcd"
    if pgo_pcd.exists():
        pgo_pcd.replace(raw_pcd)
    if raw_pcd.exists():
        try:
            processing_outputs["downsample"] = downsample_pcd(
                raw_pcd, localization_pcd, args.localization_voxel_m
            )
        except Exception as exc:  # noqa: BLE001
            errors.append(f"localization PCD generation failed: {exc}")
    try:
        consistency = compute_consistency(map2d_dir, localization_pcd, regions_path)
    except Exception as exc:  # noqa: BLE001
        consistency = {"ok": False, "reason": "alignment_processing_failed"}
        errors.append(f"2D/3D alignment failed: {exc}")
    if (map3d_dir / "poses.txt").exists() and (map3d_dir / "patches").exists():
        try:
            descriptor_result = build_scan_context_index(
                map3d_dir,
                map3d_dir / "descriptor_index" / "scan_context.yaml",
                keyframe_stride=args.descriptor_stride,
                regions_file=regions_path,
                map2d_from_map3d=consistency.get(
                    "transform_map_2d_from_map_3d"
                ),
            )
            processing_outputs["descriptor_index"] = descriptor_result
            if descriptor_result["candidate_count"] < 1:
                errors.append("Scan Context index contains no candidates")
        except Exception as exc:  # noqa: BLE001
            errors.append(f"Scan Context index generation failed: {exc}")

    destinations_path = map_root / "destinations.yaml"
    if not destinations_path.exists():
        destinations_path.write_text(
            yaml.safe_dump({"schema_version": 1, "map_id": map_name, "destinations": {}}, sort_keys=False),
            encoding="utf-8",
        )

    manifest = build_manifest(
        map_name=map_name,
        runtime_root=runtime_root,
        map_root=map_root,
        map2d_dir=map2d_dir,
        map3d_dir=map3d_dir,
        localization_pcd=localization_pcd,
        save_outputs=save_outputs,
        processing_outputs=processing_outputs,
        stationary_check=stationary_check,
        consistency=consistency,
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
