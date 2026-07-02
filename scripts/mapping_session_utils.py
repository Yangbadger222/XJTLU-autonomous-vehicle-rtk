#!/usr/bin/env python3
"""Utilities for saving and validating mapping-session artifacts."""

from __future__ import annotations

import math
import re
from pathlib import Path
from typing import Any

import numpy as np
import yaml
from PIL import Image


MAP_NAME_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$")


def validate_map_name(name: str) -> str:
    if not MAP_NAME_PATTERN.fullmatch(name):
        raise ValueError(
            "map name must be 1-64 chars and contain only letters, numbers, '.', '_', '-'"
        )
    return name


def read_pcd_xyz(path: Path) -> np.ndarray:
    with path.open("rb") as pcd_file:
        header: list[str] = []
        while True:
            line = pcd_file.readline()
            if not line:
                raise ValueError(f"{path}: missing DATA line")
            text = line.decode("ascii", errors="replace").strip()
            header.append(text)
            if line.startswith(b"DATA"):
                break
        meta: dict[str, list[str]] = {}
        for line in header:
            parts = line.split()
            if parts:
                meta[parts[0]] = parts[1:]
        fields = meta.get("FIELDS", [])
        sizes = [int(value) for value in meta.get("SIZE", [])]
        types = meta.get("TYPE", [])
        counts = [int(value) for value in meta.get("COUNT", ["1"] * len(fields))]
        points = int(meta.get("POINTS", meta.get("WIDTH", ["0"]))[0])
        data_type = meta.get("DATA", [""])[0]
        supported_layout = (
            fields[:4] == ["x", "y", "z", "intensity"]
            and sizes[:4] == [4, 4, 4, 4]
            and types[:4] == ["F", "F", "F", "F"]
            and counts[:4] == [1, 1, 1, 1]
            and data_type == "binary"
        )
        if not supported_layout:
            raise ValueError(f"{path}: unsupported PCD layout")
        payload = pcd_file.read(points * 16)
    if len(payload) != points * 16:
        raise ValueError(f"{path}: expected {points * 16} bytes, got {len(payload)}")
    points_xyzi = np.frombuffer(payload, dtype=np.float32).reshape(-1, 4)
    xyz = points_xyzi[:, :3].copy()
    return xyz[np.isfinite(xyz).all(axis=1)]


def patch_pose_integrity(map3d_dir: Path) -> dict[str, Any]:
    poses_path = map3d_dir / "poses.txt"
    patches_dir = map3d_dir / "patches"
    pose_names: list[str] = []
    if poses_path.exists():
        for line in poses_path.read_text(encoding="utf-8").splitlines():
            parts = line.split()
            if parts:
                pose_names.append(parts[0])
    patch_names = sorted(path.name for path in patches_dir.glob("*.pcd")) if patches_dir.exists() else []
    missing = sorted(name for name in pose_names if name not in set(patch_names))
    extra = sorted(name for name in patch_names if name not in set(pose_names))
    ok = bool(pose_names) and len(pose_names) == len(patch_names) and not missing and not extra
    return {
        "ok": ok,
        "pose_count": len(pose_names),
        "patch_count": len(patch_names),
        "missing_patches": missing,
        "extra_patches": extra,
    }


def load_occupied_points_from_map(map_yaml_path: Path) -> np.ndarray:
    metadata = yaml.safe_load(map_yaml_path.read_text(encoding="utf-8"))
    image_path = Path(metadata["image"])
    if not image_path.is_absolute():
        image_path = map_yaml_path.parent / image_path
    resolution = float(metadata["resolution"])
    origin_x, origin_y, _ = [float(value) for value in metadata["origin"]]
    image = np.asarray(Image.open(image_path).convert("L"))
    occupied_rows, occupied_cols = np.where(image <= 20)
    if len(occupied_rows) == 0:
        return np.empty((0, 2), dtype=np.float64)
    height = image.shape[0]
    x = origin_x + (occupied_cols.astype(np.float64) + 0.5) * resolution
    y = origin_y + ((height - 1 - occupied_rows).astype(np.float64) + 0.5) * resolution
    return np.stack([x, y], axis=1)


def _principal_yaw(points_xy: np.ndarray) -> float:
    centered = points_xy - points_xy.mean(axis=0)
    covariance = centered.T @ centered / max(len(points_xy) - 1, 1)
    values, vectors = np.linalg.eigh(covariance)
    axis = vectors[:, int(np.argmax(values))]
    return math.atan2(float(axis[1]), float(axis[0]))


def _normalize_half_turn(angle: float) -> float:
    while angle > math.pi / 2:
        angle -= math.pi
    while angle < -math.pi / 2:
        angle += math.pi
    return angle


def estimate_planar_alignment(
    occupied_2d_xy: np.ndarray,
    cloud_xy: np.ndarray,
    *,
    max_centroid_distance_m: float = 1.0,
    max_abs_yaw_deg: float = 30.0,
    min_points: int = 6,
) -> dict[str, Any]:
    if len(occupied_2d_xy) < min_points or len(cloud_xy) < min_points:
        return {
            "ok": False,
            "reason": "not_enough_points",
            "points_2d": int(len(occupied_2d_xy)),
            "points_3d": int(len(cloud_xy)),
        }

    centroid_2d = occupied_2d_xy.mean(axis=0)
    centroid_3d = cloud_xy.mean(axis=0)
    yaw_2d = _principal_yaw(occupied_2d_xy)
    yaw_3d = _principal_yaw(cloud_xy)
    yaw = _normalize_half_turn(yaw_2d - yaw_3d)
    yaw_deg = float(math.degrees(yaw))
    c = math.cos(yaw)
    s = math.sin(yaw)
    rotation = np.array([[c, -s], [s, c]], dtype=np.float64)
    translation = centroid_2d - rotation @ centroid_3d
    centroid_distance = float(np.linalg.norm(translation))
    yaw_ok = abs(yaw_deg) <= max_abs_yaw_deg
    translation_ok = centroid_distance <= max_centroid_distance_m
    if not yaw_ok:
        reason = "yaw_delta_high"
    elif not translation_ok:
        reason = "centroid_distance_high"
    else:
        reason = "ok"
    return {
        "ok": yaw_ok and translation_ok,
        "reason": reason,
        "yaw_rad": float(yaw),
        "yaw_deg": yaw_deg,
        "max_abs_yaw_deg": float(max_abs_yaw_deg),
        "translation_xy": [float(translation[0]), float(translation[1])],
        "centroid_distance_m": centroid_distance,
        "max_centroid_distance_m": float(max_centroid_distance_m),
        "points_2d": int(len(occupied_2d_xy)),
        "points_3d": int(len(cloud_xy)),
    }


def sample_cloud_xy_from_pcd(
    pcd_path: Path,
    *,
    z_min: float = -0.45,
    z_max: float = 0.65,
    max_points: int = 50000,
) -> np.ndarray:
    xyz = read_pcd_xyz(pcd_path)
    mask = (xyz[:, 2] >= z_min) & (xyz[:, 2] <= z_max)
    xy = xyz[mask, :2]
    if len(xy) > max_points:
        stride = max(1, len(xy) // max_points)
        xy = xy[::stride]
    return xy.astype(np.float64, copy=False)


def _clean_ros_scalar(value: str) -> str:
    return value.strip().strip("'\"")


def _extract_frame_id(output: str, yaml_key: str) -> str | None:
    for line in output.splitlines():
        stripped = line.strip()
        if stripped.startswith(f"{yaml_key}:"):
            return _clean_ros_scalar(stripped.split(":", 1)[1])
    return None


def _extract_echoed_scalar(output: str) -> str | None:
    for line in output.splitlines():
        stripped = line.strip()
        if not stripped or stripped == "---":
            continue
        if ":" not in stripped:
            return _clean_ros_scalar(stripped)
    return None


def evaluate_frame_snapshot(
    frame_snapshot: dict[str, Any],
    *,
    expected_base_frame: str = "base_footprint",
) -> dict[str, Any]:
    scan_capture = frame_snapshot.get("scan_header", {})
    odom_capture = frame_snapshot.get("fastlio_odom_child_frame", {})
    scan_frame_id = (
        _extract_frame_id(str(scan_capture.get("output", "")), "frame_id")
        if scan_capture.get("ok")
        else None
    )
    odom_child_frame_id = None
    if odom_capture.get("ok"):
        output = str(odom_capture.get("output", ""))
        odom_child_frame_id = _extract_frame_id(output, "child_frame_id") or _extract_echoed_scalar(output)

    missing = []
    if scan_frame_id is None:
        missing.append("scan_frame_id")
    if odom_child_frame_id is None:
        missing.append("fastlio_odom_child_frame_id")

    mismatches = []
    if scan_frame_id is not None and scan_frame_id != expected_base_frame:
        mismatches.append("scan_frame_id")
    if odom_child_frame_id is not None and odom_child_frame_id != expected_base_frame:
        mismatches.append("fastlio_odom_child_frame_id")

    if missing:
        ok = False
        reason = "frame_snapshot_missing"
    elif mismatches:
        ok = False
        reason = "frame_mismatch"
    else:
        ok = True
        reason = "ok"

    return {
        "ok": ok,
        "reason": reason,
        "expected_base_frame": expected_base_frame,
        "scan_frame_id": scan_frame_id,
        "fastlio_odom_child_frame_id": odom_child_frame_id,
        "missing": missing,
        "mismatches": mismatches,
    }
