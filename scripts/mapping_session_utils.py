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


def read_pcd_xyzi(path: Path) -> np.ndarray:
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
        if (
            data_type != "binary"
            or not fields
            or len(fields) != len(sizes)
            or len(fields) != len(types)
            or len(fields) != len(counts)
            or not {"x", "y", "z"}.issubset(fields)
        ):
            raise ValueError(f"{path}: unsupported PCD layout")
        type_map = {
            ("F", 4): "<f4",
            ("F", 8): "<f8",
            ("I", 1): "<i1",
            ("I", 2): "<i2",
            ("I", 4): "<i4",
            ("I", 8): "<i8",
            ("U", 1): "<u1",
            ("U", 2): "<u2",
            ("U", 4): "<u4",
            ("U", 8): "<u8",
        }
        dtype_fields = []
        point_step = 0
        field_counts = dict(zip(fields, counts))
        if any(field_counts.get(field) != 1 for field in ("x", "y", "z")):
            raise ValueError(f"{path}: x/y/z fields must be scalar")
        for field, size, field_type, count in zip(fields, sizes, types, counts):
            scalar_type = type_map.get((field_type, size))
            if scalar_type is None or count < 1:
                raise ValueError(f"{path}: unsupported PCD field {field}")
            dtype_fields.append(
                (field, scalar_type) if count == 1 else (field, scalar_type, (count,))
            )
            point_step += size * count
        payload = pcd_file.read(points * point_step)
    if len(payload) != points * point_step:
        raise ValueError(
            f"{path}: expected {points * point_step} bytes, got {len(payload)}"
        )
    records = np.frombuffer(payload, dtype=np.dtype(dtype_fields), count=points)
    points_xyzi = np.zeros((points, 4), dtype=np.float32)
    for column, field in enumerate(("x", "y", "z")):
        points_xyzi[:, column] = np.asarray(records[field], dtype=np.float32)
    if "intensity" in fields and field_counts["intensity"] == 1:
        points_xyzi[:, 3] = np.asarray(records["intensity"], dtype=np.float32)
    return points_xyzi[np.isfinite(points_xyzi).all(axis=1)]


def read_pcd_xyz(path: Path) -> np.ndarray:
    return read_pcd_xyzi(path)[:, :3]


def write_binary_pcd_xyzi(path: Path, points_xyzi: np.ndarray) -> None:
    points = np.asarray(points_xyzi, dtype=np.float32)
    if points.ndim != 2 or points.shape[1] != 4:
        raise ValueError("points_xyzi must have shape (N, 4)")
    points = points[np.isfinite(points).all(axis=1)]
    path.parent.mkdir(parents=True, exist_ok=True)
    header = "\n".join(
        [
            "# .PCD v0.7 - Point Cloud Data file format",
            "VERSION 0.7",
            "FIELDS x y z intensity",
            "SIZE 4 4 4 4",
            "TYPE F F F F",
            "COUNT 1 1 1 1",
            f"WIDTH {len(points)}",
            "HEIGHT 1",
            "VIEWPOINT 0 0 0 1 0 0 0",
            f"POINTS {len(points)}",
            "DATA binary",
            "",
        ]
    ).encode("ascii")
    path.write_bytes(header + points.tobytes())


def voxel_downsample_xyzi(points_xyzi: np.ndarray, leaf_size: float) -> np.ndarray:
    points = np.asarray(points_xyzi, dtype=np.float32)
    if leaf_size <= 0.0 or len(points) == 0:
        return points.copy()
    keys = np.floor(points[:, :3] / float(leaf_size)).astype(np.int64)
    _, first_indices = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(first_indices)]


def downsample_pcd(source: Path, destination: Path, leaf_size: float) -> dict[str, Any]:
    source_points = read_pcd_xyzi(source)
    output_points = voxel_downsample_xyzi(source_points, leaf_size)
    write_binary_pcd_xyzi(destination, output_points)
    return {
        "input_points": int(len(source_points)),
        "output_points": int(len(output_points)),
        "leaf_size_m": float(leaf_size),
    }


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


def transform_xy(points_xy: np.ndarray, x: float, y: float, yaw: float) -> np.ndarray:
    c = math.cos(yaw)
    s = math.sin(yaw)
    points = np.asarray(points_xy, dtype=np.float64)
    transformed_x = c * points[:, 0] - s * points[:, 1] + x
    transformed_y = s * points[:, 0] + c * points[:, 1] + y
    return np.column_stack([transformed_x, transformed_y])


def nearest_distances(source_xy: np.ndarray, target_xy: np.ndarray, *, chunk_size: int = 512) -> np.ndarray:
    if len(source_xy) == 0 or len(target_xy) == 0:
        return np.full((len(source_xy),), np.inf, dtype=np.float64)
    distances = np.empty((len(source_xy),), dtype=np.float64)
    for start in range(0, len(source_xy), chunk_size):
        chunk = source_xy[start : start + chunk_size]
        squared = np.sum((chunk[:, None, :] - target_xy[None, :, :]) ** 2, axis=2)
        distances[start : start + len(chunk)] = np.sqrt(np.min(squared, axis=1))
    return distances


class _PlanarNearestIndex:
    """Exact nearest distances up to a fixed radius using a uniform grid."""

    def __init__(self, target_xy: np.ndarray, *, radius_m: float = 0.50):
        self.radius_m = float(radius_m)
        self.target_xy = np.asarray(target_xy, dtype=np.float64)
        self.buckets: dict[tuple[int, int], np.ndarray] = {}
        if len(self.target_xy) == 0:
            return
        keys = np.floor(self.target_xy / self.radius_m).astype(np.int64)
        unique_keys, inverse = np.unique(keys, axis=0, return_inverse=True)
        for index, key in enumerate(unique_keys):
            self.buckets[(int(key[0]), int(key[1]))] = self.target_xy[inverse == index]

    def query(self, source_xy: np.ndarray) -> np.ndarray:
        source = np.asarray(source_xy, dtype=np.float64)
        distances = np.full(len(source), self.radius_m, dtype=np.float64)
        if len(source) == 0 or not self.buckets:
            return distances
        source_keys = np.floor(source / self.radius_m).astype(np.int64)
        unique_keys, inverse = np.unique(source_keys, axis=0, return_inverse=True)
        for index, key in enumerate(unique_keys):
            source_indices = np.flatnonzero(inverse == index)
            candidates = [
                self.buckets[(int(key[0]) + dx, int(key[1]) + dy)]
                for dx in (-1, 0, 1)
                for dy in (-1, 0, 1)
                if (int(key[0]) + dx, int(key[1]) + dy) in self.buckets
            ]
            if not candidates:
                continue
            target = np.vstack(candidates)
            delta = source[source_indices, None, :] - target[None, :, :]
            nearest = np.sqrt(np.min(np.sum(delta * delta, axis=2), axis=1))
            distances[source_indices] = np.minimum(nearest, self.radius_m)
        return distances


def _sample_evenly(points: np.ndarray, maximum: int) -> np.ndarray:
    if len(points) <= maximum:
        return points
    indices = np.linspace(0, len(points) - 1, maximum, dtype=np.int64)
    return points[indices]


def register_planar_maps(
    occupied_2d_xy: np.ndarray,
    cloud_3d_xy: np.ndarray,
    *,
    max_points: int = 12000,
    max_rmse_m: float = 0.10,
    max_p95_m: float = 0.15,
    min_overlap_ratio: float = 0.55,
    min_candidate_score_gap: float = 0.001,
    regions: dict[str, list[list[float]]] | None = None,
) -> dict[str, Any]:
    if len(occupied_2d_xy) < 20 or len(cloud_3d_xy) < 20:
        return {"ok": False, "reason": "not_enough_points"}

    target = _sample_evenly(np.asarray(occupied_2d_xy, dtype=np.float64), max_points)
    source = _sample_evenly(np.asarray(cloud_3d_xy, dtype=np.float64), max_points)
    target_center = target.mean(axis=0)
    source_center = source.mean(axis=0)
    yaw_seed = _normalize_half_turn(_principal_yaw(target) - _principal_yaw(source))

    nearest_index = _PlanarNearestIndex(target)

    def evaluate(x: float, y: float, yaw: float) -> tuple[float, np.ndarray]:
        transformed = transform_xy(source, x, y, yaw)
        distances = nearest_index.query(transformed)
        clipped = np.minimum(distances, 0.50)
        return float(np.mean(clipped * clipped)), distances

    candidates: list[tuple[float, float, float, float, np.ndarray]] = []
    yaw_seeds = []
    for raw_yaw in (yaw_seed, yaw_seed + math.pi, 0.0, math.pi / 2, -math.pi / 2):
        normalized_yaw = _normalize_angle(raw_yaw)
        if not any(abs(_normalize_angle(normalized_yaw - saved)) < 1.0e-6 for saved in yaw_seeds):
            yaw_seeds.append(normalized_yaw)
    for yaw in yaw_seeds:
        c = math.cos(yaw)
        s = math.sin(yaw)
        rotated_center = np.array(
            [c * source_center[0] - s * source_center[1], s * source_center[0] + c * source_center[1]]
        )
        translation = target_center - rotated_center
        score, distances = evaluate(float(translation[0]), float(translation[1]), yaw)
        candidates.append((score, float(translation[0]), float(translation[1]), yaw, distances))
    identity_score, identity_distances = evaluate(0.0, 0.0, 0.0)
    candidates.append((identity_score, 0.0, 0.0, 0.0, identity_distances))

    candidates.sort(key=lambda item: item[0])
    initial_best_score = float(candidates[0][0])
    best_seed = candidates[0]
    distinct_candidates = [
        candidate
        for candidate in candidates[1:]
        if math.hypot(candidate[1] - best_seed[1], candidate[2] - best_seed[2]) >= 0.25
        or abs(_normalize_angle(candidate[3] - best_seed[3])) >= math.radians(5.0)
    ]
    initial_second_score = (
        float(distinct_candidates[0][0]) if distinct_candidates else float("inf")
    )
    candidate_score_gap = initial_second_score - initial_best_score
    score, x, y, yaw, distances = candidates[0]
    for translation_step, yaw_step_deg in ((0.50, 5.0), (0.20, 2.0), (0.10, 1.0), (0.05, 0.5)):
        improved = True
        yaw_step = math.radians(yaw_step_deg)
        refinement_iterations = 0
        while improved and refinement_iterations < 200:
            refinement_iterations += 1
            improved = False
            for dx, dy, dyaw in (
                (translation_step, 0.0, 0.0),
                (-translation_step, 0.0, 0.0),
                (0.0, translation_step, 0.0),
                (0.0, -translation_step, 0.0),
                (0.0, 0.0, yaw_step),
                (0.0, 0.0, -yaw_step),
            ):
                trial_score, trial_distances = evaluate(x + dx, y + dy, yaw + dyaw)
                if trial_score + 1.0e-9 < score:
                    score = trial_score
                    x += dx
                    y += dy
                    yaw += dyaw
                    distances = trial_distances
                    improved = True

    inliers = distances[distances <= 0.30]
    if len(inliers) == 0:
        return {"ok": False, "reason": "no_wall_overlap"}
    rmse = float(np.sqrt(np.mean(inliers * inliers)))
    p95 = float(np.percentile(inliers, 95))
    overlap = float(len(inliers) / len(distances))
    transformed = transform_xy(source, x, y, yaw)
    region_metrics = {}
    for name, polygon in (regions or {}).items():
        mask = np.array(
            [_point_in_polygon(float(point[0]), float(point[1]), polygon) for point in transformed],
            dtype=bool,
        )
        region_distances = distances[mask]
        region_inliers = region_distances[region_distances <= 0.30]
        enough_points = len(region_distances) >= 20 and len(region_inliers) > 0
        region_rmse = (
            float(np.sqrt(np.mean(region_inliers * region_inliers))) if enough_points else float("inf")
        )
        region_p95 = float(np.percentile(region_inliers, 95)) if enough_points else float("inf")
        region_overlap = float(len(region_inliers) / len(region_distances)) if enough_points else 0.0
        region_metrics[name] = {
            "ok": enough_points
            and region_rmse <= max_rmse_m
            and region_p95 <= max_p95_m
            and region_overlap >= min_overlap_ratio,
            "points": int(len(region_distances)),
            "wall_rmse_m": region_rmse,
            "wall_p95_m": region_p95,
            "overlap_ratio": region_overlap,
        }
    regions_ok = all(metric["ok"] for metric in region_metrics.values())
    ambiguous = candidate_score_gap < min_candidate_score_gap
    ok = (
        rmse <= max_rmse_m
        and p95 <= max_p95_m
        and overlap >= min_overlap_ratio
        and regions_ok
        and not ambiguous
    )
    return {
        "ok": ok,
        "reason": "ok" if ok else "quality_gate_failed",
        "transform_map_2d_from_map_3d": {"x": x, "y": y, "yaw": _normalize_angle(yaw)},
        "wall_rmse_m": rmse,
        "wall_p95_m": p95,
        "overlap_ratio": overlap,
        "initial_best_candidate_score": initial_best_score,
        "initial_second_candidate_score": initial_second_score,
        "candidate_score_gap": candidate_score_gap,
        "ambiguous": ambiguous,
        "regional_metrics": region_metrics,
        "regions_ok": regions_ok,
        "thresholds": {
            "max_rmse_m": max_rmse_m,
            "max_p95_m": max_p95_m,
            "min_overlap_ratio": min_overlap_ratio,
            "min_candidate_score_gap": min_candidate_score_gap,
        },
        "points_2d": int(len(target)),
        "points_3d": int(len(source)),
    }


def save_alignment_overlay(
    occupied_2d_xy: np.ndarray,
    cloud_3d_xy: np.ndarray,
    transform: dict[str, float],
    output_path: Path,
    *,
    resolution_m: float = 0.05,
    max_dimension: int = 2000,
) -> None:
    target = np.asarray(occupied_2d_xy, dtype=np.float64)
    transformed = transform_xy(
        np.asarray(cloud_3d_xy, dtype=np.float64),
        float(transform["x"]),
        float(transform["y"]),
        float(transform["yaw"]),
    )
    combined = np.vstack([target, transformed])
    minimum = combined.min(axis=0) - 0.5
    maximum = combined.max(axis=0) + 0.5
    extent = np.maximum(maximum - minimum, resolution_m)
    scale = min(1.0 / resolution_m, max_dimension / float(np.max(extent)))
    width = max(1, int(math.ceil(extent[0] * scale)))
    height = max(1, int(math.ceil(extent[1] * scale)))
    image = np.full((height, width, 3), 255, dtype=np.uint8)

    def pixels(points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        columns = np.clip(((points[:, 0] - minimum[0]) * scale).astype(int), 0, width - 1)
        rows = np.clip(((maximum[1] - points[:, 1]) * scale).astype(int), 0, height - 1)
        return rows, columns

    rows, columns = pixels(_sample_evenly(target, 100000))
    image[rows, columns] = [220, 40, 40]
    rows, columns = pixels(_sample_evenly(transformed, 100000))
    existing = image[rows, columns]
    overlap = np.all(existing == np.array([220, 40, 40], dtype=np.uint8), axis=1)
    image[rows, columns] = [30, 170, 190]
    image[rows[overlap], columns[overlap]] = [30, 170, 60]
    output_path.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(image, mode="RGB").save(output_path)


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def scan_context_descriptor(
    xyz: np.ndarray,
    *,
    rings: int = 20,
    sectors: int = 60,
    max_radius_m: float = 20.0,
) -> tuple[np.ndarray, np.ndarray]:
    descriptor = np.zeros((rings, sectors), dtype=np.float32)
    if len(xyz) == 0:
        return descriptor, descriptor.mean(axis=1)
    radius = np.hypot(xyz[:, 0], xyz[:, 1])
    valid = np.isfinite(xyz).all(axis=1) & (radius > 0.0) & (radius <= max_radius_m)
    points = xyz[valid]
    radius = radius[valid]
    if len(points) == 0:
        return descriptor, descriptor.mean(axis=1)
    angles = np.mod(np.arctan2(points[:, 1], points[:, 0]), 2.0 * math.pi)
    ring_indices = np.minimum((radius / max_radius_m * rings).astype(np.int64), rings - 1)
    sector_indices = np.minimum((angles / (2.0 * math.pi) * sectors).astype(np.int64), sectors - 1)
    heights = points[:, 2] - float(np.min(points[:, 2])) + 1.0
    for ring, sector, height in zip(ring_indices, sector_indices, heights):
        descriptor[ring, sector] = max(descriptor[ring, sector], float(height))
    return descriptor, descriptor.mean(axis=1)


def load_keyframe_poses(poses_path: Path) -> list[dict[str, Any]]:
    poses: list[dict[str, Any]] = []
    for line in poses_path.read_text(encoding="utf-8").splitlines():
        parts = line.split()
        if len(parts) != 8:
            continue
        name, x, y, z, qw, qx, qy, qz = parts
        poses.append(
            {
                "patch": name,
                "position": [float(x), float(y), float(z)],
                "quaternion_wxyz": [float(qw), float(qx), float(qy), float(qz)],
            }
        )
    return poses


def _point_in_polygon(x: float, y: float, polygon: list[list[float]]) -> bool:
    inside = False
    previous = polygon[-1]
    for current in polygon:
        x1, y1 = float(previous[0]), float(previous[1])
        x2, y2 = float(current[0]), float(current[1])
        if (y1 > y) != (y2 > y):
            boundary_x = (x2 - x1) * (y - y1) / (y2 - y1) + x1
            if x < boundary_x:
                inside = not inside
        previous = current
    return inside


def load_regions(path: Path | None) -> dict[str, list[list[float]]]:
    if path is None or not path.exists():
        return {}
    payload = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    regions = payload.get("regions", {})
    result = {}
    for name, raw in regions.items():
        polygon = raw.get("polygon", []) if isinstance(raw, dict) else []
        if len(polygon) >= 3:
            result[str(name)] = polygon
    return result


def region_for_pose(position: list[float], regions: dict[str, list[list[float]]]) -> str:
    for name, polygon in regions.items():
        if _point_in_polygon(float(position[0]), float(position[1]), polygon):
            return name
    return ""


def build_scan_context_index(
    map3d_dir: Path,
    output_path: Path,
    *,
    keyframe_stride: int = 5,
    rings: int = 20,
    sectors: int = 60,
    max_radius_m: float = 20.0,
    regions_file: Path | None = None,
    map2d_from_map3d: dict[str, float] | None = None,
) -> dict[str, Any]:
    poses = load_keyframe_poses(map3d_dir / "poses.txt")
    regions = load_regions(regions_file)
    candidates = []
    for index, pose in enumerate(poses):
        if index % max(1, keyframe_stride) != 0 and index != len(poses) - 1:
            continue
        patch_path = map3d_dir / "patches" / pose["patch"]
        if not patch_path.exists():
            continue
        descriptor, ring_key = scan_context_descriptor(
            read_pcd_xyz(patch_path), rings=rings, sectors=sectors, max_radius_m=max_radius_m
        )
        region_position = pose["position"]
        if map2d_from_map3d is not None:
            transformed_position = transform_xy(
                np.asarray([pose["position"][:2]], dtype=np.float64),
                float(map2d_from_map3d["x"]),
                float(map2d_from_map3d["y"]),
                float(map2d_from_map3d["yaw"]),
            )[0]
            region_position = [
                float(transformed_position[0]),
                float(transformed_position[1]),
                float(pose["position"][2]),
            ]
        candidates.append(
            {
                **pose,
                "region": region_for_pose(region_position, regions),
                "ring_key": ring_key.astype(float).tolist(),
                "descriptor": descriptor.reshape(-1).astype(float).tolist(),
            }
        )
    payload = {
        "schema_version": 1,
        "rings": rings,
        "sectors": sectors,
        "max_radius_m": max_radius_m,
        "keyframe_stride": keyframe_stride,
        "candidates": candidates,
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
    return {
        "candidate_count": len(candidates),
        "keyframe_count": len(poses),
        "region_count": len(regions),
        "file": str(output_path),
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


def extract_vertical_structure_xy(
    xyz: np.ndarray,
    *,
    cell_size_m: float = 0.06,
    min_vertical_span_m: float = 0.50,
    min_points_per_cell: int = 3,
) -> tuple[np.ndarray, dict[str, Any]]:
    """Collapse XY cells with repeated returns over a wall-like vertical span."""
    points = np.asarray(xyz, dtype=np.float64)
    points = points[np.isfinite(points).all(axis=1)]
    if cell_size_m <= 0.0:
        raise ValueError("cell_size_m must be positive")
    if min_vertical_span_m < 0.0:
        raise ValueError("min_vertical_span_m must be non-negative")
    if min_points_per_cell < 1:
        raise ValueError("min_points_per_cell must be at least one")
    if len(points) == 0:
        return np.empty((0, 2), dtype=np.float64), {
            "method": "vertical_span_grid",
            "input_points": 0,
            "output_cells": 0,
            "cell_size_m": float(cell_size_m),
            "min_vertical_span_m": float(min_vertical_span_m),
            "min_points_per_cell": int(min_points_per_cell),
        }

    keys = np.floor(points[:, :2] / float(cell_size_m)).astype(np.int64)
    unique_keys, inverse = np.unique(keys, axis=0, return_inverse=True)
    counts = np.bincount(inverse)
    z_min = np.full(len(unique_keys), np.inf, dtype=np.float64)
    z_max = np.full(len(unique_keys), -np.inf, dtype=np.float64)
    np.minimum.at(z_min, inverse, points[:, 2])
    np.maximum.at(z_max, inverse, points[:, 2])
    xy_sums = np.zeros((len(unique_keys), 2), dtype=np.float64)
    np.add.at(xy_sums, inverse, points[:, :2])
    keep = (
        (counts >= max(1, int(min_points_per_cell)))
        & ((z_max - z_min) >= float(min_vertical_span_m))
    )
    wall_xy = xy_sums[keep] / counts[keep, None]
    return wall_xy, {
        "method": "vertical_span_grid",
        "input_points": int(len(points)),
        "output_cells": int(len(wall_xy)),
        "cell_size_m": float(cell_size_m),
        "min_vertical_span_m": float(min_vertical_span_m),
        "min_points_per_cell": int(min_points_per_cell),
    }


def extract_vertical_structure_xy_from_pcd(
    pcd_path: Path,
    **kwargs: Any,
) -> tuple[np.ndarray, dict[str, Any]]:
    return extract_vertical_structure_xy(read_pcd_xyz(pcd_path), **kwargs)


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
