from pathlib import Path

import numpy as np
import pytest
import yaml
from PIL import Image

from mapping_session_utils import (
    build_scan_context_index,
    downsample_pcd,
    evaluate_frame_snapshot,
    estimate_planar_alignment,
    load_occupied_points_from_map,
    patch_pose_integrity,
    read_pcd_xyz,
    register_planar_maps,
    scan_context_descriptor,
    save_alignment_overlay,
    transform_xy,
    validate_map_name,
)


def write_binary_pcd(path: Path, points: np.ndarray) -> None:
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
    intensities = np.zeros((len(points), 1), dtype=np.float32)
    payload = np.hstack([points.astype(np.float32), intensities]).astype(np.float32)
    path.write_bytes(header + payload.tobytes())


def test_validate_map_name_rejects_shell_and_path_metacharacters():
    assert validate_map_name("indoor_0628") == "indoor_0628"
    assert validate_map_name("door-map-a") == "door-map-a"

    for bad_name in ["../map", "map/name", "<map_name>", "name with spaces", ""]:
        with pytest.raises(ValueError):
            validate_map_name(bad_name)


def test_read_pcd_xyz_reads_binary_xyzi_layout(tmp_path):
    pcd = tmp_path / "map.pcd"
    points = np.array([[1.0, 2.0, 0.1], [-1.0, 0.5, -0.2]], dtype=np.float32)
    write_binary_pcd(pcd, points)

    loaded = read_pcd_xyz(pcd)

    np.testing.assert_allclose(loaded, points)


def test_read_pcd_xyz_reads_pcl_pointxyzinormal_layout(tmp_path):
    pcd = tmp_path / "map_normals.pcd"
    fields = np.array(
        [(1.0, 2.0, 0.1, 0.0, 0.0, 1.0, 0.2, 17.0)],
        dtype=[
            ("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
            ("normal_x", "<f4"), ("normal_y", "<f4"), ("normal_z", "<f4"),
            ("curvature", "<f4"), ("intensity", "<f4"),
        ],
    )
    header = "\n".join(
        [
            "VERSION 0.7",
            "FIELDS x y z normal_x normal_y normal_z curvature intensity",
            "SIZE 4 4 4 4 4 4 4 4",
            "TYPE F F F F F F F F",
            "COUNT 1 1 1 1 1 1 1 1",
            "WIDTH 1",
            "HEIGHT 1",
            "POINTS 1",
            "DATA binary",
            "",
        ]
    ).encode("ascii")
    pcd.write_bytes(header + fields.tobytes())

    loaded = read_pcd_xyz(pcd)

    np.testing.assert_allclose(loaded, [[1.0, 2.0, 0.1]])


def test_downsample_pcd_writes_localization_copy(tmp_path):
    source = tmp_path / "raw.pcd"
    destination = tmp_path / "localization.pcd"
    points = np.array(
        [[0.01, 0.01, 0.01], [0.02, 0.02, 0.02], [1.0, 0.0, 0.0]], dtype=np.float32
    )
    write_binary_pcd(source, points)

    result = downsample_pcd(source, destination, 0.1)

    assert result["input_points"] == 3
    assert result["output_points"] == 2
    assert len(read_pcd_xyz(destination)) == 2


def test_register_planar_maps_recovers_rigid_transform(tmp_path):
    horizontal = np.stack([np.linspace(-3.0, 3.0, 80), np.zeros(80)], axis=1)
    vertical = np.stack([np.full(60, -3.0), np.linspace(0.0, 4.0, 60)], axis=1)
    occupied = np.vstack([horizontal, vertical])
    expected = (1.2, -0.7, 0.18)
    source = transform_xy(occupied, -expected[0], -expected[1], -expected[2])

    result = register_planar_maps(
        occupied,
        source,
        max_rmse_m=0.12,
        max_p95_m=0.20,
        min_overlap_ratio=0.80,
    )

    assert result["ok"] is True
    assert result["wall_rmse_m"] < 0.12
    assert result["wall_p95_m"] < 0.20
    assert result["ambiguous"] is False
    assert result["candidate_score_gap"] >= 0.001

    output = tmp_path / "mapping_alignment_overlay_test.png"
    save_alignment_overlay(
        occupied, source, result["transform_map_2d_from_map_3d"], output
    )
    assert output.exists()


def test_register_planar_maps_reports_regional_quality():
    horizontal = np.stack([np.linspace(-3.0, 3.0, 80), np.zeros(80)], axis=1)
    vertical = np.stack([np.full(60, -3.0), np.linspace(0.0, 4.0, 60)], axis=1)
    occupied = np.vstack([horizontal, vertical])
    source = transform_xy(occupied, -1.2, 0.7, -0.18)

    result = register_planar_maps(
        occupied,
        source,
        max_rmse_m=0.12,
        max_p95_m=0.20,
        min_overlap_ratio=0.80,
        regions={"west": [[-4, -1], [0, -1], [0, 5], [-4, 5]]},
    )

    assert result["regions_ok"] is True
    assert result["regional_metrics"]["west"]["points"] >= 20


def test_scan_context_index_contains_keyframe_descriptors(tmp_path):
    map_dir = tmp_path / "localization"
    patches = map_dir / "patches"
    patches.mkdir(parents=True)
    (map_dir / "poses.txt").write_text("0.pcd 1 2 0 1 0 0 0\n", encoding="utf-8")
    points = np.array([[1.0, 0.0, 0.2], [2.0, 0.0, 0.5], [0.0, 1.0, 0.8]], dtype=np.float32)
    write_binary_pcd(patches / "0.pcd", points)
    output = map_dir / "descriptor_index" / "scan_context.yaml"
    regions = tmp_path / "regions.yaml"
    regions.write_text(
        yaml.safe_dump(
            {
                "schema_version": 1,
                "regions": {
                    "lobby": {"polygon": [[0, 0], [3, 0], [3, 3], [0, 3]]}
                },
            }
        ),
        encoding="utf-8",
    )

    result = build_scan_context_index(
        map_dir, output, keyframe_stride=1, rings=4, sectors=8, regions_file=regions
    )
    payload = yaml.safe_load(output.read_text(encoding="utf-8"))
    descriptor, ring_key = scan_context_descriptor(points, rings=4, sectors=8)

    assert result["candidate_count"] == 1
    assert payload["candidates"][0]["position"] == [1.0, 2.0, 0.0]
    assert payload["candidates"][0]["region"] == "lobby"
    assert result["region_count"] == 1
    assert len(payload["candidates"][0]["descriptor"]) == descriptor.size
    assert len(payload["candidates"][0]["ring_key"]) == len(ring_key)


def test_scan_context_region_labels_use_map2d_alignment(tmp_path):
    map_dir = tmp_path / "localization"
    patches = map_dir / "patches"
    patches.mkdir(parents=True)
    (map_dir / "poses.txt").write_text("0.pcd 1 2 0 1 0 0 0\n", encoding="utf-8")
    write_binary_pcd(
        patches / "0.pcd", np.array([[1.0, 0.0, 0.2]], dtype=np.float32)
    )
    regions = tmp_path / "regions.yaml"
    regions.write_text(
        yaml.safe_dump(
            {"regions": {"shifted": {"polygon": [[10, 0], [14, 0], [14, 4], [10, 4]]}}}
        ),
        encoding="utf-8",
    )
    output = map_dir / "descriptor_index" / "scan_context.yaml"

    build_scan_context_index(
        map_dir,
        output,
        keyframe_stride=1,
        rings=4,
        sectors=8,
        regions_file=regions,
        map2d_from_map3d={"x": 10.0, "y": 0.0, "yaw": 0.0},
    )

    payload = yaml.safe_load(output.read_text(encoding="utf-8"))
    assert payload["candidates"][0]["region"] == "shifted"
    assert payload["candidates"][0]["position"] == [1.0, 2.0, 0.0]


def test_patch_pose_integrity_requires_one_patch_per_pose(tmp_path):
    map_dir = tmp_path / "3d"
    patches = map_dir / "patches"
    patches.mkdir(parents=True)
    (map_dir / "poses.txt").write_text(
        "0.pcd 0 0 0 1 0 0 0\n1.pcd 1 0 0 1 0 0 0\n",
        encoding="utf-8",
    )
    write_binary_pcd(patches / "0.pcd", np.array([[0, 0, 0]], dtype=np.float32))

    result = patch_pose_integrity(map_dir)

    assert result["ok"] is False
    assert result["pose_count"] == 2
    assert result["patch_count"] == 1
    assert result["missing_patches"] == ["1.pcd"]


def test_load_occupied_points_and_estimate_identity_alignment(tmp_path):
    map_dir = tmp_path / "2d"
    map_dir.mkdir()
    image = np.full((10, 10), 254, dtype=np.uint8)
    image[4:6, 2:8] = 0
    Image.fromarray(image, mode="L").save(map_dir / "map.pgm")
    (map_dir / "map.yaml").write_text(
        yaml.safe_dump(
            {
                "image": "map.pgm",
                "resolution": 0.5,
                "origin": [0.0, 0.0, 0.0],
                "occupied_thresh": 0.65,
                "free_thresh": 0.25,
                "negate": 0,
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )

    occupied_2d = load_occupied_points_from_map(map_dir / "map.yaml")
    cloud_xy = occupied_2d.copy()

    result = estimate_planar_alignment(occupied_2d, cloud_xy)

    assert result["ok"] is True
    assert abs(result["yaw_rad"]) < 1.0e-6
    assert result["translation_xy"] == pytest.approx([0.0, 0.0], abs=1.0e-6)
    assert result["centroid_distance_m"] < 1.0e-6


def test_estimate_planar_alignment_rejects_large_yaw_delta():
    occupied_2d = np.array(
        [[0.0, 0.0], [1.0, 0.0], [2.0, 0.0], [3.0, 0.0], [4.0, 0.0], [5.0, 0.0]],
        dtype=np.float64,
    )
    cloud_xy = np.array(
        [[0.0, 0.0], [0.0, 1.0], [0.0, 2.0], [0.0, 3.0], [0.0, 4.0], [0.0, 5.0]],
        dtype=np.float64,
    )

    result = estimate_planar_alignment(occupied_2d, cloud_xy, max_abs_yaw_deg=30.0)

    assert result["ok"] is False
    assert result["reason"] == "yaw_delta_high"
    assert abs(result["yaw_deg"]) == pytest.approx(90.0)


def test_evaluate_frame_snapshot_checks_scan_and_odom_frames():
    snapshot = {
        "scan_header": {
            "ok": True,
            "output": "stamp:\n  sec: 1\n  nanosec: 2\nframe_id: base_footprint\n---",
        },
        "fastlio_odom_child_frame": {
            "ok": True,
            "output": "base_footprint\n---",
        },
    }

    result = evaluate_frame_snapshot(snapshot)

    assert result["ok"] is True
    assert result["expected_base_frame"] == "base_footprint"
    assert result["scan_frame_id"] == "base_footprint"
    assert result["fastlio_odom_child_frame_id"] == "base_footprint"


def test_evaluate_frame_snapshot_rejects_base_frame_mismatch():
    snapshot = {
        "scan_header": {
            "ok": True,
            "output": "stamp:\n  sec: 1\n  nanosec: 2\nframe_id: body\n---",
        },
        "fastlio_odom_child_frame": {
            "ok": True,
            "output": "base_footprint\n---",
        },
    }

    result = evaluate_frame_snapshot(snapshot)

    assert result["ok"] is False
    assert result["reason"] == "frame_mismatch"
    assert result["scan_frame_id"] == "body"
