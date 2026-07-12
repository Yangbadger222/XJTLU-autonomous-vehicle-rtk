from pathlib import Path


SAVE_SCRIPT = Path("scripts/save_mapping_session.py")
TRAVEL_LAUNCH = Path("src/bringup/launch/system_travel.launch.py")


def test_mapping_session_generates_versioned_indoor_bundle_artifacts():
    text = SAVE_SCRIPT.read_text(encoding="utf-8")

    assert 'runtime_root / "maps" / "indoor" / map_name' in text
    assert '"schema_version": 1' in text
    assert '"map_localization.pcd"' in text
    assert '"map_raw.pcd"' in text
    assert '"slam_toolbox.data"' in text
    assert "build_scan_context_index" in text
    assert "downsample_pcd" in text
    assert "verify_stationary" in text
    assert '"/odom_CBoar"' in text
    assert '"/cmd_vel"' in text
    assert "register_planar_maps" in text
    assert "extract_vertical_structure_xy_from_pcd" in text
    assert '"map_3d_to_map_2d.yaml"' in text
    assert '"artifact_integrity"' in text
    assert '"Scan Context index contains no candidates"' in text


def test_travel_rejects_unaccepted_or_incomplete_bundle():
    text = TRAVEL_LAUNCH.read_text(encoding="utf-8")

    assert "Unsupported indoor map bundle schema" in text
    assert "Indoor map bundle did not pass consistency gates" in text
    assert "Indoor map bundle calibration is not accepted" in text
    assert "Indoor map bundle artifact not found" in text
    assert "Indoor map bundle artifact escapes bundle root" in text
    assert 'context.launch_configurations["map_id"]' in text
