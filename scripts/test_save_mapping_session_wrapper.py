from pathlib import Path

import yaml

import save_mapping_session


def test_save_mapping_session_wrapper_sources_ros_without_nounset():
    text = Path("scripts/save_mapping_session.sh").read_text(encoding="utf-8")

    assert "set -euo pipefail" in text
    assert "set +u\nsource /opt/ros/humble/setup.bash\nsource install/setup.bash\nset -u" in text


def test_recalibrate_only_preserves_capture_evidence(tmp_path, monkeypatch):
    bundle = tmp_path / "floor_4"
    bundle.mkdir()
    manifest_path = bundle / "manifest.yaml"
    manifest_path.write_text(
        yaml.safe_dump(
            {
                "schema_version": 1,
                "map_id": "floor_4",
                "created_at": "original",
                "save_outputs": {"2d": "saved", "3d": "saved"},
                "artifact_integrity": {"ok": True},
                "stationary_check": {"ok": True},
                "patch_pose_integrity": {"ok": True},
                "frame_check": {"ok": True},
                "errors": [],
            }
        ),
        encoding="utf-8",
    )
    consistency = {
        "ok": True,
        "transform_map_2d_from_map_3d": {"x": 0.0, "y": 0.0, "yaw": 0.0},
        "source_extraction": {"method": "vertical_span_grid", "output_cells": 100},
    }
    monkeypatch.setattr(
        save_mapping_session, "compute_consistency", lambda *_args: consistency
    )
    monkeypatch.setattr(
        save_mapping_session, "write_alignment_artifacts", lambda *_args: None
    )

    result = save_mapping_session.recalibrate_existing_bundle(bundle)
    updated = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))

    assert result == 0
    assert updated["created_at"] == "original"
    assert updated["save_outputs"] == {"2d": "saved", "3d": "saved"}
    assert updated["calibration"]["accepted"] is True
    assert updated["consistency_ok"] is True
    assert updated["processing_outputs"]["alignment_recalibration"][
        "source_extraction"
    ]["output_cells"] == 100
