import importlib.util
import json
import math
from pathlib import Path


SCRIPT = Path("scripts/evaluate_corridor_authority_replay.py")


def _load_module():
    spec = importlib.util.spec_from_file_location("corridor_authority_replay", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_synthetic_replay_detects_heading_freeze_and_five_sample_reacquisition():
    replay = _load_module()
    headings = [
        {"stamp_s": index * 0.1 + 1.0, "correction_yaw_rad": 0.0}
        for index in range(5)
    ]
    headings.append(
        {"stamp_s": 1.5, "correction_yaw_rad": math.radians(52.0)}
    )
    headings.extend(
        {"stamp_s": 1.6 + index * 0.1, "correction_yaw_rad": 0.01}
        for index in range(5)
    )

    result = replay.evaluate_records({"heading_observations": headings})

    assert result["heading"]["outlier_rejected"] is True
    assert result["heading"]["max_rejected_innovation_deg"] >= 50.0
    assert result["heading"]["minimum_reacquire_samples"] >= 5


def test_synthetic_replay_checks_release_rates_and_saturation_runs():
    replay = _load_module()
    releases = [
        {
            "stamp_s": index * 0.1,
            "x": index * 0.0204,
            "y": 0.0,
            "yaw": math.radians(index * 0.204),
            "saturated": index < 10,
        }
        for index in range(12)
    ]

    result = replay.evaluate_records({"release_samples": releases})

    assert result["release"]["max_translation_rate_mps"] <= 0.205
    assert result["release"]["max_yaw_rate_degps"] <= 2.05
    assert result["release"]["max_consecutive_saturated"] == 10
    assert result["assertions"]["release_rates_within_limits"] is True


def test_synthetic_replay_keeps_global_correction_out_of_local_abort():
    replay = _load_module()
    records = {
        "local_odom": [
            {"stamp_s": 1.0, "x": 0.0, "y": 0.0, "yaw": 0.0},
            {"stamp_s": 1.1, "x": 0.01, "y": 0.0, "yaw": 0.0},
        ],
        "global_correction": [
            {"stamp_s": 1.0, "x": 0.0, "y": 0.0, "yaw": 0.0},
            {"stamp_s": 1.1, "x": 1.0, "y": 0.0, "yaw": 0.5},
        ],
    }

    result = replay.evaluate_records(records)

    assert result["local_watchdog"]["abort_count"] == 0
    assert result["global_correction"]["hold_count"] == 1
    assert result["assertions"]["global_never_classified_as_local_abort"] is True


def test_synthetic_replay_detects_fifteen_second_local_no_progress():
    replay = _load_module()
    samples = [
        {
            "stamp_s": float(index),
            "command_speed_mps": 0.6,
            "lio_speed_mps": 0.0,
            "gnss_speed_mps": 0.0,
        }
        for index in range(17)
    ]

    result = replay.evaluate_records({"progress_samples": samples})

    assert result["progress"]["local_no_progress"] is True
    assert result["progress"]["max_no_progress_duration_s"] >= 15.0


def test_cli_writes_json_and_returns_nonzero_for_failed_assertions(tmp_path):
    replay = _load_module()
    records_path = tmp_path / "records.json"
    output_path = tmp_path / "result.json"
    records_path.write_text(
        json.dumps(
            {
                "release_samples": [
                    {"stamp_s": 0.0, "x": 0.0, "y": 0.0, "yaw": 0.0},
                    {"stamp_s": 0.1, "x": 1.0, "y": 0.0, "yaw": 0.0},
                ]
            }
        ),
        encoding="utf-8",
    )

    return_code = replay.main(
        ["--records", str(records_path), "--out", str(output_path)]
    )

    assert return_code == 1
    payload = json.loads(output_path.read_text(encoding="utf-8"))
    assert payload["passed"] is False


def test_manifest_names_all_four_external_acceptance_fixtures():
    replay = _load_module()

    assert set(replay.BAG_MANIFEST) == {
        "2026-07-10-13-34-36",
        "2026-07-10-13-36-24",
        "2026-07-10-13-46-03",
        "2026-07-10-13-48-43",
    }
