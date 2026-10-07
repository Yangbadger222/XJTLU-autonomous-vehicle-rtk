import json
from pathlib import Path


MAPPING = Path(__file__).parents[3] / "audit" / "vehicle_baseline" / "LIO_FIELD_MAPPING.json"


def test_lio_mapping_keeps_unresolved_semantics_fail_closed():
    payload = json.loads(MAPPING.read_text())
    assert payload["vehicle_commit"] == "e54c6afbcb5a58db22d7c468085a87d658b0b932"
    assert payload["super_lio_commit"] == "f89f48dc7aea6cfa262f18e4d03b319e04e0dbd2"
    fields = {item["name"]: item for item in payload["fields"]}
    assert fields["imu_header_stamp"]["status"] == "EXACT_CARRY_OVER"
    assert fields["imu_header_stamp"]["measurement_clock_status"] == "TARGET_RUNTIME_PENDING"
    assert "cur_node_->now()" in fields["imu_header_stamp"]["vehicle_driver_source"]
    assert fields["imu_linear_acceleration_scale"]["status"] == "SOURCE_VERIFIED_DIFFERENT_NORMALIZATION"
    assert fields["imu_linear_acceleration_scale"]["unit_status"] == "SOURCE_VERIFIED"
    assert fields["livox_offset_time"]["status"] == "SOFTWARE_VERIFIED"
    assert "consumed as nanoseconds" in fields["livox_offset_time"]["source_unit_claim"]
    assert fields["health"]["status"] == "SOURCE_DERIVED_SUFFICIENT_BOUND"


def test_lio_mapping_has_no_unqualified_health_equivalence():
    payload = json.loads(MAPPING.read_text())
    health = next(item for item in payload["fields"] if item["name"] == "health")
    assert "Invalid/missing/expired certificate denies health" in health["motion_policy"]
    assert "RTK loss remains an independent stop" in health["motion_policy"]
    assert health["status"] != "EXACT_CARRY_OVER"
