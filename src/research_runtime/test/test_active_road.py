import json

import pytest

from research_runtime.active_road import (EvidenceState, EvidenceStore, GeoTransform, MaGRoadPrior,
                                           ObservationCandidate, RoadEvidence, choose_observation)
from active_road_mapping.evidence_node import _stamp_is_set


class _Stamp:
    def __init__(self, sec, nanosec):
        self.sec = sec
        self.nanosec = nanosec


def test_typed_evidence_requires_a_nonzero_acquisition_timestamp():
    assert not _stamp_is_set(_Stamp(0, 0))
    assert not _stamp_is_set(_Stamp(-1, 1))
    assert not _stamp_is_set(_Stamp(1, 1_000_000_000))
    assert _stamp_is_set(_Stamp(0, 1))


def test_evidence_replay_is_idempotent_and_persistent(tmp_path):
    store = EvidenceStore(GeoTransform("EPSG:32651", "WGS84", 0, 0, 1, 1), "v1")
    evidence = RoadEvidence("same", [(0, 0), (1, 0)], EvidenceState.OBSERVED_GEOMETRY,
                            1.0, "lidar", "s1", 0.1, 1.0, (0.2, 5.0))
    assert store.add(evidence)
    assert not store.add(evidence)
    path = tmp_path / "map.json"
    store.save(path)
    loaded = EvidenceStore.load(path)
    assert len(loaded.evidence()) == 1
    assert loaded.evidence()[0].state == EvidenceState.OBSERVED_GEOMETRY


def test_history_load_validates_all_content_without_hashing_every_prefix(tmp_path, monkeypatch):
    # A second task loads the full persisted history inside a ROS callback.
    # Rehashing every growing prefix made that callback exceed the unchanged
    # 0.20 s state and 0.50 s permission deadlines on the Humble host.
    store = EvidenceStore(GeoTransform("EPSG:32651", "WGS84", 0, 0, 1, 1), "v1")
    for index in range(224):
        store.add(RoadEvidence(str(index), [(0, 0), (1, 0)],
            EvidenceState.OBSERVED_GEOMETRY, 1.0, "lidar", "s1", .03, 1.0))
    store.anchor_submap("s1", (0, 0, 0), stamp=1., uncertainty_m=.03, authority_valid=True)
    path = tmp_path / "history.json"
    store.save(path)
    import research_runtime.active_road as active_road
    original_hash = active_road.hashlib.sha256
    hashes = []
    def count_hash(*args, **kwargs):
        hashes.append(1)
        return original_hash(*args, **kwargs)
    monkeypatch.setattr(active_road.hashlib, "sha256", count_hash)
    loaded = EvidenceStore.load(path)
    assert loaded.map_version == store.map_version
    assert loaded.evidence() == store.evidence()
    assert loaded.submap_anchors == store.submap_anchors
    assert len(hashes) == 1
    payload = json.loads(path.read_text())
    payload["evidence"][-1]["source"] = "tampered"
    path.write_text(json.dumps(payload))
    with pytest.raises(ValueError, match="content/version mismatch"):
        EvidenceStore.load(path)


def test_unsafe_high_score_candidate_is_filtered():
    selected = choose_observation([
        ObservationCandidate("unsafe", "e", True, False, True, True, 100, 1, 0.1),
        ObservationCandidate("safe", "e", True, True, True, True, 1, .5, 1),
    ])
    assert selected is not None
    assert selected.candidate_id == "safe"


def test_invalid_observation_cost_and_nonfinite_score_inputs_are_filtered():
    selected = choose_observation([
        ObservationCandidate("negative-cost", "e", True, True, True, True, 1, 1, -1),
        ObservationCandidate("negative-impact", "e", True, True, True, True, -1, 1, 1),
        ObservationCandidate("over-coverage", "e", True, True, True, True, 100, 1.1, 1),
        ObservationCandidate("nan-impact", "e", True, True, True, True, float("nan"), 1, 1),
        ObservationCandidate("valid", "e", True, True, True, True, 1, 0.5, 1),
    ])
    assert selected is not None
    assert selected.candidate_id == "valid"


def test_magroad_prior_requires_crs_and_keeps_model_version(tmp_path):
    path = tmp_path / "road.geojson"
    path.write_text(json.dumps({
        "type": "FeatureCollection",
        "crs": {"properties": {"name": "EPSG:32651"}},
        "features": [{"type": "Feature", "properties": {"id": "r1"},
                      "geometry": {"type": "LineString", "coordinates": [[0, 0], [1, 0]]}}],
    }))
    prior = MaGRoadPrior.load_geojson(path, expected_crs="EPSG:32651", model_version="magr-v1")
    assert prior.model_version == "magr-v1"
    assert prior.edges[0]["id"] == "r1"


def test_magroad_prior_rejects_missing_crs_and_invalid_geometry(tmp_path):
    with pytest.raises(ValueError, match="CRS"):
        GeoTransform("", "WGS84", 0, 0, 1, 1)

    missing_crs = tmp_path / "missing-crs.geojson"
    missing_crs.write_text(json.dumps({
        "type": "FeatureCollection",
        "features": [],
    }))
    with pytest.raises(ValueError, match="CRS"):
        MaGRoadPrior.load_geojson(missing_crs, expected_crs="EPSG:32651", model_version="magr-v1")

    invalid_geometry = tmp_path / "invalid-geometry.geojson"
    invalid_geometry.write_text(json.dumps({
        "type": "FeatureCollection",
        "crs": {"properties": {"name": "EPSG:32651"}},
        "features": [{"type": "Feature", "geometry": {
            "type": "LineString", "coordinates": [[0, 0], ["NaN", 1]]}},
    ]}))
    with pytest.raises(ValueError, match="finite"):
        MaGRoadPrior.load_geojson(invalid_geometry, expected_crs="EPSG:32651", model_version="magr-v1")


def test_persisted_evidence_rejects_bad_schema_and_invalid_measurements(tmp_path):
    store = EvidenceStore(GeoTransform("EPSG:32651", "WGS84", 0, 0, 1, 1), "v1")
    with pytest.raises(ValueError):
        store.add(RoadEvidence("bad", [(0, 0)], EvidenceState.OBSERVED_GEOMETRY,
                               1.0, "lidar", "s1", -0.1, 1.0))
    path = tmp_path / "bad-map.json"
    path.write_text(json.dumps({"schema": 99, "map_version": "v1",
                                "transform": {"crs": "EPSG:32651", "datum": "WGS84",
                                               "origin_x_m": 0, "origin_y_m": 0,
                                               "pixel_size_x_m": 1, "pixel_size_y_m": 1},
                                "evidence": []}))
    with pytest.raises(ValueError, match="schema"):
        EvidenceStore.load(path)


def test_evidence_store_rejects_unknown_map_identity():
    with pytest.raises(ValueError, match="map version"):
        EvidenceStore(GeoTransform("EPSG:32651", "WGS84", 0, 0, 1, 1), "UNKNOWN")


def test_evidence_store_rejects_uncontrolled_state():
    store = EvidenceStore(GeoTransform("EPSG:32651", "WGS84", 0, 0, 1, 1), "v1")
    with pytest.raises(ValueError, match="controlled vocabulary"):
        store.add(RoadEvidence("bad-state", [(0, 0), (1, 0)], "FREE", 1.0,
                               "lidar", "s1", 0.1, 1.0))
