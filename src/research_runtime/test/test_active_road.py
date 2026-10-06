import json

import pytest

from research_runtime.active_road import (EvidenceState, EvidenceStore, GeoTransform, MaGRoadPrior,
                                           ObservationCandidate, RoadEvidence, choose_observation)


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
