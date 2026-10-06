from research_runtime.active_road import (EvidenceState, EvidenceStore, GeoTransform,
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
