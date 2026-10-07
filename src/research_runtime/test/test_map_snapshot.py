from dataclasses import replace
import json

import pytest

from research_runtime.active_observation import FiniteObservationPolicy, load_snapshot, save_snapshot
from research_runtime.active_road import (EvidenceState, EvidenceStore, GeoTiffPrior, GeoTransform,
    MaGRoadPrior, RoadEvidence)


def evidence():
    return RoadEvidence("uuid", [(0,0),(1,0)], EvidenceState.OBSERVED_GEOMETRY,
                        1.0, "synthetic lidar", "submap", .1, 1, (.2, 2))


def test_uuid_collision_and_unseen_extension_are_rejected():
    store = EvidenceStore(GeoTransform("EPSG:32651", "WGS84", 0,0,1,1), "v1")
    store.add(evidence())
    assert not store.add(evidence())
    with pytest.raises(ValueError, match="UUID"):
        store.add(replace(evidence(), stamp=2))
    with pytest.raises(ValueError, match="beyond"):
        store.add(replace(evidence(), evidence_id="new", observed_length_m=10))


def test_full_affine_rotation_and_crop_preserve_georeferencing():
    prior = GeoTiffPrior("synthetic", "EPSG:32651", (0,-.2,100,.2,0,200), 100,100)
    point = prior.pixel_to_crs(3,4)
    assert prior.crs_to_pixel(*point) == pytest.approx((3,4))
    cropped = prior.cropped(2,3,10,10)
    assert cropped.pixel_to_crs(1,1) == pytest.approx(point)


def test_snapshot_reload_reuses_views_and_checks_prior_identity(tmp_path):
    prior_file = tmp_path / "prior.geojson"
    prior_file.write_text(json.dumps({"features": []}))
    prior = MaGRoadPrior(str(prior_file), "synthetic-model", "EPSG:32651", ())
    policy = FiniteObservationPolicy("TASK_AWARE_LOOK")
    policy.attempted.add("already-looked")
    snapshot = tmp_path / "overlay.json"
    version = save_snapshot(snapshot, prior, [evidence()], policy)
    restored = FiniteObservationPolicy("TASK_AWARE_LOOK")
    measurements, restored_version = load_snapshot(snapshot, prior, restored)
    assert restored_version == version and measurements == [evidence()]
    assert restored.attempted == {"already-looked"}
    assert save_snapshot(snapshot, prior, measurements, restored) == version
    prior_file.write_text("changed prior")
    with pytest.raises(ValueError, match="mismatch"):
        load_snapshot(snapshot, prior, restored)
