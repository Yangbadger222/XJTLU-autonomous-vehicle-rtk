import math
import json
import pytest
from research_runtime.active_road import EvidenceStore,GeoTransform,RoadEvidence,EvidenceState


def test_recovered_global_authority_reanchors_without_duplicate_road(tmp_path):
    store=EvidenceStore(GeoTransform('LOCAL_ODOM_SUBMAP','local',0,0,.3,.3),'prior')
    evidence=RoadEvidence('id',[(1,0),(2,0)],EvidenceState.OBSERVED_GEOMETRY,1.,'depth','submap',.1,1.)
    assert store.add(evidence) and store.geometry_in_map('id') is None
    store.anchor_submap('submap',(10,20,math.pi/2),stamp=2.,uncertainty_m=.2,authority_valid=True)
    assert store.geometry_in_map('id')==[(10.,21.),(10.,22.)]
    assert store.mark_anchors_stale() and store.geometry_in_map('id') is None
    store.anchor_submap('submap',(11,20,math.pi/2),stamp=3.,uncertainty_m=.2,authority_valid=True)
    assert len(store.evidence())==1 and store.evidence()[0].geometry_xy==[(1,0),(2,0)]
    file=tmp_path/'store.json';store.save(file)
    restored=EvidenceStore.load(file)
    assert restored.map_version==store.map_version
    assert restored.geometry_in_map('id')==[(11.,21.),(11.,22.)]
    assert not restored.add(evidence)


def test_authority_loss_or_unknown_uncertainty_cannot_write_global_anchor():
    store=EvidenceStore(GeoTransform('LOCAL_ODOM_SUBMAP','local',0,0,.3,.3),'prior')
    with pytest.raises(ValueError):store.anchor_submap('submap',(0,0,0),stamp=1.,uncertainty_m=0.,authority_valid=False)
    with pytest.raises(ValueError):store.anchor_submap('submap',(0,0,0),stamp=1.,uncertainty_m=-1.,authority_valid=True)


def test_content_version_detects_changed_measurement_and_external_mutation(tmp_path):
    store=EvidenceStore(GeoTransform('LOCAL_ODOM_SUBMAP','local',0,0,.3,.3),'prior')
    evidence=RoadEvidence('id',[(0,0),(1,0)],EvidenceState.OBSERVED_GEOMETRY,1.,'depth','submap',.1,1.)
    old=store.map_version;store.add(evidence);assert store.map_version!=old
    evidence.geometry_xy[0]=(100,100)
    assert store.evidence()[0].geometry_xy[0]==(0,0)
    file=tmp_path/'map.json';store.save(file)
    payload=json.loads(file.read_text());payload['evidence'][0]['pose_uncertainty_m']=.2
    file.write_text(json.dumps(payload))
    with pytest.raises(ValueError,match='version mismatch'):EvidenceStore.load(file)
