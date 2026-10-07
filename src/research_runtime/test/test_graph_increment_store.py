import math
import pytest
from research_runtime.active_road import EvidenceStore,GeoTransform,RoadEvidence,EvidenceState


def fixture():
    store=EvidenceStore(GeoTransform('LOCAL_ODOM_SUBMAP','local',0,0,.3,.3),'prior')
    store.add(RoadEvidence('sensor-uuid',[(0,0),(1,0)],EvidenceState.OBSERVED_GEOMETRY,1.,'depth','submap',.03,1.))
    update=dict(update_id='gap:a:b',prior_version='prior',start_node_id='a',end_node_id='b',
        geometry_xy=[(0.,0.),(1.,0.)],stamp=2.,supported_width_m=.61,source='supported_ground',
        local_submap_id='submap',pose_uncertainty_m=.03,evidence_ids=['sensor-uuid'])
    return store,update


def test_graph_increments_are_persistent_anchored_and_idempotent(tmp_path):
    store,update=fixture();assert store.add_graph_update(update)
    assert not store.add_graph_update(update) and not store.graph_updates_in_map()
    store.anchor_submap('submap',(10,20,math.pi/2),stamp=3.,uncertainty_m=.03,authority_valid=True)
    assert store.graph_updates_in_map()[0]['geometry_xy']==[(10.,20.),(10.,21.)]
    path=tmp_path/'evidence.json';store.save(path);restored=EvidenceStore.load(path)
    assert restored.map_version==store.map_version and len(restored.graph_updates)==1
    assert restored.mark_anchors_stale() and not restored.graph_updates_in_map()
    restored.anchor_submap('submap',(11,20,math.pi/2),stamp=4.,uncertainty_m=.03,authority_valid=True)
    assert restored.graph_updates_in_map()[0]['geometry_xy']==[(11.,20.),(11.,21.)]
    assert len(restored.evidence())==1 and len(restored.graph_updates)==1


def test_graph_update_cannot_invent_sensor_uuid_prior_or_replay_identity():
    store,update=fixture()
    with pytest.raises(ValueError):store.add_graph_update({**update,'prior_version':'different'})
    with pytest.raises(ValueError):store.add_graph_update({**update,'evidence_ids':['invented']})
    store.add_graph_update(update)
    with pytest.raises(ValueError,match='UUID'):store.add_graph_update({**update,'stamp':3.})


def test_rollback_is_persistent_and_retains_all_sensor_and_increment_provenance(tmp_path):
    store,update=fixture();store.add_graph_update(update)
    store.anchor_submap('submap',(0,0,0),stamp=3.,uncertainty_m=.03,authority_valid=True)
    before=store.map_version
    store.rollback_graph_update(update['update_id'],reason='contradicting measured support',stamp=4.)
    assert not store.graph_updates_in_map() and len(store.graph_updates)==len(store.evidence())==1
    assert store.map_version!=before and store.prior_version=='prior'
    path=tmp_path/'snapshot.json';store.save(path);restored=EvidenceStore.load(path)
    assert restored.map_version==store.map_version and not restored.graph_updates_in_map()
    assert restored.graph_updates[update['update_id']]['evidence_ids']==['sensor-uuid']
