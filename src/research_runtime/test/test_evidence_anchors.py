
def test_authority_loss_does_not_invalidate_an_unrelated_historical_session():
    from research_runtime.active_road import EvidenceStore,GeoTransform
    store=EvidenceStore(GeoTransform('LOCAL_ODOM_SUBMAP','local',0,0,.3,.3),'prior')
    store.anchor_submap('old/session',(11.,20.,0.),stamp=1.,uncertainty_m=.03,authority_valid=True)
    store.anchor_submap('new/session',(100.,200.,0.),stamp=2.,uncertainty_m=.03,authority_valid=True)
    assert store.mark_anchors_stale({'new/session'})
    assert store.submap_anchors['old/session']['state']=='ANCHORED'
    assert store.submap_anchors['old/session']['map_from_local_xyyaw']==[11.,20.,0.]
    assert store.submap_anchors['new/session']['state']=='STALE'
