import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parents[2] / "active_road_mapping"))

from active_road_mapping.evidence_node import _valid_version  # noqa: E402


def test_evidence_node_requires_known_persisted_map_version():
    assert not _valid_version("")
    assert not _valid_version("UNKNOWN")
    assert _valid_version("magr-v3")


def test_failed_persistence_never_acks_or_commits_memory_and_retry_succeeds(monkeypatch,tmp_path):
    from types import SimpleNamespace as NS
    from active_road_mapping import evidence_node as boundary
    from research_runtime.active_road import EvidenceStore,GeoTransform
    store=EvidenceStore(GeoTransform('LOCAL:odom','LOCAL_SENSOR_FRAME',0.,0.,1.,1.),'v1')
    path=tmp_path/'evidence.json';store.save(path)
    ack=[];events=[];rejections=[]
    node=NS(_store=store,_path=path,_store_mtime_ns=path.stat().st_mtime_ns,_last_rejection='',
        _ack_pub=NS(publish=lambda msg:ack.append(msg.data)),_publish_evidence_event=events.append,_reject=rejections.append)
    monkeypatch.setattr(boundary,'String',lambda **kwargs:NS(**kwargs),raising=False)
    msg=NS(header=NS(frame_id='odom',stamp=NS(sec=1,nanosec=0)),evidence_id='retryable',source='mid360',
        local_submap_id='session/native-odom',state='OBSERVED_GEOMETRY',pose_uncertainty_m=.01,
        observed_length_m=1.,valid_depth_min_m=.5,valid_depth_max_m=2.,supported_width_m=.9,
        geometry=[NS(x=0.,y=0.,z=0.),NS(x=1.,y=0.,z=0.)])
    real_save=boundary._atomic_save
    def denied(*args):raise PermissionError('controlled durable-write failure')
    monkeypatch.setattr(boundary,'_atomic_save',denied)
    for _ in range(2):boundary.ActiveRoadEvidenceNode._evidence_callback(node,msg)
    assert len(rejections)==2 and not ack and not events
    assert not node._store.evidence() and not EvidenceStore.load(path).evidence()
    monkeypatch.setattr(boundary,'_atomic_save',real_save)
    boundary.ActiveRoadEvidenceNode._evidence_callback(node,msg)
    assert ack==['retryable'] and len(events)==1
    assert len(node._store.evidence())==len(EvidenceStore.load(path).evidence())==1
    boundary.ActiveRoadEvidenceNode._evidence_callback(node,msg)
    assert ack==['retryable','retryable'] and len(EvidenceStore.load(path).evidence())==1


def test_reload_after_replacement_cannot_ack_a_directory_sync_failure(monkeypatch,tmp_path):
    from types import SimpleNamespace as NS
    import stat
    from active_road_mapping import evidence_node as boundary
    from research_runtime.active_road import EvidenceStore,GeoTransform
    store=EvidenceStore(GeoTransform('LOCAL:odom','LOCAL_SENSOR_FRAME',0.,0.,1.,1.),'v1')
    path=tmp_path/'evidence.json';store.save(path);ack=[];rejections=[]
    node=NS(_store=store,_path=path,_store_mtime_ns=path.stat().st_mtime_ns,_last_rejection='',
        _ack_pub=NS(publish=lambda msg:ack.append(msg.data)),_publish_evidence_event=lambda _:None,_reject=rejections.append)
    monkeypatch.setattr(boundary,'String',lambda **kwargs:NS(**kwargs),raising=False)
    msg=NS(header=NS(frame_id='odom',stamp=NS(sec=1,nanosec=0)),evidence_id='visible-but-unsynced',source='mid360',
        local_submap_id='session/native-odom',state='OBSERVED_GEOMETRY',pose_uncertainty_m=.01,
        observed_length_m=1.,valid_depth_min_m=.5,valid_depth_max_m=2.,supported_width_m=.9,
        geometry=[NS(x=0.,y=0.,z=0.),NS(x=1.,y=0.,z=0.)])
    real_sync=boundary.os.fsync
    def deny_directory(fd):
        if stat.S_ISDIR(boundary.os.fstat(fd).st_mode):raise OSError('controlled directory fsync failure')
        real_sync(fd)
    monkeypatch.setattr(boundary.os,'fsync',deny_directory)
    boundary.ActiveRoadEvidenceNode._evidence_callback(node,msg)
    assert not ack and not node._store.evidence() and len(EvidenceStore.load(path).evidence())==1
    node._store_mtime_ns=None
    boundary.ActiveRoadEvidenceNode._reload(node)
    assert len(node._store.evidence())==1
    boundary.ActiveRoadEvidenceNode._evidence_callback(node,msg)
    assert not ack and len(rejections)==2
    before=path.read_bytes();monkeypatch.setattr(boundary.os,'fsync',real_sync)
    boundary.ActiveRoadEvidenceNode._evidence_callback(node,msg)
    assert ack==['visible-but-unsynced'] and path.read_bytes()==before
