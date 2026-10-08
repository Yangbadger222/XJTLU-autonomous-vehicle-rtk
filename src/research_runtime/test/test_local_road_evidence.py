import json
import pytest
from research_runtime.grid_map import LocalObstacleGrid
from research_runtime.local_road_evidence import supported_strips,pose_points_xy_uncertainty
from research_runtime.active_road import EvidenceStore,GeoTransform,RoadEvidence,EvidenceState


def grid(cells,origin=(0.,0.)):
    return LocalObstacleGrid('odom','v1',.3,*origin,6,3,tuple(cells))


def test_strip_records_only_centers_inside_observed_width_and_length():
    g=grid([-1,0,0,0,0,-1]*3)
    strips=supported_strips(g,session='s1',minimum_width_m=.61)
    s=next(s for s in strips if s.geometry_xy[0][1] == s.geometry_xy[1][1])
    assert s.width_m == pytest.approx(.9)
    assert s.length_m == pytest.approx(.9)
    assert s.geometry_xy == ((.44999999999999996,.44999999999999996),(1.3499999999999999,.44999999999999996))
    assert len(s.frontier_xy)==2


def test_no_strip_bridges_unknown_hole_or_blocked_cell():
    cells=[0]*18;cells[8]=-1
    strips=supported_strips(grid(cells),session='s1',minimum_width_m=.61)
    assert all(not(a[0]<.75<b[0]) for a,b in (s.geometry_xy for s in strips) if a[1]==b[1])
    cells[8]=100
    blocked=supported_strips(grid(cells),session='s1',minimum_width_m=.61)
    assert [(s.identity,s.geometry_xy) for s in blocked]==[(s.identity,s.geometry_xy) for s in strips]
    assert not any(s.frontier_xy for s in blocked)


def test_empty_and_too_narrow_support_never_produce_a_road_observation():
    assert not supported_strips(grid([-1]*18),session='s1',minimum_width_m=.61)
    assert not supported_strips(grid([0]*6+[-1]*12),session='s1',minimum_width_m=.61)


def test_correlated_repeat_is_stable_but_new_session_is_independent():
    g=grid([0]*18)
    one=supported_strips(g,session='s1',minimum_width_m=.61)
    assert one==supported_strips(g,session='s1',minimum_width_m=.61)
    assert one[0].identity != supported_strips(g,session='s2',minimum_width_m=.61)[0].identity
    assert len(supported_strips(g,session='s1',minimum_width_m=.61,maximum=1))==1


def test_rolling_window_preserves_global_strip_identity_for_same_support():
    original=LocalObstacleGrid('odom','v1',.3,0.,0.,6,6,tuple([0]*36))
    # Shift the window north by one cell. The world-aligned upper 0.9 m
    # horizontal strip remains identical despite its new row indices.
    shifted=LocalObstacleGrid('odom','v2',.3,0.,.3,6,6,tuple([0]*36))
    a=supported_strips(original,session='s1',minimum_width_m=.61)
    b=supported_strips(shifted,session='s1',minimum_width_m=.61)
    common={s.identity for s in a if abs(s.geometry_xy[0][1]-1.35)<1e-9 and
            abs(s.geometry_xy[1][1]-1.35)<1e-9}
    assert common and common<={s.identity for s in b}


def test_width_roundtrip_and_old_unknown_width_records_remain_loadable(tmp_path):
    store=EvidenceStore(GeoTransform('LOCAL:odom','LOCAL_SENSOR_FRAME',0.,0.,1.,1.),'v1')
    old=RoadEvidence('old',[(0.,0.),(1.,0.)],EvidenceState.OBSERVED_GEOMETRY,1.,'source','s1/native',.02,1.)
    store.add(old);path=tmp_path/'old.json';store.save(path)
    assert 'supported_width_m' not in json.loads(path.read_text())['evidence'][0]
    assert EvidenceStore.load(path).map_version==store.map_version
    new=RoadEvidence('new',[(0.,0.),(1.,0.)],EvidenceState.OBSERVED_GEOMETRY,2.,'source','s1/native',.02,1.,supported_width_m=.9)
    store.add(new);store.save(path)
    assert EvidenceStore.load(path).evidence()[1].supported_width_m==.9


def test_width_may_not_be_nan_or_negative():
    for bad in (float('nan'),-.1,0.):
        with pytest.raises(ValueError):
            EvidenceStore._validate_evidence(RoadEvidence('e',[(0.,0.),(1.,0.)],EvidenceState.OBSERVED_GEOMETRY,
                1.,'source','submap',.1,1.,supported_width_m=bad))


def test_roll_pitch_and_position_orientation_cross_terms_cross_trust_threshold():
    import numpy as np
    matrix=np.eye(6)*1e-8
    matrix[0,0]=.0081;matrix[4,4]=.005
    matrix[0,4]=matrix[4,0]=-.9*(.0081*.005)**.5
    uncertainty=pose_points_xy_uncertainty(matrix.ravel(),(0.,0.,0.),[(.1,0.,-.40288)])
    expected=(.0081+.40288**2*.005+2*(-.40288)*matrix[0,4])**.5
    assert uncertainty == pytest.approx(expected,abs=1e-8)
    assert uncertainty>.10
    assert .09+.101**.5*1e-4<.10  # The former XY+sigma-yaw shortcut falsely passed.


def test_point_uncertainty_rejects_non_PSD_or_empty_contract():
    import numpy as np
    bad=np.eye(6);bad[0,1]=2.;bad[1,0]=2.
    with pytest.raises(ValueError):pose_points_xy_uncertainty(bad.ravel(),(0.,0.,0.),[(1.,0.,0.)])
    with pytest.raises(ValueError):pose_points_xy_uncertainty(np.eye(6).ravel(),(0.,0.,0.),[])
