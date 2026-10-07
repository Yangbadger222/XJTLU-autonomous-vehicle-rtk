import json
import math
from super_lio_vehicle_adapter.adapter_node import _navigation_reference_valid, _source_certificate, REFERENCE_CONTRACT


def certificate(**override):
    data=dict(source="super_lio/f89f48dc",certificate="fixed_extrinsic_observation_lower_bound_v1",
              twist_convention="imu_body_full_state_v1",
              stamp_ns=123456789,status="OK",minimum_observation_information=80.,
              legacy_min_eig_lower_bound=80.,effective_points=50,iterations=2,reason="valid")
    data.update(override)
    return json.dumps(data)


def test_original_navigation_point_is_supported_without_claiming_a_physical_extrinsic():
    assert _navigation_reference_valid("locked_fast_imu_origin",REFERENCE_CONTRACT,(0,0,0),(0,0,0,1))
    assert not _navigation_reference_valid("locked_fast_imu_origin",REFERENCE_CONTRACT,(.07,0,0),(0,0,0,1))
    assert not _navigation_reference_valid("locked_fast_imu_origin","unverified",(0,0,0),(0,0,0,1))


def test_actual_observation_bound_requires_measured_information_and_sufficient_features():
    assert _source_certificate(certificate())["eligible"]
    assert not _source_certificate(certificate(minimum_observation_information=74.,legacy_min_eig_lower_bound=74.))["eligible"]
    assert not _source_certificate(certificate(effective_points=49))["eligible"]
    assert not _source_certificate(certificate(status="FAIL"))["eligible"]
    assert not _source_certificate(certificate(iterations=0))["eligible"]


def test_fabricated_legacy_metric_nonfinite_and_headerless_healthy_are_rejected():
    assert _source_certificate("OK: healthy") is None
    assert _source_certificate(certificate(source="fast_lio2")) is None
    assert _source_certificate(certificate(minimum_observation_information=10.,legacy_min_eig_lower_bound=80.)) is None
    assert _source_certificate(certificate(minimum_observation_information=math.nan)) is None
    assert _source_certificate(certificate(stamp_ns=0)) is None
    assert _source_certificate(certificate(twist_convention="mixed_world_body")) is None


def test_deep_malformed_json_denies_health_without_terminating_the_node():
    assert _source_certificate('['*1100+']'*1100) is None
    assert _source_certificate(certificate(iterations=True)) is None
    assert _source_certificate(certificate(effective_points=50.5)) is None


def test_certificate_validation_does_not_depend_on_navigation_point_selection():
    from types import SimpleNamespace
    from super_lio_vehicle_adapter.adapter_node import SuperLioVehicleAdapter
    for legacy in (True,False):
        node=object.__new__(SuperLioVehicleAdapter)
        node._legacy_reference=legacy;node._pending_odom=None
        rejected=[];node._reject=rejected.append
        node._health_callback(SimpleNamespace(data="OK: unqualified"))
        assert not node._source_health_ok and node._source_certificate_stamp is None and rejected
        node._health_callback(SimpleNamespace(data=certificate()))
        assert node._source_health_ok and node._source_certificate_stamp==123456789


def test_unmatched_continuing_odom_expires_old_health_without_faulting_fresh_dds_ordering():
    import time
    from types import SimpleNamespace
    from super_lio_vehicle_adapter.adapter_node import SuperLioVehicleAdapter
    msg=SimpleNamespace(header=SimpleNamespace(frame_id='world',stamp=SimpleNamespace(sec=1,nanosec=0)),child_frame_id='imu')
    for legacy in (True,False):
        for age in (.01,1.):
            node=object.__new__(SuperLioVehicleAdapter)
            node._source_frame='world';node._source_child_frame='imu'
            node._legacy_reference=legacy;node._verified=not legacy
            node._source_health_ok=True;node._source_health_received=time.monotonic()-age
            node._source_certificate_stamp=900_000_000;node._reference_fault=''
            rejected=[];node._reject=rejected.append
            node._callback(msg)
            assert node._pending_odom is msg
            assert bool(rejected)==(age>.5)
            if rejected:assert 'expired' in rejected[-1]
