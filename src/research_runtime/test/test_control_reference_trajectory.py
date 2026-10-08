from dataclasses import replace
from research_runtime.trajectory import TimedTrajectory, TimedPoint, VehicleLimits, validate_trajectory
from research_runtime.grid_map import LocalObstacleGrid
from research_runtime.physical_parameter_lock import LOCKED_FOOTPRINT
from super_lio_vehicle_adapter.control_reference_lock import IMU_TO_CONTROL_TRANSLATION_M


def test_missing_or_legacy_imu_trajectory_reference_cannot_reach_tracker():
    t=TimedTrajectory.from_points('control-test','map-v1','odom',1.,2.,
        (TimedPoint(0.,0.,0.,0.,0.,0.),TimedPoint(1.,0.,0.,0.,0.,0.)))
    assert validate_trajectory(t,VehicleLimits()).valid
    for contract in ('','corridor_e54c6af_fast_imu_origin_v1'):
        rejected=validate_trajectory(replace(t,control_reference_contract=contract),VehicleLimits())
        assert not rejected.valid and 'control_reference_contract_mismatch' in rejected.reasons


def test_recorded_control_origin_catches_offset_footprint_collision_hidden_at_imu():
    cells=[0]*100;cells[2*10+5]=100
    grid=LocalObstacleGrid('odom','map-v1',.3,-.9,-1.2,10,10,tuple(cells))
    polygon=lambda x,y:tuple((x+px,y+py) for px,py in LOCKED_FOOTPRINT)
    imu=(.23,.13)
    assert not grid.polygon_occupied(polygon(*imu),.075)
    r=IMU_TO_CONTROL_TRANSLATION_M;control=(imu[0]+r[0],imu[1]+r[1])
    assert grid.polygon_occupied(polygon(*control))
    t=TimedTrajectory.from_points('offset-footprint','map-v1','odom',1.,2.,
        (TimedPoint(0.,*control,0.,0.,0.),TimedPoint(1.,*control,0.,0.,0.)))
    result=validate_trajectory(t,VehicleLimits(),footprint=LOCKED_FOOTPRINT,
        occupied=grid.occupied,occupied_polygon=grid.polygon_occupied,resolution=.3)
    assert not result.valid
