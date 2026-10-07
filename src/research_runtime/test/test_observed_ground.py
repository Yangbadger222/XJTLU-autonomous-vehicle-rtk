import math
import numpy as np
import pytest

from research_runtime.observed_ground import expected_ground_z, project_observed_ground
from research_runtime.physical_parameter_lock import MID360_GROUND_REFERENCE


def patch(z=-.40288):
    return [(.0375+.075*i,.0375+.075*j,z) for i in range(4) for j in range(4)]


def project(points):
    return project_observed_ground(points,map_version="fixture-v1",resolution_m=.30,
        origin_x_m=0.,origin_y_m=0.,width=2,height=1,floor_z_m=-.40288)


def test_positive_dense_plane_and_missing_neighbor():
    grid,stats = project(patch())
    assert grid.cells == (0,-1) and stats["supported_cells"] == 1
    assert grid.occupied(.45,.15)


def test_sparse_or_missing_downward_returns_never_establish_support():
    for points in ([],patch()[:-1],[patch()[0]]*1000,[(0.,0.,float('nan'))]):
        assert project(points)[0].cells == (-1,-1)


@pytest.mark.parametrize("hazard_height",[-.20,.06,.5])
def test_holes_steps_and_low_obstacles_override_good_ground(hazard_height):
    grid,_ = project(patch()+[(.15,.15,-.40288+hazard_height)])
    assert grid.cells == (100,-1)


def test_rough_or_too_steep_ground_is_not_free():
    points = [(x,y,z+(0.04 if i%2 else -0.04)) for i,(x,y,z) in enumerate(patch())]
    assert project(points)[0].cells[0] != 0
    points = [(x,y,z+.2*(x-.15)) for x,y,z in patch()]
    assert project(points)[0].cells[0] != 0


def test_sensor_msgs_structured_point_arrays_are_accepted():
    points = np.array(patch(),dtype=[('x','f4'),('y','f4'),('z','f4')])
    assert project(points)[0].cells == (0,-1)


def test_recorded_height_uses_factory_lever_and_acquisition_rotation():
    reference = MID360_GROUND_REFERENCE
    lever = reference['lidar_in_imu_m']
    assert expected_ground_z((0,0,0),(0,0,0,1),lever,reference['lidar_height_m']) == pytest.approx(-.40288)
    assert expected_ground_z((0,0,10),(0,math.sin(math.pi/4),0,math.cos(math.pi/4)),
        lever,reference['lidar_height_m']) == pytest.approx(10+.011-.447)
    with pytest.raises(ValueError):
        expected_ground_z((0,0,0),(0,0,0,0),lever,reference['lidar_height_m'])


def test_no_accumulation_of_support_from_preceding_scan():
    assert project(patch())[0].cells[0] == 0
    assert project(patch()[:-1])[0].cells[0] == -1


def test_adjacent_flat_surfaces_cannot_hide_a_nine_cm_step():
    points = [(x,y,z-.049) for x,y,z in patch()]
    points += [(x+.30,y,z+.049) for x,y,z in patch()]
    grid,_ = project(points)
    assert grid.cells == (100,100)
