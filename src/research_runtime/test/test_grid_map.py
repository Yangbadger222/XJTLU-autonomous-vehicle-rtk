import pytest

from research_runtime.grid_map import project_obstacle_points


def test_projection_marks_only_measured_obstacle_heights_and_keeps_unknown_blocked():
    grid, stats = project_obstacle_points(
        [(0.2, 0.2, 0.2), (0.2, 0.2, 0.05), (1.8, 1.8, 0.3),
         (float("nan"), 0.0, 0.2)],
        map_version="m1", resolution_m=0.5, origin_x_m=0.0, origin_y_m=0.0,
        width=2, height=2, obstacle_min_z_m=0.1, obstacle_max_z_m=0.4)
    assert stats.accepted_points == 1
    assert stats.rejected_height == 1
    assert stats.rejected_out_of_bounds == 1
    assert stats.rejected_nonfinite == 1
    assert grid.value_at(0.2, 0.2) == 100
    assert grid.occupied(0.7, 0.7)  # unobserved is not free


def test_projection_requires_valid_height_and_resolution_contract():
    with pytest.raises(ValueError):
        project_obstacle_points([], map_version="", resolution_m=0.5,
                                origin_x_m=0.0, origin_y_m=0.0, width=2, height=2,
                                obstacle_min_z_m=0.4, obstacle_max_z_m=0.1)
    with pytest.raises(ValueError):
        project_obstacle_points([], map_version="m1", resolution_m=0.0,
                                origin_x_m=0.0, origin_y_m=0.0, width=2, height=2,
                                obstacle_min_z_m=0.1, obstacle_max_z_m=0.4)
    with pytest.raises(ValueError):
        project_obstacle_points([], map_version="m1", resolution_m=0.5,
                                origin_x_m=0.0, origin_y_m=0.0, width=2, height=2,
                                obstacle_min_z_m=0.1, obstacle_max_z_m=0.4,
                                unknown_is_occupied=False)
