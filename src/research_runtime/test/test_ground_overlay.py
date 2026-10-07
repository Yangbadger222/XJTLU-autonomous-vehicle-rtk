import pytest

from research_runtime.grid_map import LocalObstacleGrid, project_obstacle_points


def test_only_supported_ground_is_free_and_obstacles_take_precedence():
    mask = LocalObstacleGrid("odom", "v1", .3, 0, 0, 2, 2, (0,-1,0,0))
    projected, _ = project_obstacle_points([(.1,.1,.5)], map_version="v1",
        resolution_m=.3, origin_x_m=0, origin_y_m=0, width=2, height=2,
        obstacle_min_z_m=.08, obstacle_max_z_m=1.2, observed_ground=mask)
    assert projected.cells == (100,-1,0,0)
    with pytest.raises(ValueError, match="identity"):
        project_obstacle_points([], map_version="stale", resolution_m=.3,
            origin_x_m=0, origin_y_m=0, width=2, height=2,
            obstacle_min_z_m=.08, obstacle_max_z_m=1.2, observed_ground=mask)


def test_body_filtered_obstacle_keeps_odom_altitude_and_never_creates_free_space():
    grid,stats=project_obstacle_points([(.1,.1,100.)],map_version="v1",resolution_m=.3,
        origin_x_m=0,origin_y_m=0,width=2,height=2,obstacle_min_z_m=.08,obstacle_max_z_m=1.2,
        height_filter_applied=True)
    assert grid.cells==(100,-1,-1,-1) and stats.accepted_points==1
