import math

from research_runtime.grid_map import LocalObstacleGrid
from research_runtime.trajectory import TimedPoint, TimedTrajectory, VehicleLimits, validate_trajectory


def grid_with_cell(x, y):
    cells = [0] * 400
    cells[int((y + 1.0) / 0.1) * 20 + int((x + 1.0) / 0.1)] = 100
    return LocalObstacleGrid("odom", "m1", 0.1, -1.0, -1.0, 20, 20, tuple(cells))


def check(grid, points):
    trajectory = TimedTrajectory.from_points("sweep", "m1", "odom", 0, 10, points)
    footprint = ((0.33, 0.305), (0.33, -0.305), (-0.33, -0.305), (-0.33, 0.305))
    return validate_trajectory(trajectory, VehicleLimits(), footprint=footprint,
                               occupied=grid.occupied, occupied_polygon=grid.polygon_occupied,
                               resolution=grid.resolution_m)


def test_obstacle_inside_footprint_is_collision_when_all_corners_are_free():
    grid = grid_with_cell(0.0, 0.0)
    assert not check(grid, [TimedPoint(0, 0, 0, 0, 0, 0)]).valid


def test_corner_rotation_sweep_is_checked_with_zero_translation():
    grid = grid_with_cell(0.40, 0.0)
    result = check(grid, [TimedPoint(0, 0, 0, -math.pi / 4, 0, 0.5),
                          TimedPoint(math.pi, 0, 0, math.pi / 4, 0, 0.5)])
    assert "footprint_collision_or_unknown" in result.reasons


def test_unknown_and_outside_cells_block_the_entire_polygon():
    grid = LocalObstacleGrid("odom", "m1", 0.1, -1, -1, 20, 20, (-1,) * 400)
    assert not check(grid, [TimedPoint(0, 0, 0, 0, 0, 0)]).valid
    free = LocalObstacleGrid("odom", "m1", 0.1, -1, -1, 20, 20, (0,) * 400)
    assert check(free, [TimedPoint(0, 0, 0, 0, 0, 0)]).valid
    assert not check(free, [TimedPoint(0, 0.9, 0, 0, 0, 0)]).valid


def test_between_sample_translation_intersects_cell_without_vertex_contact():
    grid = grid_with_cell(0.45, 0.0)
    result = check(grid, [TimedPoint(0, -0.3, 0, 0, 0.4, 0),
                          TimedPoint(2, 0.5, 0, 0, 0.4, 0)])
    assert "footprint_collision_or_unknown" in result.reasons
