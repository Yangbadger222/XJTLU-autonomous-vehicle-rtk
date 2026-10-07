"""Conservative 3-D obstacle evidence projection into a local 2-D grid.

The projector deliberately does not infer free space from missing returns. It
accepts points that are already expressed in the declared odom frame and
marks only measured obstacle cells. Unobserved cells remain ``-1`` and are
occupied by the safety query when ``unknown_is_occupied`` is enabled.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Sequence


def _finite(values: Sequence[float]) -> bool:
    return all(math.isfinite(float(value)) for value in values)


@dataclass(frozen=True)
class LocalObstacleGrid:
    """A row-major OccupancyGrid-compatible local obstacle map."""

    frame_id: str
    map_version: str
    resolution_m: float
    origin_x_m: float
    origin_y_m: float
    width: int
    height: int
    cells: tuple[int, ...]
    unknown_is_occupied: bool = True

    def __post_init__(self) -> None:
        if self.frame_id != "odom":
            raise ValueError("local obstacle grid must be expressed in odom")
        if not self.map_version:
            raise ValueError("map_version is required")
        if not math.isfinite(self.resolution_m) or self.resolution_m <= 0.0:
            raise ValueError("resolution_m must be positive and finite")
        if not _finite((self.origin_x_m, self.origin_y_m)):
            raise ValueError("grid origin must be finite")
        if self.width <= 0 or self.height <= 0:
            raise ValueError("grid dimensions must be positive")
        if not self.unknown_is_occupied:
            raise ValueError("unknown occupancy must remain blocked for safety")
        if len(self.cells) != self.width * self.height:
            raise ValueError("cells must be row-major width*height")
        if any(value not in (-1, 0, 100) for value in self.cells):
            raise ValueError("cells must use -1, 0 or 100 occupancy values")

    def _index(self, x_m: float, y_m: float) -> int | None:
        if not _finite((x_m, y_m)):
            return None
        col = math.floor((x_m - self.origin_x_m) / self.resolution_m)
        row = math.floor((y_m - self.origin_y_m) / self.resolution_m)
        if col < 0 or row < 0 or col >= self.width or row >= self.height:
            return None
        return row * self.width + col

    def value_at(self, x_m: float, y_m: float) -> int:
        index = self._index(x_m, y_m)
        return -1 if index is None else self.cells[index]

    def occupied(self, x_m: float, y_m: float) -> bool:
        value = self.value_at(x_m, y_m)
        return value == 100 or (value == -1 and self.unknown_is_occupied)

    def polygon_occupied(self, polygon, margin=0.0) -> bool:
        """Test the whole convex footprint against cells, including boundaries.

        ``margin`` enlarges each obstacle cell on all sides. The trajectory
        checker uses a bound on translation plus corner rotation between
        samples, so this conservative cell test covers the continuous sweep.
        """
        from .footprint import convex_hull, convex_polygons_intersect
        hull = convex_hull(polygon)
        if len(hull) < 3 or not math.isfinite(margin) or margin < 0.0:
            return True
        xs, ys = zip(*hull)
        col0 = math.floor((min(xs) - margin - self.origin_x_m) / self.resolution_m)
        col1 = math.floor((max(xs) + margin - self.origin_x_m) / self.resolution_m)
        row0 = math.floor((min(ys) - margin - self.origin_y_m) / self.resolution_m)
        row1 = math.floor((max(ys) + margin - self.origin_y_m) / self.resolution_m)
        for row in range(row0, row1 + 1):
            for col in range(col0, col1 + 1):
                unknown = row < 0 or col < 0 or row >= self.height or col >= self.width
                if not unknown and self.cells[row * self.width + col] == 0:
                    continue
                x0 = self.origin_x_m + col * self.resolution_m - margin
                y0 = self.origin_y_m + row * self.resolution_m - margin
                x1 = x0 + self.resolution_m + 2.0 * margin
                y1 = y0 + self.resolution_m + 2.0 * margin
                if convex_polygons_intersect(hull, ((x0, y0), (x1, y0), (x1, y1), (x0, y1))):
                    return True
        return False


@dataclass(frozen=True)
class ProjectionStats:
    accepted_points: int
    rejected_nonfinite: int
    rejected_height: int
    rejected_out_of_bounds: int


def project_obstacle_points(
    points_odom_xyz: Iterable[Sequence[float]],
    *,
    map_version: str,
    resolution_m: float,
    origin_x_m: float,
    origin_y_m: float,
    width: int,
    height: int,
    obstacle_min_z_m: float,
    obstacle_max_z_m: float,
    unknown_is_occupied: bool = True,
    observed_ground: LocalObstacleGrid | None = None,
    height_filter_applied: bool = False,
) -> tuple[LocalObstacleGrid, ProjectionStats]:
    """Project a measured odom-frame cloud without inventing free space.

    Height limits are explicit inputs from the locked cloud contract. A point
    at either bound is included. Points outside the local window are counted
    but cannot make an out-of-window query appear free.
    """
    if not _finite((obstacle_min_z_m, obstacle_max_z_m)) or obstacle_min_z_m > obstacle_max_z_m:
        raise ValueError("obstacle height window must be finite and ordered")
    if not unknown_is_occupied:
        raise ValueError("unknown occupancy must remain blocked for safety")
    cells = [-1] * (width * height)
    accepted = rejected_nonfinite = rejected_height = rejected_out_of_bounds = 0
    grid = LocalObstacleGrid("odom", map_version, resolution_m, origin_x_m, origin_y_m,
                             width, height, tuple(cells), unknown_is_occupied)
    if observed_ground is not None:
        fields = ("frame_id", "map_version", "resolution_m", "origin_x_m", "origin_y_m", "width", "height")
        if any(getattr(grid, field) != getattr(observed_ground, field) for field in fields):
            raise ValueError("observed ground must match the exact grid identity and geometry")
        cells = list(observed_ground.cells)
        grid = LocalObstacleGrid("odom", map_version, resolution_m, origin_x_m, origin_y_m,
                                 width, height, tuple(cells), unknown_is_occupied)
    mutable = list(grid.cells)
    for point in points_odom_xyz:
        if len(point) != 3 or not _finite(point):
            rejected_nonfinite += 1
            continue
        x_m, y_m, z_m = (float(point[0]), float(point[1]), float(point[2]))
        if not height_filter_applied and (z_m < obstacle_min_z_m or z_m > obstacle_max_z_m):
            rejected_height += 1
            continue
        index = grid._index(x_m, y_m)
        if index is None:
            rejected_out_of_bounds += 1
            continue
        mutable[index] = 100
        accepted += 1
    return (LocalObstacleGrid(grid.frame_id, grid.map_version, grid.resolution_m,
                              grid.origin_x_m, grid.origin_y_m, grid.width,
                              grid.height, tuple(mutable), grid.unknown_is_occupied),
            ProjectionStats(accepted, rejected_nonfinite, rejected_height,
                            rejected_out_of_bounds))
