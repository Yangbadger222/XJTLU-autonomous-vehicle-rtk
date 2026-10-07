"""Positive, single-acquisition support evidence; unknown space stays blocked.

This is a deliberately limited flat-ground research model. A cell needs returns
in every subcell and a finite, full-rank local plane fit. Empty rays, sparse
returns and points below the expected floor cannot establish support. Thresholds
are new algorithm settings, not measurements of the chassis' terrain ability.
No support accumulates across scans or across localization sessions.
"""
from dataclasses import dataclass
import math
from typing import Iterable, Sequence

import numpy as np

from .grid_map import LocalObstacleGrid


@dataclass(frozen=True)
class GroundPolicy:
    subcells: int = 4
    floor_band_m: float = 0.05
    max_plane_residual_m: float = 0.025
    max_slope: float = 0.10
    max_neighbor_height_jump_m: float = 0.025
    obstacle_height_m: float = 1.20

    def __post_init__(self):
        if type(self.subcells) is not int or not 2 <= self.subcells <= 8:
            raise ValueError("ground subcells must be an integer in [2,8]")
        values = (self.floor_band_m, self.max_plane_residual_m,
                  self.max_slope, self.max_neighbor_height_jump_m, self.obstacle_height_m)
        if not all(math.isfinite(x) and x > 0 for x in values):
            raise ValueError("ground policy must be finite and positive")
        if self.max_plane_residual_m > self.floor_band_m:
            raise ValueError("residual exceeds floor band")


def expected_ground_z(position, quaternion, lidar_in_imu, lidar_height_m):
    """z_floor = p_WI.z + (R_WI t_IL).z - recorded LiDAR mounting height.

    This uses the existing IMU navigation origin and factory LiDAR/IMU lever
    arm. It assumes the documented level mounting height, rather than claiming
    an IMU-to-chassis-center calibration. Tilted/unsupported terrain is rejected
    by the separate local surface test.
    """
    values = (*position, *quaternion, *lidar_in_imu, lidar_height_m)
    if not all(math.isfinite(float(x)) for x in values) or lidar_height_m <= 0:
        raise ValueError("invalid recorded ground reference")
    x, y, z, w = quaternion
    if abs(x*x+y*y+z*z+w*w-1.) > 1e-6:
        raise ValueError("ground reference requires a unit quaternion")
    tx, ty, tz = lidar_in_imu
    rotated_z = 2*(x*z-w*y)*tx + 2*(y*z+w*x)*ty + (1-2*(x*x+y*y))*tz
    return position[2]+rotated_z-lidar_height_m


def project_observed_ground(points: Iterable[Sequence[float]], *, map_version,
                            resolution_m, origin_x_m, origin_y_m, width, height,
                            floor_z_m, policy=GroundPolicy()):
    grid = LocalObstacleGrid("odom", map_version, resolution_m, origin_x_m,
        origin_y_m, width, height, (-1,)*(width*height), True)
    if not math.isfinite(floor_z_m):
        raise ValueError("ground reference height must be finite")
    groups = {}
    planes = {}
    blocked = set()
    stats = dict(finite_points=0, nonfinite_points=0, out_of_window=0,
                 supported_cells=0, blocked_cells=0, sparse_cells=0,
                 rejected_planes=0, subcell_width_m=resolution_m/policy.subcells)
    for point in points:
        x, y, z = map(float, (point[0],point[1],point[2]))
        if not all(math.isfinite(v) for v in (x,y,z)):
            stats["nonfinite_points"] += 1
            continue
        stats["finite_points"] += 1
        index = grid._index(x,y)
        if index is None:
            stats["out_of_window"] += 1
            continue
        dz = z-floor_z_m
        if dz < -policy.floor_band_m or policy.floor_band_m < dz <= policy.obstacle_height_m:
            blocked.add(index)  # Includes low obstacles the old height window excludes.
        elif abs(dz) <= policy.floor_band_m:
            groups.setdefault(index, []).append((x,y,z))
    cells = list(grid.cells)
    for index, points_cell in groups.items():
        if index in blocked:
            continue
        row, col = divmod(index,width)
        left, bottom = origin_x_m+col*resolution_m, origin_y_m+row*resolution_m
        coverage = {(min(policy.subcells-1,int((x-left)/resolution_m*policy.subcells)),
                     min(policy.subcells-1,int((y-bottom)/resolution_m*policy.subcells)))
                    for x,y,_z in points_cell}
        if len(coverage) != policy.subcells**2:
            stats["sparse_cells"] += 1
            continue
        # Work in cell-relative coordinates to avoid poor conditioning at large
        # odom coordinates. The fit never extrapolates a plane into other cells.
        samples = np.asarray(points_cell,dtype=float)
        xy = samples[:,:2]-np.asarray((left,bottom))
        design = np.column_stack((xy,np.ones(len(samples))))
        coefficients, _residual, rank, _singular = np.linalg.lstsq(design,samples[:,2]-floor_z_m,rcond=None)
        residual = float(np.max(np.abs(design@coefficients-(samples[:,2]-floor_z_m))))
        if (rank != 3 or not np.all(np.isfinite(coefficients)) or
            residual > policy.max_plane_residual_m or
            math.hypot(*coefficients[:2]) > policy.max_slope):
            stats["rejected_planes"] += 1
            continue
        cells[index] = 0
        planes[index] = (left,bottom,coefficients)
        stats["supported_cells"] += 1
    # A good plane inside each cell does not establish a connection between
    # them: disjoint flat surfaces can hide a step. Check the shared boundary
    # and centre-to-centre slope for both orthogonal neighbors. A discontinuity
    # blocks both cells, so neither footprint sweep can bridge that step.
    for index,(left,bottom,coefficients) in planes.items():
        row,col = divmod(index,width)
        for neighbor,dx,dy in ((index+1,1,0),(index+width,0,1)):
            if (dx and col+1>=width) or (dy and row+1>=height) or neighbor not in planes:
                continue
            other_left,other_bottom,other_coefficients = planes[neighbor]
            def z_at(x,y,px,py,c):
                return c[0]*(x-px)+c[1]*(y-py)+c[2]
            edge = ((left+resolution_m,bottom),(left+resolution_m,bottom+resolution_m)) if dx else (
                (left,bottom+resolution_m),(left+resolution_m,bottom+resolution_m))
            jump = max(abs(z_at(x,y,left,bottom,coefficients)-
                z_at(x,y,other_left,other_bottom,other_coefficients)) for x,y in edge)
            center = z_at(left+resolution_m/2,bottom+resolution_m/2,left,bottom,coefficients)
            other_center = z_at(other_left+resolution_m/2,other_bottom+resolution_m/2,
                other_left,other_bottom,other_coefficients)
            if (jump > policy.max_neighbor_height_jump_m or
                abs(other_center-center)/resolution_m > policy.max_slope):
                blocked.update((index,neighbor))
    for index in blocked:
        cells[index] = 100
    stats["supported_cells"] = cells.count(0)
    stats["blocked_cells"] = len(blocked)
    return LocalObstacleGrid("odom",map_version,resolution_m,origin_x_m,origin_y_m,
                             width,height,tuple(cells),True), stats
