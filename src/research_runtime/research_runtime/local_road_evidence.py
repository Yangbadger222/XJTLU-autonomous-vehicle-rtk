"""Bounded geometric strips from one positively supported acquisition.

The strips are observations, not semantic roads or travelled connections.
No interpolation crosses an unknown/blocked cell and no scan accumulation is
used. Integer cell identities suppress correlated repeat frames in a session.
"""
from dataclasses import dataclass
import hashlib
import json
import math
import numpy as np

from .grid_map import LocalObstacleGrid


@dataclass(frozen=True)
class SupportedStrip:
    identity: str
    geometry_xy: tuple[tuple[float,float],tuple[float,float]]
    width_m: float
    length_m: float
    frontier_xy: tuple[tuple[float,float], ...]


def pose_points_xy_uncertainty(covariance, origin, points) -> float:
    """Full pose propagation in the audited world small-angle convention.

    For r=p_world-p_origin, J_xy=[I_xy, -skew(r)_xy]. This retains all
    translation/roll/pitch/yaw covariance and cross terms. The result is a
    conditional one-sigma upper bound over the supplied points, not a claim
    of external LIO accuracy or sensor surface precision.
    """
    matrix=np.asarray(covariance,dtype=float)
    points=np.asarray(points,dtype=float)
    origin=np.asarray(origin,dtype=float)
    if matrix.size!=36 or points.ndim!=2 or points.shape[1]!=3 or not len(points) or origin.shape!=(3,):
        raise ValueError('full pose covariance and finite XYZ points required')
    matrix=matrix.reshape((6,6))
    if (not np.isfinite(matrix).all() or not np.isfinite(points).all() or not np.isfinite(origin).all() or
        not np.allclose(matrix,matrix.T,rtol=0,atol=1e-10) or np.linalg.eigvalsh(matrix)[0]<-1e-10):
        raise ValueError('pose covariance must be symmetric finite PSD')
    r=points-origin
    jacobian=np.zeros((len(points),2,6))
    jacobian[:,0,0]=jacobian[:,1,1]=1.
    jacobian[:,0,4],jacobian[:,0,5]=r[:,2],-r[:,1]
    jacobian[:,1,3],jacobian[:,1,5]=-r[:,2],r[:,0]
    propagated=jacobian@matrix@jacobian.transpose((0,2,1))
    return math.sqrt(max(0.,float(np.linalg.eigvalsh(propagated)[:,-1].max())))


def supported_strips(grid: LocalObstacleGrid, *, session: str, minimum_width_m: float,
                     maximum: int = 16) -> tuple[SupportedStrip, ...]:
    if (not session or session.upper() == 'UNKNOWN' or not math.isfinite(minimum_width_m) or
        minimum_width_m<=0 or type(maximum) is not int or not 1<=maximum<=64):
        raise ValueError('explicit session and bounded geometry policy required')
    tile_width=max(1,math.ceil(minimum_width_m/grid.resolution_m))
    results=[]
    # These two conservative directions cover axis-aligned support strips;
    # arbitrary-angle/semantic extraction is outside this first model.
    for vertical in (False,True):
        along,cross=(grid.height,grid.width) if vertical else (grid.width,grid.height)
        def cell(a,c):
            row,col=(a,c) if vertical else (c,a)
            return grid.cells[row*grid.width+col]
        for c in range(0,cross-tile_width+1,tile_width):
            start=None
            for a in range(along+1):
                free=a<along and all(cell(a,c+k)==0 for k in range(tile_width))
                if free and start is None:start=a
                if not free and start is not None:
                    if a-start>=2:
                        def xy(position):
                            u=position+.5;v=c+tile_width/2
                            x,y=(v,u) if vertical else (u,v)
                            return (grid.origin_x_m+x*grid.resolution_m,grid.origin_y_m+y*grid.resolution_m)
                        geometry=(xy(start),xy(a-1))
                        # Stable physical tiles, not rolling-window indices,
                        # stamp, map version, or the number of repeat frames.
                        key=[session,vertical,[[round(x/grid.resolution_m,6),round(y/grid.resolution_m,6)]
                                              for x,y in geometry],tile_width]
                        identity=hashlib.sha256(json.dumps(key,separators=(',',':')).encode()).hexdigest()
                        frontier=tuple(xy(end) for end in (start-1,a) if 0<=end<along and
                            any(cell(end,c+k)==-1 for k in range(tile_width)))
                        results.append(SupportedStrip(identity,geometry,tile_width*grid.resolution_m,
                            math.dist(*geometry),frontier))
                        if len(results)>=maximum:return tuple(results)
                    start=None
    return tuple(results)
