"""Read-only metric MaGRoad/GeoTIFF registration and confirmed local route."""
from __future__ import annotations
import hashlib
import json
import math
from pathlib import Path
from dataclasses import replace
from .active_road import GeoTiffPrior, MaGRoadPrior
from .active_observation import RoadGraph, distance


def registered_prior(manifest_path):
    path = Path(manifest_path)
    manifest = json.loads(path.read_text())
    if manifest.get('schema') != 1 or manifest.get('map_registration_verified') is not True:
        raise ValueError('an explicitly verified CRS-to-existing-map registration is required')
    def asset(key):
        candidate = Path(manifest[key])
        candidate = candidate if candidate.is_absolute() else path.parent/candidate
        if hashlib.sha256(candidate.read_bytes()).hexdigest() != manifest[key+'_sha256']:
            raise ValueError('prior asset hash mismatch: '+key)
        return candidate
    prior = MaGRoadPrior.load_geojson(asset('road_geojson'), expected_crs=manifest['crs'],
                                      model_version=manifest['model_version'])
    raster = GeoTiffPrior.load(asset('geotiff'))
    from pyproj import CRS
    if CRS.from_user_input(raster.crs) != CRS.from_user_input(prior.crs):
        raise ValueError('raster/vector CRS mismatch; explicit reprojection is required')
    transform = tuple(manifest['map_from_crs_xyyaw'])
    if len(transform) != 3 or not all(math.isfinite(v) for v in transform):
        raise ValueError('finite map_from_crs rigid registration required')
    # RoadGraph.from_prior verifies projected SI metric units. No datum/origin
    # is changed on the original RTK authority node.
    graph = transform_graph(RoadGraph.from_prior(prior), transform)
    return prior, raster, graph, manifest


def transform_graph(graph, xy_yaw):
    tx,ty,yaw = xy_yaw
    c,s = math.cos(yaw),math.sin(yaw)
    return RoadGraph([replace(edge,geometry=tuple((tx+c*x-s*y,ty+s*x+c*y)
        for x,y in edge.geometry)) for edge in graph.edges.values()])


def confirmed_route_prefix(graph, start, goal, pose, grid, footprint):
    """A graph route or current start edge, trimmed to observed safe support.

    If disconnected, do not invent a straight start-goal connection. The
    current component may approach its own boundary along its existing road.
    """
    cost,path = graph.route(start,goal)
    if not path:
        path = tuple(edge.edge_id for edge in graph.edges.values()
                     if start in (edge.start,edge.end))[:1]
    if not path:
        return (), 'UNCONFIRMED_GRAPH_CONNECTION'
    points, node = [], start
    for identity in path:
        edge = graph.edges[identity]
        geometry = edge.geometry if edge.start == node else tuple(reversed(edge.geometry))
        points.extend(geometry if not points else geometry[1:])
        node = edge.end if edge.start == node else edge.start
    # Trim driven prefix using closest projection on an existing segment.
    nearest = None
    for index,(a,b) in enumerate(zip(points,points[1:])):
        dx,dy=b[0]-a[0],b[1]-a[1]
        length=dx*dx+dy*dy
        t=max(0.,min(1.,((pose[0]-a[0])*dx+(pose[1]-a[1])*dy)/length)) if length else 0.
        projection=(a[0]+t*dx,a[1]+t*dy)
        value=(distance(pose,projection),index,projection)
        if nearest is None or value[0]<nearest[0]:nearest=value
    if nearest is None or nearest[0]>.30:return (), 'POSE_OUTSIDE_CONFIRMED_ROAD'
    remaining=[nearest[2]]+points[nearest[1]+1:]
    result=[]
    # EGO uses the original footprint's circumscribed circle against complete
    # occupied-cell AABBs. A merely polygon-free reference endpoint can still
    # be rejected by that existing planner contract. Match its envelope here.
    radius=max(math.sqrt(x*x+y*y) for x,y in footprint)
    for a,b in zip(remaining,remaining[1:]):
        yaw=math.atan2(b[1]-a[1],b[0]-a[0]);c,s=math.cos(yaw),math.sin(yaw)
        steps=max(1,math.ceil(distance(a,b)/(grid.resolution_m*.25)))
        sweep_margin=distance(a,b)/steps*.5
        for i in range(steps+1):
            xy=(a[0]+(b[0]-a[0])*i/steps,a[1]+(b[1]-a[1])*i/steps)
            polygon=[(xy[0]+c*x-s*y,xy[1]+s*x+c*y) for x,y in footprint]
            if grid.polygon_occupied(polygon) or grid.disk_occupied(xy[0],xy[1],radius+sweep_margin):
                return tuple(result), 'LOCAL_SUPPORT_BOUNDARY'
            if not result or distance(result[-1],xy)>1e-7:result.append(xy)
    return tuple(result), 'PRIOR_ROUTE_WITH_CONFIRMED_LOCAL_PREFIX' if math.isfinite(cost) else 'UNCONFIRMED_GRAPH_CONNECTION'
