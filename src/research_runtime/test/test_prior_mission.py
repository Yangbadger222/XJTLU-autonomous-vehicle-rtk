from research_runtime.active_observation import RoadGraph,GraphEdge,prior_gap_events,supported_gap_updates
from research_runtime.prior_mission import transform_graph,confirmed_route_prefix
from research_runtime.grid_map import LocalObstacleGrid
from research_runtime.physical_parameter_lock import LOCKED_FOOTPRINT
import math


def fixture(cells=None):return LocalObstacleGrid('odom','map',.3,-3,-3,40,40,tuple(cells or [0]*1600))


def test_gap_endpoints_do_not_confirm_unknown_interior_or_width():
    graph=RoadGraph([GraphEdge('a','start','left',((0,0),(1,0))),GraphEdge('b','right','goal',((2,0),(3,0)))])
    event=next(e for e in prior_gap_events(graph) if set(e.endpoints)=={'left','right'})
    cells=[0]*1600;cells[10*40+15]=-1
    updated=supported_gap_updates(graph,[event],fixture(cells),LOCKED_FOOTPRINT)
    assert event.event_id not in updated.edges
    updated=supported_gap_updates(graph,[event],fixture(),LOCKED_FOOTPRINT)
    assert event.event_id in updated.edges and updated.edges[event.event_id].state=='OBSERVED_GEOMETRY'
    assert event.event_id not in graph.edges


def test_disconnected_task_approaches_existing_edge_without_unknown_straight_shortcut():
    graph=RoadGraph([GraphEdge('a','start','left',((0,0),(1,0))),GraphEdge('b','right','goal',((2,0),(3,0)))])
    points,reason=confirmed_route_prefix(graph,'start','goal',(0,0,0),fixture(),LOCKED_FOOTPRINT)
    assert points[-1]==(1,0) and reason=='UNCONFIRMED_GRAPH_CONNECTION'
    rotated=transform_graph(graph,(10,20,math.pi/2))
    assert math.dist(rotated.nodes['left'],(10,21))<1e-12
