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


def test_reference_stops_before_polygon_free_but_ego_circle_blocked_endpoint():
    cells=[0 if col<16 else -1 for row in range(40) for col in range(40)]
    grid=fixture(cells)
    graph=RoadGraph([GraphEdge('road','start','goal',((0.,0.),(3.,0.)))])
    end=(1.4,0.)
    assert not grid.polygon_occupied([(end[0]+x,end[1]+y) for x,y in LOCKED_FOOTPRINT])
    radius=max(math.sqrt(x*x+y*y) for x,y in LOCKED_FOOTPRINT)
    assert grid.disk_occupied(*end,radius)
    prefix,reason=confirmed_route_prefix(graph,'start','goal',(0,0,0),grid,LOCKED_FOOTPRINT)
    assert reason=='LOCAL_SUPPORT_BOUNDARY' and 1.<prefix[-1][0]<1.4
    assert all(not grid.disk_occupied(x,y,radius+.0375) for x,y in prefix)


def test_disk_cell_aabb_unknown_and_map_boundary_are_blocked():
    cells=[0]*1600;cells[10*40+12]=-1
    grid=fixture(cells)
    assert not grid.disk_occupied(0.,.15,.59)
    assert grid.disk_occupied(0.,.15,.60)
    assert grid.disk_occupied(-2.9,0.,.2)
    assert grid.disk_occupied(float('nan'),0.,.1)
