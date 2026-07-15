import math

import pytest

from gps_waypoint_dispatcher.route_graph_planner import (
    RouteGraphPlanner,
    RoutePlanningError,
    densify_polyline,
)


def _planner() -> RouteGraphPlanner:
    nodes = {
        1: {"x": 0.0, "y": 0.0},
        2: {"x": 10.0, "y": 0.0},
        3: {"x": 20.0, "y": 0.0},
        4: {"x": 10.0, "y": 10.0},
    }
    return RouteGraphPlanner(nodes, [[1, 2], [2, 3], [2, 4]])


def test_astar_projects_start_and_goal_to_route_edges():
    plan = _planner().plan((2.0, 1.0), (11.0, 8.0), max_snap_distance_m=2.0)

    assert plan.points[0] == (2.0, 1.0)
    assert plan.points[1] == (2.0, 0.0)
    assert plan.points[-1] == (10.0, 8.0)
    assert math.isclose(plan.start_snap_distance_m, 1.0)
    assert math.isclose(plan.goal_snap_distance_m, 1.0)
    assert math.isclose(plan.graph_cost_m, 16.0)


def test_astar_uses_direct_connection_when_both_projections_share_edge():
    plan = _planner().plan((2.0, 0.5), (8.0, -0.5), max_snap_distance_m=1.0)

    assert plan.graph_node_ids == (
        RouteGraphPlanner.START_ID,
        RouteGraphPlanner.GOAL_ID,
    )
    assert math.isclose(plan.graph_cost_m, 6.0)


def test_astar_rejects_pose_too_far_from_network():
    with pytest.raises(RoutePlanningError, match="start is"):
        _planner().plan((2.0, 20.0), (8.0, 0.0), max_snap_distance_m=2.0)


def test_astar_reports_disconnected_route_components():
    planner = RouteGraphPlanner(
        {
            1: {"x": 0.0, "y": 0.0},
            2: {"x": 1.0, "y": 0.0},
            3: {"x": 10.0, "y": 0.0},
            4: {"x": 11.0, "y": 0.0},
        },
        [[1, 2], [3, 4]],
    )

    with pytest.raises(RoutePlanningError, match="no route exists"):
        planner.plan((0.0, 0.0), (11.0, 0.0))


def test_densify_polyline_bounds_spacing_without_duplicate_vertices():
    dense = densify_polyline([(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)], 0.3)

    assert dense[0] == (0.0, 0.0)
    assert dense[-1] == (1.0, 1.0)
    assert all(
        math.hypot(b[0] - a[0], b[1] - a[1]) <= 0.300001
        for a, b in zip(dense, dense[1:])
    )
