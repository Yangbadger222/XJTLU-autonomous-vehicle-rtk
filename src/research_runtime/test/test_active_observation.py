import math

from research_runtime.active_observation import (FiniteObservationPolicy, GraphEdge,
    ObservationEvent, PlannedView, RoadGraph, frontier_events, planned_views,
    prior_gap_events, task_impact, visible_fraction)
from research_runtime.grid_map import LocalObstacleGrid
from research_runtime.trajectory import TimedPoint, TimedTrajectory, VehicleLimits


def grid(cells=None):
    return LocalObstacleGrid("odom", "test-map", .1, -1, -1, 40, 40,
                             tuple(cells or [0]*1600))


def trajectory(version="test-map", speed=.2, end=1):
    return TimedTrajectory.from_points("query", version, "odom", 10, 10.25,
        [TimedPoint(i*.1, i*.1*speed, 0, 0, speed, 0, curvature=0) for i in range(int(end/speed/.1)+1)])


def test_gap_event_and_impact_use_only_current_prior():
    graph = RoadGraph([GraphEdge("a", "start", "left", ((0,0),(1,0))),
                       GraphEdge("b", "right", "goal", ((2,0),(3,0)))])
    event = next(e for e in prior_gap_events(graph, max_gap_m=1.1)
                 if set(e.endpoints) == {"left", "right"})
    impact, reason = task_impact(event, graph, "start", "goal", failure_cost_m=10)
    assert impact == .7 and reason == "bounded_connectivity_difference"
    assert math.isinf(graph.route("start", "goal")[0])  # hypothetical edge was not added
    assert task_impact(event, graph, None, None)[1].startswith("coverage_fallback")


def test_degree_two_support_points_do_not_create_new_gap():
    graph = RoadGraph([GraphEdge("road", "a", "b", ((0,0),(.5,0),(1,0)))])
    assert len(graph.nodes) == 2


def test_frontiers_are_current_free_unknown_boundaries():
    cells = [-1]*1600
    for row in range(10,20):
        for col in range(10,20):
            cells[row*40+col] = 0
    events = frontier_events(grid(cells))
    assert len(events) == 1
    assert events[0].kind == "geometric_entry"
    assert events[0].state == "UNOBSERVED"


def test_unknown_or_obstacle_intervening_ray_is_not_visible():
    event = ObservationEvent("event", "gap", ((1,0),))
    cells = [0]*1600
    cells[10*40+15] = -1  # x=.5, y=0
    assert visible_fraction(event, (0,0,0), grid(cells), sensor_range_m=2, fov_rad=math.pi)[0] == 0
    assert visible_fraction(event, (0,0,math.pi), grid(), sensor_range_m=2, fov_rad=math.pi)[0] == 0
    assert visible_fraction(event, (0,0,0), grid(), sensor_range_m=2, fov_rad=math.pi)[0] == 1


def test_frontier_terminal_cell_is_potentially_observable_without_free_space_promotion():
    cells = [0]*1600
    cells[10*40+20] = -1
    current = grid(cells)
    event = ObservationEvent("event", "frontier", ((1.05,.05),))
    fraction, reason = visible_fraction(event, (0,.05,0), current,
                                       sensor_range_m=2, fov_rad=math.pi)
    assert fraction == 1 and "uncertain" in reason
    assert current.occupied(1.05,.05)


def test_candidates_require_actual_timed_planner_result_and_map_version():
    event = ObservationEvent("event", "gap", ((1.5,0),))
    calls = []
    def planner(pose, version):
        calls.append((pose, version))
        return trajectory(version)
    kwargs = dict(now=10, impact=1, sensor_range_m=2, fov_rad=math.pi,
                  pose_trustworthy=True, sensor_valid=True)
    footprint = ((-.05,-.05),(.05,-.05),(.05,.05),(-.05,.05))
    views = planned_views(event, [(1,0,0)], grid(), VehicleLimits(), footprint, planner, **kwargs)
    assert len(views) == 1 and calls == [((1,0,0),"test-map")]
    assert views[0].cost_s == 6
    assert not planned_views(event, [(1,0,0)], grid(), VehicleLimits(), footprint,
                            lambda *args: trajectory("stale-map"), **kwargs)
    assert not planned_views(event, [(1,0,0)], grid(), VehicleLimits(), footprint,
                            lambda *args: trajectory(speed=2), **kwargs)
    assert not planned_views(event, [(1,0,0)], grid(), VehicleLimits(), footprint,
                            lambda *args: None, **kwargs)


def test_budget_failure_and_repeated_uncertain_angle_are_finite():
    traj = trajectory()
    view = PlannedView("view", "event", (1,0,0), 1, .5, 6, traj, "actual plan")
    policy = FiniteObservationPolicy("TASK_AWARE_LOOK", budget_s=10)
    assert policy.choose([view], elapsed_s=0)[0] == view
    policy.record(view, actual_cost_s=6, elapsed_s=6, resolved_geometry=False)
    assert policy.choose([view], elapsed_s=7)[1] == "NO_SAFE_OBSERVATION_POSE"
    assert policy.choose([view], elapsed_s=7, system_ok=False)[1] == "SYSTEM_FAULT"
    assert "event" not in policy.resolved
    passive = FiniteObservationPolicy("PASSIVE")
    assert passive.choose([view], elapsed_s=0)[1] == "PASSIVE_TASK_ONLY"
    periodic = FiniteObservationPolicy("PERIODIC_LOOK", periodic_s=10)
    periodic.record(view, actual_cost_s=6, elapsed_s=6, resolved_geometry=True)
    assert periodic.choose([view], elapsed_s=7)[1] == "PERIOD_NOT_DUE"
