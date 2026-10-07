"""Finite observation decisions from a metric prior and current evidence only.

No simulator world or future map is accepted here. Reachability is a required
planner call that returns a timed trajectory; client supplied boolean flags
cannot authorize a candidate. Unknown cells stop visibility rays and driving.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import heapq
import json
import math
from pathlib import Path
import tempfile
from typing import Callable

from .active_road import EvidenceState, MaGRoadPrior, RoadEvidence
from .grid_map import LocalObstacleGrid
from .trajectory import TimedTrajectory, VehicleLimits, validate_trajectory


def distance(a, b):
    return math.hypot(a[0]-b[0], a[1]-b[1])


@dataclass(frozen=True)
class GraphEdge:
    edge_id: str
    start: str
    end: str
    geometry: tuple[tuple[float, float], ...]
    state: str = "UNOBSERVED"
    source: str = "satellite_prior"

    @property
    def length(self):
        return sum(distance(a, b) for a, b in zip(self.geometry, self.geometry[1:]))


class RoadGraph:
    def __init__(self, edges=()):
        self.edges = {edge.edge_id: edge for edge in edges}
        self.nodes = {}
        for edge in self.edges.values():
            for node, point in ((edge.start, edge.geometry[0]), (edge.end, edge.geometry[-1])):
                if node in self.nodes and distance(self.nodes[node], point) > 1e-6:
                    raise ValueError("inconsistent endpoint geometry")
                self.nodes[node] = point

    @classmethod
    def from_prior(cls, prior: MaGRoadPrior):
        # Geographic degrees are never accepted as metric planner coordinates.
        try:
            from pyproj import CRS
            crs = CRS.from_user_input(prior.crs)
            if not crs.is_projected or any(abs(axis.unit_conversion_factor-1.0) > 1e-9 for axis in crs.axis_info[:2]):
                raise ValueError("prior graph requires a projected metric CRS")
        except ImportError:
            raise RuntimeError("pyproj is required to verify the prior graph metric CRS")
        def endpoint(point):
            return f"xy:{float(point[0]):.6f}:{float(point[1]):.6f}"
        edges = []
        for edge in prior.edges:
            points = tuple(tuple(map(float, point)) for point in edge["coordinates"])
            properties = edge["properties"]
            edges.append(GraphEdge(edge["id"], str(properties.get("start_id", endpoint(points[0]))),
                                   str(properties.get("end_id", endpoint(points[-1]))), points))
        return cls(edges)

    def route(self, start: str, goal: str, *, hypothetical: GraphEdge | None = None):
        adjacency = {node: [] for node in self.nodes}
        for edge in list(self.edges.values()) + ([hypothetical] if hypothetical else []):
            if edge.state == EvidenceState.BLOCKED_EVIDENCE.value:
                continue
            adjacency.setdefault(edge.start, []).append((edge.end, edge.length, edge))
            adjacency.setdefault(edge.end, []).append((edge.start, edge.length, edge))
        queue, best = [(0.0, start, ())], {start: 0.0}
        while queue:
            cost, node, path = heapq.heappop(queue)
            if cost != best[node]:
                continue
            if node == goal:
                return cost, path
            for next_node, increment, edge in adjacency.get(node, []):
                if cost+increment < best.get(next_node, math.inf):
                    best[next_node] = cost+increment
                    heapq.heappush(queue, (cost+increment, next_node, path+(edge.edge_id,)))
        return math.inf, ()


@dataclass(frozen=True)
class ObservationEvent:
    event_id: str
    kind: str
    targets: tuple[tuple[float, float], ...]
    state: str = "UNOBSERVED"
    endpoints: tuple[str, str] | None = None
    observed_length_m: float = 0.0
    question: str = "geometry beyond the observed boundary remains unknown"


def prior_gap_events(graph: RoadGraph, *, min_gap_m=0.3, max_gap_m=5.0, target_step_m=.15):
    if not 0 < min_gap_m <= max_gap_m:
        raise ValueError("invalid gap interval")
    degree = {node: 0 for node in graph.nodes}
    for edge in graph.edges.values():
        degree[edge.start] += 1
        degree[edge.end] += 1
    endpoints = sorted(node for node, count in degree.items() if count == 1)
    events = []
    for index, a in enumerate(endpoints):
        for b in endpoints[index+1:]:
            # Existing connected endpoints are the same road component, not
            # evidence of a missing connection or a new parallel road edge.
            if graph.route(a,b)[1]:continue
            gap = distance(graph.nodes[a], graph.nodes[b])
            if min_gap_m <= gap <= max_gap_m:
                first, last = graph.nodes[a], graph.nodes[b]
                steps = max(1, math.ceil(gap/target_step_m))
                targets = tuple((first[0]+(last[0]-first[0])*i/steps,
                                 first[1]+(last[1]-first[1])*i/steps) for i in range(steps+1))
                events.append(ObservationEvent(f"gap:{a}:{b}", "prior_gap",
                    targets, endpoints=(a, b),
                    question="does the unobserved gap contain continuous supported ground?"))
    return events


def supported_gap_updates(graph, events, grid, footprint):
    """Add geometry only after the whole gap corridor has measured support.

    Seeing its endpoints cannot connect an unobserved middle. The immutable
    satellite graph is copied, and no observation receives TRAVERSED status.
    """
    edges = list(graph.edges.values())
    for event in events:
        if event.endpoints is None or event.event_id in graph.edges:
            continue
        a,b = (graph.nodes[node] for node in event.endpoints)
        yaw = math.atan2(b[1]-a[1], b[0]-a[0])
        c,s = math.cos(yaw),math.sin(yaw)
        steps = max(1, math.ceil(distance(a,b)/(grid.resolution_m*.25)))
        polygons = [[(a[0]+(b[0]-a[0])*i/steps+c*x-s*y,
                      a[1]+(b[1]-a[1])*i/steps+s*x+c*y) for x,y in footprint]
                    for i in range(steps+1)]
        if all(not grid.polygon_occupied(polygon) for polygon in polygons):
            edges.append(GraphEdge(event.event_id, *event.endpoints, (a,b),
                                   EvidenceState.OBSERVED_GEOMETRY.value, "current_supported_ground"))
    return RoadGraph(edges)


def frontier_events(grid: LocalObstacleGrid, *, minimum_cells=3):
    """Cluster confirmed-free cells adjacent to unknown, with no truth labels."""
    frontier = set()
    for row in range(1, grid.height-1):
        for col in range(1, grid.width-1):
            index = row*grid.width+col
            if grid.cells[index] == 0 and any(grid.cells[(row+dr)*grid.width+col+dc] == -1
                                              for dr, dc in ((0,1),(0,-1),(1,0),(-1,0))):
                frontier.add((col, row))
    events = []
    while frontier:
        seed = min(frontier)
        frontier.remove(seed)
        component, queue = [seed], [seed]
        while queue:
            col, row = queue.pop()
            for dc, dr in ((0,1),(0,-1),(1,0),(-1,0)):
                neighbor = col+dc, row+dr
                if neighbor in frontier:
                    frontier.remove(neighbor)
                    queue.append(neighbor)
                    component.append(neighbor)
        if len(component) >= minimum_cells:
            unknown_neighbors = sorted({(c+dc,r+dr) for c,r in component
                for dc,dr in ((0,1),(0,-1),(1,0),(-1,0))
                if grid.cells[(r+dr)*grid.width+c+dc] == -1})
            targets = tuple((grid.origin_x_m+(c+.5)*grid.resolution_m,
                             grid.origin_y_m+(r+.5)*grid.resolution_m)
                            for c,r in unknown_neighbors)
            identity = hashlib.sha256(json.dumps(targets).encode()).hexdigest()[:16]
            events.append(ObservationEvent(f"frontier:{identity}", "geometric_entry", targets,
                observed_length_m=len(component)*grid.resolution_m))
    return events


def task_impact(event: ObservationEvent, graph: RoadGraph, start: str | None, goal: str | None,
                failure_cost_m=100.0):
    if not math.isfinite(failure_cost_m) or failure_cost_m <= 0:
        raise ValueError("finite positive failure cost required")
    if not start or not goal:
        return 1.0, "coverage_fallback:no_task"
    if event.endpoints is None:
        return 0.0, "no_current_graph_connectivity_hypothesis"
    a, b = event.endpoints
    closed_cost, _ = graph.route(start, goal)
    open_cost, _ = graph.route(start, goal, hypothetical=GraphEdge(
        event.event_id, a, b, (graph.nodes[a], graph.nodes[b])))
    # Unknown geometry remains hypothetical; this does not add an executable road.
    return max(0.0, min(closed_cost, failure_cost_m)-min(open_cost, failure_cost_m))/failure_cost_m, "bounded_connectivity_difference"


def visible_fraction(event: ObservationEvent, pose, grid: LocalObstacleGrid, *,
                     sensor_range_m: float, fov_rad: float):
    if not 0 < sensor_range_m or not 0 < fov_rad <= 2*math.pi:
        raise ValueError("explicit sensor range and FOV required")
    visible = 0
    terminal_unknown = False
    for target in event.targets:
        dx, dy = target[0]-pose[0], target[1]-pose[1]
        angle = math.atan2(math.sin(math.atan2(dy, dx)-pose[2]),
                           math.cos(math.atan2(dy, dx)-pose[2]))
        span = math.hypot(dx, dy)
        if span > sensor_range_m or abs(angle) > .5*fov_rad:
            continue
        steps = max(1, math.ceil(span/(grid.resolution_m*.25)))
        target_index = grid._index(*target)
        terminal_unknown = terminal_unknown or grid.value_at(*target) == -1
        # Every intervening sample must have actually observed support.
        if all((target_index is not None and
                grid._index(pose[0]+dx*j/steps, pose[1]+dy*j/steps) == target_index) or
               not grid.occupied(pose[0]+dx*j/steps, pose[1]+dy*j/steps)
               for j in range(steps)):
            visible += 1
    return visible/max(len(event.targets), 1), ("known_approach:terminal_cell_uncertain" if terminal_unknown
                                               else "current_known_space_only")


@dataclass(frozen=True)
class PlannedView:
    candidate_id: str
    event_id: str
    pose: tuple[float, float, float]
    impact: float
    observable_fraction: float
    cost_s: float
    trajectory: TimedTrajectory
    reason: str

    @property
    def score(self):
        return self.impact*self.observable_fraction/(self.cost_s+1e-6)


def planned_views(event: ObservationEvent, poses, grid: LocalObstacleGrid, limits: VehicleLimits,
                  footprint, planner: Callable, *, now: float, impact: float,
                  sensor_range_m: float, fov_rad: float, pose_trustworthy: bool,
                  sensor_valid: bool, settle_s=1.0, attempted=()):
    if not pose_trustworthy or not sensor_valid:
        return []
    views = []
    for pose in poses:
        if not all(math.isfinite(v) for v in pose):
            continue
        c,s = math.cos(pose[2]), math.sin(pose[2])
        polygon = [(pose[0]+c*x-s*y, pose[1]+s*x+c*y) for x,y in footprint]
        if grid.polygon_occupied(polygon):
            continue
        # A millimetre of pose-estimator jitter is not a new viewpoint.
        identity = f"{event.event_id}:cell({round(pose[0]/grid.resolution_m)},{round(pose[1]/grid.resolution_m)}):yaw({round(pose[2]/.1)})"
        if identity in attempted:
            continue
        fraction, visibility_reason = visible_fraction(event, pose, grid,
            sensor_range_m=sensor_range_m, fov_rad=fov_rad)
        if fraction <= 0:
            continue
        trajectory = planner(pose, grid.map_version)
        if trajectory is None:
            continue
        checked = validate_trajectory(trajectory, limits, now=now, expected_map_version=grid.map_version,
            footprint=footprint, occupied=grid.occupied, occupied_polygon=grid.polygon_occupied,
            resolution=grid.resolution_m)
        if not checked.valid or not trajectory.points:
            continue
        last = trajectory.points[-1]
        if distance((last.x, last.y), pose[:2]) > .10:
            continue
        # The actual planner must provide the requested body orientation too.
        yaw_error = abs(math.atan2(math.sin(last.yaw-pose[2]), math.cos(last.yaw-pose[2])))
        if yaw_error > .10:
            continue
        cost = last.t + settle_s
        if not math.isfinite(cost) or cost <= 0 or impact < 0:
            continue
        views.append(PlannedView(identity, event.event_id, pose, impact, fraction, cost,
                                 trajectory, visibility_reason+";actual_timed_planner_validated"))
    return views


class FiniteObservationPolicy:
    MODES = {"PASSIVE", "PERIODIC_LOOK", "TASK_AWARE_LOOK"}

    def __init__(self, mode, *, budget_s=60.0, periodic_s=10.0):
        if mode not in self.MODES or budget_s <= 0 or periodic_s <= 0:
            raise ValueError("invalid finite policy configuration")
        self.mode, self.budget_s, self.periodic_s = mode, budget_s, periodic_s
        self.spent_s, self.last_look_s = 0.0, -math.inf
        self.attempted, self.resolved = set(), set()

    def choose(self, views, *, elapsed_s, task_reached=False, system_ok=True):
        if not system_ok:
            return None, "SYSTEM_FAULT"
        if task_reached:
            return None, "TASK_REACHED"
        if self.spent_s >= self.budget_s:
            return None, "BUDGET_EXHAUSTED"
        eligible = [v for v in views if (self.mode != "TASK_AWARE_LOOK" or v.impact > 0) and v.event_id not in self.resolved and
                    v.candidate_id not in self.attempted and v.cost_s+self.spent_s <= self.budget_s]
        if self.mode == "PASSIVE":
            return None, "PASSIVE_TASK_ONLY"
        if self.mode == "PERIODIC_LOOK" and elapsed_s-self.last_look_s < self.periodic_s:
            return None, "PERIOD_NOT_DUE"
        if not eligible:
            return None, "NO_SAFE_OBSERVATION_POSE"
        selected = (max(eligible, key=lambda v: (v.score, v.candidate_id))
                    if self.mode == "TASK_AWARE_LOOK" else min(eligible, key=lambda v: v.candidate_id))
        return selected, "SELECTED"

    def record(self, view: PlannedView, *, actual_cost_s, elapsed_s, resolved_geometry: bool):
        if (not math.isfinite(actual_cost_s) or actual_cost_s < 0 or
                not math.isfinite(elapsed_s) or elapsed_s < 0):
            raise ValueError("invalid measured observation cost")
        self.attempted.add(view.candidate_id)
        self.spent_s += actual_cost_s
        self.last_look_s = elapsed_s
        if resolved_geometry:
            self.resolved.add(view.event_id)


def current_session_evidence(evidence,localization_session_id):
    """Historical local coordinates cannot support a new odom-session increment."""
    if not localization_session_id:return []
    return [e for e in evidence if e.local_submap_id.startswith(localization_session_id+"/")]


def save_snapshot(path, prior: MaGRoadPrior, evidence: list[RoadEvidence], policy: FiniteObservationPolicy):
    """Atomic content-addressed overlay; never changes the prior asset."""
    payload = {"schema": 1, "prior": {"path": prior.path, "crs": prior.crs,
        "model_version": prior.model_version, "sha256": hashlib.sha256(Path(prior.path).read_bytes()).hexdigest()},
        "evidence": [{**asdict(e), "state": e.state.value} for e in evidence],
        "attempted_views": sorted(policy.attempted), "resolved_geometry": sorted(policy.resolved)}
    version = hashlib.sha256(json.dumps(payload, sort_keys=True).encode()).hexdigest()
    payload["map_version"] = version
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(mode="w", dir=destination.parent, delete=False) as stream:
        json.dump(payload, stream, indent=2, sort_keys=True)
        temporary = Path(stream.name)
    temporary.replace(destination)
    return version


def load_snapshot(path, prior: MaGRoadPrior, policy: FiniteObservationPolicy):
    """Restore the overlay only against the identical read-only prior."""
    payload = json.loads(Path(path).read_text())
    claimed_version = payload.pop("map_version")
    actual_version = hashlib.sha256(json.dumps(payload, sort_keys=True).encode()).hexdigest()
    prior_hash = hashlib.sha256(Path(prior.path).read_bytes()).hexdigest()
    if (payload.get("schema") != 1 or claimed_version != actual_version or
            payload["prior"]["sha256"] != prior_hash or payload["prior"]["crs"] != prior.crs or
            payload["prior"]["model_version"] != prior.model_version):
        raise ValueError("snapshot identity or prior mismatch")
    from .active_road import EvidenceStore
    checked, identities = [], set()
    for item in payload["evidence"]:
        item["state"] = EvidenceState(item["state"])
        item["geometry_xy"] = [tuple(point) for point in item["geometry_xy"]]
        if item.get("valid_depth_m") is not None:
            item["valid_depth_m"] = tuple(item["valid_depth_m"])
        evidence = RoadEvidence(**item)
        EvidenceStore._validate_evidence(evidence)
        if evidence.evidence_id in identities:
            raise ValueError("duplicate evidence UUID in snapshot")
        identities.add(evidence.evidence_id)
        checked.append(evidence)
    policy.attempted = set(payload["attempted_views"])
    policy.resolved = set(payload["resolved_geometry"])
    # Observation budget is per task; persisted attempts suppress repeated views.
    return checked, claimed_version
