from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from typing import Hashable


class RoutePlanningError(RuntimeError):
    pass


@dataclass(frozen=True)
class EdgeProjection:
    start_id: int
    end_id: int
    x: float
    y: float
    ratio: float
    distance_m: float


@dataclass(frozen=True)
class RoutePlan:
    points: tuple[tuple[float, float], ...]
    graph_node_ids: tuple[Hashable, ...]
    graph_cost_m: float
    start_snap_distance_m: float
    goal_snap_distance_m: float


def _distance(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


def _project_to_segment(
    point: tuple[float, float],
    start: tuple[float, float],
    end: tuple[float, float],
) -> tuple[float, float, float, float]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length_sq = dx * dx + dy * dy
    if length_sq <= 1e-12:
        ratio = 0.0
    else:
        ratio = max(
            0.0,
            min(
                1.0,
                ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy)
                / length_sq,
            ),
        )
    projected = (start[0] + ratio * dx, start[1] + ratio * dy)
    return projected[0], projected[1], ratio, _distance(point, projected)


def densify_polyline(
    points: list[tuple[float, float]] | tuple[tuple[float, float], ...],
    max_spacing_m: float,
) -> list[tuple[float, float]]:
    if not math.isfinite(max_spacing_m) or max_spacing_m <= 0.0:
        raise ValueError("max_spacing_m must be finite and positive")
    if not points:
        return []

    dense = [tuple(points[0])]
    for start, end in zip(points, points[1:]):
        length = _distance(start, end)
        if length <= 1e-9:
            continue
        steps = max(1, int(math.ceil(length / max_spacing_m)))
        for index in range(1, steps + 1):
            ratio = index / steps
            dense.append(
                (
                    start[0] + ratio * (end[0] - start[0]),
                    start[1] + ratio * (end[1] - start[1]),
                )
            )
    return dense


class RouteGraphPlanner:
    START_ID = "__route_start__"
    GOAL_ID = "__route_goal__"

    def __init__(self, nodes: dict[int, dict], edges: list[list[int]]) -> None:
        self._points: dict[int, tuple[float, float]] = {}
        for raw_id, node in nodes.items():
            node_id = int(raw_id)
            point = (float(node["x"]), float(node["y"]))
            if not all(math.isfinite(value) for value in point):
                raise ValueError(f"route node {node_id} has non-finite coordinates")
            self._points[node_id] = point

        self._edges: list[tuple[int, int]] = []
        seen: set[tuple[int, int]] = set()
        for raw_edge in edges:
            if len(raw_edge) != 2:
                raise ValueError(f"invalid route edge: {raw_edge!r}")
            start_id, end_id = int(raw_edge[0]), int(raw_edge[1])
            if start_id == end_id or start_id not in self._points or end_id not in self._points:
                raise ValueError(f"invalid route edge: {raw_edge!r}")
            edge = (min(start_id, end_id), max(start_id, end_id))
            if edge not in seen:
                seen.add(edge)
                self._edges.append(edge)

        if not self._points or not self._edges:
            raise ValueError("route graph must contain nodes and edges")

    def nearest_edge(self, point: tuple[float, float]) -> EdgeProjection:
        if not all(math.isfinite(value) for value in point):
            raise RoutePlanningError("start or goal contains non-finite coordinates")
        best: EdgeProjection | None = None
        for start_id, end_id in self._edges:
            x, y, ratio, distance_m = _project_to_segment(
                point,
                self._points[start_id],
                self._points[end_id],
            )
            candidate = EdgeProjection(start_id, end_id, x, y, ratio, distance_m)
            if best is None or candidate.distance_m < best.distance_m:
                best = candidate
        if best is None:
            raise RoutePlanningError("route graph has no traversable edge")
        return best

    @staticmethod
    def _connect(
        adjacency: dict[Hashable, list[tuple[Hashable, float]]],
        start: Hashable,
        end: Hashable,
        cost: float,
    ) -> None:
        adjacency.setdefault(start, []).append((end, cost))
        adjacency.setdefault(end, []).append((start, cost))

    def plan(
        self,
        start_xy: tuple[float, float],
        goal_xy: tuple[float, float],
        *,
        max_snap_distance_m: float = 8.0,
    ) -> RoutePlan:
        if not math.isfinite(max_snap_distance_m) or max_snap_distance_m <= 0.0:
            raise ValueError("max_snap_distance_m must be finite and positive")

        start_projection = self.nearest_edge(start_xy)
        goal_projection = self.nearest_edge(goal_xy)
        if start_projection.distance_m > max_snap_distance_m:
            raise RoutePlanningError(
                "start is %.2fm from route graph (limit %.2fm)"
                % (start_projection.distance_m, max_snap_distance_m)
            )
        if goal_projection.distance_m > max_snap_distance_m:
            raise RoutePlanningError(
                "goal is %.2fm from route graph (limit %.2fm)"
                % (goal_projection.distance_m, max_snap_distance_m)
            )

        coordinates: dict[Hashable, tuple[float, float]] = dict(self._points)
        coordinates[self.START_ID] = (start_projection.x, start_projection.y)
        coordinates[self.GOAL_ID] = (goal_projection.x, goal_projection.y)
        adjacency: dict[Hashable, list[tuple[Hashable, float]]] = {
            node_id: [] for node_id in coordinates
        }
        for start_id, end_id in self._edges:
            self._connect(
                adjacency,
                start_id,
                end_id,
                _distance(self._points[start_id], self._points[end_id]),
            )

        def connect_projection(projection: EdgeProjection, virtual_id: Hashable) -> None:
            edge_length = _distance(
                self._points[projection.start_id], self._points[projection.end_id]
            )
            self._connect(
                adjacency,
                virtual_id,
                projection.start_id,
                edge_length * projection.ratio,
            )
            self._connect(
                adjacency,
                virtual_id,
                projection.end_id,
                edge_length * (1.0 - projection.ratio),
            )

        connect_projection(start_projection, self.START_ID)
        connect_projection(goal_projection, self.GOAL_ID)
        if (
            start_projection.start_id == goal_projection.start_id
            and start_projection.end_id == goal_projection.end_id
        ):
            edge_length = _distance(
                self._points[start_projection.start_id],
                self._points[start_projection.end_id],
            )
            self._connect(
                adjacency,
                self.START_ID,
                self.GOAL_ID,
                edge_length * abs(start_projection.ratio - goal_projection.ratio),
            )

        goal_point = coordinates[self.GOAL_ID]
        frontier: list[tuple[float, float, int, Hashable]] = []
        sequence = 0
        heapq.heappush(frontier, (0.0, 0.0, sequence, self.START_ID))
        cost_so_far: dict[Hashable, float] = {self.START_ID: 0.0}
        came_from: dict[Hashable, Hashable] = {}

        while frontier:
            _, current_cost, _, current = heapq.heappop(frontier)
            if current_cost > cost_so_far.get(current, math.inf) + 1e-9:
                continue
            if current == self.GOAL_ID:
                break
            for neighbor, edge_cost in adjacency.get(current, []):
                candidate_cost = current_cost + edge_cost
                if candidate_cost + 1e-9 >= cost_so_far.get(neighbor, math.inf):
                    continue
                cost_so_far[neighbor] = candidate_cost
                came_from[neighbor] = current
                sequence += 1
                heuristic = _distance(coordinates[neighbor], goal_point)
                heapq.heappush(
                    frontier,
                    (candidate_cost + heuristic, candidate_cost, sequence, neighbor),
                )

        if self.GOAL_ID not in cost_so_far:
            raise RoutePlanningError("no route exists between snapped start and goal")

        node_path: list[Hashable] = [self.GOAL_ID]
        while node_path[-1] != self.START_ID:
            node_path.append(came_from[node_path[-1]])
        node_path.reverse()

        points: list[tuple[float, float]] = []
        if start_projection.distance_m > 1e-3:
            points.append((float(start_xy[0]), float(start_xy[1])))
        for node_id in node_path:
            point = coordinates[node_id]
            if not points or _distance(points[-1], point) > 1e-6:
                points.append(point)

        return RoutePlan(
            points=tuple(points),
            graph_node_ids=tuple(node_path),
            graph_cost_m=cost_so_far[self.GOAL_ID],
            start_snap_distance_m=start_projection.distance_m,
            goal_snap_distance_m=goal_projection.distance_m,
        )
