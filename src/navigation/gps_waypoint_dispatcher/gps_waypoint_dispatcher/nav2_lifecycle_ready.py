from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Mapping


ACTIVE_LIFECYCLE_STATE_ID = 3
DEFAULT_REQUIRED_NAV2_LIFECYCLE_NODES = (
    "controller_server",
    "planner_server",
    "behavior_server",
    "bt_navigator",
)


@dataclass(frozen=True)
class LifecycleReadySummary:
    ready: bool
    missing_nodes: tuple[str, ...]
    inactive_nodes: tuple[str, ...]


def normalize_lifecycle_node_names(raw_nodes: Iterable[object]) -> tuple[str, ...]:
    if isinstance(raw_nodes, str):
        raw_nodes = raw_nodes.split(",")

    names: list[str] = []
    for raw_node in raw_nodes:
        name = str(raw_node).strip()
        if not name:
            continue
        names.append(name.lstrip("/"))
    return tuple(names)


def summarize_lifecycle_states(
    required_nodes: Iterable[object],
    state_by_node: Mapping[str, int],
) -> LifecycleReadySummary:
    missing_nodes: list[str] = []
    inactive_nodes: list[str] = []

    for node_name in normalize_lifecycle_node_names(required_nodes):
        state = state_by_node.get(node_name)
        if state is None:
            missing_nodes.append(node_name)
        elif int(state) != ACTIVE_LIFECYCLE_STATE_ID:
            inactive_nodes.append(node_name)

    return LifecycleReadySummary(
        ready=not missing_nodes and not inactive_nodes,
        missing_nodes=tuple(missing_nodes),
        inactive_nodes=tuple(inactive_nodes),
    )
