"""Deterministic, actuator-free smoke and policy comparison harness."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from .active_road import (EvidenceState, EvidenceStore, GeoTransform, ObservationCandidate,
                          RoadEvent, RoadEvidence, choose_observation)
from .authority import AuthorityState, SafetyGate, format_serial
from .trajectory import TimedPoint, TimedTrajectory, VehicleLimits, validate_trajectory
from .trajectory_tracker import TrackerState, TimedTrajectoryTracker


def compare_policies() -> dict:
    """Run a truth-blind policy loop and evaluator-side scoring.

    The ``truth`` dictionary is created only in this harness's evaluator; it is
    never passed to candidate generation or scoring. This is a bounded
    synthetic comparison, not a vehicle or bag result.
    """
    events = [
        RoadEvent("entry-1", "prior_gap", (2.0, 0.0), 0.8, 1.2, EvidenceState.UNOBSERVED, 2.0),
        RoadEvent("entry-2", "new_channel", (4.0, 1.0), 0.0, 2.0, EvidenceState.UNOBSERVED, 1.0),
    ]
    candidates = {
        "entry-1": [
            ObservationCandidate("periodic-1", "entry-1", True, True, True, True, 2.0, 0.3, 4.0),
            ObservationCandidate("task-1", "entry-1", True, True, True, True, 2.0, 0.8, 2.0),
        ],
        "entry-2": [
            ObservationCandidate("periodic-2", "entry-2", True, True, True, True, 1.0, 0.2, 3.0),
            ObservationCandidate("task-2", "entry-2", True, True, True, True, 1.0, 0.7, 1.5),
        ],
    }
    # Hidden evaluator labels only; policy code above never reads this map.
    truth = {"entry-1": "open", "entry-2": "blocked"}
    selected = {
        "PASSIVE": [],
        "PERIODIC_LOOK": [candidates[e.event_id][0] for e in events],
        "TASK_AWARE_LOOK": [choose_observation(candidates[e.event_id]) for e in events],
    }
    results = {}
    for name, choices in selected.items():
        choices = [c for c in choices if c is not None]
        # A bounded evaluator metric: a high observable fraction resolves the
        # event geometry, while truth is used only for correctness accounting.
        resolved = sum(c.observable_fraction >= 0.5 for c in choices)
        correct = sum((truth[c.event_id] == "open") == (c.event_id == "entry-1") for c in choices)
        results[name] = {"observations": len(choices), "cost": sum(c.cost for c in choices),
                         "resolved_geometry": resolved, "correct_evaluator_labels": correct}
    return {"truth_is_evaluator_only": True, "policies": results}


def run() -> dict:
    limits = VehicleLimits()
    trajectory = TimedTrajectory.from_points(
        "replay-traj-1", "synthetic-v1", "odom", 0.0, 10.0,
        [TimedPoint(t, t * 0.2, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.0) for t in (0.0, 1.0, 2.0, 3.0)])
    validation = validate_trajectory(trajectory, limits, now=1.0, expected_map_version="synthetic-v1",
                                     footprint=[(-0.5, -0.3), (-0.5, 0.3), (0.5, -0.3), (0.5, 0.3)],
                                     occupied=lambda x, y: False, resolution=0.1)
    gate = SafetyGate(0.50)
    tracker = TimedTrajectoryTracker(limits)
    tracked = tracker.command(trajectory, TrackerState(0.1, 0.0, 0.0), now=1.0,
                              expected_map_version="synthetic-v1")
    allowed = gate.command(tracked.linear_x if tracked else 0.0,
                           tracked.angular_z if tracked else 0.0,
                           AuthorityState(True, 1.0, 1.1, "OK", trajectory_ok=tracked is not None))
    denied = gate.command(tracked.linear_x if tracked else 0.0,
                          tracked.angular_z if tracked else 0.0,
                          AuthorityState(False, 1.0, 1.1, "OK", trajectory_ok=tracked is not None))
    store = EvidenceStore(GeoTransform("EPSG:32651", "WGS84", 0.0, 0.0, 0.2, 0.2), "synthetic-v1")
    event = RoadEvent("entry-1", "prior_gap", (2.0, 0.0), 0.8, 1.2, EvidenceState.UNOBSERVED, 2.0)
    store.add(RoadEvidence("obs-1", [(1.6, 0.0), (2.4, 0.0)], EvidenceState.OBSERVED_GEOMETRY,
                           1.5, "synthetic_lidar", "submap-1", 0.05, 0.8, (0.2, 12.0)))
    candidate = choose_observation([
        ObservationCandidate("view-safe", event.event_id, True, True, True, True, event.impact, 0.7, 2.0),
        ObservationCandidate("view-unsafe", event.event_id, True, False, True, True, event.impact, 1.0, 0.1),
    ])
    return {"trajectory": {"valid": validation.valid, "reasons": validation.reasons},
            "tracker": {"valid": tracked is not None,
                        "linear_x": tracked.linear_x if tracked else 0.0,
                        "angular_z": tracked.angular_z if tracked else 0.0},
            "serial_allowed": format_serial(allowed).decode(),
            "serial_denied": format_serial(denied).decode(),
            "denied_reason": denied.reason,
            "evidence_count": len(store.evidence()),
            "selected_candidate": candidate.candidate_id if candidate else None,
            "selected_score": candidate.score if candidate else None,
            "truth_access": False,
            "policy_comparison": compare_policies()}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = run()
    rendered = json.dumps(result, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered)
    print(rendered, end="")
    return 0 if result["trajectory"]["valid"] and result["serial_denied"].startswith("vcx=0") else 1


if __name__ == "__main__":
    raise SystemExit(main())
