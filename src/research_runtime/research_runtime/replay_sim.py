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
    """Labels for the required experiment, not a synthetic benefit claim.

    Hand-picked candidate scores do not constitute an interactive same-base
    policy comparison. That experiment must use the actual Super-LIO/EGO,
    restricted sensor simulator and evaluator in a separate runtime harness.
    """
    return {"status": "NOT_RUN", "truth_is_evaluator_only": True,
            "policies": {name: {"status": "NOT_RUN"} for name in
                         ("PASSIVE", "PERIODIC_LOOK", "TASK_AWARE_LOOK")},
            "reason": "contract smoke does not measure policy effect"}


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
