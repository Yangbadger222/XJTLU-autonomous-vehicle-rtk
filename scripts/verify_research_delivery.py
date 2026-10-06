#!/usr/bin/env python3
"""Evidence-first static audit for the Super-LIO/EGO research delivery.

This verifier does not claim ROS, Jetson, sensor, or actuator runtime.  It
checks the repository invariants that can be proven from the current checkout
and reports every check explicitly instead of collapsing pending gates into
one overall PASS.
"""
from __future__ import annotations

import argparse
import filecmp
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys


BASE = "e54c6afbcb5a58db22d7c468085a87d658b0b932"
BRANCH = "codex/superlio-ego-active-road"
SUPER = "f89f48dc7aea6cfa262f18e4d03b319e04e0dbd2"
EGO = "7f5be6d4cee34871e85aa1f15285cfaf17b23877"


def run(*args: str) -> str:
    return subprocess.check_output(args, text=True).strip()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    checks: list[dict[str, object]] = []

    def check(name: str, ok: bool, evidence: list[str], detail: str = "") -> None:
        checks.append({"name": name, "status": "PASS" if ok else "FAIL",
                       "evidence": evidence, "detail": detail})

    branch = run("git", "rev-parse", "--abbrev-ref", "HEAD")
    tip = run("git", "rev-parse", "HEAD")
    check("branch_and_base", branch == BRANCH and bool(run("git", "merge-base", BASE, tip)),
          ["git rev-parse --abbrev-ref HEAD", "audit/vehicle_baseline/SOURCE_IDENTITY.json"],
          f"branch={branch}, tip={tip}, base={BASE}")

    deps_text = Path("dependencies.research.repos").read_text()
    super_pin = re.search(r"super_lio:.*?\n(?:.*\n)*?\s+version:\s*([^\s]+)", deps_text)
    ego_pin = re.search(r"ego_planner_2d_ros2:.*?\n(?:.*\n)*?\s+version:\s*([^\s]+)", deps_text)
    pins = {"super_lio": super_pin.group(1) if super_pin else None,
            "ego_planner_2d_ros2": ego_pin.group(1) if ego_pin else None}
    check("external_pins", pins == {"super_lio": SUPER, "ego_planner_2d_ros2": EGO},
          ["dependencies.research.repos"], json.dumps(pins, sort_keys=True))

    verification = json.loads(Path("audit/UPSTREAM_PATCH_VERIFICATION.json").read_text())
    super_patch = Path("patches/super_lio/0001-publish-source-aware-odom-health.patch")
    ego_one = Path("patches/ego_planner_2d/0001-vehicle-state-and-feasibility.patch")
    ego_two = Path("patches/ego_planner_2d/0002-vehicle-ros-timed-trajectory.patch")
    ego_three = Path("patches/ego_planner_2d/0003-clear-stale-plan-on-failure.patch")
    expected_hashes = {
        "super_lio": verification["super_lio"]["patch_sha256"],
        "ego_one": verification["ego_planner_2d_ros2"]["patches"][0]["sha256"],
        "ego_two": verification["ego_planner_2d_ros2"]["patches"][1]["sha256"],
        "ego_three": verification["ego_planner_2d_ros2"]["patches"][2]["sha256"],
    }
    actual_hashes = {"super_lio": sha256(super_patch), "ego_one": sha256(ego_one),
                     "ego_two": sha256(ego_two), "ego_three": sha256(ego_three)}
    check("patch_hashes", actual_hashes == expected_hashes,
          ["audit/UPSTREAM_PATCH_VERIFICATION.json", str(super_patch), str(ego_one), str(ego_two)],
          json.dumps({"expected": expected_hashes, "actual": actual_hashes}, sort_keys=True))

    protected = subprocess.run(["sha256sum", "-c", "audit/vehicle_baseline/PROTECTED_FILES.sha256"],
                               text=True, capture_output=True)
    check("protected_vehicle_files", protected.returncode == 0,
          ["audit/vehicle_baseline/PROTECTED_FILES.sha256"], protected.stdout + protected.stderr)

    launch = Path("src/bringup/launch/system_active_road_research.launch.py").read_text()
    forbidden = ("system_explore", 'package="nav2', 'package="slam_toolbox',
                 'package="mppi', 'package="fastlio2', "fake_sim_node")
    check("research_launch_allowlist", all(token not in launch.lower() for token in forbidden),
          ["src/bringup/launch/system_active_road_research.launch.py", "LEGACY_REMOVAL_MANIFEST.md"])
    check("serial_gate", "' == 'live' and '" in launch and
          'DeclareLaunchArgument("enable_serial", default_value="false"' in launch,
          ["src/bringup/launch/system_active_road_research.launch.py"])

    contract = json.loads(Path("audit/vehicle_baseline/RUNTIME_CONTRACT.json").read_text())
    topics = contract["topics"]
    check("cloud_tf_boundary", topics["/lio/cloud_world"]["frame"] == "world" and
          topics["/lio/cloud_odom"]["frame"] == "odom" and
          "timestamped" in topics["/lio/cloud_odom"]["producer"],
          ["audit/vehicle_baseline/RUNTIME_CONTRACT.json",
           "src/super_lio_vehicle_adapter/super_lio_vehicle_adapter/cloud_frame_node.py"])

    interface_files = [
        "src/research_interfaces/msg/TimedTrajectory2D.msg",
        "src/research_interfaces/msg/RoadEvidence2D.msg",
        "src/active_road_mapping/active_road_mapping/evidence_node.py",
        "src/research_runtime/research_runtime/safety_bridge.py",
        "src/super_lio_vehicle_adapter/super_lio_vehicle_adapter/cloud_frame_node.py",
    ]
    check("typed_research_boundaries", all(Path(path).is_file() for path in interface_files),
          interface_files)

    required = ["AUDIT_REPORT.md", "METHOD_SPEC.md", "MIGRATION_REPORT.md", "RUN_STATE.json",
                "REPRODUCE.md", "LIVE_ACCEPTANCE_CHECKLIST.md", "ROLLBACK.md", "RESULTS.json"]
    check("required_deliverables", all(Path(path).is_file() for path in required), required)
    results = json.loads(Path("RESULTS.json").read_text())
    required_statuses = {"PASS", "FAIL", "PENDING", "NOT_RUN"}
    check("result_status_matrix", all(item.get("status") in required_statuses for item in results.values()),
          ["RESULTS.json"])

    report = Path("outputs/RESEARCH_DELIVERY_REPORT.md")
    external_report = Path("/Users/badger/Documents/Codex/2026-10-07/codex-goal-superlio-ego-research-md/outputs/RESEARCH_DELIVERY_REPORT.md")
    check("report_copy", report.is_file() and external_report.is_file() and filecmp.cmp(report, external_report, shallow=False),
          [str(report), str(external_report)])

    payload = {"schema": 1, "branch": branch, "tip": tip, "base": BASE,
               "checks": checks, "overall_static_status": "PASS" if all(c["status"] == "PASS" for c in checks) else "FAIL",
               "runtime_gates": "PENDING outside this static verifier"}
    rendered = json.dumps(payload, indent=2, ensure_ascii=False) + "\n"
    if args.output:
        args.output.write_text(rendered)
    print(rendered, end="")
    return 0 if payload["overall_static_status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
