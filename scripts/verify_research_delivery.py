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
    ancestry = subprocess.run(["git", "merge-base", "--is-ancestor", BASE, tip]).returncode == 0
    identity = json.loads(Path("audit/vehicle_baseline/SOURCE_IDENTITY.json").read_text())
    identity_matches = (identity.get("vehicle_baseline_commit") == BASE and
                        identity.get("research_base_commit") == BASE and
                        identity.get("research_branch") == BRANCH)
    check("branch_and_base", branch == BRANCH and ancestry and identity_matches,
          ["git merge-base --is-ancestor", "audit/vehicle_baseline/SOURCE_IDENTITY.json"],
          f"branch={branch}, tip={tip}, base={BASE}, ancestry={ancestry}, identity_matches={identity_matches}")

    deps_text = Path("dependencies.research.repos").read_text()
    super_pin = re.search(r"super_lio:.*?\n(?:.*\n)*?\s+version:\s*([^\s]+)", deps_text)
    ego_pin = re.search(r"ego_planner_2d_ros2:.*?\n(?:.*\n)*?\s+version:\s*([^\s]+)", deps_text)
    pins = {"super_lio": super_pin.group(1) if super_pin else None,
            "ego_planner_2d_ros2": ego_pin.group(1) if ego_pin else None}
    check("external_pins", pins == {"super_lio": SUPER, "ego_planner_2d_ros2": EGO},
          ["dependencies.research.repos"], json.dumps(pins, sort_keys=True))

    verification = json.loads(Path("audit/UPSTREAM_PATCH_VERIFICATION.json").read_text())
    proof_path=Path(verification["current_source_proof"])
    proof=json.loads(proof_path.read_text())
    for name,pin,directory,count in (("super_lio",SUPER,"super_lio",2),("ego_planner_2d_ros2",EGO,"ego_planner_2d",11)):
        item=proof.get("dependencies",{}).get(name,{})
        patches=sorted(Path("patches",directory).glob("*.patch"))
        expected=[{"path":str(p),"sha256":sha256(p),"check_apply":"PASS"} for p in patches]
        check(name+"_current_patch_source_proof",proof.get("status")=="PASS" and
              len(patches)==count and [p.name[:4] for p in patches]==[f"{i:04}" for i in range(1,count+1)] and
              item.get("pinned_commit")==pin and item.get("patches")==expected and
              item.get("sequential_apply")=="PASS" and item.get("matches_built_source") is True and
              item.get("differences")==[] and verification["dependencies"][name]["patches"]==expected,
              [str(proof_path),"audit/UPSTREAM_PATCH_VERIFICATION.json",*[str(p) for p in patches]],
              f"fresh exact-pin sequential {count} patches; actual checkout all-file byte equality; historical receipts remain historical")
    clean_path=Path(verification["current_clean_build"])
    clean=json.loads(clean_path.read_text()) if clean_path.is_file() else {}
    clean_commit=clean.get("source_commit","")
    valid_commit=bool(re.fullmatch(r"[0-9a-f]{40}",clean_commit))
    runtime_paths=["src","patches","dependencies.research.repos","scripts/build_active_road_research.sh","scripts/generate_research_parameter_lock.py",
        "scripts/apply_ego_vehicle_patch.sh","scripts/apply_super_lio_patch.sh"]
    # Compare the working tree too. Package README prose is not installed runtime
    # code; every other source/build-input difference invalidates this evidence.
    changed=run("git","diff","--name-only",clean_commit,"--",*runtime_paths).splitlines() if valid_commit else []
    untracked=run("git","ls-files","--others","--exclude-standard","--",*runtime_paths).splitlines()
    documentation_only={"src/research_interfaces/README.md","patches/super_lio/README.md",
                        "patches/ego_planner_2d/README.md"}
    runtime_changes=[p for p in changed+untracked if p not in documentation_only]
    compiled_sources_match=valid_commit and not runtime_changes
    check("current_clean_humble_build",clean.get("status")=="PASS" and clean.get("build_exit")==0 and
          clean.get("package_count")==14 and compiled_sources_match,
          [str(clean_path),verification["current_build_log"]],
          "fresh isolated SDK/14-package build; runtime changes since build: "+repr(runtime_changes))

    three_patch_recipe = Path("audit/container/ego-current-three-patch-build.Dockerfile")
    recipe_text = three_patch_recipe.read_text() if three_patch_recipe.is_file() else ""
    recipe_tokens = [
        "0001-vehicle-state-and-feasibility.patch",
        "0002-vehicle-ros-timed-trajectory.patch",
        "0003-clear-stale-plan-on-failure.patch",
        "COPY src/research_interfaces /vehicle-research/src/research_interfaces",
        "--packages-select research_interfaces ego_planner",
    ]
    check("historical_ego_three_patch_build_recipe", all(token in recipe_text for token in recipe_tokens),
          [str(three_patch_recipe)],
          "recipe applies all three pinned EGO patches before the research_interfaces/ego_planner build")

    orbstack_result_path = Path("audit/container/ego-orbstack-ros-base-three-patch-result.json")
    orbstack_recipe = Path("audit/container/ego-orbstack-ros-base-three-patch.Dockerfile")
    orbstack_build_log = Path("audit/container/ego-orbstack-ros-base-three-patch-build.log")
    orbstack_summary_log = Path("audit/container/ego-orbstack-ros-base-three-patch-build-summary.log")
    orbstack_run_log = Path("audit/container/ego-orbstack-ros-base-three-patch-run.log")
    orbstack_result = json.loads(orbstack_result_path.read_text()) if orbstack_result_path.is_file() else {}
    orbstack_recipe_text = orbstack_recipe.read_text() if orbstack_recipe.is_file() else ""
    orbstack_build_text = orbstack_build_log.read_text() if orbstack_build_log.is_file() else ""
    orbstack_summary_text = orbstack_summary_log.read_text() if orbstack_summary_log.is_file() else ""
    orbstack_run_text = orbstack_run_log.read_text() if orbstack_run_log.is_file() else ""
    orbstack_ok = (
        orbstack_result.get("executor") == "OrbStack" and
        orbstack_result.get("server_architecture") == "aarch64" and
        orbstack_result.get("platform") == "linux/arm64" and
        orbstack_result.get("base_image") == "ros:humble" and
        orbstack_result.get("source_commit") == EGO and
        orbstack_result.get("patches") == [
            "0001-vehicle-state-and-feasibility.patch",
            "0002-vehicle-ros-timed-trajectory.patch",
            "0003-clear-stale-plan-on-failure.patch",
        ] and
        orbstack_result.get("colcon_packages") == ["research_interfaces", "ego_planner"] and
        orbstack_result.get("build_exit_code") == 0 and
        "FROM ros:humble" in orbstack_recipe_text and
        "0003-clear-stale-plan-on-failure.patch" in orbstack_recipe_text and
        orbstack_result.get("runtime_smoke", {}).get("status") == "STARTED_WAITING_FOR_REQUIRED_INPUTS" and
        "Finished <<< ego_planner" in orbstack_summary_text and
        "Summary: 2 packages finished" in orbstack_summary_text and
        "motion_plan_timeout_or_exit=124" in orbstack_run_text
    )
    check("historical_ego_orbstack_three_patch_build", orbstack_ok,
          [str(orbstack_recipe), str(orbstack_result_path), str(orbstack_build_log), str(orbstack_summary_log), str(orbstack_run_log)],
          "public ros:humble ARM64 alternate-base build and bounded startup smoke; ros2-go2 and Jetson gates remain separate")

    for filename in ("PROTECTED_FILES.sha256","PROTECTED_ADDITIONAL_FILES.sha256"):
        entries=[line.split("  ",1) for line in (Path("audit/vehicle_baseline")/filename).read_text().splitlines()]
        check("source_byte_protection:"+filename,all(sha256(Path(path))==digest for digest,path in entries),["audit/vehicle_baseline/"+filename],str(len(entries))+" original files")
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
    ego_config = Path("src/bringup/config/ego_vehicle_adapter.yaml")
    check("research_config_install_path",
          ego_config.is_file() and "ego_vehicle_adapter.yaml" in launch,
          [str(ego_config), "src/bringup/CMakeLists.txt",
           "src/bringup/launch/system_active_road_research.launch.py"],
          "vehicle EGO parameters are installed with bringup rather than left in a repository-only config directory")
    bringup_package = Path("src/bringup/package.xml").read_text()
    check("bringup_launch_dependencies",
          all(f"<exec_depend>{dependency}</exec_depend>" in bringup_package
              for dependency in ("ament_index_python", "launch", "launch_ros")),
          ["src/bringup/package.xml", "src/bringup/launch/system_active_road_research.launch.py"],
          "launch imports have explicit clean-install package dependencies")
    entry_assets = [
        "src/bringup/config/master_params.yaml",
        "src/bringup/config/super_lio_vehicle.yaml",
        "src/bringup/config/super_lio_cloud_frame.yaml",
        "src/bringup/config/research_safety_bridge.yaml",
        "src/bringup/config/research_local_grid.yaml",
        "src/bringup/config/research_observed_ground.yaml",
        "src/bringup/config/active_road_mapping.yaml",
        "src/bringup/config/active_road_evidence.yaml",
        "src/bringup/config/ego_vehicle_adapter.yaml",
        "src/sensor_drivers/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py",
        "src/sensor_drivers/gnss/um982_rtk_driver/launch/um982_rtk.launch.py",
    ]
    check("research_entry_assets", all(Path(path).is_file() for path in entry_assets),
          entry_assets, "research launch config and sensor launch assets exist in the source/install inputs")

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
               "runtime_gates": "Executed runtime evidence is reported in RESULTS.json; this verifier only checks provenance/invariants"}
    rendered = json.dumps(payload, indent=2, ensure_ascii=False) + "\n"
    if args.output:
        args.output.write_text(rendered)
    print(rendered, end="")
    return 0 if payload["overall_static_status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
