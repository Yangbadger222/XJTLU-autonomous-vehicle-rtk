#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
upstream_dir="${repo_root}/src/ego_planner_2d_ros2"
expected_commit="7f5be6d4cee34871e85aa1f15285cfaf17b23877"
patch_file="${repo_root}/patches/ego_planner_2d/0001-vehicle-state-and-feasibility.patch"
ros_patch_file="${repo_root}/patches/ego_planner_2d/0002-vehicle-ros-timed-trajectory.patch"
stale_plan_patch_file="${repo_root}/patches/ego_planner_2d/0003-clear-stale-plan-on-failure.patch"
strict_contract_patch_file="${repo_root}/patches/ego_planner_2d/0004-strict-feasibility-and-grid-state-contract.patch"
world_gauge_patch_file="${repo_root}/patches/ego_planner_2d/0005-protect-owned-local-world-gauge.patch"
evidence_patch_file="${repo_root}/patches/ego_planner_2d/0006-atomic-local-evidence-grid-contract.patch"
local_frame_patch_file="${repo_root}/patches/ego_planner_2d/0007-local-perception-frame-integrity.patch"
firmware_patch_file="${repo_root}/patches/ego_planner_2d/0008-stm32-command-model-target-check.patch"
rotation_patch_file="${repo_root}/patches/ego_planner_2d/0009-explicit-stationary-heading-recovery.patch"

test -d "${upstream_dir}/.git" || { echo "missing vcs checkout: ${upstream_dir}" >&2; exit 2; }
test "$(git -C "${upstream_dir}" rev-parse HEAD)" = "${expected_commit}" || {
  echo "EGO checkout is not the pinned commit ${expected_commit}" >&2; exit 3;
}
test -z "$(git -C "${upstream_dir}" status --porcelain)" || {
  echo "EGO checkout is dirty; refusing to apply patch" >&2; exit 4;
}
git -C "${upstream_dir}" apply --check "${patch_file}"
git -C "${upstream_dir}" apply "${patch_file}"
git -C "${upstream_dir}" apply --check "${ros_patch_file}"
git -C "${upstream_dir}" apply "${ros_patch_file}"
git -C "${upstream_dir}" apply --check "${stale_plan_patch_file}"
git -C "${upstream_dir}" apply "${stale_plan_patch_file}"
git -C "${upstream_dir}" apply --check "${strict_contract_patch_file}"
git -C "${upstream_dir}" apply "${strict_contract_patch_file}"
git -C "${upstream_dir}" apply --check "${world_gauge_patch_file}"
git -C "${upstream_dir}" apply "${world_gauge_patch_file}"
git -C "${upstream_dir}" apply --check "${evidence_patch_file}"
git -C "${upstream_dir}" apply "${evidence_patch_file}"
git -C "${upstream_dir}" apply --check "${local_frame_patch_file}"
git -C "${upstream_dir}" apply "${local_frame_patch_file}"
git -C "${upstream_dir}" apply --check "${firmware_patch_file}"
git -C "${upstream_dir}" apply "${firmware_patch_file}"
git -C "${upstream_dir}" apply --check "${rotation_patch_file}"
git -C "${upstream_dir}" apply "${rotation_patch_file}"
git -C "${upstream_dir}" apply --check "${repo_root}/patches/ego_planner_2d/0010-explicit-chassis-control-reference.patch"
git -C "${upstream_dir}" apply "${repo_root}/patches/ego_planner_2d/0010-explicit-chassis-control-reference.patch"
git -C "${upstream_dir}" apply --check "${repo_root}/patches/ego_planner_2d/0011-original-stop-tolerance-and-measured-yaw-boundary.patch"
git -C "${upstream_dir}" apply "${repo_root}/patches/ego_planner_2d/0011-original-stop-tolerance-and-measured-yaw-boundary.patch"
echo "Applied Ego-Planner-2D-ROS2 vehicle core, ROS timed-trajectory and stale-plan safety patches"
