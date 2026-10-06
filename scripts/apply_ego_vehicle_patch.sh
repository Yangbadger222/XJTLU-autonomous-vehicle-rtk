#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
upstream_dir="${repo_root}/src/ego_planner_2d_ros2"
expected_commit="7f5be6d4cee34871e85aa1f15285cfaf17b23877"
patch_file="${repo_root}/patches/ego_planner_2d/0001-vehicle-state-and-feasibility.patch"
ros_patch_file="${repo_root}/patches/ego_planner_2d/0002-vehicle-ros-timed-trajectory.patch"

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
echo "Applied Ego-Planner-2D-ROS2 vehicle core and ROS timed-trajectory adaptation patches"
