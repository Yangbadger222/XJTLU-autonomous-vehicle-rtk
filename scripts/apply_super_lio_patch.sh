#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
upstream_dir="${repo_root}/src/super_lio"
expected_commit="f89f48dc7aea6cfa262f18e4d03b319e04e0dbd2"
patch_file="${repo_root}/patches/super_lio/0001-publish-source-aware-odom-health.patch"
quality_patch="${repo_root}/patches/super_lio/0002-certify-source-observations-and-covariance.patch"

test -d "${upstream_dir}/.git" || { echo "missing vcs checkout: ${upstream_dir}" >&2; exit 2; }
test "$(git -C "${upstream_dir}" rev-parse HEAD)" = "${expected_commit}" || {
  echo "Super-LIO checkout is not the pinned commit ${expected_commit}" >&2; exit 3;
}
test -z "$(git -C "${upstream_dir}" status --porcelain)" || {
  echo "Super-LIO checkout is dirty; refusing to apply patch" >&2; exit 4;
}
git -C "${upstream_dir}" apply --check "${patch_file}"
git -C "${upstream_dir}" apply "${patch_file}"
git -C "${upstream_dir}" apply --check "${quality_patch}"
git -C "${upstream_dir}" apply "${quality_patch}"
echo "Applied Super-LIO source-aware odom and conservative observation certificate patches"
