#!/usr/bin/env bash
set -eo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <bag-directory> [ros2 bag play options]" >&2
  exit 2
fi

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BAG_PATH="$1"
shift

source /opt/ros/humble/setup.bash
source "$REPO_ROOT/install/setup.bash"
set -u

exec ros2 bag play "$BAG_PATH" "$@" --clock --topics \
  /livox/lidar \
  /livox/imu \
  /fastlio2/lio_odom \
  /gnss/raw/observation_epoch \
  /gnss/raw/ephemeris \
  /gnss/pps/time_reference \
  /livox/imu_time_reference \
  /livox/lidar_time_reference
