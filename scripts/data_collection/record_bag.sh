#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_ROOT="${WORKSPACE_ROOT:-$HOME/XJTLU-autonomous-vehicle}"
if [[ $# -gt 0 ]]; then
  OUTPUT_ROOT="$1"
elif [[ -n "${FYP_LOG_SESSION_DIR:-}" ]]; then
  OUTPUT_ROOT="$FYP_LOG_SESSION_DIR"
else
  OUTPUT_ROOT="$HOME/XJTLU-autonomous-vehicle/runtime-data/bags/run_$(date +%Y%m%d_%H%M%S)"
fi
BAG_PATH="$OUTPUT_ROOT/rosbag2"
PROFILE="${PROFILE:-default}"

mkdir -p "$OUTPUT_ROOT"
source /opt/ros/humble/setup.bash
source "$WORKSPACE_ROOT/install/setup.bash"

topics=(
  /livox/lidar
  /livox/imu
  /fix
  /gnss
  /fastlio2/lio_odom
  /pgo/optimized_odom
  /tf
  /tf_static
  /cmd_vel
  /pgo/loop_markers
)

record_args=()

case "$PROFILE" in
  default)
    ;;
  frc)
    topics+=(
      /fastlio2/body_cloud
      /local_costmap/costmap_raw
      /plan
      /cmd_vel_nav
      /odom_CBoar
      /behavior_tree_log
      /chassis/status
      /frc/health
      /frc/event_marker
      /frc/risk_grid
      /pgo/keyframes
      /pgo/correction_status
      /fastlio2/degeneracy
      /navigate_to_pose/_action/status
    )
    record_args+=(--compression-mode file --compression-format zstd)
    ;;
  *)
    echo "Unknown PROFILE=$PROFILE (supported: default, frc)" >&2
    exit 1
    ;;
esac

echo "Recording bag to $BAG_PATH (PROFILE=$PROFILE)"
exec ros2 bag record -o "$BAG_PATH" "${record_args[@]}" "${topics[@]}"
