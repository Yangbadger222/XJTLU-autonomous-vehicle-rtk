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
  /rtk/health
  /rtk/status
  /rtk/nmea_sentence
  /heading
  /gnss
  /fastlio2/lio_odom
  /fastlio2/degeneracy
  /localization_authority/status
  /localization_authority/diagnostics
  /gps_goal_manager/status
  /gps_waypoint_dispatcher/path_map
  /gps_waypoint_dispatcher/goal_map
  /scan
  /local_costmap/costmap_raw
  /global_costmap/costmap_raw
  /pgo/optimized_odom
  /tf
  /tf_static
  /cmd_vel
  /cmd_vel_nav
  /cmd_vel_guarded
  /gps_nav/stop_override
  /behavior_tree_log
  /navigate_to_pose/_action/status
  /pgo/loop_markers
)

record_args=()
# Keep a bounded in-memory queue, split long recordings, and let rosbag close
# SQLite metadata cleanly after Ctrl-C instead of relying on process teardown.
record_args+=(
  --storage sqlite3
  --max-cache-size "${BAG_MAX_CACHE_SIZE_BYTES:-104857600}"
  --max-bag-size "${BAG_MAX_SIZE_BYTES:-1073741824}"
)

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
echo "cache=${BAG_MAX_CACHE_SIZE_BYTES:-104857600}B split=${BAG_MAX_SIZE_BYTES:-1073741824}B"
ros2 bag record -o "$BAG_PATH" "${record_args[@]}" "${topics[@]}" &
recorder_pid=$!

shutdown_recorder() {
  trap - INT TERM
  kill -INT "$recorder_pid" 2>/dev/null || true
  wait "$recorder_pid" || true
  exit 0
}

trap shutdown_recorder INT TERM
wait "$recorder_pid"
