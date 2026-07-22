#!/bin/bash
set -euo pipefail

MODE="${1:-explore}"
EXTRA_LAUNCH_ARGS=("${@:2}")
SESSION=$(date +%Y-%m-%d-%H-%M-%S)
SESSION_DIR="$HOME/XJTLU-autonomous-vehicle/runtime-data/logs/$SESSION"
TEGRA_PID=""
LAUNCH_PID=""
CLEANUP_DONE=0

mkdir -p "$SESSION_DIR/console"
mkdir -p "$SESSION_DIR/data"
mkdir -p "$SESSION_DIR/system"
ln -sfn "$SESSION_DIR" "$HOME/XJTLU-autonomous-vehicle/runtime-data/logs/latest"

export ROS_LOG_DIR="$SESSION_DIR/console"
export FYP_LOG_SESSION_DIR="$SESSION_DIR/data"

cleanup_runtime_nodes() {
  pkill -INT -f '[r]os2 launch|[r]os2 bag|[r]viz2|[l]ivox_ros_driver2_node|[l]io_node|[l]ocalizer_node|[p]go_node|[r]tk_fgo_node|[f]go_gil_time_sync_node|[f]go_gil_lidar_frontend_node|[f]go_gil_float_fgo_node|[f]go_map_odom_corrector|[r]tk_map_odom_corrector|[c]orridor_cmd_vel_guard|[s]erial_twistctl_node|[s]erial_reader_node|[n]mea_serial_driver|[u]m982_rtk_node|[p]lanner_server|[c]ontroller_server|[b]ehavior_server|[b]t_navigator|[s]moother_server|[v]elocity_smoother|[l]ifecycle_manager|[w]aypoint_follower|[m]ap_server|[a]mcl|[c]omponent_container(_mt)?|[g]ps_route_runner|[g]ps_global_aligner|[g]ps_anchor_localizer|[r]oute_server|[g]oal_manager_node|[r]obot_state_publisher|[j]oint_state_publisher|[p]ointcloud_to_laserscan|[a]sync_slam_toolbox_node|[m]ap_saver_server|[m]onitor_corridor_status|[f]rc_health_aggregator|[f]rc_event_marker|[f]rc_risk_pipeline|[f]rc_memory_manager|[f]rc_trial_runner' 2>/dev/null || true
  sleep 1
  pkill -KILL -f '[r]os2 launch|[r]os2 bag|[r]viz2|[l]ivox_ros_driver2_node|[l]io_node|[l]ocalizer_node|[p]go_node|[r]tk_fgo_node|[f]go_gil_time_sync_node|[f]go_gil_lidar_frontend_node|[f]go_gil_float_fgo_node|[f]go_map_odom_corrector|[r]tk_map_odom_corrector|[c]orridor_cmd_vel_guard|[s]erial_twistctl_node|[s]erial_reader_node|[n]mea_serial_driver|[u]m982_rtk_node|[p]lanner_server|[c]ontroller_server|[b]ehavior_server|[b]t_navigator|[s]moother_server|[v]elocity_smoother|[l]ifecycle_manager|[w]aypoint_follower|[m]ap_server|[a]mcl|[c]omponent_container(_mt)?|[g]ps_route_runner|[g]ps_global_aligner|[g]ps_anchor_localizer|[r]oute_server|[g]oal_manager_node|[r]obot_state_publisher|[j]oint_state_publisher|[p]ointcloud_to_laserscan|[a]sync_slam_toolbox_node|[m]ap_saver_server|[m]onitor_corridor_status|[f]rc_health_aggregator|[f]rc_event_marker|[f]rc_risk_pipeline|[f]rc_memory_manager|[f]rc_trial_runner' 2>/dev/null || true
  ros2 daemon stop 2>/dev/null || true
  for dev in /dev/serial_twistctl /dev/wheeltec_gps /dev/rtk_um982; do
    if [ -e "$dev" ] && fuser "$dev" >/dev/null 2>&1; then
      fuser -k "$dev" 2>/dev/null || true
    fi
  done
}

cleanup() {
  if [[ "$CLEANUP_DONE" == "1" ]]; then
    return
  fi
  CLEANUP_DONE=1

  if [[ -n "${TEGRA_PID:-}" ]]; then
    kill "$TEGRA_PID" 2>/dev/null || true
    wait "$TEGRA_PID" 2>/dev/null || true
  fi

  if [[ -n "${LAUNCH_PID:-}" ]]; then
    kill "$LAUNCH_PID" 2>/dev/null || true
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi

  cleanup_runtime_nodes

  if [[ -f "$SESSION_DIR/system/session_info.yaml" ]]; then
    echo "end_time: $(date -Iseconds)" >> "$SESSION_DIR/system/session_info.yaml"
  fi
}
trap cleanup EXIT INT TERM

tegrastats --interval 1000 --logfile "$SESSION_DIR/system/tegrastats.log" &
TEGRA_PID=$!

cat > "$SESSION_DIR/system/session_info.yaml" <<EOF
mode: $MODE
start_time: $(date -Iseconds)
session_dir: $SESSION_DIR
git_branch: $(cd ~/XJTLU-autonomous-vehicle && git branch --show-current 2>/dev/null || echo unknown)
git_commit: $(cd ~/XJTLU-autonomous-vehicle && git rev-parse --short HEAD 2>/dev/null || echo unknown)
frc_mode: ${FRC_MODE:-off}
ros_log_dir: $SESSION_DIR/console
data_log_dir: $SESSION_DIR/data
system_log_dir: $SESSION_DIR/system
EOF

echo "=== Log session: $SESSION ==="
echo "  Console: $SESSION_DIR/console/"
echo "  Data:    $SESSION_DIR/data/"
echo "  System:  $SESSION_DIR/system/"
echo "=== Launching mode: $MODE ==="

cleanup_runtime_nodes

set +u
source /opt/ros/humble/setup.bash
source ~/XJTLU-autonomous-vehicle/install/setup.bash
set -u

case "$MODE" in
  slam)         LAUNCH_FILE="system_slam.launch.py" ;;
  explore)      LAUNCH_FILE="system_explore.launch.py" ;;
  indoor-nav)   LAUNCH_FILE="system_explore.launch.py" ;;
  corridor)     LAUNCH_FILE="system_gps_corridor.launch.py" ;;
  corridor-fgo) LAUNCH_FILE="system_gps_corridor.launch.py" ;;
  travel)       LAUNCH_FILE="system_travel.launch.py" ;;
  explore-gps)  LAUNCH_FILE="system_explore_gps.launch.py" ;;
  nav-gps)      LAUNCH_FILE="system_nav_gps.launch.py" ;;
  rtk-basic)    LAUNCH_FILE="system_rtk_basic.launch.py" ;;
  tightly-coupled) LAUNCH_FILE="system_tightly_coupled.launch.py" ;;
  *)            echo "Unknown mode: $MODE"; exit 1 ;;
esac

LAUNCH_ARGS=()
# FRC 双锚风险记忆栈：FRC_MODE=shadow|full 时透传（仅 explore 系模式支持）
if [[ -n "${FRC_MODE:-}" && ( "$MODE" == "explore" || "$MODE" == "indoor-nav" || "$MODE" == "explore-gps" ) ]]; then
  LAUNCH_ARGS+=("frc_mode:=${FRC_MODE}")
  if [[ -n "${FRC_EXTRA_PARAMS:-}" ]]; then
    LAUNCH_ARGS+=("frc_extra_params_file:=${FRC_EXTRA_PARAMS}")
  fi
fi
if [[ -n "${FYP_RTK_PARAMS_FILE:-}" ]]; then
  case "$MODE" in
    rtk-basic)
      LAUNCH_ARGS+=("params_file:=${FYP_RTK_PARAMS_FILE}")
      ;;
    explore-gps|nav-gps|corridor|corridor-fgo|tightly-coupled)
      LAUNCH_ARGS+=("rtk_params_file:=${FYP_RTK_PARAMS_FILE}")
      ;;
  esac
fi
if [[ "$MODE" == "corridor-fgo" ]]; then
  LAUNCH_ARGS+=("fgo_authority:=true")
  if [[ -n "${FYP_FGO_GIL_PARAMS_FILE:-}" ]]; then
    LAUNCH_ARGS+=("fgo_params_file:=${FYP_FGO_GIL_PARAMS_FILE}")
  fi
fi
if [[ "$MODE" == "corridor" || "$MODE" == "corridor-fgo" || "$MODE" == "nav-gps" || "$MODE" == "indoor-nav" || "$MODE" == "tightly-coupled" || "$MODE" == "travel" ]]; then
  if [[ -n "${FYP_USE_RVIZ:-}" ]]; then
    LAUNCH_ARGS+=("use_rviz:=${FYP_USE_RVIZ}")
  elif [[ -n "${DISPLAY:-}" || -n "${WAYLAND_DISPLAY:-}" ]]; then
    LAUNCH_ARGS+=("use_rviz:=true")
  else
    LAUNCH_ARGS+=("use_rviz:=false")
  fi
fi

if [[ ( "$MODE" == "corridor" || "$MODE" == "corridor-fgo" ) && "${FYP_CORRIDOR_CONSOLE_MODE:-quiet}" != "raw" ]]; then
  ROUTE_FILE="$HOME/XJTLU-autonomous-vehicle/runtime-data/gnss/current_route.yaml"
  ROUTE_FIX_TIMEOUT_S=""
  if [[ -f "$ROUTE_FILE" ]]; then
    ROUTE_FIX_TIMEOUT_S="$(
      awk -F: '
        /^[[:space:]]*startup_fix_timeout_s[[:space:]]*:/ {
          value=$2
          gsub(/[[:space:]]/, "", value)
          if (value != "") {
            print value
          }
        }
      ' "$ROUTE_FILE" | tail -n 1
    )"
  fi

  if [[ -n "${FYP_CORRIDOR_STARTUP_TIMEOUT_S:-}" ]]; then
    STARTUP_TIMEOUT_S="${FYP_CORRIDOR_STARTUP_TIMEOUT_S}"
  elif [[ -n "$ROUTE_FIX_TIMEOUT_S" ]]; then
    STARTUP_TIMEOUT_S="$(awk "BEGIN { printf \"%.0f\", (${ROUTE_FIX_TIMEOUT_S} + 30.0) }")"
  else
    STARTUP_TIMEOUT_S="45"
  fi

  LAUNCH_STDOUT_LOG="$SESSION_DIR/system/launch_stdout.log"
  echo "=== Corridor Console: quiet ==="
  echo "  Status: foreground concise monitor"
  echo "  Full launch output: $LAUNCH_STDOUT_LOG"
  echo "  Startup timeout: ${STARTUP_TIMEOUT_S}s"
  if [[ -n "$ROUTE_FIX_TIMEOUT_S" && -z "${FYP_CORRIDOR_STARTUP_TIMEOUT_S:-}" ]]; then
    echo "  Route stable-fix timeout: ${ROUTE_FIX_TIMEOUT_S}s (+30s buffer)"
  fi

  ros2 launch bringup "$LAUNCH_FILE" "${LAUNCH_ARGS[@]}" "${EXTRA_LAUNCH_ARGS[@]}" >"$LAUNCH_STDOUT_LOG" 2>&1 &
  LAUNCH_PID=$!

  set +e
  python3 ~/XJTLU-autonomous-vehicle/scripts/monitor_corridor_status.py \
    --startup-timeout-s "$STARTUP_TIMEOUT_S" \
    --launch-log "$LAUNCH_STDOUT_LOG" \
    --launch-pid "$LAUNCH_PID"
  MONITOR_RC=$?
  set -e

  if [[ -n "${LAUNCH_PID:-}" ]]; then
    kill "$LAUNCH_PID" 2>/dev/null || true
    wait "$LAUNCH_PID" 2>/dev/null || true
    LAUNCH_PID=""
  fi
  exit "$MONITOR_RC"
fi

ros2 launch "${LAUNCH_PACKAGE:-bringup}" "$LAUNCH_FILE" "${LAUNCH_ARGS[@]}" "${EXTRA_LAUNCH_ARGS[@]}"
