#!/usr/bin/env bash
# Explicit source/package allowlist; never reuses the production install.
set -eo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
research_build_root="${1:?usage: build_active_road_research.sh ABSOLUTE_ISOLATED_BUILD_ROOT}"
case "$research_build_root" in /*) ;; *) echo "build root must be absolute" >&2; exit 2;; esac
test "$research_build_root" != "$repo_root" || exit 2
test -f /opt/ros/humble/setup.bash
# An inherited production/research overlay must not satisfy dependencies.
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH
unset PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
export MAKEFLAGS=-j1 CMAKE_BUILD_PARALLEL_LEVEL=1
mkdir -p "$research_build_root"
cmake -S "$repo_root/src/sensor_drivers/Livox-SDK2" -B "$research_build_root/sdk-build" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$research_build_root/sdk-install"
cmake --build "$research_build_root/sdk-build" --parallel 1
cmake --install "$research_build_root/sdk-build"
colcon --log-base "$research_build_root/log" build \
  --base-paths "$repo_root/src/bringup" "$repo_root/src/research_interfaces" \
    "$repo_root/src/research_runtime" "$repo_root/src/active_road_mapping" \
    "$repo_root/src/super_lio_vehicle_adapter" "$repo_root/src/ego_vehicle_adapter" \
    "$repo_root/src/super_lio/src" "$repo_root/src/ego_planner_2d_ros2/src/EgoPlanner-ROS2" \
    "$repo_root/src/sensor_drivers/livox_ros_driver2" "$repo_root/src/sensor_drivers/serial" \
    "$repo_root/src/sensor_drivers/serial_twistctl" "$repo_root/src/sensor_drivers/gnss/um982_rtk_driver" \
    "$repo_root/src/navigation/gps_waypoint_dispatcher" \
  --packages-select bringup research_interfaces research_runtime active_road_mapping \
    super_lio_vehicle_adapter ego_vehicle_adapter basic super_lio ego_planner \
    livox_ros_driver2 serial serial_twistctl um982_rtk_driver gps_waypoint_dispatcher \
  --build-base "$research_build_root/build" --install-base "$research_build_root/install" \
  --parallel-workers 1 --event-handlers console_direct+ \
  --cmake-args -DBUILD_TESTING=OFF -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble \
    -DLIVOX_LIDAR_SDK_LIBRARY="$research_build_root/sdk-install/lib/liblivox_lidar_sdk_shared.so" \
    -DLIVOX_LIDAR_SDK_INCLUDE_DIR="$research_build_root/sdk-install/include"
