#!/usr/bin/env bash
set -eo pipefail
export PYTHONDONTWRITEBYTECODE=1 CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1
unset QEMU_STRACE PYTHONOPTIMIZE LD_PRELOAD FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
source /research-ws/fresh14/install/setup.bash
test "$(uname -m)" = aarch64
test "$(git -C /vehicle-research rev-parse HEAD)" = 0848422f6bd21ff72933883e77a4502854aed07e
mkdir -p /research-ws/shutdown-overlay/src
ln -s /research-ws/shutdown-source/research_runtime /research-ws/shutdown-overlay/src/research_runtime
ln -s /research-ws/shutdown-source/super_lio_vehicle_adapter /research-ws/shutdown-overlay/src/super_lio_vehicle_adapter
cd /research-ws/shutdown-overlay
colcon build --packages-select research_runtime super_lio_vehicle_adapter --allow-overriding research_runtime super_lio_vehicle_adapter --parallel-workers 1
printf 'ARM64_SHUTDOWN_OVERLAY_BUILD_COMPLETED\n'
