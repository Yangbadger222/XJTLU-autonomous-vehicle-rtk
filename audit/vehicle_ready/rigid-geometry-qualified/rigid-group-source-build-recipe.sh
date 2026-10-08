#!/usr/bin/env bash
set -eo pipefail
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=105 PYTHONDONTWRITEBYTECODE=1
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
export ROS_LOG_DIR=/research-ws/qualification/rigid-group-ros TMPDIR=/research-ws/tmp
unset QEMU_STRACE PYTHONOPTIMIZE LD_PRELOAD FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
source /research-ws/fresh14/install/setup.bash
source /research-ws/terminal-overlay/install/setup.bash
test "$(uname -m)" = aarch64
test "$(git -C /vehicle-research rev-parse HEAD)" = 49b44ee17812f8bc8dd124799d06cc0e2283eb56
mkdir -p /research-ws/rigid-group-overlay
cd /research-ws/rigid-group-overlay
timeout --signal=INT --kill-after=20s 1200s colcon build --base-paths /research-ws/rigid-group-source/basic /research-ws/rigid-group-source/super_lio --packages-select basic super_lio --parallel-workers 1
printf 'ARM64_RIGID_GROUP_BUILD_COMPLETED\n'
