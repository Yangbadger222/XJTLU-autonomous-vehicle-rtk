#!/usr/bin/env bash
set -eo pipefail
export PYTHONDONTWRITEBYTECODE=1 ROS_LOCALHOST_ONLY=1 CMAKE_BUILD_PARALLEL_LEVEL=1
unset QEMU_STRACE LD_PRELOAD FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
source /research-ws/fresh14/install/setup.bash
test "$(uname -m)" = aarch64
test "$(git -C /vehicle-research rev-parse HEAD)" = a66cc7e41c80f7999ddb3a1e4aec179cfa0e3c6f
cmake -S /research-ws/shadow-source -B /research-ws/shadow-build -DCMAKE_BUILD_TYPE=Release
cmake --build /research-ws/shadow-build --parallel 1
file /research-ws/shadow-build/research_shadow_sensor_ingress
sha256sum /research-ws/shadow-build/research_shadow_sensor_ingress
printf 'ARM64_AUX_SHADOW_BUILD_COMPLETED\n'
