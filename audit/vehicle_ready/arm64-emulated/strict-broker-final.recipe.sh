#!/usr/bin/env bash
set -eo pipefail
export PYTHONDONTWRITEBYTECODE=1 PYTHONOPTIMIZE=1 ROS_LOCALHOST_ONLY=1
unset QEMU_STRACE LD_PRELOAD FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
source /research-ws/fresh14/install/setup.bash
test "$(uname -m)" = aarch64
test "$(git -C /vehicle-research rev-parse HEAD)" = a66cc7e41c80f7999ddb3a1e4aec179cfa0e3c6f
python3 /research-ws/fd-broker-denials-optimize.py
export RESEARCH_QEMU_FD_BROKER=/research-ws/socket-broker.sock LD_PRELOAD=/research-ws/qemu62-native-fd-shim.so
timeout --signal=INT --kill-after=10s 150s python3 /research-ws/minimal-dds-probe.py --domain 158 --output /research-ws/qualification/minimal-dds-native-fd-strict-observed.json
printf 'ARM64_STRICT_BROKER_DIAG_COMPLETED\n'
