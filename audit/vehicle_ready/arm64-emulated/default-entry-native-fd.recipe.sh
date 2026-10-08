#!/usr/bin/env bash
set -eo pipefail
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=93 PYTHONDONTWRITEBYTECODE=1
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
export ROS_LOG_DIR=/research-ws/qualification/default-fd-ros FYP_RUNTIME_ROOT=/research-ws/runtime TMPDIR=/research-ws/tmp
unset QEMU_STRACE LD_PRELOAD FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
source /research-ws/fresh14/install/setup.bash
test "$(uname -m)" = aarch64
test "$(git -C /vehicle-research rev-parse HEAD)" = a66cc7e41c80f7999ddb3a1e4aec179cfa0e3c6f
export RESEARCH_QEMU_FD_BROKER=/research-ws/socket-broker.sock LD_PRELOAD=/research-ws/qemu62-native-fd-shim.so
cd /vehicle-research
python3 scripts/generate_research_parameter_lock.py --check
timeout --signal=INT --kill-after=15s 240s python3 scripts/validate_research_launch.py --repo /vehicle-research --console-port 8886 --output /research-ws/qualification/default-entry-native-fd.json
printf 'ARM64_DEFAULT_FD_ENTRY_COMPLETED\n'
timeout --signal=INT --kill-after=15s 100s python3 /research-ws/shadow-source/validate_shadow_sensor_ingress_ros.py --launcher /research-ws/shadow-source/run_shadow_sensor_ingress.py --executable /research-ws/shadow-build/research_shadow_sensor_ingress --source-root /research-ws/shadow-source --output /research-ws/qualification/shadow-native-fd-qualification.json
printf 'ARM64_SHADOW_FD_QUALIFICATION_COMPLETED\n'
