#!/usr/bin/env bash
set -eo pipefail
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=93 PYTHONDONTWRITEBYTECODE=1
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
export ROS_LOG_DIR=/research-ws/qualification/terminal-default-ros FYP_RUNTIME_ROOT=/research-ws/terminal-runtime TMPDIR=/research-ws/tmp
unset QEMU_STRACE PYTHONOPTIMIZE LD_PRELOAD FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
source /research-ws/fresh14/install/setup.bash
source /research-ws/terminal-overlay/install/setup.bash
test "$(uname -m)" = aarch64
test "$(git -C /vehicle-research rev-parse HEAD)" = 0848422f6bd21ff72933883e77a4502854aed07e
export RESEARCH_QEMU_FD_BROKER=/research-ws/socket-broker.sock LD_PRELOAD=/research-ws/qemu62-native-fd-shim.so
cd /vehicle-research
python3 scripts/generate_research_parameter_lock.py --check
python3 /research-ws/terminal-source/installed_source_receipt.py --source /research-ws/terminal-source --runtime-commit a7dc39774c4421b40a6a0abb7d1709a81bdc72b0 --output /research-ws/qualification/terminal-current-build.json
timeout --signal=INT --kill-after=15s 240s python3 /research-ws/terminal-source/validate_research_launch.py --repo /vehicle-research --console-port 8886 --output /research-ws/qualification/terminal-default-qualified.json
printf 'ARM64_SHUTDOWN_DEFAULT_QUALIFIED_COMPLETED\n'
