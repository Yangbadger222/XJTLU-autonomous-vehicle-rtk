#!/usr/bin/env bash
set -eo pipefail
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=93 PYTHONDONTWRITEBYTECODE=1
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
export ROS_LOG_DIR=/research-ws/qualification/ground-signal-ros TMPDIR=/research-ws/tmp
unset QEMU_STRACE PYTHONOPTIMIZE LD_PRELOAD FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
source /research-ws/fresh14/install/setup.bash
source /research-ws/terminal-overlay/install/setup.bash
source /research-ws/rigid-group-overlay/install/setup.bash
source /research-ws/mapping-signal-overlay/install/setup.bash
source /research-ws/adapter-signal-overlay/install/setup.bash
test "$(uname -m)" = aarch64
test "$(git -C /vehicle-research rev-parse HEAD)" = 77d5c5a5ce26b52b22d5631d67ba88a71ad410b3
cd /research-ws
colcon --log-base /research-ws/ground-signal-overlay/log build --base-paths /research-ws/ground-signal-source/research_runtime --packages-select research_runtime --build-base /research-ws/ground-signal-overlay/build --install-base /research-ws/ground-signal-overlay/install > /research-ws/qualification/ground-signal-arm-build.log 2>&1
source /research-ws/ground-signal-overlay/install/setup.bash
python3 /research-ws/ground-signal-source/installed_source_receipt.py --source /vehicle-research/src --runtime-commit 77d5c5a5ce26b52b22d5631d67ba88a71ad410b3 --output /research-ws/qualification/ground-signal-arm-python-source.json
export RESEARCH_QEMU_FD_BROKER=/research-ws/socket-broker.sock LD_PRELOAD=/research-ws/qemu62-native-fd-shim.so FYP_RUNTIME_ROOT=/research-ws/ground-signal-default-runtime
timeout --signal=INT --kill-after=15s 480s python3 /research-ws/ground-signal-qa/acceptance-cli-timing.py --repo /vehicle-research --console-port 8897 --output /research-ws/qualification/ground-signal-arm-default.json > /research-ws/qualification/ground-signal-arm-default.stdout 2>&1
printf 'ARM64_GROUND_SIGNAL_QUALIFIED_COMPLETED\n'
