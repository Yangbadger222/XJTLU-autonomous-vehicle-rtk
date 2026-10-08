#!/usr/bin/env bash
set -eo pipefail
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=93 PYTHONDONTWRITEBYTECODE=1
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
export ROS_LOG_DIR=/research-ws/qualification/adapter-signal-ros TMPDIR=/research-ws/tmp
unset QEMU_STRACE PYTHONOPTIMIZE LD_PRELOAD FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
source /research-ws/fresh14/install/setup.bash
source /research-ws/terminal-overlay/install/setup.bash
source /research-ws/rigid-group-overlay/install/setup.bash
source /research-ws/mapping-signal-overlay/install/setup.bash
test "$(uname -m)" = aarch64
test "$(git -C /vehicle-research rev-parse HEAD)" = 6b15c7dda33d4110ba247cf37425abc67da1dd6d
cd /research-ws
colcon --log-base /research-ws/adapter-signal-overlay/log build --base-paths /research-ws/adapter-signal-source/super_lio_vehicle_adapter --packages-select super_lio_vehicle_adapter --build-base /research-ws/adapter-signal-overlay/build --install-base /research-ws/adapter-signal-overlay/install > /research-ws/qualification/adapter-signal-arm-build.log 2>&1
source /research-ws/adapter-signal-overlay/install/setup.bash
python3 /research-ws/adapter-signal-source/installed_source_receipt.py --source /vehicle-research/src --runtime-commit 6b15c7dda33d4110ba247cf37425abc67da1dd6d --output /research-ws/qualification/adapter-signal-arm-python-source.json
export RESEARCH_QEMU_FD_BROKER=/research-ws/socket-broker.sock LD_PRELOAD=/research-ws/qemu62-native-fd-shim.so FYP_RUNTIME_ROOT=/research-ws/adapter-signal-default-runtime
timeout --signal=INT --kill-after=15s 480s python3 /research-ws/adapter-signal-qa/acceptance-cli-timing.py --repo /vehicle-research --console-port 8897 --output /research-ws/qualification/adapter-signal-arm-default.json > /research-ws/qualification/adapter-signal-arm-default.stdout 2>&1
printf 'ARM64_ADAPTER_SIGNAL_QUALIFIED_COMPLETED\n'
