#!/usr/bin/env bash
set -eo pipefail
export DEBIAN_FRONTEND=noninteractive MAKEFLAGS=-j1 CMAKE_BUILD_PARALLEL_LEVEL=1
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=93 PYTHONDONTWRITEBYTECODE=1
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
export ROS_LOG_DIR=/research-ws/qualification/ros FYP_RUNTIME_ROOT=/research-ws/runtime
export TMPDIR=/research-ws/tmp
mkdir -p /research-ws/qualification /research-ws/tmp
# Supplement the runtime-only joint_state_publisher inside this task container.
dpkg-query -W > /research-ws/qualification/installed-packages.txt
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
source /research-ws/fresh14/install/setup.bash
test "$(uname -m)" = aarch64
test "$(git -C /vehicle-research rev-parse HEAD)" = a66cc7e41c80f7999ddb3a1e4aec179cfa0e3c6f
cd /vehicle-research
python3 scripts/generate_research_parameter_lock.py --check
python3 scripts/validate_research_launch.py --repo /vehicle-research --console-port 8886 --output /research-ws/qualification/default-entry.json
printf 'ARM64_DEFAULT_ENTRY_COMPLETED\n'
