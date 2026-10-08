#!/usr/bin/env bash
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive MAKEFLAGS=-j1 CMAKE_BUILD_PARALLEL_LEVEL=1
export ROS_DOMAIN_ID=117 ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1
export ROS_LOG_DIR=/research-ws/ros-log FYP_RUNTIME_ROOT=/research-ws/runtime
printf 'architecture='; uname -m
test "$(uname -m)" = aarch64
grep -E '^(ID|VERSION_ID)=' /etc/os-release
apt-get -o Acquire::http::Proxy=http://127.0.0.1:17997 -o Acquire::https::Proxy=http://127.0.0.1:17997 -o Dir::Etc::sourcelist=/etc/apt/sources.list.d/ros2.sources -o Dir::Etc::sourceparts=- -o APT::Get::List-Cleanup=0 update
apt-get -o Acquire::http::Proxy=http://127.0.0.1:17997 -o Acquire::https::Proxy=http://127.0.0.1:17997 install -y --no-install-recommends \
 build-essential cmake git file libeigen3-dev libpcl-dev libgoogle-glog-dev libtbb-dev libyaml-cpp-dev libfmt-dev libboost-all-dev \
 python3-colcon-common-extensions python3-numpy python3-yaml python3-pytest python3-rasterio python3-pyproj \
 ros-humble-pcl-ros ros-humble-pcl-conversions ros-humble-tf2-sensor-msgs ros-humble-sensor-msgs-py ros-humble-xacro ros-humble-nmea-msgs
# Only newly created container apt download/list caches; no host assets.
rm -rf /var/lib/apt/lists/* /var/cache/apt/archives/*.deb
gcc -dumpmachine
test "$(gcc -dumpmachine)" = aarch64-linux-gnu
dpkg-query -W > /research-ws/installed-packages.txt
cd /vehicle-research
git rev-parse HEAD
python3 scripts/generate_research_parameter_lock.py --check
bash scripts/build_active_road_research.sh /research-ws/fresh14
printf 'ARM64_BUILD_COMPLETED\n'
