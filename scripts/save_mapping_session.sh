#!/bin/bash
set -euo pipefail

cd "$(dirname "$0")/.."
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u
python3 scripts/save_mapping_session.py "$@"
