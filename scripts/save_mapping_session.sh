#!/bin/bash
set -euo pipefail

cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/save_mapping_session.py "$@"
