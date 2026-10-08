#!/usr/bin/env bash
# Reconstructed reproduction recipe; not an OS command capture.
# Actual results and exec exit receipts are separate. Reserve fresh paths first.
set -e
if [[ $# -ne 2 ]]; then
  printf '%s\n' 'Usage: bash reproduce-six.sh /absolute/fresh-output-dir /absolute/fresh-runtime-dir' >&2
  exit 2
fi
trial_output_dir="$1"
trial_runtime_dir="$2"
python3 - "$trial_output_dir" "$trial_runtime_dir" <<'PY_FRESH'
import os,sys
from pathlib import Path
a,b=[Path(v).expanduser().resolve() for v in sys.argv[1:]]
if not all(Path(v).is_absolute() for v in sys.argv[1:]):
 raise SystemExit('requires absolute paths')
if a==b or a in b.parents or b in a.parents:
 raise SystemExit('output and runtime roots must be separate nonnested paths')
if any(os.path.lexists(v) for v in (a,b)):
 raise SystemExit('refusing existing output/runtime root; retained evidence must not be overwritten or reused')
a.mkdir(parents=True,exist_ok=False)
b.mkdir(parents=True,exist_ok=False)
print('reserved fresh output and runtime roots')
PY_FRESH
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
source /dev/shm/codex-stop-fresh-ws/install/setup.bash
source /dev/shm/codex-shutdown-native-ws/install/setup.bash
source /home/badger/codex-research/superlio-ego-20261007/research-venv/bin/activate
export ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1 TMPDIR=/dev/shm
source /dev/shm/codex-terminal-native-ws/install/setup.bash
source /dev/shm/codex-super-unit-q-ws/install/setup.bash
source /dev/shm/codex-super-rigid-group-ws/install/setup.bash
source /dev/shm/codex-mapping-signal-native-ws/install/setup.bash
source /dev/shm/codex-adapter-signal-native-ws/install/setup.bash
source /dev/shm/codex-ground-signal-native-ws/install/setup.bash
export ROS_LOG_DIR=/home/badger/codex-research/superlio-ego-20261007/logs/tf-ground-diagnosis/ros
export ROS_DOMAIN_ID=94
cd /dev/shm/codex-shadow-delivery
export FYP_RUNTIME_ROOT="$trial_runtime_dir"
timeout --signal=INT --kill-after=25s 540s python3 scripts/validate_restricted_policy_ros.py --repo /dev/shm/codex-shadow-delivery --install /dev/shm/codex-ground-signal-native-fixture-install --budget-s 60 --default-profile --output "$trial_output_dir/result.json"
