# Reproduce

```bash
git clone https://github.com/Yangbadger222/XJTLU-autonomous-vehicle-rtk.git
git switch --detach e54c6afbcb5a58db22d7c468085a87d658b0b932
git switch -c codex/superlio-ego-active-road
vcs import src < dependencies.research.repos
scripts/apply_super_lio_patch.sh
scripts/apply_ego_vehicle_patch.sh
colcon build --symlink-install --parallel-workers 1
source install/setup.bash
```

Before deployment, extract the source and corridor launch layers (the target
runtime layer is intentionally pending until a Jetson `ros2 param dump`):

```bash
python3 scripts/audit_vehicle_baseline.py
```

The local actuator-free smoke test requires only Python:

```bash
PYTHONPATH=src/research_runtime python3 -m pytest -q src/research_runtime/test
PYTHONPATH=src/research_runtime python3 -m research_runtime.replay_sim \
  --output runtime-data/research/active_road/replay_smoke.json
```

On Jetson, use a clean worktree and the exact commit, source ROS 2 Humble,
build with one worker, and run the new launch with `execution_mode:=shadow`.
Shadow/live starts the pinned Super-LIO node; replay intentionally does not
start sensor processes. Do not attach a production serial device to a replay container. `live` remains
blocked until the acceptance checklist is completed by a human operator.
