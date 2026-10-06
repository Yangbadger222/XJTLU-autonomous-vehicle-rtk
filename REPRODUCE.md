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
PYTHONPATH=src/research_runtime:src/active_road_mapping:src/super_lio_vehicle_adapter \
  python3 -m pytest -q src/research_runtime/test src/super_lio_vehicle_adapter/test
PYTHONPATH=src/research_runtime python3 -m research_runtime.replay_sim \
  --output runtime-data/research/active_road/replay_smoke.json
PYTHONPATH=src/research_runtime python3 -c 'import sys; sys.argv=["research_safety_bridge","--mode","replay","--output","/tmp/research-safety-bridge-replay.json"]; from research_runtime.safety_bridge import main; raise SystemExit(main())'
```

On Jetson, use a clean worktree and the exact commit, source ROS 2 Humble,
build with one worker, and run the new launch with `execution_mode:=shadow`.
Shadow/live starts the pinned Super-LIO node, the patched EGO vehicle ROS edge,
the source-aware odometry adapter, the timestamped cloud-frame adapter,
`active_road_map`, the typed `active_road_evidence` ingest/policy boundary,
and the fail-closed local obstacle grid node; replay
intentionally does not start sensor,
authority, planner or serial processes. The physical serial sink requires the
separate explicit gate `execution_mode:=live enable_serial:=true`; never attach a
production serial device to replay or shadow. `live` remains blocked until the
acceptance checklist is completed by a human operator.

The recorded ARM64 compile evidence for EGO patches 0001+0002 is reproducible
with `audit/container/ego-current-build.Dockerfile`; its completed BuildKit log
is `audit/container/ego-current-build.log`. Patch 0003 is included in the
exact sparse-checkout apply audit, but the current three-patch rebuild is still
pending. The recipe proves compilation only and does not replace Jetson shadow
or live acceptance.

The complete current three-patch ARM64 recipe is
`audit/container/ego-current-three-patch-build.Dockerfile`. When the Docker
daemon is available, run it from the repository root with:

```bash
docker build --platform linux/arm64 \
  -f audit/container/ego-current-three-patch-build.Dockerfile \
  -t ego-current-three-patch:research .
```

It applies 0001, 0002 and 0003 at the pinned EGO commit before building
`research_interfaces` and `ego_planner`; until that command produces a
complete log, the current three-patch build remains pending.

An isolated ARM64 OrbStack run using the public `ros:humble` base has now
completed this three-patch build and startup smoke. The reproducible recipe is
`audit/container/ego-orbstack-ros-base-three-patch.Dockerfile`; evidence is in
`audit/container/ego-orbstack-ros-base-three-patch-build.log`,
`audit/container/ego-orbstack-ros-base-three-patch-build-summary.log`,
`audit/container/ego-orbstack-ros-base-three-patch-run.log`, and the structured
result JSON beside them. The node starts and waits for measured odom, road
reference, obstacle grid and map version, then exits only at the five-second
smoke timeout. This is an alternate-base compile/start result; it does not
prove the unavailable `ros2-go2:humble` image or Jetson runtime.

The current Python package install path can be checked without ROS:

```bash
python3 -m pip wheel --no-deps --wheel-dir <temporary-dir> \
  src/research_runtime src/active_road_mapping
```

The installed entry points include `active_road_map`,
`active_road_evidence`, `research_local_obstacle_grid`, and
`research_safety_bridge`. `RoadEvidence2D` accepts only measured odom-frame
geometry; the evidence node refuses a missing/unknown persisted map identity
and writes accepted UUIDs atomically/idempotently.
