# Corridor Authority Stability Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Keep corridor navigation stable when RTK heading, quality, or global correction changes without hiding real FAST-LIO2 divergence.

**Architecture:** Pure Python modules own stamped LIO interpolation, Fixed-quality association, correction gates, release limits, watchdog classification, and velocity containment. ROS nodes own queues, action cancellation/retry, TF publication, heartbeats, and launch lifecycle; the corrector remains the only corridor `map -> odom` authority.

**Tech Stack:** Python 3, ROS 2 Humble `rclpy`, Nav2 actions, tf2, pytest, YAML, rosbag2 replay.

---

## Chunk 1: Authority Mathematics And State

### Task 1: Stamped LIO history and deterministic correction gates

**Files:**
- Modify: `src/navigation/gps_waypoint_dispatcher/gps_waypoint_dispatcher/rtk_authority.py`
- Modify: `src/navigation/gps_waypoint_dispatcher/test/test_rtk_authority.py`

- [ ] **Step 1: Write failing pure-unit tests**

Add tests for `StampedPoseHistory`: frame/stamp/finite rejection, bounded 2 s/200 samples, linear/circular interpolation, no extrapolation, and rejection when bracket span exceeds 0.20 s. Add `CorrectionGate`: `UNINITIALIZED/LOCKED/SUSPECT/REACQUIRING/DEGRADED`, one locked innovation rejection, five consecutive recovery samples spanning at least 0.30 s, window replacement on inconsistency, 20-sample cap, circular yaw mean, translation arithmetic mean, and persistent prerequisite failure.

- [ ] **Step 2: Run focused tests and verify RED**

Run: `PYTHONPATH=src/navigation/gps_waypoint_dispatcher python3 -m pytest src/navigation/gps_waypoint_dispatcher/test/test_rtk_authority.py -q`

Expected: imports fail for the new history/gate interfaces.

- [ ] **Step 3: Implement minimal pure state objects**

Use dataclasses and enums with no ROS imports. Preserve existing pose helpers used by legacy tests. Represent all yaw errors with shortest angular distance; do not average Euler angles arithmetically.

- [ ] **Step 4: Run and verify GREEN**

Run the focused command above.

Expected: all old and new authority tests pass.

- [ ] **Step 5: Commit explicitly**

```bash
git add src/navigation/gps_waypoint_dispatcher/gps_waypoint_dispatcher/rtk_authority.py src/navigation/gps_waypoint_dispatcher/test/test_rtk_authority.py
git commit -m "feat: add timestamped RTK correction gates"
```

### Task 2: Coupled base-space release and backlog state

**Files:**
- Modify: `src/navigation/gps_waypoint_dispatcher/gps_waypoint_dispatcher/rtk_authority.py`
- Modify: `src/navigation/gps_waypoint_dispatcher/test/test_rtk_authority.py`

- [ ] **Step 1: Write failing release tests**

Specify monotonic `dt<=0` freeze, 0.10 s cap, base-space 0.20 m/s and 2 deg/s limits, ordinary/moderate/fault backlog classification, fresh advancing LIO requirement, 1 s stopped confirmation, normal-rate stopped release, and 1 s recovery below 0.15 m/2 degrees.

- [ ] **Step 2: Run and verify RED**

Run the Task 1 pytest command and confirm failures are the missing release state.

- [ ] **Step 3: Implement `CorrectionReleaseState`**

Compose previous/target `map -> odom` with current local pose, limit the Nav2-visible base delta, then solve `map -> odom`. Keep target evaluation separate from output release. Return explicit `NORMAL`, `CORRECTION_BACKLOG`, and `FAULT_HOLD` results plus `motion_allowed`.

- [ ] **Step 4: Run and verify GREEN**

Run the focused authority tests.

- [ ] **Step 5: Commit explicitly**

```bash
git add src/navigation/gps_waypoint_dispatcher/gps_waypoint_dispatcher/rtk_authority.py src/navigation/gps_waypoint_dispatcher/test/test_rtk_authority.py
git commit -m "fix: rate limit global correction in vehicle space"
```

## Chunk 2: ROS Authority And Route Containment

### Task 3: Refactor the RTK corrector around stamped observations

**Files:**
- Modify: `src/navigation/gps_waypoint_dispatcher/gps_waypoint_dispatcher/rtk_map_odom_corrector_node.py`
- Modify: `src/navigation/gps_waypoint_dispatcher/test/test_rtk_authority.py`
- Create: `src/sensor_drivers/gnss/nmea_navsat_driver/test/test_corridor_gga_quality.py`
- Modify: `src/navigation/gps_waypoint_dispatcher/package.xml`

- [ ] **Step 1: Write failing node-contract tests**

Assert subscriptions to `/fastlio2/lio_odom` and `/rtk/nmea_sentence`; future/zero stamp rejection; duplicate/older no-state-change; ten-item FIFO dropping newest while the oldest blocks; exact 0.25 s fix-quality and 0.30 s heading/odometry prerequisite waits; same-stamp GGA/fix and prior-GGA/heading association; non-4 GGA clearing both FIFOs; degradation after five prerequisite failures or 1 s; no later heading paired backward to an old fix; no latest-TF historical lookup; preservation of non-authoritative bootstrap; 10 Hz Bool motion heartbeat; fixed diagnostics indices 12-18; and shutdown-safe rebroadcast. Add raw valid q=4, q=5, bad-checksum, missing-quality, and truncated GGA samples under the mandated GNSS test directory. Assert `package.xml` adds runtime `nmea_msgs` dependency.

- [ ] **Step 2: Run and verify RED**

Run: `PYTHONPATH=src/navigation/gps_waypoint_dispatcher python3 -m pytest src/navigation/gps_waypoint_dispatcher/test/test_rtk_authority.py -q`

Run: `PYTHONPATH=src/navigation/gps_waypoint_dispatcher python3 -m pytest src/sensor_drivers/gnss/nmea_navsat_driver/test/test_corridor_gga_quality.py -q`

Expected: source/node contracts fail against latest-value implementation.

- [ ] **Step 3: Implement callbacks and timer orchestration**

Parse GGA quality with a pure checksum-aware helper exercised by the raw samples, enqueue stamped heading/fix, process oldest first, associate quality exactly as specified, and interpolate `StampedPoseHistory`. Keep heading and translation gates independent; translation drops fixes whenever heading is not locked. Drive `CorrectionReleaseState`, publish `map -> odom`, existing mode/status, diagnostics, and `/localization_authority/motion_allowed` every timer tick. Add `<depend>nmea_msgs</depend>` explicitly.

- [ ] **Step 4: Run authority regression and verify GREEN**

Run the Task 3 pytest command.

Expected: all authority tests pass.

- [ ] **Step 5: Commit explicitly**

```bash
git add src/navigation/gps_waypoint_dispatcher/gps_waypoint_dispatcher/rtk_map_odom_corrector_node.py src/navigation/gps_waypoint_dispatcher/test/test_rtk_authority.py src/sensor_drivers/gnss/nmea_navsat_driver/test/test_corridor_gga_quality.py src/navigation/gps_waypoint_dispatcher/package.xml
git commit -m "fix: align RTK authority observations with LIO time"
```

### Task 4: Separate local divergence from global correction holds

**Files:**
- Modify: `src/navigation/gps_waypoint_dispatcher/gps_waypoint_dispatcher/route_safety.py`
- Modify: `src/navigation/gps_waypoint_dispatcher/test/test_route_safety.py`
- Create: `src/navigation/gps_waypoint_dispatcher/test/test_route_hold_integration.py`
- Modify: `src/navigation/gps_waypoint_dispatcher/gps_waypoint_dispatcher/gps_route_runner_node.py`

- [ ] **Step 1: Write failing watchdog and hold tests**

Pure tests must classify stamped local odom: duplicate ignore, regressing/non-finite immediate abort, >10 m/s or rad/s immediate abort, >3 m/s or rad/s for three consecutive samples abort, and counter reset on a good sample. Test global correction >0.50 m/s or 5 deg/s and false/stale authority as `GLOBAL_CORRECTION_HOLD`, never `ODOM_DIVERGENCE_ABORT`.

Add runner contracts for 10 Hz stop heartbeat, stop-before-cancel, 2 s cancel acknowledgement, 1 s continuous readiness, same ENU subgoal recomputation/retry, 15 s total hold timeout, and terminal stop ownership.

In `test_route_hold_integration.py`, use a fake ROS 2 Humble `NavigateToPose` ActionServer to exercise `cancel_goal_async()`, require a nonempty `goals_canceling` acknowledgement, cover cancellation rejection/timeout, reset readiness on one false heartbeat inside the 1 s window, and verify the retried goal is recomputed from the same ENU subgoal.

- [ ] **Step 2: Run and verify RED**

On a ROS 2 Humble environment, run: `PYTHONPATH=src/navigation/gps_waypoint_dispatcher python3 -m pytest src/navigation/gps_waypoint_dispatcher/test/test_route_safety.py src/navigation/gps_waypoint_dispatcher/test/test_route_hold_integration.py -q`

Expected: missing classifier/hold interfaces and old map-base single-step abort contract failures.

- [ ] **Step 3: Implement classifier and runner hold state**

Subscribe to `/fastlio2/lio_odom`, `map -> odom`, authority Bool/status, and use message/TF stamps for rates. Route cancellation and retry through one explicit state machine. Remove direct Twist publication from the runner; publish only `/gps_corridor/stop_override` Bool heartbeat.

- [ ] **Step 4: Run navigation unit regression**

On the workstation run the ROS-free test: `PYTHONPATH=src/navigation/gps_waypoint_dispatcher python3 -m pytest src/navigation/gps_waypoint_dispatcher/test/test_route_safety.py -q`.

After pushing/pulling the task branch on Jetson and sourcing Humble, run: `PYTHONPATH=src/navigation/gps_waypoint_dispatcher python3 -m pytest src/navigation/gps_waypoint_dispatcher/test/test_route_hold_integration.py -q`.

Expected: all dispatcher tests pass.

- [ ] **Step 5: Commit explicitly**

```bash
git add src/navigation/gps_waypoint_dispatcher/gps_waypoint_dispatcher/route_safety.py src/navigation/gps_waypoint_dispatcher/test/test_route_safety.py src/navigation/gps_waypoint_dispatcher/test/test_route_hold_integration.py src/navigation/gps_waypoint_dispatcher/gps_waypoint_dispatcher/gps_route_runner_node.py
git commit -m "fix: recover route goals from global correction holds"
```

### Task 5: Corridor command guard

**Files:**
- Create: `src/navigation/gps_waypoint_dispatcher/gps_waypoint_dispatcher/corridor_cmd_guard.py`
- Create: `src/navigation/gps_waypoint_dispatcher/gps_waypoint_dispatcher/corridor_cmd_vel_guard_node.py`
- Create: `src/navigation/gps_waypoint_dispatcher/test/test_corridor_cmd_guard.py`
- Modify: `src/navigation/gps_waypoint_dispatcher/setup.py`

- [ ] **Step 1: Write failing pure guard tests**

Test `v_limit=min(0.85, 0.25/max(abs(w),0.05))`, reverse symmetry, non-finite zero, command age 0.25 s, authority/override age 0.50 s, false authority, true override, startup fail-closed, and unchanged bounded angular velocity.

- [ ] **Step 2: Run and verify RED**

Run: `PYTHONPATH=src/navigation/gps_waypoint_dispatcher python3 -m pytest src/navigation/gps_waypoint_dispatcher/test/test_corridor_cmd_guard.py -q`

Expected: missing guard module.

- [ ] **Step 3: Implement pure limiter and 20 Hz node**

The node stores steady receipt times for `/cmd_vel_nav`, motion Bool, and stop Bool, publishes guarded `/cmd_vel`, and emits zero at startup/stale/error. It is the only corridor publisher to `/cmd_vel`.

- [ ] **Step 4: Run and verify GREEN**

Run the focused guard test, then all dispatcher tests.

- [ ] **Step 5: Commit explicitly**

```bash
git add src/navigation/gps_waypoint_dispatcher/gps_waypoint_dispatcher/corridor_cmd_guard.py src/navigation/gps_waypoint_dispatcher/gps_waypoint_dispatcher/corridor_cmd_vel_guard_node.py src/navigation/gps_waypoint_dispatcher/test/test_corridor_cmd_guard.py src/navigation/gps_waypoint_dispatcher/setup.py
git commit -m "feat: contain unstable corridor velocity commands"
```

## Chunk 3: Launch, Replay, And Bilingual Operations

### Task 6: Wire corridor topology and parameters

**Files:**
- Modify: `src/bringup/launch/system_gps_corridor.launch.py`
- Modify: `src/bringup/launch/system_explore.launch.py`
- Modify: `src/bringup/config/master_params.yaml`
- Modify: `src/bringup/test/test_system_gps_corridor_launch.py`
- Modify: `Makefile`
- Modify: `scripts/launch_with_logs.sh`

- [ ] **Step 1: Write failing launch/config tests**

Assert the included explore launch remaps controller output to a raw controller topic, smoother input to that raw topic, smoother output to `/cmd_vel_nav`, while `serial_twistctl` remains subscribed to guarded `/cmd_vel`. Assert guard is required with `on_exit=Shutdown`, runner never owns `/cmd_vel`, authority inputs include raw GGA/LIO, new topics are bagged, PGO TF remains false, all new thresholds equal spec defaults, and corridor topology has exactly one `/cmd_vel` publisher. Assert `make kill-runtime` and launch cleanup include the guard process.

- [ ] **Step 2: Run and verify RED**

Run: `PYTHONPATH=src/navigation/gps_waypoint_dispatcher python3 -m pytest src/bringup/test/test_system_gps_corridor_launch.py -q`

Expected: guard/topology assertions fail.

- [ ] **Step 3: Update launch and tuned parameters**

Add the corrector/runner/guard topics and parameters. Scope the explore-launch remaps behind corridor launch arguments so other modes retain their current command topics. Keep straight speed 0.85 and existing MPPI angular limit; the guard supplies only context-sensitive linear limiting. Add the guard to all runtime cleanup lists.

- [ ] **Step 4: Run launch and navigation regression**

Run: `PYTHONPATH=src/navigation/gps_waypoint_dispatcher python3 -m pytest src/navigation/gps_waypoint_dispatcher/test/test_alignment_math.py src/navigation/gps_waypoint_dispatcher/test/test_nav2_lifecycle_ready.py src/navigation/gps_waypoint_dispatcher/test/test_route_safety.py src/navigation/gps_waypoint_dispatcher/test/test_rtk_authority.py src/navigation/gps_waypoint_dispatcher/test/test_corridor_cmd_guard.py src/bringup/test/test_system_gps_corridor_launch.py -q`

Expected: all tests pass.

- [ ] **Step 5: Commit explicitly**

```bash
git add src/bringup/launch/system_gps_corridor.launch.py src/bringup/launch/system_explore.launch.py src/bringup/config/master_params.yaml src/bringup/test/test_system_gps_corridor_launch.py Makefile scripts/launch_with_logs.sh
git commit -m "feat: wire corridor localization containment"
```

### Task 7: Replay evaluator and mirrored documentation

**Files:**
- Create: `scripts/evaluate_corridor_authority_replay.py`
- Create: `src/navigation/gps_waypoint_dispatcher/test/test_corridor_authority_replay.py`
- Modify: `docs-CN/knowledge/nav2_tuning.md`
- Modify: `docs-EN/knowledge/nav2_tuning.md`
- Modify: `docs-CN/knowledge/rtk_fgo.md`
- Modify: `docs-EN/knowledge/rtk_fgo.md`
- Modify: `docs-CN/devlog/2026-07.md`
- Modify: `docs-EN/devlog/2026-07.md`
- Modify: `docs-CN/commands.md`
- Modify: `docs-EN/commands.md`
- Modify: `docs-CN/architecture.md`
- Modify: `docs-EN/architecture.md`
- Modify: `docs-CN/known_issues.md`
- Modify: `docs-EN/known_issues.md`
- Modify: `docs-CN/knowledge/gps_planning.md`
- Modify: `docs-EN/knowledge/gps_planning.md`
- Modify: `docs-CN/knowledge/pgo.md`
- Modify: `docs-EN/knowledge/pgo.md`

- [ ] **Step 1: Write failing evaluator tests**

Use synthetic timestamped records to prove heading outlier freezes in one sample and requires recovery window, release rates respect 0.205 m/s and 2.05 deg/s replay tolerances, global correction cannot become local abort, no run has more than ten consecutive rate-saturated release samples, and local no-progress classification triggers after the specified 15 s condition. Test JSON output, bag-specific expected results for all four fixtures, and nonzero failure exit.

- [ ] **Step 2: Run and verify RED**

Run: `PYTHONPATH=src/navigation/gps_waypoint_dispatcher python3 -m pytest src/navigation/gps_waypoint_dispatcher/test/test_corridor_authority_replay.py -q`

Expected: evaluator script is missing.

- [ ] **Step 3: Implement evaluator and docs**

Read rosbag2 SQLite through available ROS serialization only when a bag path is supplied; keep synthetic evaluator functions importable without ROS. Resolve external bags under `FYP_CORRIDOR_BAG_ROOT`. Document architecture, commands, tuned YAML old/new values and reasons, failure/known-issue state, GPS/PGO ownership, low-speed acceptance, serial-branch prerequisite, and rollback in all listed mirrored English/Chinese docs plus four-element devlogs.

- [ ] **Step 4: Run final workstation verification**

Run the ROS-free workstation set explicitly (the action integration file is Jetson-only): `PYTHONPATH=src/navigation/gps_waypoint_dispatcher python3 -m pytest src/navigation/gps_waypoint_dispatcher/test/test_alignment_math.py src/navigation/gps_waypoint_dispatcher/test/test_nav2_lifecycle_ready.py src/navigation/gps_waypoint_dispatcher/test/test_route_safety.py src/navigation/gps_waypoint_dispatcher/test/test_rtk_authority.py src/navigation/gps_waypoint_dispatcher/test/test_corridor_cmd_guard.py src/navigation/gps_waypoint_dispatcher/test/test_corridor_authority_replay.py src/bringup/test/test_system_gps_corridor_launch.py -q`.

Run against each available 2026-07-10 bag with `scripts/evaluate_corridor_authority_replay.py`; record unavailable external fixtures explicitly.

On Jetson, first deploy and verify the companion serial branch: `colcon build --packages-select serial_reader serial_twistctl --symlink-install --parallel-workers 1 && source install/setup.bash && colcon test --packages-select serial_reader serial_twistctl && colcon test-result --verbose`. Record the specified motor-disabled STM32 500 ms command-watchdog bench result; if it has not been run, corridor vehicle acceptance remains blocked.

Then run: `colcon build --packages-select gps_waypoint_dispatcher bringup --symlink-install --parallel-workers 1 && source install/setup.bash && colcon test --packages-select gps_waypoint_dispatcher && PYTHONPATH=src/navigation/gps_waypoint_dispatcher python3 -m pytest src/navigation/gps_waypoint_dispatcher/test/test_route_hold_integration.py src/bringup/test/test_system_gps_corridor_launch.py -q && colcon test-result --verbose`. The explicit pytest is required because bringup does not register that file with colcon.

Expected: workstation regression passes, replay JSON meets numeric criteria, and Jetson verification is reported separately if not run.

- [ ] **Step 5: Commit explicitly**

```bash
git add scripts/evaluate_corridor_authority_replay.py src/navigation/gps_waypoint_dispatcher/test/test_corridor_authority_replay.py docs-CN/knowledge/nav2_tuning.md docs-EN/knowledge/nav2_tuning.md docs-CN/knowledge/rtk_fgo.md docs-EN/knowledge/rtk_fgo.md docs-CN/knowledge/gps_planning.md docs-EN/knowledge/gps_planning.md docs-CN/knowledge/pgo.md docs-EN/knowledge/pgo.md docs-CN/commands.md docs-EN/commands.md docs-CN/architecture.md docs-EN/architecture.md docs-CN/known_issues.md docs-EN/known_issues.md docs-CN/devlog/2026-07.md docs-EN/devlog/2026-07.md
git commit -m "docs: add corridor authority replay acceptance"
```
