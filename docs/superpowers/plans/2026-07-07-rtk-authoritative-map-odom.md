# RTK Authoritative Map Odom Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an RTK-authoritative `map -> odom` publisher for corridor mode while leaving an authority interface for later indoor prior-map handoff.

**Architecture:** FAST-LIO2 keeps publishing `odom -> base_link`. Corridor mode disables PGO TF ownership and starts a new `gps_waypoint_dispatcher` node that computes `map -> odom` from RTK `map -> base` and current local `odom -> base`. Safety gates prevent stale, invalid, or jumpy RTK from moving the TF.

**Tech Stack:** ROS 2 Humble, `rclpy`, `tf2_ros`, `sensor_msgs/NavSatFix`, `geometry_msgs/QuaternionStamped`, `std_msgs/String`, `std_msgs/Float64MultiArray`, existing `FixedENUProjector` and alignment math helpers.

---

## Chunk 1: Pure Math And Gates

### Task 1: Add ROS-free authority math helpers

**Files:**
- Create: `src/navigation/gps_waypoint_dispatcher/gps_waypoint_dispatcher/rtk_authority.py`
- Test: `src/navigation/gps_waypoint_dispatcher/test/test_rtk_authority.py`

- [ ] **Step 1: Write failing tests**
  - Verify `compute_map_to_odom()` returns `T_map_base * inverse(T_odom_base)`.
  - Verify `limit_pose_step()` caps translation and yaw.
  - Verify `summarize_authority_inputs()` rejects stale fix, stale heading, missing alignment, and hard divergence.

- [ ] **Step 2: Run tests and confirm RED**

```bash
PYTHONPATH=src/navigation/gps_waypoint_dispatcher python3 -m pytest src/navigation/gps_waypoint_dispatcher/test/test_rtk_authority.py -q
```

- [ ] **Step 3: Implement minimal helpers**

- [ ] **Step 4: Run tests and confirm GREEN**

## Chunk 2: ROS Node

### Task 2: Add `rtk_map_odom_corrector_node`

**Files:**
- Create: `src/navigation/gps_waypoint_dispatcher/gps_waypoint_dispatcher/rtk_map_odom_corrector_node.py`
- Modify: `src/navigation/gps_waypoint_dispatcher/setup.py`
- Test: extend `src/navigation/gps_waypoint_dispatcher/test/test_rtk_authority.py`

- [ ] **Step 1: Write failing tests for entry point and status strings**
- [ ] **Step 2: Implement node subscriptions, TF lookup, TF publish, and status publishers**
- [ ] **Step 3: Run Python compile and tests**

## Chunk 3: Corridor Launch Ownership

### Task 3: Make corridor use RTK authority for `map -> odom`

**Files:**
- Modify: `src/bringup/config/pgo_corridor_no_gps.yaml`
- Modify: `src/bringup/launch/system_gps_corridor.launch.py`
- Modify: `src/bringup/test/test_system_gps_corridor_launch.py`

- [ ] **Step 1: Write failing launch/config tests**
  - PGO corridor override sets `publish_tf: false`.
  - Corridor launch starts `rtk_map_odom_corrector_node`.
  - Corridor bag records `/localization_authority/*`.

- [ ] **Step 2: Implement launch/config changes**
- [ ] **Step 3: Run bringup tests**

## Chunk 4: Documentation And Verification

### Task 4: Update bilingual docs and verify locally/Jetson

**Files:**
- Modify: `docs-CN/architecture.md`
- Modify: `docs-EN/architecture.md`
- Modify: `docs-CN/knowledge/nav2_tuning.md`
- Modify: `docs-EN/knowledge/nav2_tuning.md`
- Modify: `docs-CN/devlog/2026-07.md`
- Modify: `docs-EN/devlog/2026-07.md`

- [ ] **Step 1: Document the single `map -> odom` owner rule and authority interface**
- [ ] **Step 2: Run local tests**
- [ ] **Step 3: Commit and push**
- [ ] **Step 4: Pull on Jetson, build with `--parallel-workers 1`, source install, and run tests**
