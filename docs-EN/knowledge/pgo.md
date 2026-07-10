# PGO Pose Graph Optimization

## 1. Current Engineering Implementation (2026-03)

The current primary PGO implementation is located at:

- Directory: `src/perception/pgo_gps_fusion/`
- Colcon package name: `pgo`
- Launch entry point: `ros2 launch pgo pgo_launch.py`

This implementation is no longer a pure loop-closure PGO. It consists of:
- FAST-LIO2 local odometry
- PGO loop-closure graph optimization
- Optional GPS factor absolute position constraints

## 2. Current Inputs and Outputs

### Inputs

- Point cloud: `/fastlio2/body_cloud`
- Odometry: `/fastlio2/lio_odom`
- GNSS: `/gnss` (when `gps.enable=true`)

### Outputs

- TF: `map -> odom`
- Odometry: `/pgo/optimized_odom`
- Visualization: `/pgo/loop_markers`

## 3. Node Responsibilities

PGO currently handles 5 tasks:

1. Collect keyframes from FAST-LIO2
2. Perform loop search and ICP confirmation
3. Run pose graph optimization with GTSAM
4. Periodically add GPS factors when enabled
5. Publish `map -> odom` and `/pgo/optimized_odom`

## 4. Current Main Configuration Parameters

These values come from the current mainline `master_params.yaml`:

- `key_pose_delta_deg: 5.0`
- `key_pose_delta_trans: 0.1`
- `loop_search_radius: 1.0`
- `loop_time_tresh: 60.0`
- `loop_score_tresh: 0.15`
- `loop_submap_half_range: 5`
- `submap_resolution: 0.1`
- `min_loop_detect_duration: 5.0`

## 5. GPS Factor and Fixed ENU Origin

When `gps.enable=true`, PGO uses the following GNSS-related parameters:

- `gps.topic: /fix` (changed from `/gnss` to `/fix` in 2026-03-22 corridor v2)
- `gps.noise_xy: 2.5`
- `gps.noise_z: 5.0`
- `gps.factor_interval: 10`
- `gps.quality_hdop_max: 3.0`
- `gps.quality_sat_min: 6`
- `gps.drift_threshold: 2.0`

New additions as of 2026-03-20:
- `gps.origin_mode: auto | fixed`
- `gps.origin_lat`
- `gps.origin_lon`
- `gps.origin_alt`
- `gps.topic` in `nav-gps` mode can be continuously published as scene-calibrated `/gnss` by `gps_anchor_localizer`

Behavioral constraints:
- `auto`: Maintains historical compatibility; uses the first valid GPS to initialize LocalCartesian
- `fixed`: Uses the configured fixed ENU origin directly at startup
- In `fixed` mode, the origin must not be overwritten after the first GPS arrives

This allows PGO's `map` coordinate frame to share the same geographic reference as `gps_anchor_localizer`, the goal manager, and the scene route graph.

## 6. TF Relationships

```text
map -> odom -> base_link
```

- FAST-LIO2 provides `odom -> base_link`
- PGO provides `map -> odom`
- Combined, they yield the global pose

If PGO has not entered normal keyframe and optimization flow, `map -> odom` may disappear, causing RViz to display blank point clouds and costmaps when the fixed frame is set to `map`.

## 7. 2026-03-18 Startup Regression Fix

### Symptom

- `pgo_node` continuously printed `Received out of order message` after startup
- No stable keyframes
- No stable `map -> odom`
- `/pgo/optimized_odom` unstable or not publishing

### Root Cause

`last_message_time` in the synchronization state was uninitialized, causing the first pair of synchronized messages to be incorrectly flagged as out-of-order.

### Fix

Initialized `last_message_time` to `0.0`, ensuring the first pair of matched messages is accepted normally.

### Result

This fix has been merged to `main`. The current mainline PGO startup synchronization state uses the stable initialization version.

## 8. Interface Quick Reference

| Interface | Type | Description |
|-----------|------|-------------|
| `/fastlio2/body_cloud` | `sensor_msgs/PointCloud2` | PGO keyframe point cloud input |
| `/fastlio2/lio_odom` | `nav_msgs/Odometry` | PGO pose input |
| `/gnss` | `sensor_msgs/NavSatFix` | GPS factor input |
| `/pgo/optimized_odom` | `nav_msgs/Odometry` | Optimized pose output |
| `map -> odom` | TF | Global correction offset |
| `/pgo/loop_markers` | Marker | Loop closure visualization |

## 9. 2026-03-20 Real-Vehicle `nav-gps` Blocking Conclusion

The first round of real scene bundle outdoor testing confirmed that the current `pgo` has a higher-priority stability issue.

### Symptom

- `gps_anchor_localizer` can successfully transition through:
  - `NO_FIX -> UNSTABLE_FIX -> GNSS_READY -> NAV_READY`
- Goal manager can successfully reach:
  - `GOAL_REQUESTED`
  - `COMPUTING_ROUTE`
  - `FOLLOWING_ROUTE`
- The vehicle briefly moves but quickly stops
- Goal manager final state:
  - `FAILED; follow_path_status=6`

### Direct Evidence

- `controller_server` logs:
  - `Controller patience exceeded`
  - `Transform data too old when converting from odom to map`
- Launch logs:
  - `pgo_node ... process has died ... exit code -11`
- On-site TF results:
  - Only `odom -> base_link` remains
  - `map -> odom` disappeared

### Engineering Conclusion

- The current failure is not a problem with the route graph, destination number menu, or `NAV_READY` gating
- The real failure point is the `pgo_node` segfault
- Once `pgo` exits, the Nav2 controller cannot continue transforming the robot pose from `odom` to `map`
- The result is `follow_path` being aborted

### Attempted But Not Fully Resolved Fixes

- Fixed one clear concurrency bug in `syncCB()`:
  - Changed anonymous `lock_guard` to a named lock
- `pgo` single package rebuilt
- However, upon re-testing on the real vehicle, `pgo_node` still exits with `exit code -11`

### Next Round of Investigation Entry Points

- `SimplePGO::addKeyPose()`
- `SimplePGO::smoothAndUpdate()`
- GPS factor insertion path
- `map -> odom` offset update chain before broadcast

## 10. Corridor v2 ENU->map Alignment Mechanism Evolution (2026-03-22~24)

### 10.1 Design Background

- GPS provides WGS84 latitude/longitude
- PGO uses a fixed ENU origin to convert to ENU (x=East, y=North)
- However, FAST-LIO2's `map` frame yaw is arbitrary (depends on IMU attitude at startup)
- Therefore, the ENU->map rotation angle theta and translation t must be estimated

### 10.2 Architecture Evolution

**v2 initial version (PGO-side ENU->map estimation)**:
- The original plan was to have PGO accumulate (enu_xy, map_xy) pairs during operation and publish to `/gps_corridor/enu_to_map`
- Real-vehicle testing revealed: after PGO took over, `ENU->map` continued to drift, and the runner recomputed subgoals leading to backtracking/recovery
- **This approach has been deprecated**

**v2 final version (independent Global Aligner)**:
- Added independent `gps_global_aligner_node.py` (commit `e51a46a`), decoupled from PGO
- The aligner independently collects GPS->ENU and map->base_link pairs, estimating a smooth ENU->map transform online
- Outputs to `/gps_corridor/enu_to_map` (same topic, but source switched from PGO to the independent aligner)
- Applies rate limiting and smoothing to estimates to prevent jumps
- In corridor mode, PGO's `gps.enable` is turned off; PGO only performs loop closure

### 10.3 Current PGO Role in Corridor Mode

- **Retained**: Graph optimization / loop detection / publishing `map -> odom` / `optimized_odom`
- **Disabled**: GPS factor (`gps.enable: false`), no longer directly participates in corridor alignment
- **Independent aligner takes over**: GPS global correction is handled by `gps_global_aligner_node`

### 10.4 Bootstrap Mechanism

The runner uses fixed yaw bootstrap at startup to begin navigation immediately:
- Read the yaw0 of `map->base_link` from TF
- Compute `θ_bootstrap = yaw0 - radians(launch_yaw_deg)` using the route YAML's `launch_yaw_deg`
- Construct the initial ENU->map transform
- Issue the first goal immediately without waiting for the aligner to converge

This resolves the "stationary startup deadlock" problem from the early v2 planning phase.
## 11. Corridor TF Ownership After Authority Containment

In corridor mode PGO must keep both GPS factors and `publish_tf` disabled. The only production `map->odom` broadcaster is `rtk_map_odom_corrector`; FAST-LIO2 publishes `odom->base_footprint`, and URDF supplies `base_footprint->base_link`. The corrector uses stamped LIO history rather than latest tf2 lookup. A fresh timestamp on a frozen correction preserves TF availability but `/localization_authority/motion_allowed=false` prevents Nav2 commands from reaching the chassis.
