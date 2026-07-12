# SLAM Mapping and Indoor Navigation Design Plan

> Status: phases 1-5 implemented in code; phase-0 vehicle evidence, Jetson build, parameter calibration, and vehicle acceptance remain
> Date: 2026-07-11
> Scope: `slam` mapping mode and `travel` indoor prior-map navigation mode
> Out of scope: automatic indoor/outdoor switching, floor changes, elevator integration, and the outdoor RTK navigation algorithm

## 1. Goals and boundaries

This plan turns the current experimental chain into two independently testable product capabilities:

1. `slam` produces a 2D navigation map, a 3D localization map, keyframes, and a validated map bundle from one mapping session.
2. `travel` performs initialization, relocalization, static global planning, dynamic local avoidance, smooth control, and named-destination navigation on that bundle.

RViz `2D Pose Estimate` and `Nav2 Goal` remain as debugging inputs during this phase. The final user entry point is named-destination navigation, but indoor/outdoor switching is not implemented in this phase.

Design principles:

- The Slam Toolbox 2D OccupancyGrid is the authoritative Nav2 navigation map.
- The PGO-optimized 3D PCD and keyframes are the authoritative map for 3D global relocalization.
- The maps are not assumed to share a coordinate frame. The bundle must store and validate `T_map_2d_map_3d`.
- Only the localizer may publish indoor `map -> odom`; FAST-LIO2 publishes `odom -> base_footprint`.
- Motion output is prohibited when localization quality is not accepted, localization is lost, or TF ownership conflicts.
- The vehicle is a build, runtime, and validation target only; code is not edited directly on it.

## 2. Current baseline

### 2.1 Implemented software baseline

- `system_slam.launch.py` runs FAST-LIO2, PGO, pointcloud-to-LaserScan, Slam Toolbox, and the map saver.
- `slam_toolbox_mapping.yaml` now versions the Humble async-mapper baseline in the repository and consistently uses `base_footprint`.
- FAST-LIO2 publishes `/fastlio2/body_cloud_localization` independently from the low LaserScan slice and Nav2 obstacle cloud; PGO and localizer consume the same structural localization stream.
- `save_mapping_session.py` produces a versioned indoor bundle with the 2D map, pose graph, raw/downsampled 3D PCD, keyframes, Scan Context index, regions, destinations, 2D-to-3D report, and overlay.
- Saving enforces stationary, chassis feedback, frame, patch/pose, RMSE/p95/overlap, and artifact integrity gates; failures return nonzero and set `consistency_ok=false`.
- `system_travel.launch.py` accepts `map_bundle` as the primary input and rejects unsupported schemas, failed consistency, unaccepted calibration, and missing artifacts.
- The localizer supports RViz-seeded, region-assisted, and multi-candidate Scan Context startup, publishes structured status, and performs low-rate ICP correction with jump gates and low-pass updates.
- Travel uses a static-only global costmap and live point cloud in the local costmap.
- Travel uses a polygon footprint, MPPI, collision-checked smoothing, Collision Monitor, and fail-stop trees; unhealthy localization gates velocity before the serial controller.
- Travel now runs its controller at `20Hz`, matching MPPI `model_dt=0.05s`.
- `indoor_navigation_manager` exposes `NavigateNamedDestination` with map/backend validation, aliases, feedback, cancel, concurrency rejection, and cancellation on localization degradation.

### 2.2 Remaining vehicle work

- Build all ROS interfaces/C++ packages on Jetson Orin NX with `--parallel-workers 1` and complete a 30-minute resource run.
- Calibrate localization height windows, voxel size, ICP score/overlap, candidate gap, and 2D-to-3D thresholds from real PCDs.
- Measure the physical outer envelope and STM32 minimum effective speeds, then validate the provisional `650x500mm + 25mm` footprint and Collision Monitor zones.
- Collect region/arbitrary-start datasets and report top-k recall, false localization, startup latency, and repeated-corridor rejection.
- Pass narrow-door, in-place-turn, pedestrian crossing, sustained occlusion, localization-loss stop, and ten repeated-route tests.

## 3. Target architecture

```text
Mapping:
Livox + IMU
    -> FAST-LIO2 -> odom -> base_footprint
                  -> navigation slice -> LaserScan -> Slam Toolbox -> 2D map
                  -> localization cloud -> PGO -> 3D map + patches + poses

Save and calibration:
2D map + 3D map
    -> offline SE(2) registration
    -> alignment metrics + T_map_2d_map_3d
    -> versioned indoor map bundle

Navigation:
localization cloud + FAST-LIO2 odom
    -> place candidate retrieval
    -> coarse registration
    -> fine ICP/GICP
    -> compose T_map_2d_odom
    -> localization quality gate
    -> Nav2 static planner + MPPI + local obstacle costmap
    -> velocity smoother -> serial_twistctl -> STM32
```

## 4. Frames and TF ownership

The map assets use two logical frames:

- `map_2d`: Slam Toolbox 2D map frame; exposed to Nav2 as runtime `map`.
- `map_3d`: PGO 3D localization map frame; used in map assets and localization math, not as a second runtime TF root.

The map bundle stores:

```text
T_map_2d_map_3d
```

After the 3D localizer estimates `T_map_3d_odom`, runtime output is:

```text
T_map_2d_odom = T_map_2d_map_3d * T_map_3d_odom
```

The production chain remains:

```text
map -> odom -> base_footprint -> base_link -> sensor frames
```

| TF | Sole publisher |
|---|---|
| `map -> odom` | indoor localization manager/localizer |
| `odom -> base_footprint` | FAST-LIO2 |
| `base_footprint -> base_link` and static sensor TF | robot_state_publisher |

AMCL and PGO must not broadcast `map -> odom` in Travel. Launch tests and runtime health checks must verify a single owner.

## 5. Indoor map bundle contract

Proposed layout:

```text
runtime-data/maps/indoor/<map_id>/
  manifest.yaml
  navigation/
    map.yaml
    map.pgm
    slam_toolbox.posegraph
    slam_toolbox.data
  localization/
    map_raw.pcd
    map_localization.pcd
    poses.txt
    patches/
    descriptor_index/
  calibration/
    map_3d_to_map_2d.yaml
    alignment_report.yaml
    alignment_overlay.png
  destinations.yaml
  regions.yaml
```

Minimum manifest content:

```yaml
schema_version: 1
map_id: building_a_floor_1
created_at: "..."
git:
  branch: "..."
  commit: "..."
frames:
  navigation: map_2d
  localization: map_3d
artifacts:
  navigation_map: navigation/map.yaml
  localization_map: localization/map_localization.pcd
  raw_map: localization/map_raw.pcd
calibration:
  file: calibration/map_3d_to_map_2d.yaml
  accepted: true
quality:
  frame_check_ok: true
  patch_pose_integrity_ok: true
  alignment_wall_rmse_m: 0.0
  alignment_wall_p95_m: 0.0
  map_coverage_ok: true
```

Travel must reject missing files, unsupported schemas, or `calibration.accepted=false`. The final runtime entry point accepts one `map_bundle` or `map_id`, rather than unrelated operator-entered `map_yaml` and `pcd_map` paths.

## 6. SLAM mapping design

### 6.1 Point-cloud responsibility split

One low-height cloud must not continue serving mapping, relocalization, and obstacle avoidance. Target responsibilities:

| Cloud | Content | Consumers |
|---|---|---|
| navigation slice | stable structure near the vehicle collision height | pointcloud_to_laserscan, Slam Toolbox |
| localization cloud | wider height range retaining walls, door frames, and columns | PGO, localizer, descriptor builder |
| Nav2 obstacle cloud | current `[-0.20, 1.20]m` near-field obstacle semantics | local costmap, Collision Monitor |

Exact height ranges must be determined from vehicle PCD statistics. Mapping and Travel must use identical localization-cloud filtering rules.

Mapping projection enables a rectangular self-filter in `base_footprint`. The measured `650x500mm` body receives a `25mm` margin per side, excluding points inside `x=[-0.35,0.35]m, y=[-0.275,0.275]m`. A rectangle is preferable to one large `range_min`: covering the vehicle corners with a radius would also discard useful near-field wall returns beyond the rectangular front and sides. This filter affects mapping `/scan` only; it does not change FAST-LIO2 matching, the PGO localization structure cloud, or Travel/Corridor live obstacle clouds. A real overhanging object inside the body envelope will also be excluded, so the bounds must remain tied to measured dimensions plus a small margin rather than the Nav2 inflation radius. Rebuilt maps still require narrow-door, table-leg, and low-obstacle inspection.

### 6.2 Repository-owned Slam Toolbox configuration

Add `slam_toolbox_mapping.yaml` under version control instead of loading the installed default YAML. At minimum, pin:

- `map_frame/odom_frame/base_frame`, with `base_frame=base_footprint`.
- Map resolution, maximum range, and update interval.
- Minimum mapping translation and rotation.
- Scan matching, loop closure, scan queue, and TF publish period.
- Serialized pose-graph output conventions.

Parameter changes require corresponding updates to this document, the relevant SLAM/Nav2 knowledge documents, and both devlogs.

### 6.3 Mapping gates

Startup:

1. Check Livox, IMU, FAST-LIO2 odometry, and TF.
2. Wait for IMU initialization and stable LIO status.
3. Confirm that only Slam Toolbox publishes mapping-time `map -> odom`.
4. Then allow the operator to move the vehicle.

Collection:

- Cover main corridors, room entrances, door frames, and loop areas at low and steady speed.
- Revisit structures from different directions.
- Record LIO degeneracy, PGO loops, CPU/GPU, temperature, and dropped frames.
- Pause mapping around heavy pedestrian traffic to avoid persistent dynamic clutter.

Save:

1. Keep the vehicle stationary for at least 2 seconds.
2. Require zero `/cmd_vel` and chassis feedback below a velocity threshold.
3. Require fresh FAST-LIO2 and TF timestamps.
4. Require PGO to be outside loop-optimization commit and meet keyframe/coverage thresholds.
5. Save 2D, 3D, pose graph, patches, and session metadata.
6. Generate downsampled `map_localization.pcd`.
7. Run 2D-to-3D registration and quality checks.
8. Mark the bundle `accepted=true` only after every hard gate passes.

### 6.4 2D-to-3D registration

The clean Slam Toolbox OccupancyGrid remains the 2D map. A simple raw PGO projection does not replace it.

The registration tool must:

1. Extract occupied wall points from PGM/YAML.
2. Extract walls by vertical span in localization-PCD XY cells: default `0.06m` cells, at least three points, and `>=0.50m` z span, rejecting single-height floor/furniture projections.
3. Solve planar `SE(2)` with multiple seeds. Because both maps start from a shared session origin, explicitly include identity while retaining principal-axis/centroid seeds for nonzero offsets.
4. Report the first and second candidate scores and final transform.
5. Compute global and regional RMSE, p95 wall distance, and overlap.
6. Produce a visual overlay for manual review.

Initial thresholds, to be finalized using the first vehicle maps:

- Global wall RMSE `<= 0.10m`.
- p95 wall distance `<= 0.15m`.
- No visible local mismatch at major doors or corners.
- The first candidate must clearly outperform the second, otherwise manual approval is required.

A rigid transform cannot repair nonlinear deformation from two different optimizers. If regional metrics fail, reduce map extent, remap, or adopt a shared trajectory solution; do not only loosen thresholds.

Quality evaluation uses the complete wall sample (up to 12,000 points) and exact nearest neighbors within 0.50m through a spatial grid. `alignment_report.yaml` records the extraction method, input count, output-cell count, and parameters; the overlay uses the same wall sample.

## 7. Travel indoor navigation design

### 7.1 Startup and localization state machine

```text
UNINITIALIZED
  -> MAP_LOADING
  -> WAITING_FOR_SENSORS
  -> GLOBAL_SEARCHING or WAITING_FOR_INITIAL_POSE
  -> LOCALIZED
  -> DEGRADED
  -> RELOCALIZING
  -> LOST
```

Only `LOCALIZED` permits navigation. `DEGRADED`, `RELOCALIZING`, and `LOST` cancel the goal and continuously command zero velocity.

Localization status must publish:

- State and reason.
- Map ID.
- Cloud and odometry freshness.
- Coarse and fine registration scores.
- Point-cloud overlap.
- Current correction and rate of change.
- Gap between first and second global candidates.
- Last trusted localization timestamp.

### 7.2 Staged initialization

Phase A keeps RViz `/initialpose` for current debugging and baseline acceptance.

Phase B adds region-assisted localization. The user selects a coarse area such as lobby, east corridor, or laboratory zone, and the system searches multiple initial poses in that submap.

Phase C adds automatic global localization. Build a Scan Context or equivalent descriptor index from `patches + poses`, retrieve several candidates, then run coarse and fine GICP/ICP registration.

Automatic localization must not blindly accept the highest score. Ambiguous repeated corridors must produce a stopped state and request a region hint.

### 7.3 Runtime drift control

Freezing one successful ICP correction is appropriate for the early safety baseline, but not for long operation. Target behavior:

- FAST-LIO2 provides high-rate continuous local motion.
- The localizer performs low-rate prior-map matching.
- Small, continuous, trusted corrections update `map -> odom` through rate limiting and low-pass filtering.
- Large jumps, candidate switches, and low-overlap results are not written directly to TF.
- Indoor `map -> odom` must be projected to planar `SE(2)`: retain only XY and yaw, and never let ICP write z, roll, or pitch into Nav2's global TF.
- Large corrections require a stop and multi-frame-consistent relocalization.
- Gates consider the actual effect on `map -> base_footprint`, not only raw `map -> odom` values.

### 7.4 Planning and control

Retain:

```text
NavFn/A* -> Savitzky-Golay SmoothPath -> MPPI -> velocity_smoother
```

First required changes:

- Set `controller_frequency=20Hz` to match `model_dt=0.05s`.
- Configure a polygon footprint from the documented `650 x 500mm` dimensions and measured outermost body points, with safety margin.
- Set MPPI `CostCritic.consider_footprint=true`.
- Collision-check the smoothed path to prevent corner cutting.
- Measure the STM32 minimum effective `vx/wz` before setting velocity deadbands.

The vehicle can turn in place, so the controller retains the `DiffDrive` motion model and zero-linear-speed rotation samples. A polygon footprint does not disable in-place turning; it makes rotational collision checking match the real body.

### 7.5 Dynamic obstacles and safety

- The global costmap contains only the static map and static inflation.
- The local costmap uses the Nav2 obstacle cloud for people, chairs, and temporary obstacles.
- Add Collision Monitor with independent slowdown and stop zones.
- Preserve Travel fail-stop behavior: no automatic BackUp, blind Spin, or full-map clear.
- PS2 `X` is the highest-priority software disable; the physical red e-stop overrides all software.
- Localization state must gate the velocity chain, not only display a warning.

### 7.6 Named-destination navigation

Destination data remains independent of the algorithm:

```yaml
schema_version: 1
map_id: building_a_floor_1
destinations:
  lab_101:
    display_name: Laboratory 101
    aliases: [laboratory, lab 101]
    pose: {x: 12.4, y: 6.8, yaw: 1.57}
    approach: forward
```

Add a ROS 2 Action instead of relying on a String topic:

```text
NavigateNamedDestination
  goal: map_id, destination_name
  feedback: state, resolved_name, remaining_distance, localization_state
  result: success, error_code, message
```

The first implementation only calls indoor `NavigateToPose`. Keep `map_id` and a backend field for future outdoor support, but do not implement switching in this phase.

Foxglove uses a separate adapter without bypassing Actions or safety gates: a clicked `PoseStamped` becomes `NavigateToPose`, a destination string becomes `NavigateNamedDestination`, and a Trigger service cancels the current goal. The adapter sends only under healthy structured localization, publishes destination markers/catalog/status, and never subscribes or publishes to `/cmd_vel*`.

Named-navigation cancellation must cover the window before the underlying Nav2 goal handle is returned. The request is latched and issued as soon as the handle is accepted; rejection, exception, and cancellation paths must all release the concurrency reservation.

The velocity chain uses three command-loss layers: the localization gate requires fresh localization, obstacle cloud, and velocity command; the host serial node resends zero after 300 ms; and the STM32 independently clears `Vcx/Wc` after 500 ms without a valid serial command. Collision Monitor source timeout is not itself a stop guarantee, so obstacle-cloud freshness is checked by the upstream fail-closed gate.

## 8. Development phases and file scope

### Phase 0: freeze the baseline and evidence

- Launch contract tests for `system_slam.launch.py` are implemented.
- The Slam Toolbox configuration is pinned with `base_frame=base_footprint`.
- Record a baseline SLAM/Travel bag and map bundle.
- The Travel 20Hz controller-period mismatch is fixed.

Acceptance: the current manual initialization flow does not regress, and both modes build and run on Jetson with `--parallel-workers 1`.

### Phase 1: production map bundle (software complete)

Implemented scope:

- `src/bringup/launch/system_slam.launch.py`
- `src/bringup/config/slam_toolbox_mapping.yaml`
- `src/perception/fastlio2/`
- `scripts/save_mapping_session.py`
- New map registration/acceptance tools and tests

Acceptance: one command generates the full bundle; failed gates return nonzero; Travel starts from `map_bundle`.

### Phase 2: Travel safety and control (software complete)

Implemented scope:

- `src/bringup/config/nav2_travel.yaml`
- `src/bringup/behavior_trees/`
- `src/bringup/launch/system_travel.launch.py`
- Collision Monitor configuration and launch tests

Acceptance: straight, 90-degree turn, in-place turn, narrow doorway, pedestrian crossing, and U-shaped obstacle tests pass; localization failure reliably stops the vehicle.

### Phase 3: localization state and low-rate drift correction (software complete)

- Extend `src/perception/localizer/`.
- Add localization status messages/diagnostics.
- Add TF smoothing and jump-gate tests.

Acceptance: map alignment stays within the agreed tolerance after a long corridor return; bad registration cannot be written directly to TF.

### Phase 4: arbitrary-start global localization (software complete)

- Build the descriptor index.
- Add multi-candidate retrieval, coarse/fine registration, and ambiguity rejection.
- Add region-hint fallback.

Acceptance: measure top-k recall, success rate, false-localization rate, and startup latency on predefined unknown starts. False-localization rate has priority over success rate.

### Phase 5: named-destination navigation (software complete)

- Add the destination schema, loader, and Action server.
- Test cancel, duplicate requests, unknown destinations, and localization degradation.

Acceptance: complete repeated trips using only destination names, without the RViz goal tool.

## 9. Test matrix and metrics

| Category | Scenario | Core metrics |
|---|---|---|
| Map | loop corridor, room entrance, glass area | 2D/3D error, coverage, ghosting |
| Initialization | known seed, coarse region, arbitrary start | success, false localization, latency, candidate gap |
| Localization | corridor return, long run, temporary occlusion | drift, TF step, recovery time |
| Planning | straight, 90-degree turn, narrow door, U obstacle | planning success, corner cutting, replans |
| Control | in-place turn, path tracking, terminal stop | lateral error, angular sign changes, stop error |
| Dynamic obstacles | crossing, sustained block, obstacle removal | stop distance, resume time, false clears |
| Resources | 30-minute continuous run | CPU/GPU/RAM, temperature, missed control periods |

Formal acceptance includes at least ten repeated round trips on the same route. Every failure retains its session logs and map version; successful videos alone are insufficient evidence.

## 10. Main risks and decision gates

1. **Non-rigid 2D/3D mismatch:** split the map or use a shared optimization source if one `SE(2)` fails regional metrics.
2. **Repeated-corridor false localization:** require ambiguity rejection and region hints.
3. **Dynamic-map contamination:** inspect the area and apply outlier/temporal filtering before accepting a map.
4. **Oversized PGO map:** separate raw archive from the downsampled localization map; load only the latter on Orin NX.
5. **Low-speed chassis deadband:** identify commanded-versus-measured response before MPPI tuning.
6. **TF jumps:** large corrections require a stop, multi-frame agreement, and smoothed release.
7. **Premature indoor/outdoor integration:** do not add mode switching before this indoor plan is complete.

## 11. Definition of done

Software phases 1-5 are implemented. Indoor/outdoor switching design starts only after these vehicle conditions pass:

- Mapping repeatably generates versioned, accepted map bundles.
- The 2D navigation and 3D localization map relation is explicit and strictly validated.
- Arbitrary-start or region-assisted localization works and rejects ambiguity instead of mislocalizing.
- Low-rate correction controls runtime drift, and localization faults reliably stop motion.
- The measured footprint, MPPI, local obstacles, and Collision Monitor pass vehicle tests.
- Named navigation works without RViz and supports cancel, status feedback, and correct arrival.
- Bilingual documentation, tests, vehicle logs, and Jetson resource metrics are complete.
