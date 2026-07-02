# Hybrid Overlap Localization Design

Date: 2026-07-02
Status: Draft for review
Scope: Architecture and staged design only. No implementation is approved by this document alone.

## 1. Problem

The vehicle has two localization failure modes that the current stack does not fully solve.

First, cold-start relocalization is fragile in campus-scale environments. Buildings and corridors can have repeated geometry, so local point-cloud alignment against a large prior map can converge to a wrong local optimum.

Second, outdoor-to-indoor transition is fragile. Outdoor navigation uses RTK/ENU absolute coordinates, while indoor navigation depends on a relative prior map. Doorway areas are exactly where RTK may degrade because of multipath, tree cover, walls, and partial sky occlusion. A hard coordinate-frame switch at the doorway can therefore lose localization.

## 2. Design Goal

Introduce a hybrid localization architecture that uses:

- RTK Fixed position and dual-antenna heading for outdoor global pose and initial alignment.
- A deliberately mapped overlap zone around building entrances.
- Local 3D prior point-cloud registration only where it is useful.
- Building-level map streaming instead of one campus-wide dense 3D map.
- Shadow-mode validation before any node is allowed to own production TF.

The first deliverable must prove one building entrance can be relocalized reliably in shadow mode. It must not replace the current `explore`, `corridor`, `nav-gps`, or `tightly-coupled` modes.

## 3. Non-Goals

The first implementation must not:

- Build a dense 3D map of the entire campus.
- Replace FAST-LIO2, PGO, Nav2, or `rtk_fgo_localizer`.
- Publish production `map -> odom`.
- Remap Nav2 to a new localization output.
- Depend on a learned model such as OverlapNet or AirLoop.
- Require Scan Context for the MVP.

These capabilities may be added after the overlap-zone shadow pipeline is validated.

## 4. Existing System Constraints

The current vehicle stack uses:

```text
map -> odom -> base_link
```

- FAST-LIO2 publishes high-rate local odometry and `odom -> base_link`.
- PGO publishes global correction and `map -> odom`.
- `corridor` mode uses UM982 RTK plus a standalone ENU-to-map aligner.
- `nav-gps` mode uses scene bundles, anchors, route graphs, and `gps_waypoint_dispatcher`.
- `tightly-coupled` mode runs `rtk_fgo_localizer` in shadow mode by default.
- Mapping sessions can save 2D maps, 3D PGO maps, keyframe poses, patches, and a manifest.

This design must preserve those boundaries until the new localization output has passed replay, shadow, and vehicle validation.

## 5. Architecture Overview

The proposed architecture has four layers.

```text
Outdoor RTK / route graph / corridor
  -> approaches a building entrance in the campus ENU frame

Multi-map manager
  -> preloads the target building's overlap-zone 3D map

Overlap relocalizer
  -> uses RTK position + heading as an initial guess
  -> aligns live LiDAR cloud against the preloaded PCD patch
  -> publishes candidate pose, confidence, and diagnostics

Hybrid localization supervisor
  -> accepts or rejects candidate localization
  -> shadow mode first; experimental TF ownership only in a later phase
```

The first phase implements only the multi-map metadata reader and overlap relocalizer in shadow mode. The supervisor can initially be a lightweight status gate or an offline evaluator.

## 6. Coordinate Frames

Use a single campus-level `map` frame aligned to the selected ENU origin.

Each building map has a local frame:

```text
map -> building_<id>_map -> building point cloud
```

The building manifest stores the transform from the campus `map` frame to the building map frame. For the MVP, a single building can be stored directly in the campus `map` frame if the mapping workflow already produces a consistent ENU-registered map. The manifest must still include the transform fields so the data model is future-proof.

The overlap relocalizer publishes candidate poses in the campus `map` frame:

```text
/overlap_relocalization/candidate_pose  geometry_msgs/PoseStamped
```

It does not publish TF in Phase 1.

## 7. Overlap Zone

An overlap zone is a deliberately mapped area around a building entrance:

- 10-20 m outside the entrance where RTK Fixed is still expected.
- 5-10 m inside the entrance where indoor map localization must take over.
- Stable geometry such as walls, door frames, pillars, railings, and corners.
- Minimal dependence on dynamic objects, trees, vehicles, or crowds.

The zone is collected under RTK Fixed when possible, with dual-antenna heading recorded. Old G60-collected routes or maps must not be used for RTK acceptance.

The map artifact should include:

```text
runtime-data/maps/buildings/<building_id>/
  manifest.yaml
  entry_zones.yaml
  map.pcd
  poses.txt
  patches/
```

## 8. Runtime Data Model

### 8.1 Building Index

`runtime-data/maps/buildings/building_index.yaml` lists known building maps.

Example:

```yaml
buildings:
  - id: eb
    name: engineering_building
    map_dir: runtime-data/maps/buildings/eb
    preload_radius_m: 60.0
    unload_radius_m: 100.0
    geofence:
      center_lat: 31.274927
      center_lon: 120.737548
      radius_m: 80.0
    entry_zones:
      - id: eb_main_door
        file: entry_zones.yaml
```

### 8.2 Building Manifest

Each building map has `manifest.yaml`.

```yaml
building_id: eb
map_frame: map
building_frame: building_eb_map
created_at: "2026-07-02T00:00:00+09:00"
enu_origin:
  lat: 31.274927
  lon: 120.737548
  alt: 0.0
map_to_building:
  xyz: [0.0, 0.0, 0.0]
  rpy: [0.0, 0.0, 0.0]
artifacts:
  pcd: map.pcd
  poses: poses.txt
  patches_dir: patches
quality:
  rtk_fixed_required: true
  heading_required: true
  consistency_ok: true
```

### 8.3 Entry Zones

`entry_zones.yaml` defines relocalization gates.

```yaml
entry_zones:
  - id: eb_main_door
    building_id: eb
    trigger:
      center_lat: 31.274927
      center_lon: 120.737548
      radius_m: 35.0
    overlap_patch:
      pcd: map.pcd
      crop_box_map:
        min: [-20.0, -15.0, -2.0]
        max: [20.0, 25.0, 3.0]
    acceptance:
      min_rtk_quality: 4
      max_hdop: 2.0
      require_heading: true
      max_initial_position_error_m: 3.0
      max_initial_yaw_error_deg: 10.0
      max_fitness_score: 0.6
      min_inlier_ratio: 0.45
      required_consecutive_successes: 5
```

## 9. ROS Components

### 9.1 `building_map_manager`

Responsibility:

- Read `building_index.yaml`.
- Subscribe to `/fix` and `/rtk/status`.
- Detect nearby building geofences.
- Preload the target building PCD and entry-zone metadata in the background.
- Publish map availability and selected entry zone.

Suggested package:

```text
src/navigation/building_map_manager/
```

Suggested topics:

```text
/building_map_manager/status              std_msgs/String
/building_map_manager/active_entry_zone   std_msgs/String
/building_map_manager/preload_status      std_msgs/String
```

Phase 1 can implement a simpler file-path parameter instead of full multi-building indexing. The data model should still match this design.

### 9.2 `overlap_relocalizer`

Responsibility:

- Subscribe to live local cloud and odometry.
- Build a short local submap from recent LiDAR frames or consume a prepared body/world cloud.
- Use RTK `/fix` and `/heading` to form an initial campus-frame pose.
- Crop the preloaded overlap-zone PCD.
- Run ICP or NDT registration.
- Publish candidate pose and diagnostics.
- Reject ambiguous or low-confidence alignments.

Suggested package:

```text
src/perception/overlap_relocalizer/
```

Suggested inputs:

```text
/fastlio2/body_cloud        sensor_msgs/PointCloud2
/fastlio2/lio_odom          nav_msgs/Odometry
/fix                        sensor_msgs/NavSatFix
/heading                    geometry_msgs/QuaternionStamped
/rtk/status                 std_msgs/String
/building_map_manager/active_entry_zone
```

Suggested outputs:

```text
/overlap_relocalization/candidate_pose       geometry_msgs/PoseStamped
/overlap_relocalization/status               std_msgs/String
/overlap_relocalization/score                std_msgs/Float32MultiArray
/overlap_relocalization/aligned_cloud        sensor_msgs/PointCloud2
/overlap_relocalization/reference_patch      sensor_msgs/PointCloud2
```

`score` should include at least:

```text
[fitness_score, inlier_ratio, translation_delta_m, yaw_delta_deg, consecutive_successes]
```

### 9.3 `hybrid_localization_supervisor`

Responsibility:

- Watch RTK quality, overlap relocalization score, PGO health, FAST-LIO2 odometry continuity, and current mode.
- Decide whether a candidate pose is accepted.
- In Phase 1, publish only a status result.
- In a later experimental mode, publish a smoothed experimental TF chain.

Suggested outputs:

```text
/hybrid_localization/status        std_msgs/String
/hybrid_localization/state         std_msgs/String
/hybrid_localization/accepted_pose geometry_msgs/PoseStamped
```

No production `map -> odom` publication is allowed in Phase 1.

## 10. Registration Method

The MVP should use deterministic PCL-based registration before learned models.

Recommended first choice:

- Voxel downsample live cloud and reference patch.
- Remove ground or use the existing vehicle-height-filtered cloud.
- Use RTK position and heading to create the initial transform.
- Run Generalized ICP or NDT.
- Check score and pose continuity.

Scan Context can be introduced later as a candidate recall method when cold-start ambiguity remains. OverlapNet can be introduced later as a learned overlap score. AirLoop is a long-term learning enhancement and should not block MVP.

## 11. State Machine

The complete state machine is:

```text
UNKNOWN_START
OUTDOOR_RTK_READY
OUTDOOR_NAV
BUILDING_APPROACH
MAP_PRELOADED
OVERLAP_ALIGNING
OVERLAP_LOCKED
INDOOR_LOCALIZED
RELOCALIZATION_FAILED
LOCAL_DEGRADED
MANUAL_HOLD
```

Phase 1 only needs:

```text
IDLE
WAITING_FOR_RTK_FIXED
WAITING_FOR_MAP
ALIGNING
CANDIDATE_STABLE
REJECTED
FAULT
```

Acceptance requires consecutive success, not one successful registration.

Reject conditions include:

- RTK quality is not Fixed when Fixed is required.
- Heading is missing or unstable.
- ICP/NDT fitness is too high.
- Inlier ratio is too low.
- Candidate pose jumps too far from the RTK/FGO/PGO prior.
- Multiple candidate patches have similar scores.
- TF or FAST-LIO2 odometry is stale.

## 12. Launch Strategy

Add a new experimental mode only after the shadow nodes exist:

```text
make launch-hybrid-overlap
ros2 launch bringup system_hybrid_overlap.launch.py
```

The launch file should start:

- Existing explore baseline.
- UM982 RTK driver.
- Building map manager.
- Overlap relocalizer.
- Hybrid supervisor in shadow mode.
- Rosbag recording for all relevant source and diagnostic topics.

It must not modify existing `corridor`, `nav-gps`, `explore-gps`, or `tightly-coupled` launch behavior.

## 13. Logging and Bagging

Record at least:

```text
/fix
/heading
/rtk/status
/rtk/nmea_sentence
/fastlio2/lio_odom
/fastlio2/body_cloud
/pgo/optimized_odom
/tf
/tf_static
/building_map_manager/status
/building_map_manager/active_entry_zone
/overlap_relocalization/candidate_pose
/overlap_relocalization/status
/overlap_relocalization/score
/hybrid_localization/status
/hybrid_localization/state
```

The session must use the existing `scripts/launch_with_logs.sh` logging pattern and write under `runtime-data/logs/<timestamp>/`.

## 14. Validation Plan

### 14.1 Offline Unit Tests

- Parse building index and entry-zone YAML.
- Validate geofence trigger math.
- Validate RTK quality parsing and Fixed/Float handling.
- Validate transform composition between ENU, campus `map`, and building map.
- Validate candidate acceptance/rejection logic.

### 14.2 Bag Replay

Use bags containing:

- Outdoor RTK Fixed approach.
- Doorway overlap traversal.
- RTK degraded doorway case.
- Similar-looking non-target building facade.

Measure:

- Registration success rate.
- Fitness score distribution.
- Pose jitter.
- False-positive acceptance rate.
- Processing time per registration attempt.

### 14.3 Shadow Vehicle Test

Acceptance targets for a single entrance:

- Map preload completes before the vehicle reaches the overlap zone.
- Candidate pose is stable for at least 5 consecutive frames.
- Candidate translation jitter is below 0.3 m.
- Candidate yaw jitter is below 2 degrees.
- No accepted pose occurs when RTK is degraded or the vehicle is at the wrong entrance.
- Nav2 behavior is unchanged because the system is shadow-only.

### 14.4 Experimental TF Test

Only after shadow success:

- Enable experimental TF in a new launch mode.
- Smooth correction steps to at most 0.1-0.2 m per update and 0.2-0.5 degrees per update.
- Confirm Nav2 does not see sudden transform jumps.
- Keep manual stop and PS2 `X` motor-disable safety active.

## 15. Staged Roadmap

### Phase 0: Data and Map Preparation

- Define building map directory format.
- Extend or wrap mapping save workflow to produce building manifests.
- Collect one entrance overlap zone under RTK Fixed.

### Phase 1: Single-Entrance Shadow Relocalization

- Implement map loading from explicit parameters.
- Implement RTK-heading initial guess.
- Implement ICP/NDT registration and diagnostics.
- Publish candidate pose and score only.

### Phase 2: Supervisor and Replay Metrics

- Implement acceptance gate.
- Add bag replay evaluator.
- Define pass/fail thresholds from real logs.

### Phase 3: Experimental Indoor Transition

- Add experimental launch mode.
- Allow supervisor to publish experimental TF after explicit launch flag.
- Test one entrance transition from outdoor route to indoor localization.

### Phase 4: Multi-Building Streaming

- Add `building_index.yaml`.
- Add geofence-triggered background preload.
- Add unload policy.
- Support multiple entry zones per building.

### Phase 5: Place Recognition Enhancements

- Add Scan Context or Scan Context++ for cold-start candidate recall.
- Add learned overlap scoring only if deterministic scoring is insufficient.
- Consider long-term map maintenance inspired by LT-mapper/AirLoop.

## 16. Risks

- RTK Fixed may not hold near the target entrance.
- Doorway geometry may be too symmetric for ICP/NDT alone.
- Dynamic objects may contaminate overlap-zone scans.
- PCD map may not be ENU-registered accurately enough.
- Loading large PCD files may stall the Jetson if not done off the control path.
- Publishing a new TF source too early can destabilize Nav2.

The mitigation is to keep Phase 1 shadow-only, log all diagnostics, and reject ambiguous candidates rather than forcing a localization switch.

## 17. Open Decisions

- Whether Phase 1 uses ICP, GICP, NDT, or multiple backends behind one interface.
- Whether live registration uses a single current scan or a short local submap.
- How building map `map_to_building` is calibrated in the first field workflow.
- Whether the first map artifact should be saved directly in campus `map` or in a building-local frame.
- Final score thresholds after replay on real doorway bags.

## 18. Recommended First Milestone

The first milestone should be:

> Single-building, single-entrance, RTK Fixed overlap-zone shadow relocalization.

Success means the system can preload one PCD patch, use RTK + heading as the initial guess, register live LiDAR to the overlap patch, and publish a stable candidate pose with diagnostics, without changing Nav2 or production TF.
