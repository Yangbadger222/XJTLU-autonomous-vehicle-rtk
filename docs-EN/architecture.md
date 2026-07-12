# System Architecture

## 1. Deployment Locations

- Jetson code repository: `~/XJTLU-autonomous-vehicle`
- Runtime data root directory: `~/XJTLU-autonomous-vehicle/runtime-data`
- GitHub remote: `Yangbadger222/XJTLU-autonomous-vehicle-rtk`
- AI collaboration control plane: located in a separate PC repository, not within this code repository

## 2. Hardware Platform

- Jetson Orin NX, 16 GB RAM, Ubuntu 22.04, ROS 2 Humble
- Livox MID360 LiDAR
- WIT IMU
- T-RTK UM982 Dual Antenna Mobile Kit + 4G Module (Rover End)
- Serial connection to STM32 lower-level controller
- PS2 gamepad as the highest-priority manual override

## 3. Eight Operating Modes

| Mode | Command | Current Purpose |
|------|---------|-----------------|
| SLAM | `make launch-slam` | Pure mapping: produces 2D Nav2 maps, 3D PGO point-cloud maps, and a manifest |
| Explore | `make launch-explore` | Current primary operating mode, local obstacle avoidance navigation |
| Indoor Nav | `make launch-indoor-nav` | RViz click-to-go navigation without GNSS |
| Corridor | `make launch-corridor` | GPS Corridor v2 main runtime on the MPPI controller |
| Travel | `make launch-travel` | Indoor map-bundle navigation: automatic/assisted localization, static planning, dynamic avoidance, and named Action |
| Explore GPS | `make launch-explore-gps` | Explore with GNSS and PGO GPS factor added |
| Nav GPS | `make launch-nav-gps` | Scene bundle + anchor ready + GPS route-graph navigation mode |
| RTK Basic | `make launch-rtk-basic` | RTK signal testing with CORS account |
| Tightly Coupled | `make launch-tightly-coupled` | Experimental RTK FGO shadow mode publishing `/rtk_fgo/*` beside the main stack |

All `make launch-*` entry points go through `scripts/launch_with_logs.sh`, so session-isolated log directories are created by default.

## 4. SLAM Pure Mapping Data Flow

```text
Livox MID360 + IMU -> FAST-LIO2 -> /fastlio2/body_cloud (low 2D LaserScan slice)
                                  -> /fastlio2/body_cloud_localization (3D structural localization cloud)
                                  -> /fastlio2/lio_odom
                                  -> TF: odom -> base_footprint -> base_link

/fastlio2/body_cloud -> pointcloud_to_laserscan -> /scan
                                             |
                                             v
                                      SLAM Toolbox -> /map
                                                   -> TF: map -> odom

/fastlio2/body_cloud_localization + /fastlio2/lio_odom -> PGO(publish_tf=false)
                                             -> /pgo/global_map
                                             -> /pgo/save_maps

scripts/save_mapping_session.sh <map_name>
  -> saves 2D map + pose graph, raw/downsampled 3D PCD, poses/patches
  -> builds a Scan Context index, solves T_map_2d_map_3d, and writes metrics/overlay
  -> writes manifest.yaml; failed stationary, frame, integrity, or calibration gates block Travel
```

SLAM mode does not run Nav2 planners/controllers. Slam Toolbox uses the repository-owned mapping profile and exclusively owns `map -> odom`; PGO saves the 3D assets with `publish_tf=false`. Launch records a lean evidence bag by default and only debug adds raw clouds. Saving requires stationary LIO/chassis feedback, consistent frames, complete keyframes, and accepted 2D/3D registration. `use_rtk:=true` records future geo-registration evidence only; indoor invalid/float RTK is not a strong constraint.

## 5. Explore Mode Data Flow

```text
Livox MID360 -> /livox/lidar ------+
                                   |
Livox IMU   -> /livox/imu -------->+-> FAST-LIO2 -> /fastlio2/body_cloud
                                   |              -> /fastlio2/lio_odom
                                   |
                                   +-> PGO -> TF: map -> odom
                                           -> /pgo/optimized_odom
                                           -> /pgo/loop_markers

PGO / registered cloud -> pointcloud_to_laserscan / pointcloud_to_grid -> Nav2 costmaps
Nav2 -> /cmd_vel -> serial_twistctl -> STM32 -> motors
STM32 -> serial_reader -> chassis feedback / odom_CBoard
```

## 6. GPS-Related Chains

### 5.1 Explore GPS Mode

```text
GNSS serial -> nmea_navsat_driver -> /fix
                                   |
                                   v
                          gnss_calibration -> /gnss
                                              |
                                              v
                                   PGO GPS Factor constraints
```

The purpose of `make launch-explore-gps` is still to inject the calibrated `/gnss` into PGO, improving the global position constraint capability of the `map -> odom` transform.

### 5.2 Nav GPS Mode (scene bundle + route graph)

```text
scene_gps_bundle.yaml -> build_scene_runtime.py
                       -> current_scene/master_params_scene.yaml
                       -> current_scene/scene_points.yaml
                       -> current_scene/scene_route_graph.geojson

GNSS serial -> /fix + /heading + /rtk/status -------------------+
                       |                                       |
                       |                                       v
                       |                         rtk_map_odom_corrector
                       |                         scene fixed ENU -> map identity
                       |                         TF: map -> odom
                       v
                gps_anchor_localizer -> /gnss + /gps_system/*
                       |
                       +-> lock startup anchor for route-selection readiness

scene_points.yaml + route_graph.geojson ------------------------+
                                                               |
goto_name -> gps_waypoint_dispatcher(goal manager) ------------+
            |  reads scene_points.yaml
            |  reads current map->base_link pose
            |  Stage A: route_server nearest traversable graph-node search
            |  Stage B: ComputeRoute(use_poses=true)
            v
     dense graph path -> FollowPath -> Nav2 -> /cmd_vel
```

The core of `nav-gps` is:
- `gps_anchor_localizer` still owns anchor matching, `NAV_READY`, and `/gnss` publishing
- `map -> odom` is no longer published by PGO in this mode; PGO uses the corridor no-TF/no-GPS configuration and remains a point-cloud / optimization side channel only
- `rtk_map_odom_corrector` reads the scene fixed origin and uses a fixed ENU-to-map identity alignment to compute the RTK-authoritative `map -> odom`
- Nav2 uses the corridor RTK MPPI profile and the high-window `/fastlio2/body_cloud_nav2_obstacles` obstacle cloud instead of the old DWB-based `nav2_gps.yaml` profile
- route server runs with `enable_nn_search=true`, and the goal manager starts from the current pose, so the vehicle no longer has to be near a small set of anchors before navigating
- `scene_gps_bundle.yaml` is the single source of truth
- At runtime, only compiled artifacts under `~/XJTLU-autonomous-vehicle/runtime-data/gnss/current_scene/` are read
- `goto_name` is the main entry point; users input only English destination names

### 5.3 RTK FGO Tight-Coupled Experimental Mode (Shadow)

```text
Explore stack + UM982 RTK
  -> rtk_fgo_localizer
       inputs: /fastlio2/lio_odom, /livox/imu, /odom_CBoar, /fix, /heading, /rtk/*
       outputs: /rtk_fgo/odom, /rtk_fgo/path, /rtk_fgo/status, /rtk_fgo/rtk_gate
                /rtk_fgo/correction_status, /rtk_fgo/factor_diagnostics
```

This mode can be launched standalone with `make launch-tightly-coupled`; `corridor` and `nav-gps` also start the same shadow node by default for bag-based evaluation only:
- `publish_tf=false` by default; it does not broadcast production `map -> odom`
- Nav2 is not remapped to FGO output, and the production localization output of `corridor`, `explore-gps`, and `nav-gps` is not replaced
- Source sensor topics and `/rtk_fgo/*` are recorded automatically for rosbag replay and vehicle shadow validation

## 7. TF Chain

```text
map -> odom -> base_footprint -> base_link
```

- In Explore / explore-gps production navigation modes, `map -> odom` is published by PGO, representing global correction offset
- In Corridor and RTK nav-gps modes, PGO disables `publish_tf`; the only production `map -> odom` owner is `rtk_map_odom_corrector`
- In pure SLAM mapping mode, `map -> odom` is published by SLAM Toolbox; PGO only saves 3D maps and does not publish TF
- In Travel, localizer exclusively owns `map -> odom`. Bundle startup first attempts multi-candidate Scan Context plus ICP; a region service or RViz seed provides fallback. Once localized, 1Hz map matching admits only score/overlap/jump-gated corrections and releases them with `alpha=0.15` smoothing.
- Travel mode bridges RViz `2D Pose Estimate` (`/initialpose`) into `/localizer/relocalize`, has FAST-LIO2 publish the taller Nav2 obstacle cloud `/fastlio2/body_cloud_nav2_obstacles`, and retimes it as `/fastlio2/body_cloud_nav2` for the local costmap. The global costmap stays static-map based so live point-cloud obstacles cannot mark the robot start cell and block NavFn planning.
- Travel's velocity chain is `/cmd_vel -> localization_cmd_gate -> /cmd_vel_localized -> Collision Monitor -> /cmd_vel_safe -> serial_twistctl`. The gate requires fresh structured localization, Nav2 obstacle cloud, and velocity input; any timeout continuously emits zero. Travel enables a final acceleration limiter in `serial_twistctl`: zero, deceleration, and safety stops remain immediate, while a nonzero command released after a Stop ramps from zero at `0.30m/s2` and `0.80rad/s2`, preventing an upstream smoother's retained high command from being released abruptly. `serial_twistctl` resends zero after 300 ms of command loss, and the STM32 firmware independently clears `Vcx/Wc` with a 500 ms watchdog. The Foxglove adapter also requires `/chassis/status.ctrl_mode=0`, rejecting or canceling goals while motors are disabled or the chassis remains in gamepad mode.
- `indoor_navigation_manager` resolves bundle destinations/aliases into `NavigateToPose` and exposes feedback, cancellation, and cancellation on localization degradation through `/navigate_named_destination`.
- Travel starts `foxglove_bridge:8765` and `foxglove_navigation_adapter` by default. The adapter accepts both the repository layout's `/foxglove/goal_pose` and Foxglove's default `/move_base_simple/goal`, uses TF to convert poses from source frames such as `base_link` or `odom` into `map`, and then routes both through the same localization-gated `NavigateToPose`; destination strings become `NavigateNamedDestination`. It publishes destination markers, catalog, and status, never accesses `/cmd_vel*`, and rejects or cancels goals when localization is unhealthy or the goal frame cannot be transformed.
- PGO is off by default, or runs only with `publish_tf=false`
- `odom -> base_footprint` is published by FAST-LIO2, representing high-frequency local odometry; `base_footprint -> base_link` is provided by URDF static TF
- The combination of both yields the global pose

If `map -> odom` does not exist, RViz under the `map` fixed frame will appear as if point clouds or costmaps are blank, even if Livox and FAST-LIO2 are still running.

## 8. Configuration Architecture

- `src/bringup/config/master_params.yaml`
  - Repository template parameter entry point
- `src/bringup/config/nav2_default.yaml`
- `src/bringup/config/nav2_explore.yaml`
- `src/bringup/config/nav2_gps.yaml`
- `src/bringup/config/nav2_travel.yaml`
- `~/XJTLU-autonomous-vehicle/runtime-data/gnss/scene_gps_bundle.yaml`
- `~/XJTLU-autonomous-vehicle/runtime-data/gnss/current_scene/master_params_scene.yaml`
- `~/XJTLU-autonomous-vehicle/runtime-data/gnss/current_scene/scene_points.yaml`
- `~/XJTLU-autonomous-vehicle/runtime-data/gnss/current_scene/scene_route_graph.geojson`
- `src/sensor_drivers/gnss/gnss_calibration/config/calibration_points.yaml`
- `src/perception/pgo_gps_fusion/config/pgo.yaml`
- `src/perception/pgo_gps_fusion/config/pgo_no_gps.yaml`

## 8. Logs and Runtime Data

`~/XJTLU-autonomous-vehicle/runtime-data/` lives inside the workspace and currently contains:

```text
~/XJTLU-autonomous-vehicle/runtime-data/
├── bags/
├── config/
│   └── log_switch.yaml
├── gnss/
│   ├── scene_gps_bundle.yaml
│   ├── scene_gps_bundle_*.yaml
│   └── current_scene/
│       ├── master_params_scene.yaml
│       ├── scene_points.yaml
│       ├── scene_route_graph.geojson
│       └── scene_gps_bundle.yaml
├── logs/
│   ├── <timestamp>/
│   │   ├── console/
│   │   ├── data/
│   │   └── system/
│   └── latest -> <timestamp>
├── maps/
├── perf/
└── planning/
    └── angle_offset.txt
```

- `console/`: ROS 2 stdout/stderr
- `data/`: per-node custom data logs
- `system/`: `tegrastats.log` and `session_info.yaml`

## 9. Source Code Layout

```text
src/
├── sensor_drivers/
├── perception/
├── planning/
├── navigation/
└── bringup/
```

Notes:
- `sensor_drivers/`: Livox, IMU, GNSS, serial
- `perception/`: FAST-LIO2, PGO GPS fusion, point cloud to grid related; `rtk_fgo_localizer` is the experimental tight-coupled RTK FGO package and is currently wired only into shadow mode
- `planning/`: Historical GPS global planning and coordinate transformation experiments
- `navigation/`: `waypoint_collector` and scene-graph goal manager `gps_waypoint_dispatcher`
- `bringup/`: System launch files, parameters, maps, RViz configurations
- Upstream dependencies are fetched through `vcs import < dependencies.repos` and are not treated as project-specific development space

## 10. Package Build Types

- `ament_cmake`
  - `livox_ros_driver2`
  - `serial`
  - `serial_reader`
  - `serial_twistctl`
  - `fastlio2`
  - `pointcloud_to_grid`
  - `pointcloud_to_laserscan`
  - `pgo`
  - `pgo_original`
  - `rtk_fgo_localizer` (experimental package skeleton)
  - `hba`
  - `localizer`
  - `interface`
- `ament_python`
  - `global2local_tf`
  - `gnss_global_path_planner`
  - `waypoint_collector`
  - `gps_waypoint_dispatcher`
  - `wit_ros2_imu`
  - `gnss_calibration`
  - `gyro_odometry`

## 11. Key Dependencies

- PCL
- OpenCV
- Eigen3
- yaml-cpp
- NLopt
- Livox SDK2
- GTSAM
- GeographicLib
- pyproj (recommended for exact GPS projection; QGIS scene compilation and nav-gps scene loading have a local-ENU fallback)
- `ros-humble-geographic-msgs`


## 5.3 GPS Corridor v2 Mode (Standalone Global Aligner Architecture)

```text
current_route.yaml
  -> start_ref / waypoints[] / launch_yaw_deg / enu_origin

/fix -----> gps_global_aligner -----> /gps_corridor/enu_to_map (smoothed ENU->map transform)
  |                              |
  |                              +---> gps_route_runner
  |                                    1. bootstrap: yaw0 + launch_yaw_deg -> initial ENU->map
  |                                    2. wait for stable /fix
  |                                    3. check start point <= start_ref tolerance
  |                                    4. GPS waypoints -> ENU -> map (using aligner output)
  |                                    5. freeze alignment within waypoint, split subgoals per segment
  |                                    6. sequential NavigateToPose
  |
  +-- /heading + /rtk/status + odom->base_link TF
      -> rtk_map_odom_corrector
      -> TF: map->odom (RTK authoritative)
                                                     |
                                                     v
                                               Nav2 Explore stack (MPPI controller)
                                               -> planner/controller/costmaps
                                               -> /cmd_vel
```

Differences from the `nav-gps` mode:
- Does not use scene graph / `route_server`
- Does not use `gps_waypoint_dispatcher`'s `goto_name` / menu interaction
- Does not use runtime `current_scene/` compiled artifacts
- Uses standalone `gps_global_aligner_node` instead of PGO live handoff

Positioning assumptions for this mode:
- The vehicle starts from a fixed physical Launch Pose with a physically fixed heading
- `launch_yaw_deg` records the vehicle's geographic heading at startup (ENU convention)
- Supports multi-waypoint routes (not limited to two-point straight lines)

Key architectural decisions for this mode:
- **Standalone global aligner**: Decoupled from PGO, smoothly publishes the `ENU->map` transform
- **Live alignment subgoal recomputation**: Continuously projects active subgoals using the latest alignment output instead of the old per-waypoint frozen model
- **Bootstrap startup**: Immediately computes initial alignment using `yaw0 - radians(launch_yaw_deg)`, without waiting for GPS
- **Corridor-specific Nav2 BT**: Corridor uses NavigateToPose / NavigateThroughPoses trees without `Spin` / `BackUp`; planning or tracking failures are stopped and reported by `gps_route_runner` with route-progress context instead of physical recovery actions
- **Nav2 action timeout**: Corridor raises the BT `default_server_timeout` to 1000ms so a slow FollowPath acknowledgement under Jetson load does not falsely trigger recovery
- **Nav2-dedicated obstacle cloud**: The corridor local costmap uses `/fastlio2/body_cloud_nav2_obstacles` with a `[-0.20, 1.20]m` height window; PGO/LIO still use the low-window `/fastlio2/body_cloud`, so structure-cloud tuning does not make Nav2 blind to taller obstacles
- **RTK-authoritative `map->odom`**: In corridor mode, `pgo_corridor_no_gps.yaml` disables PGO `publish_tf`; `rtk_map_odom_corrector` becomes the only `map->odom` owner and computes it from RTK fix, dual-antenna heading, `ENU->map`, and current `odom->base_link`
- **RTK bootstrap**: Before `gps_global_aligner` publishes `ENU->map`, `rtk_map_odom_corrector` uses the current RTK fix, heading, and `odom->base_link` to publish a temporary `map->odom`, breaking the startup loop where the aligner waits for map TF
- **RTK degraded hold**: When RTK fix, heading, or target-jump gating rejects the latest input, `rtk_map_odom_corrector` freezes the global pose update but keeps rebroadcasting the last trusted `map->odom` with a fresh timestamp, so Nav2 does not fail only because the TF tree expired
- **RTK target smoothing and jump gating**: `rtk_map_odom_corrector` adds target deadband + low-pass filtering between the raw `map->odom` target and final per-step limiting, reducing small RTK/heading jitter before it can continuously enter `map->odom`; target-jump gating compares the current raw RTK `map_base` against the last trusted raw RTK `map_base`, so `map->odom` target translation amplified by the odom-origin lever arm is not mistaken for an RTK jump
- **Indoor/outdoor handoff interface**: `rtk_map_odom_corrector` publishes `/localization_authority/mode`, `/localization_authority/status`, and `/localization_authority/diagnostics`; a later prior-map relocalizer can become another authority source on the same `map->odom` interface

Data plane for this mode:
- `~/XJTLU-autonomous-vehicle/runtime-data/gnss/current_route.yaml` (generated by `collect_gps_route.py`)
- `start_ref` + multiple `waypoints[]` GPS coordinates
- `launch_yaw_deg` is a required field
- `/localization_authority/*` records the current `map->odom` authority source, rejection reason, raw RTK `map_base` jump, raw `map->odom` target/output gap, and limited output
