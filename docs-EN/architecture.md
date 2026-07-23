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
| Travel | `make launch-travel` | Experimental prior-map navigation: 2D-map global planning + PCD point-cloud relocalization |
| Explore GPS | `make launch-explore-gps` | Explore with GNSS and PGO GPS factor added |
| Nav GPS | `make launch-nav-gps` | Scene bundle + RTK authority + GPS route-graph navigation mode |
| RTK Basic | `make launch-rtk-basic` | RTK signal testing with CORS account |
| Tightly Coupled | `make launch-tightly-coupled` | Experimental RTK FGO shadow mode publishing `/rtk_fgo/*` beside the main stack |

All `make launch-*` entry points go through `scripts/launch_with_logs.sh`, so session-isolated log directories are created by default.

## 4. SLAM Pure Mapping Data Flow

```text
Livox MID360 + IMU -> FAST-LIO2 -> /fastlio2/body_cloud
                                  -> /fastlio2/lio_odom
                                  -> TF: odom -> base_footprint -> base_link

/fastlio2/body_cloud -> pointcloud_to_laserscan -> /scan
                                             |
                                             v
                                      SLAM Toolbox -> /map
                                                   -> TF: map -> odom

/fastlio2/body_cloud + /fastlio2/lio_odom -> PGO(publish_tf=false)
                                             -> /pgo/global_map
                                             -> /pgo/save_maps

scripts/save_mapping_session.sh <map_name>
  -> saves 2D map.yaml/map.pgm
  -> saves 3D map.pcd/poses.txt/patches
  -> writes manifest.yaml with 2D/3D consistency, patch/pose integrity, and frame checks
```

SLAM mode does not start Nav2 planners/controllers and does not execute navigation behavior. In this mode PGO uses `pgo_slam.yaml` with `publish_tf=false` by default, so it does not compete with SLAM Toolbox for `map -> odom`. The save script checks the current FAST-LIO2 child frame `base_footprint` by default, but the actual child frame must be confirmed with TF tools before changing `base_frame` on the vehicle. RTK can be enabled with `use_rtk:=true` to record outdoor Fixed samples during mapping, but indoor invalid/float RTK samples are records only, not strong constraints.

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
                       -> current_scene/road_keepout.{yaml,pgm}

GNSS serial -> /fix + /heading + /rtk/status
                       |
                       v
                rtk_map_odom_corrector
                scene fixed ENU -> map identity
                TF: map -> odom

Optional legacy: gps_anchor_localizer -> /gnss + /gps_system/*

goto_name / /goal_pose / /gps_goal
            -> gps_waypoint_dispatcher
            |  project current pose and goal onto nearest graph edges
            |  insert virtual endpoints and run Euclidean-heuristic A*
            |  produce one continuous NavPath at 0.35m density
            v
     FollowPath -> MPPI + obstacle cloud + road keepout
                -> /cmd_vel_nav -> velocity smoother -> /cmd_vel
                -> authority guard -> /cmd_vel_guarded -> serial
```

The core of `nav-gps` is:
- The legacy `gps_anchor_localizer` is disabled by default. The current A*/RTK-authority production chain consumes neither `/gnss` nor anchor readiness; compatibility experiments may opt in explicitly.
- `map -> odom` is no longer published by PGO in this mode; PGO uses the corridor no-TF/no-GPS configuration and remains a point-cloud / optimization side channel only
- `rtk_map_odom_corrector` reads the scene fixed origin and uses a fixed ENU-to-map identity alignment to compute the RTK-authoritative `map -> odom`
- Nav2 uses the corridor RTK MPPI profile and the high-window `/fastlio2/body_cloud_nav2_obstacles` obstacle cloud instead of the old DWB-based `nav2_gps.yaml` profile
- the goal manager runs graph A* directly and projects both endpoints onto graph edges; it no longer depends on route-server Dijkstra or a small anchor set
- the QGIS drivable polygon is compiled into a KeepoutFilter mask; MPPI may avoid obstacles inside the road while remaining blocked outside it
- RTK is the sole motion authority for `map -> odom`. When RTK position/heading gates are not locked, the last trusted transform is frozen and motion stops; FAST-LIO remains only for local odometry and obstacle clouds, not global-localization bridging. Fixed-RTK normal innovation uses `20deg / 1.5m`, with a `7.5deg / 0.50m` recovery window and tolerance for up to `10` failures or `2s` of temporarily unprocessable input; the `q=4` requirement and `2m / 20deg` fault boundary remain unchanged.
- The vehicle profile keeps `controller_frequency=20Hz` aligned with `model_dt=0.05s`, while reducing MPPI work to `350x40` samples with a two-second horizon.
- The default lean bag retains the smaller local costmap for obstacle review but omits the global costmap, which accounted for `77.5%` of this bag. The debug profile adds the global costmap, raw point clouds, and legacy anchor status.
- `scene_gps_bundle.yaml` is the single source of truth
- At runtime, only compiled artifacts under `~/XJTLU-autonomous-vehicle/runtime-data/gnss/current_scene/` are read
- named `goto_name`, map-frame `/goal_pose`, and geographic `/gps_goal` destinations are supported

### 5.3 RTK FGO Tight-Coupled Experimental Mode (Shadow)

```text
Explore stack + UM982 RTK
  -> rtk_fgo_localizer
       inputs: /fastlio2/lio_odom, /livox/imu, /odom_CBoar, /fix, /heading, /rtk/*
       outputs: /rtk_fgo/odom, /rtk_fgo/path, /rtk_fgo/status, /rtk_fgo/rtk_gate
                /rtk_fgo/correction_status, /rtk_fgo/factor_diagnostics
```

This mode can be launched standalone with `make launch-tightly-coupled`. Both corridor and `nav-gps` vehicle entry points disable it by default for CPU budget; enable it explicitly with `FYP_CORRIDOR_ENABLE_FGO_SHADOW=true` or `FYP_NAV_GPS_ENABLE_FGO_SHADOW=true`, respectively:
- `publish_tf=false` by default; it does not broadcast production `map -> odom`
- Nav2 is not remapped to FGO output, and the production localization output of `corridor`, `explore-gps`, and `nav-gps` is not replaced
- Source sensor topics and `/rtk_fgo/*` are recorded automatically for rosbag replay and vehicle shadow validation
- when FGO later takes authority, it must be the sole `map -> odom` owner and keep publishing the common `/localization_authority/motion_allowed` contract; A*, MPPI, and the menu do not depend on an RTK-specific mode name

## 7. TF Chain

```text
map -> odom -> base_footprint -> base_link
```

- In Explore / explore-gps production navigation modes, `map -> odom` is published by PGO, representing global correction offset
- In Corridor and RTK nav-gps modes, PGO disables `publish_tf`; the only production `map -> odom` owner is `rtk_map_odom_corrector`
- In pure SLAM mapping mode, `map -> odom` is published by SLAM Toolbox; PGO only saves 3D maps and does not publish TF
- In Travel prior-map mode, `map -> odom` is published by the `localizer` ICP point-cloud relocalizer; after startup PCD preload, `/localizer/relocalize` must succeed before TF broadcasting starts, avoiding unvalidated or stale-stamped TF in Nav2
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
- Does not use the scene graph / A* goal manager
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
- **RTK degraded hold**: When RTK fix, heading, or alignment gating rejects the latest input, `rtk_map_odom_corrector` freezes the last trusted `map->odom` and revokes motion authority; FAST-LIO no longer bridges global localization during RTK degradation. To reduce short stops caused by local sky obstruction, the Fixed-RTK normal innovation gate is relaxed to `20 deg / 1.5 m`, the recovery window to `7.5 deg / 0.50 m`, and up to `10` failures or `2 s` of temporarily unprocessable input is tolerated; the `q=4` and `2 m / 20 deg` fault boundaries remain fail-closed.
- **RTK target smoothing and jump gating**: `rtk_map_odom_corrector` adds target deadband + low-pass filtering between the raw `map->odom` target and final per-step limiting, reducing small RTK/heading jitter before it can continuously enter `map->odom`; target-jump gating compares the current raw RTK `map_base` against the last trusted raw RTK `map_base`, so `map->odom` target translation amplified by the odom-origin lever arm is not mistaken for an RTK jump
- **Indoor/outdoor handoff interface**: `rtk_map_odom_corrector` publishes `/localization_authority/mode`, `/localization_authority/status`, and `/localization_authority/diagnostics`; a later prior-map relocalizer can become another authority source on the same `map->odom` interface

Data plane for this mode:
- `~/XJTLU-autonomous-vehicle/runtime-data/gnss/current_route.yaml` (generated by `collect_gps_route.py`)
- `start_ref` + multiple `waypoints[]` GPS coordinates
- `launch_yaw_deg` is a required field
- `/localization_authority/*` records the current `map->odom` authority source, rejection reason, raw RTK `map_base` jump, raw `map->odom` target/output gap, limited output, and the authority speed-limit heartbeat.
## 5.4 Corridor Localization And Command Containment (2026-07-10)

```text
raw GGA + stamped fix/heading + stamped LIO history
                         -> independent yaw/position gates
                         -> base-space correction release
                         -> map -> odom + motion_allowed heartbeat

controller_server -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel
motion_allowed ----------------------------------------------------------+
gps_route_runner -> /gps_corridor/stop_override ------------------------+-> corridor_cmd_vel_guard -> /cmd_vel_guarded -> serial_twistctl
```

The corrector is the only corridor `map->odom` owner. The runner owns stop intent and action retry, but never Twist. The guard is the only corridor `/cmd_vel_guarded` publisher and is a required launch process. In guarded mode serial subscribes only to that topic; missing either 10 Hz Bool heartbeat for 0.50 s stops output. Explore and other modes continue to use Nav2 `/cmd_vel` directly.
