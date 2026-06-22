# System Architecture

## 1. Deployment Locations

- Jetson code repository: `~/XJTLU-autonomous-vehicle`
- Runtime data root directory: `~/XJTLU-autonomous-vehicle/runtime-data`
- GitHub remote: `kevinlasnh/XJTLU-autonomous-vehicle`
- AI collaboration control plane: located in a separate PC repository, not within this code repository

## 2. Hardware Platform

- Jetson Orin NX, 16 GB RAM, Ubuntu 22.04, ROS 2 Humble
- Livox MID360 LiDAR
- WIT IMU
- Basic GNSS module, currently used at approximately 2.5 m accuracy level, not an RTK workflow
- Serial connection to STM32 lower-level controller
- PS2 gamepad as the highest-priority manual override

## 3. Seven Operating Modes

| Mode | Command | Current Purpose |
|------|---------|-----------------|
| SLAM | `make launch-slam` | Mapping and perception chain verification |
| Explore | `make launch-explore` | Current primary operating mode, local obstacle avoidance navigation |
| Indoor Nav | `make launch-indoor-nav` | RViz click-to-go navigation without GNSS |
| Corridor | `make launch-corridor` | GPS Corridor v2 main runtime on the MPPI controller |
| Explore GPS | `make launch-explore-gps` | Explore with GNSS and PGO GPS factor added |
| Nav GPS | `make launch-nav-gps` | Scene bundle + anchor ready + GPS route-graph navigation mode |
| Travel | `make launch-travel` | Static map navigation framework, currently paused |

All `make launch-*` entry points go through `scripts/launch_with_logs.sh`, so session-isolated log directories are created by default.

## 4. Explore Mode Data Flow

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

## 5. GPS-Related Chains

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

GNSS serial -> /fix -> gps_anchor_localizer -> /gnss -----------+
                       |                    |                   |
                       |                    +-> /gps_system/*   +-> PGO GPS Factor
                       |
                       +-> lock startup anchor + session offset

scene_points.yaml + route_graph.geojson ------------------------+
                                                               |
goto_name -> gps_waypoint_dispatcher(goal manager) ------------+
            |  reads scene_points.yaml
            |  checks NAV_READY
            |  locks startup anchor
            |  Stage A: navigate_to_pose (when needed)
            |  Stage B: ComputeRoute(start_id, goal_id)
            v
     dense graph path -> FollowPath -> Nav2 -> /cmd_vel
```

The core of `nav-gps` is:
- Current pose uniformly uses `/gnss` published by `gps_anchor_localizer`
- PGO, localizer, and goal manager all read the same fixed ENU origin
- `scene_gps_bundle.yaml` is the single source of truth
- At runtime, only compiled artifacts under `~/XJTLU-autonomous-vehicle/runtime-data/gnss/current_scene/` are read
- `goto_name` is the main entry point; users input only English destination names

## 6. TF Chain

```text
map -> odom -> base_link
```

- `map -> odom` is published by PGO, representing global correction offset
- `odom -> base_link` is published by FAST-LIO2, representing high-frequency local odometry
- The combination of both yields the global pose

If `map -> odom` does not exist, RViz under the `map` fixed frame will appear as if point clouds or costmaps are blank, even if Livox and FAST-LIO2 are still running.

## 7. Configuration Architecture

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
- `perception/`: FAST-LIO2, PGO GPS fusion, point cloud to grid related; `rtk_fgo_localizer` is currently an experimental tight-coupled RTK FGO package skeleton and is not wired into any runtime chain yet
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
- pyproj
- `ros-humble-geographic-msgs`


## 5.3 GPS Corridor v2 Mode (Standalone Global Aligner Architecture)

```text
current_route.yaml
  -> start_ref / waypoints[] / launch_yaw_deg / enu_origin

/fix -----> gps_global_aligner -----> /gps_corridor/enu_to_map (smoothed ENU->map transform)
                 |                              |
                 | TF: map->base_link           |
                 +------------------------------+---> gps_route_runner
                                                      1. bootstrap: yaw0 + launch_yaw_deg -> initial ENU->map
                                                      2. wait for stable /fix
                                                      3. check start point <= start_ref tolerance
                                                      4. GPS waypoints -> ENU -> map (using aligner output)
                                                      5. freeze alignment within waypoint, split subgoals per segment
                                                      6. sequential NavigateToPose
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

Data plane for this mode:
- `~/XJTLU-autonomous-vehicle/runtime-data/gnss/current_route.yaml` (generated by `collect_gps_route.py`)
- `start_ref` + multiple `waypoints[]` GPS coordinates
- `launch_yaw_deg` is a required field
