# Operations Command Manual

This document only records commands confirmed to be executable in the current repository and current Jetson environment.

## Initial Setup

All commands below rely on the following conditions:
1. The robot repository is cloned into a folder `~/XJTLU-autonomous-vehicle`
2. The robot has ROS2 Humble installed
3. `~/.bashrc` matches exactly the version in [/scripts/.bashrc](/scripts/.bashrc)

To create the directory, run:
```bash
cd ~/
mkdir XJTLU-autonomous-vehicle
cd ~/XJTLU-autonomous-vehicle
```

To install ROS2 Humble, follow their [official documentation](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html).

To set `~/.bashrc` for the first time:
- Copy the contents of [/scripts/.bashrc](/scripts/.bashrc) into your clipboard
- SSH into the Jetson
- Run:
```bash
vi ~/.bashrc
```
- Then, type `:%d`, press `Enter`
- Then, paste your clipboard contents
- After, press `Esc`, then type `:wq`, press `Enter`
- You should be back to the terminal. Run:
```bash
source ~/.bashrc
```

## 1. Build and Source

```bash
# Initial dependency setup
make setup

# Full build
make build

# Layered build
make build-bringup
make build-fastlio2
make build-sensor
make build-perception
make build-planning
make build-navigation

# Single package build
colcon build --packages-select <pkg> --symlink-install --parallel-workers 1

# Must re-source after every build
ss
```

## 2. Initialize Runtime Data

```bash
bash scripts/init_runtime_data.sh

ls ~/XJTLU-autonomous-vehicle/runtime-data
```

## 3. Launch Operating Modes

```bash
make launch-slam
make launch-explore
make launch-indoor-nav
make launch-corridor
make launch-travel
make launch-explore-gps
make launch-nav-gps
make launch-rtk-basic
make launch-tightly-coupled
```

Equivalent wrapper direct invocation:

```bash
bash scripts/launch_with_logs.sh slam
bash scripts/launch_with_logs.sh explore
bash scripts/launch_with_logs.sh indoor-nav
bash scripts/launch_with_logs.sh corridor
bash scripts/launch_with_logs.sh travel
bash scripts/launch_with_logs.sh explore-gps
bash scripts/launch_with_logs.sh nav-gps
bash scripts/launch_with_logs.sh rtk-basic
bash scripts/launch_with_logs.sh tightly-coupled
```

Equivalent `ros2 launch` invocation:

```bash
ros2 launch bringup system_slam.launch.py
ros2 launch bringup system_explore.launch.py
ros2 launch bringup system_gps_corridor.launch.py
ros2 launch bringup system_tightly_coupled.launch.py
ros2 launch bringup system_explore_gps.launch.py
ros2 launch bringup system_nav_gps.launch.py
ros2 launch bringup system_travel.launch.py
```

Optional RTK recording in pure SLAM mapping:

```bash
# Default mapping run: build 2D/3D maps without starting RTK
bash scripts/launch_with_logs.sh slam

# Enable RTK only when outdoor Fixed samples are needed for later indoor/outdoor geo-registration
ros2 launch bringup system_slam.launch.py use_rtk:=true
```

One-line command for indoor click-to-go navigation without GPS:

> Compatibility note: `FYP_*` names are legacy runtime interface variables still read by the current scripts. This documentation pass updates public project wording, not runtime interface names.

```bash
FYP_USE_RVIZ=true bash scripts/launch_with_logs.sh indoor-nav
```

Notes:
- `indoor-nav` does not start the GNSS driver, `gps_global_aligner`, or `gps_route_runner`
- It keeps Livox, FAST-LIO2, PGO, Nav2, and the serial control chain running
- In RViz, use `2D Goal Pose` to publish goals to `/goal_pose` for indoor click-to-go navigation

One-line command for prior-map Travel navigation:

```bash
FYP_USE_RVIZ=true bash scripts/launch_with_logs.sh travel \
  map_yaml:=/home/badger/XJTLU-autonomous-vehicle/runtime-data/maps/2d/<map_name>/map.yaml \
  pcd_map:=/home/badger/XJTLU-autonomous-vehicle/runtime-data/maps/3d/<map_name>/map.pcd
```

Notes:
- `travel` uses the 2D `map.yaml` for Nav2 global planning and the 3D `map.pcd` for ICP point-cloud relocalization in `localizer`
- `localizer` owns `map -> odom`; FAST-LIO2 owns `odom -> base_footprint`, and URDF provides `base_footprint -> base_link`
- `localizer` only preloads the PCD map at startup; it does not publish `map -> odom` until `/localizer/relocalize` succeeds
- PGO is off by default; if `use_pgo:=true` is passed, it uses `pgo_slam.yaml` and does not publish TF
- After startup, use `/localizer/relocalize` to reload the PCD map and set the initial pose:

```bash
ros2 service call /localizer/relocalize interface/srv/Relocalize \
  "{pcd_path: '/home/badger/XJTLU-autonomous-vehicle/runtime-data/maps/3d/<map_name>/map.pcd', x: 0.0, y: 0.0, z: 0.0, yaw: 0.0, pitch: 0.0, roll: 0.0}"
```

Verification:

```bash
ros2 run tf2_ros tf2_monitor odom base_footprint
ros2 service call /localizer/relocalize_check interface/srv/IsValid "{code: 0}"
ros2 run tf2_ros tf2_monitor map odom
```

One-line command for GPS Corridor v2:

```bash
FYP_USE_RVIZ=true bash scripts/launch_with_logs.sh corridor
```

Notes:
- Corridor-specific route capture, startup watchdog, and runtime behavior are documented in Section 14
- The wrapper maintains both session logs and the foreground status monitor output

## 4. Launch Individual Core Components

```bash
# Livox
ros2 launch livox_ros_driver2 msg_MID360_launch.py

# WIT IMU
ros2 run wit_ros2_imu wit_ros2_imu

# GNSS raw driver
ros2 launch nmea_navsat_driver nmea_serial_driver.launch.py

# GNSS calibration
ros2 launch gnss_calibration gnss_calibration_launch.py

# GNSS scene-ready localizer (new GPS route-graph architecture)
ros2 run gnss_calibration gps_anchor_localizer_node \
  --ros-args --params-file ~/XJTLU-autonomous-vehicle/runtime-data/gnss/current_scene/master_params_scene.yaml

# FAST-LIO2
ros2 launch fastlio2 lio_no_rviz.py params_file:=~/XJTLU-autonomous-vehicle/src/bringup/config/master_params.yaml

# PGO + FAST-LIO2
ros2 launch pgo pgo_launch.py params_file:=~/XJTLU-autonomous-vehicle/src/bringup/config/master_params.yaml

# Compatible with legacy flat PGO config
ros2 launch pgo pgo_launch.py pgo_config:=pgo_no_gps.yaml

# Serial nodes
ros2 run serial_reader serial_reader_node
ros2 run serial_twistctl serial_twistctl_node

# waypoint_collector
ros2 run waypoint_collector waypoint_node

# GPS goal manager CLI
ros2 run gps_waypoint_dispatcher goto_name <destination_name>
ros2 run gps_waypoint_dispatcher list_destinations
ros2 run gps_waypoint_dispatcher stop
```

## 5. Debugging and Status Checks

```bash
# topic / node / action
ros2 topic list
ros2 node list
ros2 action list
ros2 action info /compute_route
ros2 action info /follow_path
ros2 action info /navigate_to_pose
ros2 node info /pgo/pgo_node

# Frequency and messages
ros2 topic hz /livox/lidar
ros2 topic hz /fastlio2/body_cloud
ros2 topic echo /pgo/optimized_odom --once
ros2 topic echo /fix --once
ros2 topic echo /gnss --once
ros2 topic echo /gps_system/status --once
ros2 topic echo /gps_system/nearest_anchor --once
ros2 topic echo /gps_system/nearest_anchor_id --once
ros2 topic echo /gps_goal_manager/status --once
ros2 topic echo /gps_waypoint_dispatcher/goal_map --once
ros2 topic echo /gps_waypoint_dispatcher/path_map --once

# Parameters
ros2 param get /fastlio2/lio_node lidar_max_range
ros2 param get /pgo/pgo_node gps.enable
ros2 param get /pgo/pgo_node gps.topic
ros2 param get /pgo/pgo_node gps.origin_mode

# TF
ros2 run tf2_ros tf2_monitor
ros2 run tf2_ros tf2_monitor map odom
ros2 run tf2_ros tf2_monitor odom base_footprint
ros2 run tf2_tools tf2_echo map base_link
```

Common diagnostic focus points:

- Whether `map -> odom` exists
- Whether `/pgo/optimized_odom` is being published continuously
- Whether `/gps_system/status` has reached `NAV_READY`
- Whether `/gnss` contains valid scene-calibrated GNSS data published by `gps_anchor_localizer`
- Whether `/compute_route` / `/follow_path` / `/navigate_to_pose` actions are online
- Whether the RViz fixed frame is set to `map`

## 6. Logs and Runtime Data

```bash
# Check what latest points to
readlink -f ~/XJTLU-autonomous-vehicle/runtime-data/logs/latest

# View current session metadata
cat ~/XJTLU-autonomous-vehicle/runtime-data/logs/latest/system/session_info.yaml

# View tegrastats
tail -f ~/XJTLU-autonomous-vehicle/runtime-data/logs/latest/system/tegrastats.log

# View console log directory
ls ~/XJTLU-autonomous-vehicle/runtime-data/logs/latest/console

# View data log directory
ls ~/XJTLU-autonomous-vehicle/runtime-data/logs/latest/data
```

## 7. Data Collection and Evaluation

```bash
# Record rosbag
bash scripts/data_collection/record_bag.sh
bash scripts/data_collection/record_bag.sh ~/XJTLU-autonomous-vehicle/runtime-data/bags/my_run

# Record tegrastats separately
bash scripts/data_collection/record_perf.sh
bash scripts/data_collection/record_perf.sh ~/XJTLU-autonomous-vehicle/runtime-data/perf/my_run.log

# Export TUM trajectory
python3 scripts/data_collection/bag_to_tum.py   ~/XJTLU-autonomous-vehicle/runtime-data/bags/my_run/rosbag2   /pgo/optimized_odom   ~/XJTLU-autonomous-vehicle/runtime-data/bags/my_run/pgo_optimized.tum
```

## 8. Map Saving

```bash
# Save the current SLAM session's 2D + 3D maps and write a manifest
scripts/save_mapping_session.sh <map_name>
```

Output:

```text
runtime-data/maps/<map_name>/manifest.yaml
runtime-data/maps/2d/<map_name>/map.yaml
runtime-data/maps/2d/<map_name>/map.pgm
runtime-data/maps/3d/<map_name>/map.pcd
runtime-data/maps/3d/<map_name>/poses.txt
runtime-data/maps/3d/<map_name>/patches/*.pcd
```

Notes:
- `manifest.yaml` records `consistency_ok` to flag likely 2D/3D map drift; it is gated by the 2D/3D alignment diagnostic, patch/pose integrity, and frame checks. This is a save-time diagnostic, not a replacement for later relocalization validation
- `patch_pose_integrity.ok` must be `true`, meaning `patches/*.pcd` and `poses.txt` keyframes are one-to-one
- `frame_check.ok` must be `true`; by default `/scan.header.frame_id` and `/fastlio2/lio_odom.child_frame_id` are expected to be `base_footprint`. If the vehicle's FAST-LIO2 child frame is different, confirm it with `view_frames`/`tf2_echo` first, then save with `--expected-base-frame <frame>`
- Later indoor/outdoor geo-registration must use RTK Fixed samples plus heading; indoor invalid/float RTK samples are records only, not strong constraints

Low-level troubleshooting commands:

```bash
# Confirm TF and frame names before saving; do not change base_frame blindly.
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo odom base_footprint

# Save 3D point cloud map; file_path must be absolute because ROS service requests do not expand ~
ros2 service call /pgo/save_maps interface/srv/SaveMaps "{file_path: '/home/badger/XJTLU-autonomous-vehicle/runtime-data/maps/3d/<map_name>', save_patches: true}"

# Save 2D occupancy grid map
ros2 run nav2_map_server map_saver_cli -f ~/XJTLU-autonomous-vehicle/runtime-data/maps/2d/<map_name>/map --ros-args -p map_subscribe_transient_local:=true

# View PCD
pcl_viewer -bc 1,1,1 -ps 3 <map.pcd>
```

## 9. Stop System and Emergency Stop

```bash
# Clean up after system shutdown to ensure a clean state for the next launch
make kill
```

Stop and emergency-stop priority:

1. PS2 gamepad `X` button disables motors as the highest-priority software stop
2. Red physical emergency stop button on the vehicle body overrides all software commands

The lower-controller `B` button path now performs damped active braking: it keeps motor control enabled, applies current opposite to wheel speed, preserves a high current limit at higher speed for short stopping distance, tapers current at low speed, rate-limits current changes, then clears state and keeps sending zero-current frames near stop. The `B` brake latch initializes only on the first trigger, so holding `B` does not repeatedly clear the current ramp; the `B` indication is solid pink and no longer uses a blocking blink. Do not use `B` as a replacement for `X` or the red physical e-stop until bench and vehicle validation are complete.

## 10. Git and PR

```bash
# Sync main
git checkout main
git pull --ff-only

# Create branch
git checkout -b <BRANCH_NAME>

# Check status
git status
git branch -v
git log --oneline -5

# Push branch
git push -u origin <BRANCH_NAME>
```

GitHub CLI:

```bash
gh auth status
gh pr create
gh pr merge --merge --delete-branch
```

If `gh auth status` on the Jetson returns an invalid token, you can run `gh pr create` / `gh pr merge` on a local workstation that is already logged into GitHub CLI for the same branch, then return to the Jetson to execute:

```bash
git checkout main
git pull --ff-only
git fetch --prune
```

## 11. System Maintenance

```bash
# Disk / memory
df -h /
free -h
htop

# Disk usage per folder
sudo du -h --max-depth=1 / | sort -hr

# JetPack / model
cat /etc/nv_tegra_release
cat /proc/device-tree/model

# NetworkManager and wired interface auto-start status
systemctl is-enabled NetworkManager
systemctl is-active NetworkManager
nmcli -t -f NAME,AUTOCONNECT,AUTOCONNECT-PRIORITY,DEVICE connection show --active

# Set network autoconnection settings
sudo nmcli connection modify "WiFi-Name" connection.autoconnect yes
sudo nmcli connection modify "WiFi-Name" connection.autoconnect-priority 100
sudo nmcli connection modify "WiFi-Name" connection.autoconnect-retries 3

# Check if current machine has passwordless sudo
sudo -n true && echo sudo_ok

# Switch Jetson WiFi and restart ToDesk on the Jetson side (execute directly on the Linux host)
bash scripts/switch_jetson_wifi.sh --status
bash scripts/switch_jetson_wifi.sh
bash scripts/switch_jetson_wifi.sh outdoor
bash scripts/switch_jetson_wifi.sh indoor
bash scripts/switch_jetson_wifi.sh Pixel
bash scripts/switch_jetson_wifi.sh XJTLU

# GPS dispatcher dependencies
apt list --installed | grep ros-humble-geographic-msgs
python3 -c "import pyproj; print(pyproj.__version__)"
```

Notes:
- Without arguments, the script toggles between `XJTLU` and `Pixel` by default
- The commands above are complete one-line commands to run directly on the Jetson / Linux host
- `pyproj` remains the recommended dependency; if it is temporarily missing, QGIS scene compilation and nav-gps scene loading fall back to a local ENU approximation instead of failing at startup
- When the script runs locally on the Jetson, it automatically switches to local mode; if the current shell is SSH/Tailscale, the session may disconnect during the network switch
- Each network switch restarts `todeskd` on the Jetson side, with logs written to `/tmp/wifi-switch.log`

## 12. GPS Data Collection

Minimum two-line launch commands:

```bash
ros2 launch nmea_navsat_driver nmea_serial_driver.launch.py params_file:=/home/jetson/XJTLU-autonomous-vehicle/src/bringup/config/master_params.yaml
python3 scripts/collect_gps_scene.py
```

```bash
python3 scripts/collect_gps_scene.py
```

Script description:
- Coordinate source: **uses /fix only**
- Sampling: 10 samples per point, averaged
- Quality threshold: sample spread < 2 m, otherwise collection is rejected
- Output file: `~/XJTLU-autonomous-vehicle/runtime-data/gnss/scene_gps_bundle.yaml`
- A single file simultaneously maintains:
  - fixed origin
  - graph nodes
  - `anchor`
  - `dest`
  - edges

Interactive commands:
- `Enter`: collect a map point
- `e`: add an edge between two points, treated as bidirectional
- `o`: select a fixed origin from existing points
- `u`: modify name / anchor / destination of an existing point
- `l`: list all points and edges, showing anchor / dest / origin
- `d`: delete a specific point by ID
- `q`: save and exit

Compile runtime files after collection:

```bash
python3 scripts/build_scene_runtime.py
```

QGIS route-network import flow, for example `~/Desktop/maps/qgis_4_package/3.geojson`:

```bash
python3 scripts/compile_qgis_scene.py \
  --input ~/Desktop/maps/qgis_4_package/3.geojson \
  --road-area-gpkg ~/Desktop/maps/qgis_4_package/road_wide_all.gpkg \
  --road-area-layer road_wide \
  --scene-name qgis_4 \
  --densify-step-m 5.0 \
  --road-mask-resolution-m 0.10 \
  --road-mask-expansion-m 1.0 \
  --output ~/XJTLU-autonomous-vehicle/runtime-data/gnss/scene_gps_bundle.yaml

python3 scripts/build_scene_runtime.py
```

`compile_qgis_scene.py` converts `feature_type=route` LineStrings into a connected scene graph, densifies edges to at most 5m, and rasterizes the GeoPackage road polygon into `road_keepout.yaml/.pgm`. With the current package and `1.0m` expansion on each side, it produces 557 nodes, 558 edges, 12 destinations, and an approximately `5958x3572 @ 0.10m` road mask. Disconnected graphs are rejected so intersections can be split/snapped in QGIS first.

The current `road_wide_all.gpkg` extends only about `0.50m` to each side of the centerline, or roughly `1.0m` total. That leaves little margin around the configured navigation radius of about `0.386m`. The current vehicle compile command uses `--road-mask-expansion-m 1.0` to expand each drivable boundary by `1.0m`, producing roughly `3.0m` of effective total width. This is a test-stage compensation; redraw the polygon from real road boundaries and reduce the expansion toward `0` before final acceptance.

Collection guidelines:
- All turns, intersections, and destination entrances must have waypoints
- Nearby `anchor` points are required only for legacy-anchor compatibility experiments; current RTK-authority A* may start anywhere near the route graph
- Graph edges are understood as straight-line segments between nodes; curves must be discretized by adding more nodes
- The script will prompt whether to automatically create an edge with the previous point

## 13. GPS Navigation Debugging

```bash
# Launch nav-gps
make launch-nav-gps

# View scene destination list
ros2 run gps_waypoint_dispatcher list_destinations

# Legacy anchor-chain indoor smoke: opt in before publishing mock /fix
FYP_NAV_GPS_ENABLE_LEGACY_ANCHOR_LOCALIZER=true make launch-nav-gps
ros2 topic pub /fix sensor_msgs/msg/NavSatFix \
  "{header: {frame_id: 'gps'}, status: {status: 0, service: 1}, latitude: 31.274927, longitude: 120.737548, altitude: 0.0, position_covariance: [4.0, 0.0, 0.0, 0.0, 4.0, 0.0, 0.0, 0.0, 25.0], position_covariance_type: 2}" \
  --rate 5

# Observe ready status
ros2 topic echo /gps_system/status
ros2 topic echo /gps_goal_manager/status

# Send English-named destination
ros2 run gps_waypoint_dispatcher goto_name anchor_a

# Arbitrary geographic destination: snap to nearest route edge, then run A*
ros2 run gps_waypoint_dispatcher goto_latlon 31.2749432 120.7380295

# RViz/Foxglove may also publish a map-frame /goal_pose
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 10.0, y: 5.0}, orientation: {w: 1.0}}}"

# Check FollowPath
ros2 action list | grep follow_path

# Inspect the common authority contract and guarded command path
ros2 topic echo /localization_authority/motion_allowed
ros2 topic echo /gps_nav/stop_override
ros2 topic echo /cmd_vel_nav
ros2 topic echo /cmd_vel

# Stop current task
ros2 run gps_waypoint_dispatcher stop

# Launch nav-gps, wait for the active authority to allow motion, and select by number
python3 scripts/nav_gps_menu.py
```

Runtime notes:
- After modifying or importing a new QGIS/scene map, rerun `python3 scripts/build_scene_runtime.py` so `master_params_scene.yaml` records the scene fixed origin plus `rtk_map_odom_corrector`'s `scene_points_file` and `use_scene_identity_alignment=true`.
- `nav-gps` does not require a nearby anchor. The goal manager projects the current pose and destination onto graph edges, inserts virtual endpoints, runs A*, and sends the whole route as one `FollowPath`.
- After the RTK position and heading gates first lock, scene-identity mode seeds the absolute `map->odom` directly. This permits startup anywhere near the route network; subsequent updates still pass through corridor-authority smoothing, jump, and fault gates.
- When `current_scene/road_keepout.yaml` exists, both costmaps enable KeepoutFilter so local avoidance remains inside the QGIS road polygon.
- `nav-gps` now reuses the corridor RTK-authoritative chain: PGO disables `publish_tf` and GPS factors, while `rtk_map_odom_corrector` is the only `map->odom` owner.
- `nav-gps` also uses `/cmd_vel_nav -> guard -> /cmd_vel`; authority loss cancels the active path and stops, and continuous recovery replans A* from the current pose.
- Nav2 uses the corridor RTK MPPI profile and the high-window `/fastlio2/body_cloud_nav2_obstacles` obstacle cloud; the older DWB-based `nav2_gps.yaml` profile is no longer the vehicle entry point for destination-by-name navigation.
- MPPI keeps `controller_frequency=20Hz` aligned with `model_dt=0.05s`, while the vehicle profile uses `batch_size=350`, `time_steps=40`, and densifies the continuous A* path at `0.35m`. This cuts trajectory simulation per cycle by about 42% from the previous `500x48` workload.
- The `nav-gps` vehicle entry point disables RTK FGO shadow by default so it does not compete with FAST-LIO2/Nav2 for Jetson CPU. Set `FYP_NAV_GPS_ENABLE_FGO_SHADOW=true` only for shadow evidence; it still forces `publish_tf=false` and `nav2_use_fgo=false`. A future FGO takeover must first disable RTK-corrector TF output and keep the common `motion_allowed` contract.
- The current RTK-authority/A* chain disables the legacy `gps_anchor_localizer` by default because planning and motion permission consume neither anchors nor `/gnss`; set `FYP_NAV_GPS_ENABLE_LEGACY_ANCHOR_LOCALIZER=true` for compatibility experiments.
- The default lean bag records RTK, FAST-LIO2 odom, Livox IMU, chassis `/odom_CBoar`, `/rtk_fgo/*`, TF, goal/authority status, all three velocity stages, the local costmap, and `/plan`. The global costmap, legacy anchor status, and raw point clouds are added only with `FYP_NAV_GPS_BAG_PROFILE=debug`.
- On the vehicle, prefer `FYP_USE_RVIZ=false bash scripts/launch_with_logs.sh nav-gps` to avoid spending Jetson resources on RViz.

## 14. Fixed-Launch GPS Corridor

### GPS Route Collection (Waypoint Survey)

```bash
python3 scripts/collect_gps_route.py
```

Interactive workflow:
1. Enter route name
2. Place the vehicle at the start point, press Enter to collect `start_ref` (10 samples, spread < 2 m)
3. Move to each waypoint sequentially, press Enter to collect
   - After each point, ENU coordinate preview and spread are displayed
   - `Accept / Retry? [A/r]` -- poor signal allows immediate re-collection
   - Altitude anomalies (> 10 m jump) trigger automatic warnings
4. Confirm `launch_yaw_deg` (auto-suggested if first segment > 5 m, otherwise manual input)
5. Route summary table displayed before saving (segment distances, bearings, ENU coordinates)
6. Confirm save -> `~/XJTLU-autonomous-vehicle/runtime-data/gnss/current_route.yaml`

### Automatic Corridor Navigation

```bash
bash scripts/launch_with_logs.sh corridor
```

One-line launch with RTK/CORS parameters for field testing:

> Do not commit the real CORS password. Use `make ntrip-login` in the Jetson to handle CORS credentials.

```bash
FYP_USE_RVIZ=false FYP_CORRIDOR_CONSOLE_MODE=quiet bash scripts/launch_with_logs.sh corridor
```

Confirm the route that will be used before launching:

```bash
sed -n '1,120p' runtime-data/gnss/current_route.yaml
```

Clean up residual processes after completion:

```bash
make kill
```

Makefile shortcut launch:

```bash
make launch-corridor
```

Debug observation:

```bash
ros2 topic echo /gps_corridor/status
ros2 topic echo /gps_corridor/goal_map
ros2 topic echo /gps_corridor/path_map
ros2 topic echo /gps_corridor/enu_to_map
```

Check whether the default automatic corridor bag contains the lean RTK / FAST-LIO2 / FGO shadow / Nav2 diagnostic topics:

```bash
ros2 bag info runtime-data/logs/latest/bag | grep -E '/fix|/heading|/rtk/status|/rtk/nmea_sentence|/fastlio2/lio_odom|/livox/imu|/odom_CBoar|/rtk_fgo|/cmd_vel|/plan'
```

If raw Livox replay is needed, opt in to the heavier debug bag profile before launch:

```bash
FYP_CORRIDOR_BAG_PROFILE=debug FYP_USE_RVIZ=false FYP_CORRIDOR_CONSOLE_MODE=quiet bash scripts/launch_with_logs.sh corridor
ros2 bag info runtime-data/logs/latest/bag | grep -E '/livox/lidar|/fastlio2/body_cloud'
```

Notes:
- This mode assumes the vehicle is already placed at the fixed Launch Pose with the heading aligned
- `collect_gps_route.py` collects `start_ref + multiple key waypoints` and generates `~/XJTLU-autonomous-vehicle/runtime-data/gnss/current_route.yaml`
- If `collect_gps_route.py` does not detect `/fix`, it automatically launches `nmea_navsat_driver` in the background and stops it after collection
- The collection process explicitly confirms `launch_yaw_deg`; if the start point is too close to the first waypoint, manual input is required
- Default subgoal spacing is 5 m, automatically written to the route file during collection; long RTK route legs are split into short subgoals to reduce rolling-costmap and local-tracking coupling risk
- At runtime, no menu appears and no additional commands are awaited
- The wrapper writes logs and bags to `~/XJTLU-autonomous-vehicle/runtime-data/logs/<session>/`
- Corridor currently generates a temporary Nav2 parameter file from `nav2_corridor_rtk.yaml` at launch time and applies the RTK-authoritative corridor profile: `vx_max=0.85`, `wz_max=0.70`, `ax_max=0.85`, `ax_min=-1.2`, `az_max=1.4`, `temperature=0.45`, `regenerate_noises=true`, `open_loop=false`, `failure_tolerance=1.5s`, `controller_frequency=20Hz`, and `batch_size=500`; the local costmap is near-field `12m x 12m`, STVL marking uses `obstacle_range=5m`, and `CostCritic.cost_weight=7.0`; MPPI keeps `model_dt=0.05s`, so the control period must not be larger than the model step and cannot be lowered to `15Hz`
- After the final waypoint is reached, `gps_route_runner` publishes `STOPPING_BEFORE_EXIT`, holds zero `/cmd_vel` for 1.2s at 20Hz, then publishes `SUCCEEDED`; quiet mode exits only after this hold, so the bag should contain a visible zero-speed tail.
- Corridor disables the RTK FGO shadow node by default to reduce Jetson CPU load; set `FYP_CORRIDOR_ENABLE_FGO_SHADOW=true` when shadow evidence is needed. When enabled, it still forces `publish_tf=false` and `nav2_use_fgo=false`, so it does not own `map->odom` or feed Nav2
- The default corridor bag uses the lean profile and records RTK, FAST-LIO2 odom, Livox IMU, chassis `/odom_CBoar`, `/rtk_fgo/*`, TF, corridor status, goals, costmaps, `/cmd_vel`, and `/plan`; raw Livox point cloud, `/fastlio2/body_cloud`, and `/fastlio2/body_cloud_nav2_obstacles` are recorded only with `FYP_CORRIDOR_BAG_PROFILE=debug`
- Livox packet-scale console/CSV logging is disabled during normal runs. Use `LIVOX_VERBOSE_PACKET_LOGS=1` only for short bench diagnostics because it prints and flushes per packet.
- During startup, if the current `/fix` deviates from `start_ref` beyond tolerance, `gps_route_runner` will abort immediately without moving the vehicle
- **Ctrl+C automatically cleans up all nodes, ros2 daemon, and serial port occupancy** -- no need for manual `make kill-runtime`

**Quiet mode** (default):
- Only simplified status messages are shown in the foreground
- Full launch output is written to `~/XJTLU-autonomous-vehicle/runtime-data/logs/<session>/system/launch_stdout.log`
- Default startup timeout is 45 s, adjustable via the `FYP_CORRIDOR_STARTUP_TIMEOUT_S` environment variable

**Raw mode** (for debugging):
```bash
FYP_CORRIDOR_CONSOLE_MODE=raw bash scripts/launch_with_logs.sh corridor
```

## RTK FGO Tight-Coupled Shadow Mode

Build:

```bash
make build-perception
ss
```

Launch the experimental shadow mode:

```bash
make launch-tightly-coupled
FYP_USE_RVIZ=false bash scripts/launch_with_logs.sh tightly-coupled
```

Launch with field RTK/CORS parameters:

```bash
FYP_RTK_PARAMS_FILE=/tmp/um982_cors.yaml FYP_USE_RVIZ=false bash scripts/launch_with_logs.sh tightly-coupled
```

Observe shadow outputs:

```bash
ros2 topic echo /rtk_fgo/status
ros2 topic echo /rtk_fgo/rtk_gate
ros2 topic echo /rtk_fgo/correction_status
ros2 topic echo /rtk_fgo/factor_diagnostics
```

Check chassis feedback and bag capture:

```bash
ros2 topic hz /cmd_vel
ros2 topic hz /odom_CBoar
ros2 topic echo /odom_CBoar --once
ros2 bag info runtime-data/logs/latest/bag | grep -E '/odom_CBoar|/cmd_vel|/fix|/heading|/rtk_fgo|/pgo/optimized_odom|/pgo/loop_markers|/livox/lidar|/fastlio2/body_cloud'
tail -f runtime-data/logs/latest/data/serial_reader.log
```

Generate replay metrics from the latest tightly-coupled bag:

```bash
python3 scripts/evaluate_rtk_fgo_bag.py \
  --bag runtime-data/logs/latest/bag \
  --out runtime-data/logs/latest/system/rtk_fgo_metrics.json
```

Experimental TF must be enabled explicitly and only for guarded tests:

```bash
ros2 launch bringup system_tightly_coupled.launch.py publish_fgo_tf:=true nav2_use_fgo:=false
```

Notes:
- This mode defaults to `publish_tf=false` and does not broadcast production `map -> odom`
- Nav2 is not remapped, and `corridor`, `explore-gps`, and `nav-gps` are not replaced
- The automatic bag includes `/rtk_fgo/*`, `/fix`, `/heading`, `/rtk/status`, `/rtk/nmea_sentence`, `/livox/lidar`, `/livox/imu`, `/fastlio2/lio_odom`, `/fastlio2/body_cloud`, `/pgo/optimized_odom`, `/pgo/loop_markers`, and `/tf`
- `/rtk_fgo/factor_diagnostics` includes frame-anchor, wheel-factor, and graph-window health keys

***

## Huggingface

To upload rosbags into Huggingface:

```bash
hf upload frogcar/rtk-data-2026-surf ./runtime-data --repo-type dataset
```

To clone into your own computer:

1. First-time setup
```bash
pip install -U "huggingface_hub[cli]"
export HF_ENDPOINT=https://hf-mirror.com
hf auth login
```

When logging in, use our organization's access token.

2. Clone the repo:
```bash
hf download frogcar/rtk-data-2026-surf --repo-type dataset --local-dir ./rtk-data-2026-surf
```

***

## NTRIP Account Setting

When using the RTK antenna, the robot must have an NTRIP account to receive full-quality signal. These can be bought in Taobao, for example in here: https://e.tb.cn/h.Ry4kJCGRkkS8a8n?tk=VpOEgN1OG2z

In addition, this repo counts with a script that handles these credentials.

To login onto an NTRIP account, run:
```bash
make ntrip-login
```

To change the account parameters, such as the server IP and mountpoint, run:
```bash
make ntrip-setup
```

To check current credentials and connection test, run:
```bash
make ntrip-status
```

To log out, run:
```bash
make ntrip-logout
```

Equivalent wrapper direct invocation:
```bash
@python3 scripts/setup_ntrip.py
@python3 scripts/setup_ntrip.py --setup
@python3 scripts/setup_ntrip.py --status
@python3 scripts/setup_ntrip.py --logout
```

Once logged in, the credentials are stored in the robot. You will be logged in automatically every time until you manually log out or change the credentials.

***

## Foxglove

### Initial Setup

On your personal computer:
1. Create an account at https://app.foxglove.dev/signin
2. Download Foxglove https://foxglove.dev/download

In the Jetson, download Foxglove:
```bash
sudo apt update
sudo apt install ros-$ROS_DISTRO-foxglove-bridge
```

### Live Connection

To start a connection, SSH into the Jetson and run:
```bash
ros2 run foxglove_bridge foxglove_bridge
```

Then from your computer, open Foxglove and click on "Open Connection" -> "Foxglove WebSocket (default)" and enter `ws://100.79.128.22:8765`

After entering, click anywhere on the center view, and the left panel will load many options. This may take some time, up to a minute.

**Note**: the Tailscale IP may be slightly different per account. If you are not sure about the correct IP:
1. Open another terminal in the Jetson and run: `tailscale ip -4`
2. Open Tailscale from your computer or the [browser](https://login.tailscale.com/admin/machines) and check the robot address "badger"
3. Go back to Foxglove and type the correct IP address in here: `ws://<TAILSCALE_IP>:8765`

#### Foxglove Settings

To correctly render the robot URDF along with other data, such as the point cloud, use the following settings:
- Fixed frame: `<Root frame>`
- Display frame: `base_link`
- Follow mode: `Pose` (position + attitude)
- Sync timestamps: `Off`
- Location topic: `Auto`
- ENU frame: `<Fixed frame>`
- Grid Frame: `base_footprint`

### Debugging

Cannot connect:
- Double-check the Tailscale address is correct, and you have Tailscale on
- Ping the Jetson with the tailscale address from your computer with `ping <TAILSCALE_IP>`
- If using a VPN, go to your proxy settings and add all Tailscale IPs to the exception list: `100.*.*.*` (or try turning your VPN off and connecting again)
- If using Clash verge:
  1. Go to Settings -> System Proxy, click on the gear icon
  2. Set `Always use Default Bypass` to OFF
  3. A text input box should appear in the bottom (below `Proxy Bypass Settings`). Type this IP: `100.64.0.0/10`, then click on the NEW button, then SAVE.
  4. Go bach to the Settings page, look for "Tun Mode", click on the gear icon
  5. In the bottom text input box (below `Route Exclude Address`), type the same IP: `100.64.0.0/10`. Click on NEW, then SAVE.
  6. Go back to Foxglove and try again.

Connection too slow:
- Change the Jetson WiFi to your phone hotspot and ping it again

Topics not rendering properly (URDF or point cloud missing):
- Make sure in `Panel` -> `Topics`, the topics `/fastlio2/world_cloud` and `/robot_description` are visible (click on the eye icon)
- Close the current foxglove session and open another one, it usually fixes itself

## `~/.bashrc`

This script runs whenever a terminal is opened in the jetson (including SSH). We have modified it to include common commands and give us an overview of the robot's current state.

The script is being tracked in [/scripts/.bashrc](/scripts/.bashrc). To set it up in the Jetson:

1. Copy the script in [/scripts/.bashrc](/scripts/.bashrc) into your clipboard
2. Open a terminal in the Jetson (SSH or local are both ok)
3. Type the following command to open `~/.bashrc` with Vim:
```bash
rc
```
4. After it opens, type `:%d` to delete all contents in the file
5. Use `Ctrl + V` to paste the new script from your clipboard
6. Press `Esc`, then `:wq` to write and quit (save and exit)
7. To test it, run: `s1`

Whenever you want to update `~/.bashrc`, modify it first from [/scripts/.bashrc](/scripts/.bashrc), then follow the steps above to make sure we keep track of the file. Do not modify it in the Jetson without tracking it in this repo.
## 18. Corridor Authority Replay And Acceptance

Run the four 2026-07-10 external fixtures from a workstation with `rosbags` installed:

```bash
export FYP_CORRIDOR_BAG_ROOT=/path/to/rosbags-jetson-20260710
PYTHONPATH=src/navigation/gps_waypoint_dispatcher \
  python3 scripts/evaluate_corridor_authority_replay.py \
  --manifest --out /tmp/corridor-authority-replay.json
```

The command returns nonzero if an assertion fails. Current reference replay detects 20 s `LOCAL_NO_PROGRESS` in `13:34`, a 51.75-degree rejected heading correction in `13:36`, and release limits at or below `0.20 m/s` and `2 deg/s` in `13:46`/`13:48`. `max_consecutive_saturated` remains in the result as a diagnostic, but sustained limiting no longer fails acceptance by itself; safety is enforced by the release-rate limits, the `0.50 m/5 deg` backlog thresholds, and the `2.0 m/20 deg` fault thresholds.

After deploying to Jetson, build with the required single worker and re-source:

```bash
colcon build --packages-select gps_waypoint_dispatcher bringup --symlink-install --parallel-workers 1
source install/setup.bash
colcon test --packages-select gps_waypoint_dispatcher
PYTHONPATH=src/navigation/gps_waypoint_dispatcher python3 -m pytest \
  src/navigation/gps_waypoint_dispatcher/test/test_route_hold_integration.py \
  src/bringup/test/test_system_gps_corridor_launch.py -q
colcon test-result --verbose
```

Before vehicle acceptance, separately deploy the serial branch, flash the STM32, and perform the motor-disabled 500 ms command-loss bench. During runtime monitor `/localization_authority/motion_allowed`, `/gps_corridor/stop_override`, `/cmd_vel_nav`, and guarded `/cmd_vel`.
