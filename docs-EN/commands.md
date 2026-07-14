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
make launch-rtk-raw
make launch-fgo-gil-time-sync
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
bash scripts/launch_with_logs.sh rtk-raw
bash scripts/launch_with_logs.sh fgo-gil-time-sync
bash scripts/launch_with_logs.sh tightly-coupled
```

Equivalent `ros2 launch` invocation:

```bash
ros2 launch bringup system_slam.launch.py
ros2 launch bringup system_explore.launch.py
ros2 launch bringup system_gps_corridor.launch.py
ros2 launch bringup system_rtk_raw.launch.py
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
- After relocalization, Travel defaults to `continuous_icp: false`: `localizer` freezes the valid `map -> odom` correction and republishes it with the current ROS time at `tf_republish_hz`, avoiding map drift from partial local scans during navigation; sending `/initialpose` or calling `/localizer/relocalize` runs ICP again
- Travel now starts `initialpose_relocalize_bridge.py`, so RViz `2D Pose Estimate` on `/initialpose` calls `/localizer/relocalize` automatically with the launch-time `pcd_map`
- Travel also starts `nav2_cloud_retime.py`; the local costmap reads `/fastlio2/body_cloud_nav2`, a current-stamp copy of `/fastlio2/body_cloud_nav2_obstacles`, while the global costmap plans on the static 2D map and `localizer`/mapping nodes keep using the original `/fastlio2/body_cloud`
- Travel `NavigateToPose` / `NavigateThroughPoses` use dedicated fail-stop behavior trees: if the local controller or planner fails, navigation stops and returns failure instead of automatically running `Spin`, `BackUp`, or costmap-clearing recovery actions
- Travel uses the MPPI indoor safety profile (`vx_max=0.35`, `wz_max=0.65`, `controller_frequency=20 Hz`) while keeping the fail-stop behavior trees and the `6 m x 6 m @ 0.05 m` local costmap
- Before sending a navigation goal, verify in RViz that the live point cloud/scan overlaps the static map; Travel freezes the successful `map -> odom` correction by default, so a rough pose or heading error shifts the whole subsequent path
- PGO is off by default; if `use_pgo:=true` is passed, it uses `pgo_slam.yaml` and does not publish TF
- After startup, use `/localizer/relocalize` to reload the PCD map and set the initial pose:

```bash
ros2 service call /localizer/relocalize interface/srv/Relocalize \
  "{pcd_path: '/home/badger/XJTLU-autonomous-vehicle/runtime-data/maps/3d/<map_name>/map.pcd', x: 0.0, y: 0.0, z: 0.0, yaw: 0.0, pitch: 0.0, roll: 0.0}"
```

For normal field operation, prefer RViz `2D Pose Estimate` over the manual service call: click the vehicle's current map position and drag the arrow along the vehicle heading.

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

QGIS route-network import flow, for example `/Users/badger/Desktop/maps/qgis_4_package/3.geojson`:

```bash
python3 scripts/compile_qgis_scene.py \
  --input /Users/badger/Desktop/maps/qgis_4_package/3.geojson \
  --scene-name qgis_4 \
  --densify-step-m 5.0 \
  --output ~/XJTLU-autonomous-vehicle/runtime-data/gnss/scene_gps_bundle.yaml

python3 scripts/build_scene_runtime.py
```

`compile_qgis_scene.py` converts `feature_type=route` LineStrings into a scene graph, densifies route edges to at most 5m, and writes usable destinations into `scene_gps_bundle.yaml`. For the current `qgis_4_package/3.geojson`, the default output is about `557` route nodes, `558` edges, and these initial destination families: `math_building`, `environment_building`, `route_end_*`, and `junction_*`.

Collection guidelines:
- All turns, intersections, and destination entrances must have waypoints
- Areas where the system may be powered on must have nearby `anchor` points
- Graph edges are understood as straight-line segments between nodes; curves must be discretized by adding more nodes
- The script will prompt whether to automatically create an edge with the previous point

## 13. GPS Navigation Debugging

```bash
# Launch nav-gps
make launch-nav-gps

# View scene destination list
ros2 run gps_waypoint_dispatcher list_destinations

# Indoor software smoke test can use mock /fix to drive gps_anchor_localizer
ros2 topic pub /fix sensor_msgs/msg/NavSatFix \
  "{header: {frame_id: 'gps'}, status: {status: 0, service: 1}, latitude: 31.274927, longitude: 120.737548, altitude: 0.0, position_covariance: [4.0, 0.0, 0.0, 0.0, 4.0, 0.0, 0.0, 0.0, 25.0], position_covariance_type: 2}" \
  --rate 5

# Observe ready status
ros2 topic echo /gps_system/status
ros2 topic echo /gps_goal_manager/status

# Send English-named destination
ros2 run gps_waypoint_dispatcher goto_name anchor_a

# Check if route / local planner actions are online
ros2 action list | grep -E 'compute_route|follow_path'

# Stop current task
ros2 run gps_waypoint_dispatcher stop

# One-command launch nav-gps, wait for NAV_READY or RTK_AUTHORITATIVE, and select destination by number
python3 scripts/nav_gps_menu.py
```

Runtime notes:
- After modifying or importing a new QGIS/scene map, rerun `python3 scripts/build_scene_runtime.py` so `master_params_scene.yaml` records the scene fixed origin plus `rtk_map_odom_corrector`'s `scene_points_file` and `use_scene_identity_alignment=true`.
- `nav-gps` does not require the vehicle to start near an anchor before accepting a destination; the goal manager reads the current `map->base_link` pose and sends `ComputeRoute(use_poses=true)` so the route server starts from the nearest traversable graph node.
- `route_server` runs with `enable_nn_search=true` in `nav-gps`, so as long as the vehicle is near the route network, the start pose is snapped to the nearest traversable graph node and the route follows the graph to the destination.
- `nav-gps` now reuses the corridor RTK-authoritative chain: PGO disables `publish_tf` and GPS factors, while `rtk_map_odom_corrector` is the only `map->odom` owner.
- Nav2 uses the corridor RTK MPPI profile and the high-window `/fastlio2/body_cloud_nav2_obstacles` obstacle cloud; the older DWB-based `nav2_gps.yaml` profile is no longer the vehicle entry point for destination-by-name navigation.
- The RTK FGO shadow node starts by default with `publish_tf=false` and `nav2_use_fgo=false`; set `FYP_NAV_GPS_ENABLE_FGO_SHADOW=false` to disable it.
- The default lean bag records RTK, FAST-LIO2 odom, Livox IMU, chassis `/odom_CBoar`, `/rtk_fgo/*`, TF, GPS/goal status, costmaps, `/cmd_vel`, and `/plan`; use `FYP_NAV_GPS_BAG_PROFILE=debug` only when raw point-cloud replay is needed.
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
- Corridor currently generates a temporary Nav2 parameter file from `nav2_corridor_rtk.yaml` at launch time and applies the RTK-authoritative corridor profile: `vx_max=0.85`, `wz_max=0.70`, `ax_max=0.85`, `ax_min=-1.2`, `az_max=1.4`, `temperature=0.45`, `regenerate_noises=true`, `failure_tolerance=1.5s`, `controller_frequency=20Hz`, and `batch_size=500`; the local costmap is near-field `12m x 12m`, STVL marking uses `obstacle_range=5m`, and `CostCritic.cost_weight=7.0`; MPPI keeps `model_dt=0.05s`, so the control period must not be larger than the model step and cannot be lowered to `15Hz`
- After the final waypoint is reached, `gps_route_runner` publishes `STOPPING_BEFORE_EXIT`, holds zero `/cmd_vel` for 1.2s at 20Hz, then publishes `SUCCEEDED`; quiet mode exits only after this hold, so the bag should contain a visible zero-speed tail.
- Corridor starts the RTK FGO shadow node by default, with `publish_tf=false` and `nav2_use_fgo=false`, so it does not own `map->odom` or feed Nav2; set `FYP_CORRIDOR_ENABLE_FGO_SHADOW=false` to disable it
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

## UM982 Raw Binary Shadow Capture

This mode reads only the dedicated `/dev/rtk_um982_raw`. Never override it with the production NMEA/NTRIP device `/dev/rtk_um982`.

```bash
make build-rtk-raw
ss
make launch-rtk-raw
```

Override the device through an uncommitted vehicle parameter file:

```bash
FYP_UM982_RAW_PARAMS_FILE=/tmp/um982_raw_vehicle.yaml make launch-rtk-raw
```

Inspect checksum-valid frames, GNSS week/TOW, message ID, and diagnostics:

```bash
ros2 topic hz /gnss/raw/frame
ros2 topic echo /gnss/raw/frame --once
ros2 topic hz /gnss/raw/observation_epoch
ros2 topic echo /gnss/raw/observation_epoch --once
ros2 topic hz /gnss/raw/ephemeris
ros2 topic echo /gnss/raw/ephemeris --once
ros2 topic echo /gnss/raw/diagnostics --once
ros2 bag info runtime-data/logs/latest/bag | grep -E '/gnss/raw/frame|/gnss/raw/observation_epoch|/gnss/raw/ephemeris|/gnss/raw/diagnostics'
```

Phase 1 plus the uncompressed-observation and broadcast-ephemeris canonicalization subsets of Phase 2 are implemented. IDs 12/13/284 publish master/secondary/base epochs; IDs 106/107/108/109/110 publish GPS/GLONASS/BDS/Galileo/QZSS ephemerides. Every payload receives exact-length, PRN, time, finite-value, and orbit-range validation. Phase 5 now propagates these ephemerides to transmit-time satellite states with Sagnac correction. Compressed observations, RTCM fallback, and a real UART fixture remain pending. Without a dedicated raw UART, `SERIAL_DISCONNECTED` or `NO_RECENT_VALID_FRAME` is the expected fail-closed diagnostic.

## FGO-GIL Phase 3 Time Sync And IMU Frontend

Start the source stack that provides `/gnss/raw/observation_epoch`, `/livox/imu`, and `/livox/lidar`, then launch the shadow frontend:

```bash
make build-fgo-gil
ss
make launch-fgo-gil-time-sync
```

Inspect clock state and buffer diagnostics:

```bash
ros2 topic echo /fgo_gil/time_sync_diagnostics
```

State semantics:

- `UNSYNCED`: fewer than five valid clock pairs; high-weight joint factors are forbidden.
- `COARSE_NO_PPS`: coarse mapping from raw-GNSS reception time, with a default 20 ms uncertainty floor; five seconds without a new clock pair returns to `UNSYNCED`.
- `PPS_LOCKED`: at least two consecutive `/gnss/pps/time_reference` samples and the newest no older than 2 s; default uncertainty floor is 0.1 ms.

The current Livox driver stamps production `/livox/imu` and `/livox/lidar` with ROS `now()`, so `fgo_gil.yaml` defaults to the `ros` domain. Change a domain to `device` only after the corresponding `/livox/*_time_reference` is genuinely published and validated. This launch starts no TF/Nav2 owner and publishes no `/cmd_vel`.

Audit a SQLite rosbag without a ROS Python environment:

```bash
python3 scripts/analyze_fgo_gil_imu_bag.py <bag-directory> --max-gap-s 0.05
```

All 26,004 IMU messages in the local `jetson_2026-06-24-13-54-59` bag decode without duplicates, reversals, or non-finite measurements. Its effective rate is only about 83.47 Hz, however, with 1,109 gaps over 50 ms and a maximum gap of about 9.94 s, so it is not a continuous-preintegration acceptance bag. The documented 2026-07-10 bag still needs to be retrieved from the analysis machine and rechecked with the same tool for the expected approximately 193.5 Hz stream.

## FGO-GIL Phase 4 Raw LiDAR Factor Frontend

Start Livox, the FAST-LIO initialization input, and the UM982 raw-observation source, then launch the combined Phase 3+4 shadow frontend:

```bash
make build-fgo-gil
ss
make launch-fgo-gil-lidar
```

Inspect the only Phase 4 output:

```bash
ros2 topic echo /fgo_gil/lidar_diagnostics
```

Key states:

- `WAITING_FOR_TIME_SYNC`: Phase 3 has not reached `COARSE`/`PPS_LOCKED`; scans cannot enter the map.
- `WAITING_FOR_LIO_INITIALIZATION` / `WAITING_FOR_IMU_COVERAGE`: no initializer at or before scan start, or no continuous IMU coverage through scan end.
- `MAP_INITIALIZED`: the first keyframe passed timing, de-skew, and minimum-feature checks, but no valid constraint is claimed yet.
- `DEGENERATE_NO_CONSTRAINT` / `MATCHES_INSUFFICIENT`: residuals remain observable, but `constraint_valid=false` and no keyframe is added.
- `TRACKING`: line/plane match counts, minimum information eigenvalue, and condition number all pass.

Diagnostics include `edge_features`, `plane_features`, `line_matches`, `plane_matches`, `residual_rms_m`, `information_min_eigenvalue`, `information_condition`, `latency_ms`, and all rejection counters. This mode publishes no TF, odometry, path, or `/cmd_vel`. Available local bags do not contain `/livox/lidar`, so real MID360 feature counts, latency, and thresholds still require the 2026-07-10 bag or a new recording; current YAML values cannot support paper results.

## FGO-GIL Phase 5-6 GNSS DD, Float FGO, And Integer Fixing

Start the Livox/FAST-LIO initialization source and the dedicated UM982 raw source first, then launch the Phase 3-6 shadow graph:

```bash
make build-fgo-gil
ss
make launch-fgo-gil-float
```

Inspect factor batches, float/fixed ECEF odometry, and graph diagnostics:

```bash
ros2 topic echo /fgo_gil/lidar_constraints --once
ros2 topic echo /fgo_gil/float_diagnostics
ros2 topic echo /fgo_gil/float_odom_ecef --once
ros2 topic echo /fgo_gil/fixed_odom_ecef --once
```

The checked-in parameters intentionally set both of these to `false`:

```yaml
calibration.ecef_from_lidar_world.calibrated: false
calibration.gnss.base_ecef_calibrated: false
```

Before field replay, provide an uncommitted parameter override with the measured `T_ecef_lidar_world` and the CORS station ECEF coordinate. Do not set either flag to `true` with zero placeholders. The default master lever arm in IMU coordinates is `[0.0, -0.184, 0.134] m`; it still inherits the uncalibrated 2 cm IMU-height assumption.

Expected fail-closed states are `WAITING_FOR_CALIBRATION`, `WAITING_FOR_LIDAR_KEYFRAME`, `WAITING_FOR_CONTINUOUS_IMU`, and `LIO_ONLY_WAITING_BASE`. `FLOAT_ACTIVE` only means the graph is optimizing; `FIXED_ACTIVE` only means the current candidate passed configured gates. Neither is field acceptance. `/fgo_gil/float_odom_ecef` always remains the float solution, while `/fgo_gil/fixed_odom_ecef` is published only after the ratio, success-rate, residual, and back-substitution checks pass.

Phase 6 fields on `/fgo_gil/float_diagnostics` include `solution_status`, `ambiguity_ratio`, `ambiguity_success_rate`, `fixed_ambiguities`, `fix_rejection_reason`, `back_substitution_rejection`, and fixed correction/cost metrics. GLONASS FDMA is excluded from integer fixing in Phase 6. The node publishes no TF, `/cmd_vel`, or Nav2 input; the path added in Phase 7 is shadow output only. `make kill-runtime` includes all three FGO-GIL executables.

## FGO-GIL Phase 7 Full Shadow Runtime, Bagging, And Evaluation

Live mode starts Livox, the FAST-LIO2 comparator, the dedicated UM982 raw driver, and the Phase 3-7 FGO-GIL path. It records the `full` profile by default:

```bash
make build-fgo-gil
source install/setup.bash
make launch-fgo-gil-shadow
```

Use the `minimal` profile when bag size matters:

```bash
bash scripts/launch_with_logs.sh fgo-gil-shadow bag_profile:=minimal
```

For an existing bag, start the algorithm-only path first, then publish `/clock` from another terminal:

```bash
bash scripts/launch_with_logs.sh fgo-gil-shadow \
  use_sim_time:=true start_livox:=false start_fastlio:=false \
  start_raw_driver:=false record_bag:=false

ros2 bag play <bag-directory> --clock
```

Inspect the selected solution, path, and layered diagnostics:

```bash
ros2 topic echo /fgo_gil/odom --once
ros2 topic echo /fgo_gil/path --once
ros2 topic echo /fgo_gil/factor_diagnostics
ros2 topic echo /fgo_gil/ambiguity_status
ros2 topic echo /fgo_gil/timing_status
ros2 topic echo /fgo_gil/performance
```

Full decoding requires the ROS 2 and workspace setup to be sourced. A workstation can audit topic evidence in an older bag with `--metadata-only`:

```bash
python3 scripts/evaluate_fgo_gil_bag.py \
  --bag <bag-directory> --out /tmp/fgo_gil_metrics.json

python3 scripts/evaluate_fgo_gil_bag.py \
  --bag <old-bag-directory> --out /tmp/fgo_gil_metadata.json --metadata-only
```

The result contains SE(3)-aligned APE/RPE, availability, fixing rate, outage drift, optimization latency, real-time factor, and CPU/RAM parsed from the same session's `tegrastats.log`. If an old bag lacks `/gnss/raw/observation_epoch`, or that topic has zero messages, the result must say `RAW_GNSS_UNAVAILABLE`. Such a bag validates comparators and non-raw paths only; it is not paper-level GNSS acceptance.

Phase 7 forces `publish_tf=false` and `nav2_use_fgo=false`. Startup fails if either is set to `true`; this mode starts neither serial control nor Nav2. Use `make kill-runtime` to stop rosbag, Livox, FAST-LIO, the raw driver, and all three FGO-GIL executables.

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
