# 系统架构

## 1. 运行位置

- Jetson 代码仓: `~/XJTLU-autonomous-vehicle`
- 运行时数据根目录: `~/XJTLU-autonomous-vehicle/runtime-data`
- GitHub 远端: `kevinlasnh/XJTLU-autonomous-vehicle`
- AI 协作控制面: 位于独立 PC 仓库，不在本代码仓内

## 2. 硬件平台

- Jetson Orin NX, 16 GB RAM, Ubuntu 22.04, ROS 2 Humble
- Livox MID360 LiDAR
- WIT IMU
- 基础 GNSS 模块，当前按约 2.5 m 级精度使用，不是 RTK 工作流
- 串口连接到 STM32 下位机
- PS2 手柄作为最高优先级人工接管

## 3. 八种运行模式

| 模式 | 命令 | 当前用途 |
|------|------|----------|
| SLAM | `make launch-slam` | 纯建图：生成 2D Nav2 地图、3D PGO 点云地图与 manifest |
| Explore | `make launch-explore` | 当前主运行模式，局部避障导航 |
| Indoor Nav | `make launch-indoor-nav` | 不启 GNSS 的 RViz 点击点导航 |
| Corridor | `make launch-corridor` | GPS Corridor v2 主链，基于 MPPI 控制器 |
| Explore GPS | `make launch-explore-gps` | Explore 基础上加入 GNSS 与 PGO GPS 因子 |
| Nav GPS | `make launch-nav-gps` | scene bundle + anchor ready + GPS 路网导航模式 |
| Tightly Coupled | `make launch-tightly-coupled` | 实验性 RTK FGO shadow mode，旁路发布 `/rtk_fgo/*` |
| Travel | `make launch-travel` | 实验性先验地图导航：2D map 全局规划 + PCD 点云重定位 |

所有 `make launch-*` 入口都通过 `scripts/launch_with_logs.sh` 启动，因此默认会生成按 session 隔离的日志目录。

## 4. SLAM 纯建图数据流

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
  -> 保存 2D map.yaml/map.pgm
  -> 保存 3D map.pcd/poses.txt/patches
  -> 写 manifest.yaml，包括 2D/3D 一致性、patch/pose 完整性与 frame 检查
```

SLAM 模式不启动 Nav2 planner/controller，也不执行导航行为。PGO 在该模式下使用 `pgo_slam.yaml`，默认 `publish_tf=false`，避免与 SLAM Toolbox 同时发布 `map -> odom`。保存脚本默认检查 FAST-LIO2 当前子坐标系 `base_footprint`，但现场修改 `base_frame` 前必须先用 TF 工具确认实际子坐标系。RTK 可通过 `use_rtk:=true` 在建图时记录室外 Fixed 样本，但室内 invalid/float RTK 只作为记录，不作为强约束。

## 5. Explore 模式数据流

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

## 6. GPS 相关链路

### 5.1 Explore GPS 模式

```text
GNSS serial -> nmea_navsat_driver -> /fix
                                   |
                                   v
                          gnss_calibration -> /gnss
                                              |
                                              v
                                   PGO GPS Factor constraints
```

`make launch-explore-gps` 的职责仍然是把校准后的 `/gnss` 注入 PGO，提升 `map -> odom` 的全局位置约束能力。

### 5.2 Nav GPS 模式（scene bundle + route graph）

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
            |  读取 scene_points.yaml
            |  检查 NAV_READY
            |  锁定 startup anchor
            |  Stage A: navigate_to_pose (需要时)
            |  Stage B: ComputeRoute(start_id, goal_id)
            v
     dense graph path -> FollowPath -> Nav2 -> /cmd_vel
```

`nav-gps` 的核心是：
- 当前位姿统一使用 `gps_anchor_localizer` 发布的 `/gnss`
- PGO、localizer、goal manager 读取同一 fixed ENU origin
- `scene_gps_bundle.yaml` 是唯一 source of truth
- 运行时只读取 `~/XJTLU-autonomous-vehicle/runtime-data/gnss/current_scene/` 下的编译产物
- `goto_name` 是主入口；用户只输入英文目标名

### 5.3 RTK FGO 紧耦合实验模式（shadow）

```text
Explore stack + UM982 RTK
  -> rtk_fgo_localizer
       inputs: /fastlio2/lio_odom, /livox/imu, /odom_CBoar, /fix, /heading, /rtk/*
       outputs: /rtk_fgo/odom, /rtk_fgo/path, /rtk_fgo/status, /rtk_fgo/rtk_gate
                /rtk_fgo/correction_status, /rtk_fgo/factor_diagnostics
```

该模式通过 `make launch-tightly-coupled` 启动，第一版是 shadow mode：
- 默认 `publish_tf=false`，不广播生产 `map -> odom`
- 不 remap Nav2，不替换 `corridor`、`explore-gps`、`nav-gps`
- 自动录制源传感器 topic 与 `/rtk_fgo/*`，用于 rosbag replay 和实车旁路验证

## 7. TF 链

```text
map -> odom -> base_footprint -> base_link
```

- 生产导航模式下，`map -> odom` 由 PGO 发布，表示全局校正偏移
- SLAM 纯建图模式下，`map -> odom` 由 SLAM Toolbox 发布；PGO 只保存 3D 地图，不发布 TF
- Travel 先验地图模式下，`map -> odom` 由 `localizer` 的 ICP 点云重定位发布；启动预加载 PCD 后仍需 `/localizer/relocalize` 成功才开始广播，避免未验证或旧时间戳 TF 污染 Nav2
- PGO 默认不启动，或只以 `publish_tf=false` 运行
- `odom -> base_footprint` 由 FAST-LIO2 发布，表示高频局部里程计；`base_footprint -> base_link` 由 URDF 静态 TF 提供
- 两者组合后得到全局位姿

如果 `map -> odom` 不存在，RViz 在 `map` fixed frame 下会表现为点云或 costmap 看起来空白，即使 Livox 和 FAST-LIO2 本身还在运行。

## 8. 配置架构

- `src/bringup/config/master_params.yaml`
  - 仓库模板参数入口
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

## 8. 日志与运行时数据

`~/XJTLU-autonomous-vehicle/runtime-data/` 位于工作区内部，当前主要包含：

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
- `data/`: 各节点自定义数据日志
- `system/`: `tegrastats.log` 与 `session_info.yaml`

## 9. 源码层级

```text
src/
├── sensor_drivers/
├── perception/
├── planning/
├── navigation/
└── bringup/
```

说明：
- `sensor_drivers/`: Livox、IMU、GNSS、串口
- `perception/`: FAST-LIO2、PGO GPS 融合、点云转栅格相关；`rtk_fgo_localizer` 是紧耦合 RTK FGO 实验包，当前只接入 shadow mode
- `planning/`: 历史 GPS 全局规划与坐标转换试验区
- `navigation/`: `waypoint_collector` 与 scene-graph goal manager `gps_waypoint_dispatcher`
- `bringup/`: 系统 launch、参数、地图、RViz 配置
- 上游依赖通过 `vcs import < dependencies.repos` 拉取，不作为项目自定义开发区

## 10. 包构建类型

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
  - `rtk_fgo_localizer`（实验包骨架）
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

## 11. 关键依赖

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


## 5.3 GPS Corridor v2 模式（独立 Global Aligner 架构）

```text
current_route.yaml
  -> start_ref / waypoints[] / launch_yaw_deg / enu_origin

/fix -----> gps_global_aligner -----> /gps_corridor/enu_to_map (平滑 ENU→map 变换)
                 |                              |
                 | TF: map->base_link           |
                 +------------------------------+---> gps_route_runner
                                                      1. bootstrap: yaw0 + launch_yaw_deg → 初始 ENU→map
                                                      2. 等稳定 /fix
                                                      3. 检查启动点 ≤ start_ref 容差
                                                      4. GPS waypoints → ENU → map (用 aligner 输出)
                                                      5. waypoint 内冻结 alignment，按段切 subgoals
                                                      6. 串行 NavigateToPose
                                                      v
                                               Nav2 Explore stack (MPPI controller)
                                               -> planner/controller/costmaps
                                               -> /cmd_vel
```

该模式与 `nav-gps` 的区别：
- 不使用 scene graph / `route_server`
- 不使用 `gps_waypoint_dispatcher` 的 `goto_name` / menu 交互
- 不使用 runtime `current_scene/` 编译产物
- 使用独立 `gps_global_aligner_node` 替代 PGO live handoff

该模式的定位假设：
- 车辆从固定物理 Launch Pose 启动，朝向物理固定
- `launch_yaw_deg` 记录车辆启动时的地理朝向（ENU 约定）
- 支持多点 waypoint 路线（不限于两点直线）

该模式的关键架构决策：
- **独立 global aligner**: 与 PGO 解耦，平滑发布 `ENU→map` 变换
- **Live alignment 重算 subgoal**: 运行中持续使用当前对齐结果重算有效 subgoal，不再使用 per-waypoint frozen 机制
- **Bootstrap 启动**: 用 `yaw0 - radians(launch_yaw_deg)` 立即计算初始对齐，不等 GPS
- **Corridor 专用 Nav2 BT**: corridor 使用无 `Spin` / `BackUp` 的 NavigateToPose / NavigateThroughPoses 行为树；规划或跟踪失败时由 `gps_route_runner` 停车并上报进度，不执行物理 recovery 动作
- **Nav2 action 超时**: corridor 运行时将 BT `default_server_timeout` 提高到 1000ms，避免 Jetson 负载下 FollowPath ack 稍慢就误触发 recovery
- **PGO TF 保鲜**: PGO 在无新 cloud 时按 timer 重发最近一次 `map→odom`，保证 Nav2 controller/costmap 查询当前时刻 TF 时不会因 LIO 低频或掉帧误判

该模式的数据面：
- `~/XJTLU-autonomous-vehicle/runtime-data/gnss/current_route.yaml`（`collect_gps_route.py` 生成）
- `start_ref` + 多个 `waypoints[]` 的 GPS 坐标
- `launch_yaw_deg` 为必填字段
