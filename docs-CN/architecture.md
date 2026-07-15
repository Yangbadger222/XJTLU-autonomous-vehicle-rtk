# 系统架构

## 1. 运行位置

- Jetson 代码仓: `~/XJTLU-autonomous-vehicle`
- 运行时数据根目录: `~/XJTLU-autonomous-vehicle/runtime-data`
- GitHub 远端: `Yangbadger222/XJTLU-autonomous-vehicle-rtk`
- AI 协作控制面: 位于独立 PC 仓库，不在本代码仓内

## 2. 硬件平台

- Jetson Orin NX, 16 GB RAM, Ubuntu 22.04, ROS 2 Humble
- Livox MID360 LiDAR
- WIT IMU
- T-RTK UM982 双天线 Mobile 套装 + 4G 模块 (移动端)
- 串口连接到 STM32 下位机
- PS2 手柄作为最高优先级人工接管

## 3. 八种运行模式

| 模式 | 命令 | 当前用途 |
|------|------|----------|
| SLAM | `make launch-slam` | 纯建图：生成 2D Nav2 地图、3D PGO 点云地图与 manifest |
| Explore | `make launch-explore` | 当前主运行模式，局部避障导航 |
| Indoor Nav | `make launch-indoor-nav` | 不启 GNSS 的 RViz 点击点导航 |
| Corridor | `make launch-corridor` | GPS Corridor v2 主链，基于 MPPI 控制器 |
| Travel | `make launch-travel` | 实验性先验地图导航：2D map 全局规划 + PCD 点云重定位 |
| Explore GPS | `make launch-explore-gps` | Explore 基础上加入 GNSS 与 PGO GPS 因子 |
| Nav GPS | `make launch-nav-gps` | scene bundle + RTK authority + GPS 路网导航模式 |
| RTK Basic | `make launch-rtk-basic` | RTK 信号检测 |
| Tightly Coupled | `make launch-tightly-coupled` | 实验性 RTK FGO shadow mode，旁路发布 `/rtk_fgo/*` |

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
                       -> current_scene/road_keepout.{yaml,pgm}

GNSS serial -> /fix + /heading + /rtk/status
                       |
                       v
                rtk_map_odom_corrector
                scene fixed ENU -> map identity
                TF: map -> odom

可选 legacy：gps_anchor_localizer -> /gnss + /gps_system/*

goto_name / /goal_pose / /gps_goal
            -> gps_waypoint_dispatcher
            |  当前位姿和终点投影到最近 graph edge
            |  插入虚拟起点/终点，欧氏启发式 A*
            |  生成一条 0.35m 密度的连续 NavPath
            v
     FollowPath -> MPPI + obstacle cloud + road keepout
                -> /cmd_vel_nav -> authority guard -> /cmd_vel
```

`nav-gps` 的核心是：
- 默认不启动旧 `gps_anchor_localizer`；当前 A*/RTK-authority 生产链不消费 `/gnss` 或 anchor readiness，需要兼容实验时可显式启用
- `map -> odom` 不再由 PGO 抢发布；PGO 使用 corridor no-TF/no-GPS 配置，仅保留点云/优化旁路能力
- `rtk_map_odom_corrector` 读取 scene fixed origin，并使用固定 ENU→map identity alignment 计算 RTK authoritative `map -> odom`
- Nav2 使用 corridor RTK MPPI profile 和 `/fastlio2/body_cloud_nav2_obstacles` 高窗障碍点云，而不是旧 `nav2_gps.yaml` 的 DWB profile
- goal manager 自己执行图 A*，起终点投影到最近 graph edge，不再依赖 `route_server` 的 Dijkstra 或少数 anchor
- QGIS 道路面编译成 KeepoutFilter mask；MPPI 可在道路内部避障，但道路外部保持禁止通行
- authority 失效时 guard 立即清零，goal manager 取消当前 `FollowPath`；authority 连续恢复后从当前位置重新 A* 规划
- 实车 profile 保持 `controller_frequency=20Hz` 与 `model_dt=0.05s` 匹配，但将 MPPI 工作量收敛到 `350x40` samples、2秒预测视野，并关闭 critic statistics
- 默认 lean bag 保留较小的 local costmap 用于避障复盘，但不录占本次 bag `77.5%` 的 global costmap；debug profile 才追加 global costmap、原始点云和 legacy anchor 状态
- `scene_gps_bundle.yaml` 是唯一 source of truth
- 运行时只读取 `~/XJTLU-autonomous-vehicle/runtime-data/gnss/current_scene/` 下的编译产物
- 支持 `goto_name`、地图 `/goal_pose` 和经纬度 `/gps_goal` 三种终点入口

### 5.3 RTK FGO 紧耦合实验模式（shadow）

```text
Explore stack + UM982 RTK
  -> rtk_fgo_localizer
       inputs: /fastlio2/lio_odom, /livox/imu, /odom_CBoar, /fix, /heading, /rtk/*
       outputs: /rtk_fgo/odom, /rtk_fgo/path, /rtk_fgo/status, /rtk_fgo/rtk_gate
                /rtk_fgo/correction_status, /rtk_fgo/factor_diagnostics
```

该模式可通过 `make launch-tightly-coupled` 单独启动；`corridor` 默认旁路启动，`nav-gps` 实车入口因 CPU 预算默认关闭，可通过 `FYP_NAV_GPS_ENABLE_FGO_SHADOW=true` 显式启用：
- 默认 `publish_tf=false`，不广播生产 `map -> odom`
- 不 remap Nav2，不替换 `corridor`、`explore-gps`、`nav-gps` 的生产定位输出
- 自动录制源传感器 topic 与 `/rtk_fgo/*`，用于 rosbag replay 和实车旁路验证
- 后续 FGO 接管时只允许 FGO 成为唯一 `map -> odom` owner，并继续发布统一的 `/localization_authority/motion_allowed`；A*、MPPI 与菜单不绑定具体 authority 名称

## 7. TF 链

```text
map -> odom -> base_footprint -> base_link
```

- Explore / explore-gps 等生产导航模式下，`map -> odom` 由 PGO 发布，表示全局校正偏移
- Corridor 与 RTK nav-gps 模式下，PGO 关闭 `publish_tf`，唯一生产 `map -> odom` owner 是 `rtk_map_odom_corrector`
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
- pyproj（推荐用于精确 GPS 投影；QGIS scene 编译和 nav-gps 读取有本地 ENU fallback）
- `ros-humble-geographic-msgs`


## 5.3 GPS Corridor v2 模式（独立 Global Aligner 架构）

```text
current_route.yaml
  -> start_ref / waypoints[] / launch_yaw_deg / enu_origin

/fix -----> gps_global_aligner -----> /gps_corridor/enu_to_map (平滑 ENU→map 变换)
  |                              |
  |                              +---> gps_route_runner
  |                                    1. bootstrap: yaw0 + launch_yaw_deg → 初始 ENU→map
  |                                    2. 等稳定 /fix
  |                                    3. 检查启动点 ≤ start_ref 容差
  |                                    4. GPS waypoints → ENU → map (用 aligner 输出)
  |                                    5. waypoint 内冻结 alignment，按段切 subgoals
  |                                    6. 串行 NavigateToPose
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

该模式与 `nav-gps` 的区别：
- 不使用 scene graph / A* goal manager
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
- **Nav2 专用障碍点云**: corridor local costmap 使用 `/fastlio2/body_cloud_nav2_obstacles`，高度窗为 `[-0.20, 1.20]m`；PGO/LIO 仍使用低窗 `/fastlio2/body_cloud`，避免为了建图稳定而裁掉的高障碍同时让 Nav2 失明
- **RTK authoritative `map→odom`**: corridor 中 PGO 通过 `pgo_corridor_no_gps.yaml` 关闭 `publish_tf`，由 `rtk_map_odom_corrector` 根据 RTK fix、双天线 heading、`ENU→map` 和当前 `odom→base_link` 计算唯一的 `map→odom`
- **RTK bootstrap**: 在 `gps_global_aligner` 尚未发布 `ENU→map` 前，`rtk_map_odom_corrector` 会用当前 RTK fix、heading 和 `odom→base_link` 先发布临时 `map→odom`，打破启动时 aligner 等待 map TF 的闭环
- **RTK degraded hold**: 当 RTK fix、heading 或目标跳变被 gating 拒绝时，`rtk_map_odom_corrector` 不更新全局位姿，但会继续用最后一次可信输出刷新 `map→odom` 时间戳，避免 Nav2 因 TF 过期误判导航失败
- **RTK target 平滑与跳变门控**: `rtk_map_odom_corrector` 在 raw `map→odom` target 和最终单步限幅之间加入 target deadband + 低通，抑制 RTK/heading 微抖持续写入 `map→odom` 后造成后段“画龙”；target-jump gate 的平移/yaw 安全判断比较当前 raw RTK `map_base` 与上一帧可信 raw RTK `map_base`，避免把 `odom` 原点杠杆放大的 `map→odom` target 平移误判为 RTK 跳变
- **室内外切换接口**: `rtk_map_odom_corrector` 发布 `/localization_authority/mode`、`/localization_authority/status` 和 `/localization_authority/diagnostics`；后续室内先验地图 relocalization 可作为新的 authority source 接管同一 `map→odom` 接口

该模式的数据面：
- `~/XJTLU-autonomous-vehicle/runtime-data/gnss/current_route.yaml`（`collect_gps_route.py` 生成）
- `start_ref` + 多个 `waypoints[]` 的 GPS 坐标
- `launch_yaw_deg` 为必填字段
- `/localization_authority/*` 记录当前 `map→odom` authority 来源、拒绝原因、raw RTK `map_base` jump、raw `map→odom` target/output gap 和限幅后的输出
## 5.4 Corridor 定位与命令收敛链（2026-07-10）

```text
raw GGA + stamped fix/heading + stamped LIO history
                         -> 独立 yaw/position gates
                         -> base-space correction release
                         -> map -> odom + motion_allowed heartbeat

controller_server -> /cmd_vel_controller -> velocity_smoother -> /cmd_vel_nav
motion_allowed ----------------------------------------------------------+
gps_route_runner -> /gps_corridor/stop_override ------------------------+-> corridor_cmd_vel_guard -> /cmd_vel -> serial_twistctl
```

corrector 是 corridor 唯一 `map→odom` owner。runner 只拥有 stop intent 与 action retry，不发布 Twist。guard 是 corridor 唯一 `/cmd_vel` publisher，也是 required launch process；任一 10 Hz Bool heartbeat 超过 0.50 秒缺失都会输出停车。Explore 与其他模式不启用这些 remap。
