# Nav2 参数调优记录

## 1. 基本概念

- 路径（Path）: 由 planner 生成的空间几何点集合
- 轨迹（Trajectory）: 由 controller 生成的带时间和速度约束的可执行运动序列

## 2. 当前主线速度边界（2026-04-15）

- `robot_radius`: `0.38625`
- `vx_max`: `1.0`（2026-04-15 吸收 IEEE demo 抗推头 baseline，从 1.5 下调）
- `wz_max`: `1.2`
- `ax_max`: `1.2`
- `ax_min`: `-3.0`
- `az_max`: `6.0`

## 3. 速度平滑器

- `feedback: OPEN_LOOP`
- `odom_topic: /fastlio2/lio_odom`
- `max_velocity: [1.0, 0.0, 1.2]`
- `max_accel: [1.2, 0.0, 6.0]`
- 原因: 已确认闭环接入现阶段不稳定，继续使用开环是当前真实配置

## 4. Explore / Corridor 主模式当前控制器（2026-07-06 更新）

Explore 模式使用 `nav2_explore.yaml` 的 MPPI 主线配置：

- `plugin: nav2_mppi_controller::MPPIController`
- `time_steps: 48`，`model_dt: 0.05`（前向仿真 2.4s）
- `batch_size: 1000`（每次采样 1000 条轨迹）
- `vx_max: 1.0`，`wz_max: 1.2`，`ax_max: 1.2`
- `temperature: 0.3`，`gamma: 0.015`
- `motion_model: DiffDrive`
- Critics: `ConstraintCritic`, `CostCritic(4.5)`, `GoalCritic(5.0)`, `GoalAngleCritic(3.0)`, `PathAlignCritic(cost_weight=12.0, offset=6)`, `PathFollowCritic(cost_weight=16.0)`, `PathAngleCritic(4.0)`, `PreferForwardCritic(5.0)`
- `controller_frequency: 20.0`
- `yaw_goal_tolerance: 6.28`（实质上禁用朝向检查）
- 仍保留 2026-04-05 收口的 `5 Hz` 全局重规划、A* 搜索和 5 级恢复行为

Corridor 模式沿用同一 MPPI 结构，但 `system_gps_corridor.launch.py` 会在启动时从 `nav2_explore.yaml` 生成临时 Nav2 参数文件，注入 RTK 首次室外验收低速 profile：

- `controller_frequency: 20.0`（保持与 `model_dt=0.05s` 一致；15Hz 会让 MPPI 配置失败）
- `batch_size: 500`
- `vx_max: 0.45`，`wz_max: 0.65`，`ax_max: 0.45`，`ax_min: -0.8`，`az_max: 2.0`
- `vx_std: 0.14`，`wz_std: 0.14`
- `velocity_smoother.max_velocity: [0.45, 0.0, 0.65]`
- `velocity_smoother.max_accel: [0.45, 0.0, 1.4]`
- `velocity_smoother.max_decel: [-0.8, 0.0, -1.8]`

原因：2026-07-06 RTK corridor 实车日志显示 RTK 本身保持 `RTK Fixed q=4`，但轨迹出现右偏，且 `/fastlio2/lio_odom` 有约 `1.18m/0.19s` 跳变。低速 profile 用于降低首次 RTK 验收时的右偏、过大角速度和 FAST-LIO2 退化风险。2026-07-06 20:06 的现场日志证明 `controller_frequency=15Hz` 会触发 MPPI `Controller period more then model dt` 配置失败，因此 corridor 保持 `20Hz` 并通过速度/采样量降负载。历史 DWB 配置已不再作为 Explore/Corridor 主线使用；`nav2_gps.yaml` 和 `nav2_travel.yaml` 保持独立配置。

## 4b. 2026-04-05 走廊高速基线（历史记录）

> 以下组合已经不再是当前主线代码值，但作为历史背景保留。

- `vx_max: 1.5`
- `wz_max: 1.75`
- `ax_max: 3.0`
- `velocity_smoother.max_velocity: [1.5, 0.0, 1.75]`
- `velocity_smoother.max_accel: [3.0, 0.0, 6.0]`
- 该轮同时把全局重规划提升到 `5 Hz` 并启用 `A*`
- 2026-04-15 吸收 IEEE demo 抗推头 baseline 时，仅收回了线速度/角速度/加速度上限；`5 Hz` 重规划与 `A*` 仍保留在当前基线中

## 4a. Corridor 模式 RPP 控制器（2026-03-22，已废弃）

> 以下参数仅供历史参考。Corridor 已于 2026-03-31 切换到 MPPI（commit `9d71823`）。

Corridor v2 使用 Rotation Shim + Regulated Pure Pursuit 替代 DWB：

- `RotationShimController.angular_dist_threshold: 0.785`（45 度）
- `RotationShimController.angular_disengage_threshold: 0.39`
- `RotationShimController.rotate_to_heading_angular_vel: 1.0`
- `RotationShimController.max_angular_accel: 1.6`
- `RotationShimController.simulate_ahead_time: 1.0`
- `RPP.desired_linear_vel: 0.5`
- `RPP.lookahead_dist: 1.0`
- `RPP.min_lookahead_dist: 0.45`
- `RPP.max_lookahead_dist: 1.5`
- `RPP.lookahead_time: 1.5`
- `RPP.max_allowed_time_to_collision_up_to_carrot: 0.30`
- `RPP.use_cost_regulated_linear_velocity_scaling: true`（2026-03-26 启用）
- `RPP.allow_reversing: false`

## 5. 代价地图相关结论（2026-04-15 更新）

### Local Costmap（当前 `nav2_explore.yaml` 实际值）

- 更新频率: `12 Hz`
- 发布频率: `6 Hz`
- `resolution: 0.05`
- `width/height: 30`（单位是米，不是 cells）
- STVL `voxel_decay: 0.8`
- `obstacle_range: 15.0`
- `min_obstacle_height: -0.33` / `max_obstacle_height: 0.30`
- `inflation_radius: 0.43`，`cost_scaling_factor: 2.0`
- `denoise_layer.minimal_group_size: 4`

### Global Costmap（当前 `nav2_explore.yaml` 实际值）

- 更新频率: `5 Hz`（2026-04-05 从 3Hz 提升以支持 5Hz 重规划）
- 发布频率: `2.0 Hz`
- `resolution: 0.10`
- `width/height: 50`（单位是米，2026-04-05 缩小以降低开销）
- STVL `voxel_decay: 1.5`
- `obstacle_range: 15.0`
- `min_obstacle_height: -0.33` / `max_obstacle_height: 0.30`
- `inflation_radius: 0.63`，`cost_scaling_factor: 1.0`

### 已知待改进

- 当前 costmap 画布本身不是主要瓶颈；真正的约束来自 rolling window 语义、局部可视范围和场景几何
- `5 Hz` 重规划对动态障碍响应更快，但室外大尺度绕路能力仍需继续实测
- 当前 MPPI 基线更偏向“稳定贴路径 + 保留动态避障”，不是纯高速 corridor 配置

### 代表性室内 full-system session（2026-03-31）

- Session：`runtime-data/logs/2026-03-31-20-51-45/`
- 模式 / 版本：`indoor-nav`，`gps-mppi@2c2b8e6`
- 时长：约 `16 分 59 秒`
- 完整 use case：Livox + FAST-LIO2 + PGO + Nav2(MPPI) + 串口底盘控制 + RViz 点击点导航；不启 GNSS 相关链路
- 导航交互证据：`rviz2` 记录 13 次 `Setting goal pose`，`bt_navigator` 记录 13 次 `Begin navigating from current location`
- 控制链证据：`controller_server` 记录 17 次 goal 接收、1251 次 path handoff；`data/serial_twistctl.log` 持续写入
- 性能指标（来自 `system/tegrastats.log` 的 1009 个 1Hz 样本）：
  - RAM：`2.676-3.387 GB`，均值 `3.224 GB`，总内存 `15.289 GB`
  - CPU：8 核平均占用 `57.92%`，单核峰值 `83%`
  - GR3D：平均 `55.79%`，峰值 `97%`
  - `VDD_IN`：平均 `10.27 W`，峰值 `11.87 W`
  - 温度：`tj/cpu` 峰值 `62.312C`，`gpu` 峰值 `59.875C`
- 用途：这是当前用于答辩资源稳定性图和室内无 GPS 全链路验证的代表性 session
- 远端保存：该 session 的 `system/`、`console/`、`data/` 已保存在 runtime-data Hugging Face 数据仓库远端主分支

## 6. GPS 专用配置（`nav2_gps.yaml`）

GPS 目标导航模式不直接改 `nav2_explore.yaml`，而是新建独立的 `nav2_gps.yaml`。

相对 Explore 配置的最小必要差异：

- `general_goal_checker.xy_goal_tolerance = 3.0`
- `general_goal_checker.yaw_goal_tolerance = 0.5`
- `GridBased.tolerance = 2.5`
- `BaseObstacle.scale = 0.02`
- `GoalAlign.scale = 24.0`
- `RotateToGoal.scale = 32.0`

调参原则：
- 低精度 GNSS 环境下放宽 goal tolerance
- 保持现有 DWB / costmap 主结构不动
- 不在 GPS MVP 分支中顺手引入 MPPI、VoxelLayer 等更大变更

## 7. 当前运行注意事项（2026-07）

1. RViz 的 fixed frame 必须设为 `map`。
2. 如果 `map -> odom` 没建立，即使 Livox 和 FAST-LIO2 在跑，RViz 也可能表现为空白或 costmap 不显示。
3. Explore 使用 MPPI 主线 baseline；Corridor 启动时生成临时 Nav2 参数文件来使用 RTK 低速验收 profile。
4. `velocity_smoother.max_velocity[0]` 在 Explore 中为 `1.0`，在 Corridor 中为 `0.45`。
5. Corridor 生成 Nav2 参数时强制 `general_goal_checker.stateful=false`；这样前一个 goal 的“已到点”状态不会残留到后续相距很远的 RTK subgoal。
6. `nav2_gps.yaml` 与 `nav2_travel.yaml` 均独立于 Explore/Corridor profile。
7. FAST-LIO2 发布点云已在 C++ 端按高度窗口 `[-0.33, 0.30]` 过滤（commit `f619fa6`），下游 STVL 收到的是干净数据。
8. Corridor rosbag 默认使用 lean profile，记录 `/rtk/status`、`/fix`、`/heading`、`/fastlio2/lio_odom`、TF、corridor 状态、目标、costmap、`/cmd_vel` 和 `/plan`。只有需要回放原始 `/livox/lidar`、`/livox/imu`、`/fastlio2/body_cloud` 时才设置 `FYP_CORRIDOR_BAG_PROFILE=debug`；全量原始 profile 在验收跑车时可能让 Jetson 上的 Nav2 / FAST-LIO2 饿死。

## 8. 航点系统

- `waypoint_collector` 订阅 RViz 的 `/clicked_point`
- `gps_waypoint_dispatcher` 通过 `FollowWaypoints` 把 GPS 目标交给 Nav2
- `goto_name` 走路网模式
- `goto_latlon` 仅做调试直达模式
