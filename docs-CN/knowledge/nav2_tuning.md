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

## 4. Explore / Corridor 主模式当前控制器（2026-07-09 更新）

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

Corridor 模式沿用同一 MPPI 结构，但 `system_gps_corridor.launch.py` 会在启动时从 `nav2_corridor_rtk.yaml` 生成临时 Nav2 参数文件，注入 RTK 小步提速、中等原地转头与横摆抑制 profile：

- `controller_frequency: 20.0`（保持与 `model_dt=0.05s` 一致；15Hz 会让 MPPI 配置失败）
- `batch_size: 500`
- `failure_tolerance: 1.5`，让局部障碍或人群导致的短暂 MPPI optimizer failure 有缓冲时间，不在第一帧失败时直接 abort。
- `progress_checker: 0.10m / 15.0s`，允许直角弯原地调车头时短时间没有平移进展。
- `vx_max: 0.85`，`wz_max: 0.70`，`ax_max: 0.85`，`ax_min: -1.2`，`az_max: 1.4`
- `vx_std: 0.20`，`wz_std: 0.15`，`temperature: 0.45`，`regenerate_noises: true`
- `CostCritic.cost_weight: 7.0`，让动态障碍代价比贴线优先级更硬，但不关闭 fail-stop 行为。
- `velocity_smoother.max_velocity: [0.85, 0.0, 0.70]`
- `velocity_smoother.max_accel: [0.85, 0.0, 1.4]`
- `velocity_smoother.max_decel: [-1.2, 0.0, -1.8]`
- `nav2_corridor_rtk.yaml` 将 global costmap 收敛为路线级规划画布：只启用 `inflation_layer`，`track_unknown_space=false`，`robot_radius=0.22`，`inflation_radius=0.30`，让 NavFn 可以持续给 RTK 子目标生成全局路径。
- Corridor 的实时安全边界仍由 local costmap 承担：保留 `stvl_layer`、`frc_layer` 和 `inflation_layer`，继续使用 `robot_radius=0.38625` 与 `inflation_radius=0.43` 处理 Livox 障碍；local rolling window 收敛到 `12m x 12m`，STVL marking `obstacle_range=5.0m`，避免 15m 远处稀疏点云和 30m 大图负载干扰 MPPI。
- `gps_route_runner` 的默认 `segment_length_m` 从 `30.0m` 缩短到 `5.0m`；长 RTK 线段会被拆成更短子目标，降低 rolling costmap 边缘、局部路径和全局规划之间的耦合风险。
- 发布终止 `SUCCEEDED` 前，`gps_route_runner` 会先发布 `STOPPING_BEFORE_EXIT`，并按 `20Hz` 保持 `terminal_stop_hold_s=1.2` 的零 `/cmd_vel`；这样 quiet monitor 不会在下位机收到明显零速度尾巴前杀掉 launch。
- corridor 通过 launch 注入无 `Spin` / `BackUp` 的 NavigateToPose 与 NavigateThroughPoses BT；`nav2_explore.yaml` 与 `nav2_corridor_rtk.yaml` 必须同时保留 `default_nav_to_pose_bt_xml` 和 `default_nav_through_poses_bt_xml` 两个空默认槽位，否则 Nav2 会对 through-poses action 回退到上游 recovery 树
- corridor 由 `rtk_map_odom_corrector` 发布 RTK authoritative `map→odom`；PGO 在 `pgo_corridor_no_gps.yaml` 中关闭 `publish_tf`，只保留点云/优化输出，不再作为 corridor 的全局 TF owner
- 启动阶段若 external `/gps_corridor/enu_to_map` 尚未出现，`rtk_map_odom_corrector` 会复用 bootstrap alignment，并允许在 RTK Fixed 前先发布 `RTK_BOOTSTRAP` 的 `map→odom`；这只用于让 Nav2 lifecycle 获得 `map` frame，路线执行仍由 `gps_route_runner` 的 stable-fix / alignment gate 控制
- RTK corrector 在 RTK/heading/alignment 新鲜且平移跳变仍安全时允许 `YAW_REACQUIRE`：`max_yaw_reacquire_jump_deg=45.0`，但实际 `map→odom` yaw 每周期先受 `max_yaw_step_deg=0.5` 限制，再按 `max_base_yaw_step_m=0.12` 基于当前 `odom→base_link` 半径动态压小；这样车离 odom 原点几十米时，yaw release 不会被杠杆效应放大成 `map→base_link` 米级平移跳变

原因：2026-07-06 RTK corridor 实车日志显示 RTK 本身保持 `RTK Fixed q=4`，但轨迹出现右偏，且 `/fastlio2/lio_odom` 有约 `1.18m/0.19s` 跳变。低速 profile 先用于降低 RTK 验收时的右偏、过大角速度和 FAST-LIO2 退化风险。2026-07-06 20:06 的现场日志证明 `controller_frequency=15Hz` 会触发 MPPI `Controller period more then model dt` 配置失败，因此 corridor 保持 `20Hz` 并通过速度/采样量降负载。2026-07-06 22:33 的实车日志又证明缺少 `default_nav_through_poses_bt_xml` 槽位时，`bt_navigator` 会继续加载 Nav2 默认 `navigate_through_poses_w_replanning_and_recovery.xml`，在 corridor 只加载 `wait` 行为后因缺少 `spin` action server 而启动失败。随后现场判断确认：LIO 只要约 1m 漂移，Nav2 的 `map→base_link` 就会偏离路线并停止；因此 corridor 需要 RTK authoritative `map→odom` 把全局位姿压回 RTK，而不是继续让 PGO/LIO 漂移主导 Nav2。2026-07-07 成功跑通的 `2026-07-07-08-48-14` bag 显示，运动段 `vx` 中位数 `0.428m/s` 且 p95/max 已达旧的 `0.45m/s` 上限，因此第一轮小步提到 `0.55m/s` 是合理的；但 `|wz|` p95 为 `0.537rad/s`、角速度正负切换约 `0.53次/s`，并出现 `TARGET_YAW_JUMP` 窗口，所以角速度限制保持保守。随后成功的 `2026-07-07-10-39-15` bag 显示 RTK 全程 `q=4`，运动段 `vx` 中位数/p95/max 均顶到 `0.55m/s`，且 yaw 命令已明显平滑（`|wz|` p95 `0.094rad/s`、max `0.137rad/s`）。该 bag 也显示 Nav2 宣布 `SUCCEEDED` 时终点速度仍在衰减（成功前 33ms 为 `0.073m/s`，成功后 17ms 为 `0.028m/s`），quiet launch 退出前零速度尾巴太短。因此上一轮把 corridor 线速度上限提到 `0.65m/s`，小幅增强线加/减速度，并新增终点零速度保持。2026-07-08 的失败日志指向 global planner 对长 RTK 子目标和全局滚动障碍层过敏，容易在未知/障碍代价图边缘判定失败；因此 corridor 将全局层拆成轻量路线规划，动态避障留给 local costmap，并把 RTK 子目标默认缩短到 `5m`。2026-07-09 11:14 与 11:16 的 route `2` 失败复盘显示，第一个子目标到达后第二段近似 90 度转向，Nav2 在 10s 内只发布平均 `vcx≈0.001m/s`、`wc≈0.13rad/s` 的慢速原地转向命令，串口实际发出的也是同样的 `vcx,wc`，LIO 位移仅约 `0.10-0.14m`，随后触发 `Failed to make progress`。底盘硬件虽是 90 度全向轮，但当前上位机/STM32 协议仍只有 `vcx,wc`，没有 `vcy`；本轮不引入横移，而是提高 MPPI 对原地调车头的采样和角速度/角加速度限幅，并把 progress checker 放宽到 `0.10m / 15s`，同时把直线速度小步提到 `0.75m/s`。2026-07-09 17:55/18:00/18:02 的 corridor bag 进一步显示：有人阻挡的一次属于 local costmap 正确让车辆停住并尝试绕行；另外两次失败集中在 MPPI 对短暂无可行控制过早 abort，以及 `rtk_map_odom_corrector` 的跳变判断被 `map→odom` target 杠杆放大影响。离线回放 6 个 corridor bag 后确认，旧 `map→odom` target gate 会在部分 bag 里出现数百次 reject，而 raw RTK `map_base` 最大单步仅约 `0.20-0.36m`；因此本轮保留避障停车能力，但把 local costmap 收敛为近场 `12m`/`5m`、提高 CostCritic 权重，并把 target-jump gating 改为 raw RTK `map_base` 到上一帧可信 raw RTK `map_base`。历史 DWB 配置已不再作为 Explore/Corridor 主线使用；`nav2_gps.yaml` 和 `nav2_travel.yaml` 保持独立配置。

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

## 6. GPS 路网选点配置（RTK nav-gps）

旧 `nav2_gps.yaml` 仍保留在仓库中，但当前实车 `nav-gps` 入口不再使用它作为主 profile。`system_nav_gps.launch.py` 会复用 corridor RTK profile：

- 从 `nav2_corridor_rtk.yaml` 生成临时 Nav2 参数文件。
- 使用 MPPI，保持 `controller_frequency=20Hz` 与 `model_dt=0.05s` 匹配；`batch_size=350`、`time_steps=40`，保留2秒预测视野并比旧 `500x48` 降低约42%轨迹仿真量。其余保持 `failure_tolerance=1.5s`、`vx_max=0.85`、`wz_max=0.70`、`temperature=0.45`、`regenerate_noises=true`。
- local costmap 使用 `/fastlio2/body_cloud_nav2_obstacles`，保留 `[-0.20, 1.20]m` 级别的高窗障碍点云。
- global costmap 继续保持 route-planning-only 语义，避免实时点云/unknown space 阻断路网目标。
- `general_goal_checker.stateful=false`，避免一个目的地的到点状态残留到下一个 route graph 目标。
- goal manager 将当前位置和终点投影到最近 graph edge，插入虚拟端点后执行欧氏启发式 A*；不再调用 route server 的 Dijkstra，也不依赖少数 anchor。
- QGIS 道路 Polygon 编译为 local/global costmap 的 KeepoutFilter；MPPI 继续使用高窗点云在道路内部避障。
- `nav-gps` 与 corridor 共用 guarded command 拓扑；authority/stop heartbeat 任一失效都输出零速度，恢复后从当前 pose 重新 A*。

定位语义：
- PGO 关闭 `publish_tf` 和 GPS 因子，不再抢 `map→odom`。
- `rtk_map_odom_corrector` 使用 scene fixed origin 和 ENU→map identity alignment，成为唯一 `map→odom` owner。
- 默认关闭旧 `gps_anchor_localizer`；goal manager 支持命名、地图 pose 和经纬度终点，实际起跑只依赖统一 authority、当前 TF 和 FollowPath。旧 `/gnss`/anchor 实验可显式恢复该节点。
- goal manager 不把 `NAV_READY`/anchor 作为硬门槛；统一的 `motion_allowed` heartbeat、当前 TF 与稳定 authority 才是实际起跑条件，因此未来可由 FGO 接管而无需改规划层。

## 7. 当前运行注意事项（2026-07）

1. RViz 的 fixed frame 必须设为 `map`。
2. 如果 `map -> odom` 没建立，即使 Livox 和 FAST-LIO2 在跑，RViz 也可能表现为空白或 costmap 不显示。
3. Explore 使用 MPPI 主线 baseline；Corridor 启动时从 `nav2_corridor_rtk.yaml` 生成临时 Nav2 参数文件来使用 RTK 小步提速、中等原地转头、近场 local costmap、全局/局部代价地图分离与横摆抑制 profile。
4. `velocity_smoother.max_velocity[0]` 在 Explore 中为 `1.0`，在 Corridor 中为 `0.85`；Corridor 角速度上限为 `0.70rad/s`，但仍只使用 `vcx,wc` 控制链路，不发布横向 `vcy`。
5. Corridor 生成 Nav2 参数时强制 `general_goal_checker.stateful=false`；这样前一个 goal 的“已到点”状态不会残留到后续相距很远的 RTK subgoal。
6. `nav2_gps.yaml` 保留为旧 GPS MVP profile；当前 RTK `nav-gps` 实车入口复用 corridor RTK MPPI profile，`nav2_travel.yaml` 仍独立于 Explore/Corridor/nav-gps。
7. FAST-LIO2 发布点云已在 C++ 端按高度窗口 `[-0.33, 0.30]` 过滤（commit `f619fa6`），下游 STVL 收到的是干净数据。
8. Corridor 和 `nav-gps` 实车入口都默认关闭 RTK FGO shadow，以给 FAST-LIO2、costmap 和 MPPI 留出 CPU；分别只有设置 `FYP_CORRIDOR_ENABLE_FGO_SHADOW=true` 或 `FYP_NAV_GPS_ENABLE_FGO_SHADOW=true` 时启用。Shadow 始终设置 `publish_tf=false`、`nav2_use_fgo=false`，不接管 `map→odom` 或 Nav2。`nav-gps` lean bag 保留 RTK、FAST-LIO2 odom、Livox IMU、底盘 `/odom_CBoar`、TF、状态、目标、三层速度、local costmap 和 `/plan`；占本次 bag `77.5%` 的 global costmap、旧 anchor 状态与原始点云只进入 debug profile，避免录包线程挤占控制循环。
9. Corridor 与 `nav-gps` 的 RTK MPPI 固定使用 `open_loop=false`，用实际 odometry 速度初始化每轮预测；底盘丢帧或转向执行不足时，不再把上一条命令误当成已执行运动。

## 8. 航点系统

- `waypoint_collector` 订阅 RViz 的 `/clicked_point`
- `gps_waypoint_dispatcher` 将整条 A* 路线作为一次 `FollowPath` 交给 Nav2，中间图节点不会停车
- `goto_name`、`goto_latlon` 和 `/goal_pose` 都先吸附到路网再规划
## 2026-07-10 Corridor Authority 收敛链

Corridor 现已拆分 local motion、global correction 与 command authority。`rtk_map_odom_corrector` 使用 2 秒 `/fastlio2/lio_odom` 时间戳历史对齐 RTK 观测，要求 5 个一致的 Fixed 样本，并在 `map→base_footprint` 空间以不超过 `0.20 m/s`、`2 deg/s` 慢释放。低于 backlog 阈值的 NORMAL correction 即使连续受速率限制也保持运动权限，直至收敛；中等 backlog（`0.50-2.0 m` 或 `5-20 deg`）才要求连续停车 1 秒后慢释放，更大 backlog 锁存 `FAULT_HOLD`。这样避免合法的 0.49 m 或 4.9 deg correction 因固定样本数超时而反复触发停车。

命令链现为 `controller_server -> /cmd_vel_controller -> velocity_smoother -> /cmd_vel_nav -> corridor_cmd_vel_guard -> /cmd_vel`。guard 保留最高 `0.85 m/s` 直线速度，但按 `min(0.85, 0.25/max(|w|, 0.05))` 限制转弯线速度；命令、authority 或 stop override 心跳过期时发布零速度。这些参数仅用于 corridor；Explore 未显式传入 `guarded_cmd_vel=true` 时保持原拓扑。

原因：2026-07-10 bags 分别暴露了 51.75 度 heading outlier、旧 authority 的 8-13 m target gap，以及 20 秒底盘/LIO no-progress。把三者都当成 `map→base` odom divergence 会导致全局坐标快速移动，或把错误归因到 local LIO。
