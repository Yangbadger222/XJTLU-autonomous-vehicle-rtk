# Super-LIO / EGO2D active-road research delivery

本交付在 `codex/superlio-ego-active-road` 分支完成，车辆基线为
`Yangbadger222/XJTLU-autonomous-vehicle-rtk` 的
`corridor-authority-stability`，基线 HEAD 为
`e54c6afbcb5a58db22d7c468085a87d658b0b932`。原分支与车端生产工作区未改动。

## 已完成的软件与研究闭环

- 锁定车辆源参数、解析后的 launch 覆盖和保护文件哈希；运行时参数 dump 仍需目标机证据。
- 固定 Super-LIO ROS2 `f89f48dc7aea6cfa262f18e4d03b319e04e0dbd2`，提供可复现补丁、源健康/协方差输出和失效关闭的车体适配器。未把 `world→imu` 冒充 `base_footprint`。
- 固定 JackJu-HIT/Ego-Planner-2D-ROS2 `develop@7f5be6d4cee34871e85aa1f15285cfaf17b23877`。两个顺序补丁现在将真实 odom、odom-frame road reference、OccupancyGrid（unknown=occupied）接入上游 `PlannerInterface`，并输出 typed `TimedTrajectory2D`；`nav_msgs/Path` 只用于可视化。
- 轨迹检查补充了动态导数、实际切向速度、body 横向速度、曲率/yaw-rate、footprint 和地图版本拒绝；不可行轨迹不 clip。
- 新增保守的局部障碍 GridMap 投影合同：只接受 odom 帧、显式高度窗和地图版本，未知栅格保持阻塞，不把无回波当 free；目标机传感器/TF wiring 仍待现场接入。
- 新入口排除 Nav2/MPPI/SLAM/旧实验旁路，只保留传感器、RTK authority、安全命令、研究桥。replay 不启动传感器、authority、规划器或串口；shadow/live 才启动规划链。物理串口还必须显式 `execution_mode:=live enable_serial:=true`。
- 实现 CRS 绑定的 GeoTIFF/MaGRoad 证据接口、像素+深度→相机→车体→odom 几何合同、局部证据持久化、待观察入口、主动观察候选评分和 evaluator-only 策略对照。
- 已把 Super-LIO/FAST-LIO2 的时间、单位、同步、云过滤和健康差异固化到 `audit/vehicle_baseline/LIO_FIELD_MAPPING.json`；未解析项保持 fail-closed。
- 已在隔离 ARM64 ROS 2 Humble 容器中真实回放一个本地 135.1 秒 bag，收到旧 `/fastlio2/lio_odom`、costmap 和 `/cmd_vel`；该 bag 没有 `/livox/lidar` 或 `/livox/imu`，所以回放 PASS 仅限旧 topic，不宣称 Super-LIO 回放通过。

## 已执行验证

- 研究运行时、适配器、入口合同和 LIO 字段映射测试：`47 passed`；launch 合同还验证车辆配置不会激活上游 demo 的速度/加速度/jerk/地图分辨率/膨胀默认值，并检查 active-road bringup 声明了所有研究运行时、安全边界和传感器依赖。`research_safety_bridge --mode replay` 入口也实际执行并写出回放 JSON；该模拟闭环先用合成 odom 位姿经过 timed-trajectory tracker，再注入 RTK authority loss。TimedTrajectory 安全边界现在消费 `/lio/odom_vehicle`、`/lio/vehicle_health`、odom-frame `/research/local_obstacle_grid` 和 `/research/map_version`，按时间插值并用实测位姿做跟踪反馈后才进入原 command guard，不再直接使用轨迹第一点；vehicle health 超过 0.50 秒未更新、footprint/地图版本缺失或未知栅格合同时即按 UNKNOWN 停车。最终 mock 串口故障矩阵覆盖 authority/health/TF/map/trajectory/stop_override/manual-stop/non-finite command，均输出 `vcx=0,wc=0\\n`；轨迹验证器另有地图/帧不匹配、过期、时间倒退、位姿突跳、曲率/yaw-rate 不一致，以及按地图分辨率连续 footprint 扫掠的反例；主动观察评分也拒绝负/非有限代价输入，证据存储拒绝坏 schema、重复 UUID 和非法几何/不确定度/深度区间，MaGRoad 先验拒绝缺失 CRS 和非有限几何。适配器现在对非单位四元数、非有限协方差和未实现协方差旋转的非恒等外参保持停车。
- `research_runtime` 已构建 wheel 并检查安装内容，GridMap 投影器、安全桥和轨迹检查器均随包发布。
- 在隔离 ARM64 ROS 2 Humble 容器中，固定提交的原始 EGO `motion_plan` 完成编译并启动冒烟；顺序补丁后的 EGO 车辆边界完成编译；Super-LIO C++ 核心和 ROS 接口完成编译。EGO timed wrapper 的 yaw-rate/curvature 现在使用 `cross(v,a)/|v|^2` 与 `yaw-rate/|v|` 的量纲一致公式，并拒绝标量速度与切向速度不一致的样本；补丁 provenance hash 已刷新。Super-LIO 构建使用仅含 `livox_ros_driver2` 消息的编译合同，真实 Livox SDK/驱动、传感器运行和 Jetson 运行仍为 PENDING。
- launch 与适配器 Python 语法编译通过。
- 两个 EGO 补丁在精确提交的全新 checkout 上顺序 `git apply --check` 通过；原始与 patched EGO 的隔离 Humble 编译日志见 `audit/container/`。
- 研究回放 smoke 输出包含轨迹可行性、策略对照和最终 mock 串口停车字节；真实 bag 完整播放也在隔离容器中退出码 0。
- 保护的原始车辆文件未修改；未刷固件、未改标定/串口协议/运动限值/安全权限、未自动开车、未 force-push。

## 仍需现场或目标环境完成的门槛

`RESULTS.json` 已逐项区分软件 PASS、研究合成 PASS 与实车 PENDING：Super-LIO 的核心/消息合同编译已通过，但合并的 build/runtime 项保持 PENDING；真实 LiDAR/IMU bag 回放、IMU→base 外参与协方差/健康等价性、运行时有效参数 dump、Jetson shadow、物理急停最终串口字节、RTK authority holdover 和 live motion acceptance 均未冒充完成。已完成的 bag 回放只覆盖旧 odom/costmap/cmd topic。RTK 失 authority 时仍由既有保护停车；Super-LIO 健康不能绕过该停车条件。

详细证据见仓库中的 `RESULTS.json`、`AUDIT_REPORT.md`、`audit/UPSTREAM_PATCH_VERIFICATION.json`、`audit/VEHICLE_BAG_REPLAY_INPUT.json` 和 `LIVE_ACCEPTANCE_CHECKLIST.md`。
