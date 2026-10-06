# Super-LIO / EGO2D active-road research delivery

本交付在 `codex/superlio-ego-active-road` 分支完成，车辆基线为
`Yangbadger222/XJTLU-autonomous-vehicle-rtk` 的
`corridor-authority-stability`，基线 HEAD 为
`e54c6afbcb5a58db22d7c468085a87d658b0b932`。原分支与车端生产工作区未改动。

## 已完成的软件与研究闭环

- 锁定车辆源参数、解析后的 launch 覆盖和保护文件哈希；运行时参数 dump 仍需目标机证据。
- 固定 Super-LIO ROS2 `f89f48dc7aea6cfa262f18e4d03b319e04e0dbd2`，提供可复现补丁、源健康/协方差输出和失效关闭的车体适配器。未把 `world→imu` 冒充 `base_footprint`。
- 固定 JackJu-HIT/Ego-Planner-2D-ROS2 `develop@7f5be6d4cee34871e85aa1f15285cfaf17b23877`，提供可复现补丁，接入实测状态入口、动态可行性、差速/滑移转向曲率与 yaw-rate 限制，并通过 typed `TimedTrajectory2D` 合同传输。
- 新 active-road 入口排除 Nav2/MPPI/SLAM/旧实验旁路，只保留传感器、RTK authority、安全命令、串口与研究桥；replay 模式不启动传感器或执行器。
- 实现 CRS 绑定的 GeoTIFF/MaGRoad 证据接口、局部证据持久化、待观察入口、主动观察候选评分和 evaluator-only 策略对照。
- 明确记录已有 bag：该 15.5 秒 bag 只有旧 `/fastlio2/lio_odom`、RTK、TF 和 costmap，没有 `/livox/lidar` 或 `/livox/imu`，因此不宣称 Super-LIO 回放通过。

## 已执行验证

- 研究运行时与 Super-LIO 适配器纯 Python 测试：`13 passed`。
- launch 与适配器 Python 语法编译通过。
- 研究回放 smoke 输出包含轨迹可行性、策略对照和最终 mock 串口停车字节。
- 上游两个补丁在精确锁定提交上 `git apply --check` 通过；ROS/Humble target build 尚未运行。
- 保护的原始车辆文件未修改；未刷固件、未改标定/串口协议/运动限值/安全权限、未自动开车、未 force-push。

## 仍需现场或目标环境完成的门槛

`RESULTS.json` 已逐项区分软件 PASS、研究合成 PASS 与实车 PENDING：ROS2 Humble/Jetson 编译与运行、真实 LiDAR/IMU bag 回放、IMU→base 外参与协方差/健康等价性、Jetson shadow、物理急停最终串口字节、RTK authority holdover 和 live motion acceptance 均未冒充完成。RTK 失 authority 时仍由既有保护停车；Super-LIO 健康不能绕过该停车条件。

详细证据见仓库中的 `RESULTS.json`、`AUDIT_REPORT.md`、`audit/UPSTREAM_PATCH_VERIFICATION.json`、`audit/VEHICLE_BAG_REPLAY_INPUT.json` 和 `LIVE_ACCEPTANCE_CHECKLIST.md`。
