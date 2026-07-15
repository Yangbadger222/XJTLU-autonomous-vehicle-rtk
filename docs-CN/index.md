# XJTLU 自主导航车辆项目文档索引

> 最后更新: 2026-07-11

## 当前系统摘要

- 当前常用运行入口: `make launch-explore` / `make launch-indoor-nav` / `make launch-corridor`
- **室内无 GPS 点击点导航**: `make launch-indoor-nav`（RViz 发目标，无需 GNSS）
- GPS 融合模式已部署: `make launch-explore-gps`
- GPS 目标导航模式已在 `feature/gps-navigation-v4` 完成软件部署并通过室内 smoke: `make launch-nav-gps`
- **GPS Corridor v2 当前已收口到 MPPI 主线基线**: `make launch-corridor`
  - 控制器从 RotationShim + RPP 切换到 MPPI（commit `9d71823`），获得原生采样避障能力
  - Costmap 高度过滤调优（commit `ce5226f`）：只保留车体高度范围内障碍，消除假障碍
  - 障碍地图范围扩展到 15m（commit `2c2b8e6`），匹配 Livox MID360 量程
  - **2026-04-15 收口**：吸收 IEEE demo 的 MPPI 抗推头基线，当前主线参数为 `vx_max=1.0`、`wz_max=1.2`、`ax_max=1.2`、`PathAlignCritic.offset_from_furthest=6`、`PathFollowCritic.cost_weight=16.0`
  - FAST-LIO2 发布点云前置高度过滤（commit `f619fa6`），C++ 端裁剪后交给下游
  - **GPS live alignment 已部署**（commit `ebc26e2` + `fe3933e` + `e73c2bf`）：
    - Calibration 改为 translation-only，避免旋转翻转
    - 启动时直接吸收 stable GPS offset
    - Runner 使用实时 alignment 重新计算 subgoal，移除 per-waypoint frozen 机制
  - **室内实车验证通过**：代表性 full-stack session `2026-03-31-20-51-45`（`indoor-nav`, `gps-mppi@2c2b8e6`）持续约 16 分 59 秒并记录 1009 个 `tegrastats` 样本；MPPI 成功绕开人体、走廊整圈巡航无漂移，RAM 处于 `2.676-3.387 GB / 15.289 GB`
  - **GPS 户外回归测试已收口**（2026-04-01）：live alignment 机制已部署，基本对齐问题已修复
  - **直线稳定性调优已完成**（2026-04-02）：引入 Savitzky-Golay 路径平滑 + MPPI critic 调优，直线跟踪已达到"完美成功"标准
  - **动态避障恢复与重规划优化**（2026-04-05）：全局重规划提升至 5Hz，Navfn 启用 A* 搜索；BT 恢复机制升级为 5 级渐进式（局部清理 -> 等待 -> 原地旋转 -> 全局清理 -> 后退 1m）。这组 5Hz 重规划/A* 仍保留在当前基线中。
  - **PGO 可视化增强已并入当前基线**：支持按需发布 `/pgo/global_map`，并可通过 `pgo_launch.py rviz_config:=...` 注入自定义 RViz 布局。
  - **2026-05-08 部署修复**：alignment shift 改为 Nav2 goal preemption、global costmap 扩到 `100m x 100m`、corridor/explore 默认恢复 bringup 完整 RViz 布局。
  - 当前状态：**室内导航已验证；GPS corridor 软件链可部署并能进入路线执行；普通 GNSS `/fix` 的物理坐标精度仍是当前主阻塞**
- 当前导航与建图主栈: FAST-LIO2 + PGO + Nav2 (MPPI)
- 运行时数据根目录: `~/XJTLU-autonomous-vehicle/runtime-data`
- 参数统一入口: `src/bringup/config/master_params.yaml`
- 日志统一入口: `scripts/launch_with_logs.sh`

## 核心文档

| 主题 | 文件 |
|------|------|
| 系统结构与数据流 | [architecture.md](architecture.md) |
| 常用命令 | [commands.md](commands.md) |
| 贡献指南（中文） | [`../CONTRIBUTING.md`](../CONTRIBUTING.md) |
| 贡献指南（英文） | [`../CONTRIBUTING-EN.md`](../CONTRIBUTING-EN.md) |
| 已知问题与当前阻塞 | [known_issues.md](known_issues.md) |
| 硬件规格参数 | [hardware_spec.md](hardware_spec.md) |

## 技术深度文档

| 主题 | 文件 |
|------|------|
| FAST-LIO2 工作原理 | [knowledge/fastlio2.md](knowledge/fastlio2.md) |
| PGO + GPS Factor | [knowledge/pgo.md](knowledge/pgo.md) |
| Nav2 调参与运行约束 | [knowledge/nav2_tuning.md](knowledge/nav2_tuning.md) |
| SLAM 建图与室内导航设计开发方案 | [knowledge/indoor_mapping_navigation_design.md](knowledge/indoor_mapping_navigation_design.md) |
| GPS 全局导航与路网规划 | [knowledge/gps_planning.md](knowledge/gps_planning.md) |

## 开发日志

- 在 `devlog/YYYY-MM.md`

## 固件代码

- STM32 RM C Board 下位机完整目录现已并入当前仓库：[`../src/firmware/rm_c_board/`](../src/firmware/rm_c_board/)
- 该目录按仓库内快照保留了源码、工程文件以及同目录原有构建产物，方便直接回看和对照
- 固件目录内说明见 [`../src/firmware/rm_c_board/README.md`](../src/firmware/rm_c_board/README.md)

## 归档说明

- `docs-CN/devlog/legacy/` 与 `docs-EN/devlog/legacy/` 为历史原始文档归档，仅保留参考价值，不按当前结构继续维护。
- 通过 `dependencies.repos` 拉取的上游依赖文档不属于本项目自维护文档范围。

## 维护原则

1. 代码、launch、参数、系统环境、流程有变动时，同步更新对应文档。
2. 当月开发记录统一追加到 `devlog/YYYY-MM.md`。
3. 新问题或状态变化同步到 `known_issues.md`。
4. 命令和流程文档必须反映当前仓库真实可执行状态，而不是历史习惯。
