# PGO 位姿图优化说明

## 1. 当前工程实现（2026-03）

当前主用 PGO 实现在：

- 目录: `src/perception/pgo_gps_fusion/`
- colcon 包名: `pgo`
- 启动入口: `ros2 launch pgo pgo_launch.py`

这个实现已经不是纯回环 PGO，而是：
- FAST-LIO2 局部里程计
- PGO 回环图优化
- 可选 GPS factor 绝对位置约束

## 2. 当前输入与输出

### 输入

- 点云: `/fastlio2/body_cloud`
- 里程计: `/fastlio2/lio_odom`
- GNSS: `/gnss`（当 `gps.enable=true` 时）

### 输出

- TF: `map -> odom`
- 里程计: `/pgo/optimized_odom`
- 可视化: `/pgo/loop_markers`

## 3. 节点职责

PGO 当前承担 5 件事：

1. 从 FAST-LIO2 收集关键帧
2. 做回环搜索与 ICP 确认
3. 用 GTSAM 做位姿图优化
4. 在启用时周期性加入 GPS factor
5. 发布 `map -> odom` 和 `/pgo/optimized_odom`

## 4. 当前主配置参数

这些值来自当前主线 `master_params.yaml`：

- `key_pose_delta_deg: 5.0`
- `key_pose_delta_trans: 0.1`
- `loop_search_radius: 1.0`
- `loop_time_tresh: 60.0`
- `loop_score_tresh: 0.15`
- `loop_submap_half_range: 5`
- `submap_resolution: 0.1`
- `min_loop_detect_duration: 5.0`

## 5. GPS Factor 与固定 ENU 原点

当 `gps.enable=true` 时，PGO 会使用以下 GNSS 相关参数：

- `gps.topic: /fix`（2026-03-22 corridor v2 已从 `/gnss` 改为 `/fix`）
- `gps.noise_xy: 2.5`
- `gps.noise_z: 5.0`
- `gps.factor_interval: 10`
- `gps.quality_hdop_max: 3.0`
- `gps.quality_sat_min: 6`
- `gps.drift_threshold: 2.0`

2026-03-20 的新增点是：
- `gps.origin_mode: auto | fixed`
- `gps.origin_lat`
- `gps.origin_lon`
- `gps.origin_alt`
- `gps.topic` 在 `nav-gps` 模式下可由 `gps_anchor_localizer` 持续发布 scene-calibrated `/gnss`

行为约束：
- `auto`: 保持历史兼容，用首条有效 GPS 初始化 LocalCartesian
- `fixed`: 启动时直接使用配置中的固定 ENU 原点
- 在 `fixed` 模式下，首条 GPS 到来后不得覆写 origin

这让 PGO 的 `map` 坐标系能和 `gps_anchor_localizer`、goal manager、scene route graph 使用同一地理参考。

## 6. TF 关系

```text
map -> odom -> base_link
```

- FAST-LIO2 提供 `odom -> base_link`
- PGO 提供 `map -> odom`
- 组合后得到全局位姿

如果 PGO 没有进入正常关键帧和优化流程，`map -> odom` 就可能消失，RViz 在 `map` fixed frame 下会表现为点云和 costmap 看起来空白。

## 7. 2026-03-18 启动回归修复

### 现象

- `pgo_node` 启动后持续刷 `Received out of order message`
- 没有稳定关键帧
- 没有稳定 `map -> odom`
- `/pgo/optimized_odom` 不稳定或不发布

### 根因

同步状态中的 `last_message_time` 未初始化，第一对同步消息会被错误判定为 out-of-order。

### 修复

将 `last_message_time` 初始化为 `0.0`，保证第一对匹配消息被正常接受。

### 结果

该修复已经合并到 `main`，当前主线的 PGO 启动同步状态是稳定初始化版本。

## 8. 接口速查

| 接口 | 类型 | 说明 |
|------|------|------|
| `/fastlio2/body_cloud` | `sensor_msgs/PointCloud2` | PGO 关键帧点云输入 |
| `/fastlio2/lio_odom` | `nav_msgs/Odometry` | PGO 位姿输入 |
| `/gnss` | `sensor_msgs/NavSatFix` | GPS factor 输入 |
| `/pgo/optimized_odom` | `nav_msgs/Odometry` | 优化后位姿输出 |
| `map -> odom` | TF | 全局校正偏移 |
| `/pgo/loop_markers` | Marker | 回环可视化 |

## 9. 2026-03-20 实车 `nav-gps` 阻塞结论

首轮真实 scene bundle 室外测试已经确认，当前 `pgo` 还有一个更高优先级的稳定性问题。

### 现象

- `gps_anchor_localizer` 可以正常进入：
  - `NO_FIX -> UNSTABLE_FIX -> GNSS_READY -> NAV_READY`
- goal manager 可以正常进入：
  - `GOAL_REQUESTED`
  - `COMPUTING_ROUTE`
  - `FOLLOWING_ROUTE`
- 车辆会短暂动作，但很快停止
- goal manager 最终状态：
  - `FAILED; follow_path_status=6`

### 直接证据

- `controller_server` 日志：
  - `Controller patience exceeded`
  - `Transform data too old when converting from odom to map`
- launch 日志：
  - `pgo_node ... process has died ... exit code -11`
- 现场 TF 结果：
  - 只剩 `odom -> base_link`
  - `map -> odom` 消失

### 工程结论

- 当前失败不是 route graph、destination 编号菜单或 `NAV_READY` gating 的问题
- 真正失败点在 `pgo_node` 段错误
- 一旦 `pgo` 退出，Nav2 控制器就无法继续把机器人位姿从 `odom` 转到 `map`
- 结果就是 `follow_path` 被 abort

### 已做但未完全收口的修复

- 已修复 `syncCB()` 中一处明确的并发 bug：
  - 匿名 `lock_guard` 改为具名锁
- `pgo` 单包已重编
- 但再次实车复测后，`pgo_node` 仍然继续 `exit code -11`

### 下一轮排查入口

- `SimplePGO::addKeyPose()`
- `SimplePGO::smoothAndUpdate()`
- GPS factor 加入路径
- `map -> odom` 广播前的 offset 更新链

## 10. Corridor v2 ENU→map 对齐机制演变（2026-03-22~24）

### 10.1 设计背景

- GPS 给出的是 WGS84 经纬度
- PGO 用固定 ENU 原点将其转为 ENU (x=东, y=北)
- 但 FAST-LIO2 的 `map` 坐标系 yaw 是随机的（取决于启动时 IMU 姿态）
- 因此需要估计 ENU→map 的旋转角 θ 和平移 t

### 10.2 架构演变

**v2 初版（PGO 侧 ENU→map 估计）**:
- 最初计划让 PGO 在运行中累积 (enu_xy, map_xy) 配对，发布到 `/gps_corridor/enu_to_map`
- 实车发现：PGO 接管后 `ENU→map` 仍继续漂移，runner 重算 subgoal 导致回头/recovery
- **该方案已淘汰**

**v2 最终版（独立 Global Aligner）**:
- 新增独立 `gps_global_aligner_node.py`（commit `e51a46a`），与 PGO 解耦
- Aligner 自行采集 GPS→ENU 与 map→base_link 配对，在线估计平滑 ENU→map 变换
- 输出到 `/gps_corridor/enu_to_map`（同一 topic，但来源从 PGO 切换为独立 aligner）
- 对估算结果做限速与平滑，防止跳变
- Corridor 模式下 PGO 的 `gps.enable` 被关闭，PGO 仅做 loop closure

### 10.3 当前 PGO 在 Corridor 模式中的角色

- **保留**: 图优化 / 回环检测 / 发布 `map → odom` / `optimized_odom`
- **关闭**: GPS factor（`gps.enable: false`），不再直接参与 corridor 对齐
- **独立 aligner 接管**: GPS 全局纠��通过 `gps_global_aligner_node` 完成

### 10.4 Bootstrap 机制

Runner 在启动时用固定 yaw bootstrap 立即开始导航：
- 从 TF 读取 `map->base_link` 的 yaw0
- 用 route YAML 的 `launch_yaw_deg` 计算 `θ_bootstrap = yaw0 - radians(launch_yaw_deg)`
- 构造初始 ENU→map 变换
- 不等 aligner 收敛，立即发出第一个目标

这解决了 v2 计划初期的"静止启动死锁"问题。
## 11. Authority 收敛后的 Corridor TF 归属

Corridor 模式下 PGO 必须同时关闭 GPS factor 与 `publish_tf`。生产链唯一的 `map→odom` broadcaster 是 `rtk_map_odom_corrector`；FAST-LIO2 发布 `odom→base_footprint`，URDF 提供 `base_footprint→base_link`。corrector 使用带时间戳的 LIO history，不再用 latest tf2 lookup 代替历史对齐。冻结 correction 时仍以新时间戳保持 TF 可用，但 `/localization_authority/motion_allowed=false` 会阻止 Nav2 命令到达底盘。
