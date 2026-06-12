# FRC 双锚风险记忆

## 定位

FRC（Failure/Risk Context）栈用于在 Explore/Nav2 主线旁边记录、回放并可选注入残余风险。默认运行模式是 `off`，实验时先用 `shadow` 只发布风险场，确认风险图与基线行为稳定后再用 `full` 调用 `/frc/enable` 打开 costmap 注入。

## 运行模式

- `off`：不启动 FRC 在线节点。
- `shadow`：启动 `frc_health_aggregator`、`frc_event_marker`、`frc_risk_pipeline`、`frc_memory_manager`，发布 `/frc/risk_grid`，但 `frc_layer.enabled=false`，Nav2 costmap 行为应与基线一致。
- `full`：节点集与 `shadow` 完全相同，延时调用 `/frc/enable`，让 `frc_costmap_layer` 读取 `/frc/risk_grid` 并只增不减地提高局部 costmap cost。

启动示例：

```bash
FRC_MODE=shadow make launch-explore
FRC_MODE=full make launch-explore
FRC_MODE=shadow FRC_EXTRA_PARAMS=/tmp/frc_overrides.yaml make launch-explore
```

`explore-gps` 也支持同样的 `FRC_MODE` / `FRC_EXTRA_PARAMS` 透传。`trial_runner` 不随 FRC stack 自动启动，正式实验时手动运行并显式指定路线文件。

## 主要话题

- `/fastlio2/degeneracy`：FAST-LIO2 发布的 `Float32MultiArray[min_eig, cond, regularized]`，供健康状态和离线归因使用。
- `/pgo/keyframes`：PGO 关键帧数组，供锚挂载与回环后重算 map 位姿。
- `/pgo/correction_status`：PGO 修正窗口，供训练过滤和健康状态使用。
- `/chassis/status`：STM32 上行控制模式和 PS2 按键，旧 16 字段固件下字段默认为 0。
- `/frc/health`：FRC 健康状态聚合。
- `/frc/event_marker`：在线自动或人工产生的失败/风险事件。
- `/frc/risk_grid`：`odom` 系风险场，供 `frc_layer` 消费。
- `/frc/anchor_states`：memory manager 发布的锚状态。

## Costmap 注入规则

`nav2_explore.yaml` 的 local costmap 插件顺序为：

```yaml
plugins: ["stvl_layer", "denoise_layer", "frc_layer", "inflation_layer"]
```

`frc_layer` 默认 `enabled: false`。这是有意的 tuned YAML 改动：单一配置同时覆盖基线、shadow、full，shadow 期可以录到与 full 同构的 bag，同时 disabled 时 costmap 应逐字节等价于未注入。闭环只通过 `/frc/enable` 或动态参数打开。

安全不变量：

- 不触碰 `LETHAL_OBSTACLE` 和 `NO_INFORMATION`。
- 只增不减：输出 cost 不低于原始 cost。
- 有 `tau` 置信度门和 `max_cost` 上限。
- `/frc/risk_grid` 超过 watchdog 后自动旁路，回到几何基线。

## 参数与覆盖

FRC 参数集中在 `src/bringup/config/master_params.yaml`：

- `/frc_health_aggregator`
- `/frc_event_marker`
- `/frc_risk_pipeline`
- `/frc_memory_manager`
- `/frc_trial_runner`
- `/pgo.pgo_node` 下的 `"frc.*"` PGO 导出参数

实验期不要直接改 tuned YAML；使用 `FRC_EXTRA_PARAMS=/path/to/overrides.yaml` 追加覆盖。路径类参数按 Jetson 默认目录写入，非默认用户名或工作区路径时也用 overrides 覆盖。

## 数据与录包

初始化运行时目录：

```bash
bash scripts/init_runtime_data.sh
```

该脚本会创建：

```text
runtime-data/frc/{models,routes,trials,events,anchor_log}
```

FRC 录包：

```bash
PROFILE=frc bash scripts/data_collection/record_bag.sh
```

`PROFILE=frc` 会在默认传感器/定位 topic 基础上追加 FRC、costmap、PGO keyframes、退化度量和 Nav2 状态 topic，并启用 zstd file compression。
如果 `FYP_LOG_SESSION_DIR` 已由 `launch_with_logs.sh` 设置，bag 会默认写到同一 session 的 `data/rosbag2`；否则写到 `runtime-data/bags/run_*`。
