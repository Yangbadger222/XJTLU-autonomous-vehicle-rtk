# RTK FGO 紧耦合室外导航设计

## 1. 状态与范围

本文是设计说明和早期实现记录。当前已经有 `rtk_fgo_localizer` 的最小 `ament_cmake` 包骨架、RTK GGA quality 解析与 gate 决策核心库、室内外/RTK 恢复状态机、校正平滑器、最小 GTSAM graph core、ROS shadow node、实验 launch/Make target，以及对应 gtest；但还没有 Nav2 remap。

计划中的系统必须作为新的实验模式引入，不能替换或暗中改变现有 `corridor`、`explore-gps`、`nav-gps` 链路。

第一版目标边界：

- 新包：`src/perception/rtk_fgo_localizer/`（已建立最小骨架）
- 计划 launch 文件：`src/bringup/launch/system_tightly_coupled.launch.py`
- 计划参数文件：`src/bringup/config/rtk_fgo.yaml`
- 计划运行命令：`make launch-tightly-coupled`
- 默认行为：先旁路输出定位和诊断；在实验模式显式开启前，不接管现有 `map -> odom` TF

当前已实现范围覆盖：

- `rtk_quality.hpp/.cpp`: 解析 raw GGA 中的 quality、satellite count、HDOP，并按 RTK Fixed/Float、innovation 和 heading residual 做第一层 gate。
- `test_rtk_quality.cpp`: 覆盖 Fixed strong candidate、Float weak candidate 和 Fixed 大 innovation 拒绝。
- `state_machine.hpp/.cpp`: 实现 `LOCAL_ONLY`、`RTK_CANDIDATE`、`RTK_RECOVERY`、`RTK_LOCKED`、`RTK_DEGRADED`、`FAULT_HOLD` 的基础转换规则；shadow commit 被拒绝时保持可恢复降级，真正图残差异常才进入 `FAULT_HOLD`。
- `correction_smoother.hpp/.cpp`: 对平移与 yaw 校正做单步限幅，避免可信 RTK 恢复时一次性跳变输出。
- `frame_anchor.hpp/.cpp`: 把原始 RTK fix 显式锚定到 FGO `map` 坐标系。连续通过质量检查的 Fixed 样本会初始化 `GeographicLib::LocalCartesian` 原点和 `ENU -> map` 的 yaw/translation 变换；普通 RTK innovation gate 只在 anchor 建立后才开始。
- `fgo_graph.hpp/.cpp`: 实现 GTSAM graph API，支持初始状态、FAST-LIO relative pose、latest-transition wheel planar 因子、RTK position shadow commit/reject、RTK heading yaw factor、可选 IMU 预积分，以及有界窗口重建诊断。
- `heading_conventions.hpp/.cpp`: 将当前 UM982 `/heading` 中按罗盘 heading 编码的 quaternion yaw 转换为 FGO/GTSAM 使用的 ENU yaw；若后续驱动改为标准 ROS ENU yaw，可通过参数关闭该转换。
- `heading_stability.hpp/.cpp`: 在 frame-anchor bootstrap 阶段做支持 wraparound 的 RTK heading spread 检查，避免瞬时双天线 heading 跳变初始化 `ENU -> map` yaw。
- `diagnostic_snapshot.hpp/.cpp`: 通过 `/rtk_fgo/factor_diagnostics` 发布 RTK quality、收星数、HDOP、位置/航向 innovation、fix 隐含速度和各输入 age，方便 rosbag 回放排查。
- `yaw_factor.hpp/.cpp`: 实现 yaw-only Pose3 因子，供双天线 RTK heading 作为绝对 yaw 候选约束。
- IMU 预积分在 `factors.imu_enabled=true` 时使用保守的 GTSAM `ImuFactor` + bias `BetweenFactor` 路径。YAML 默认仍保持 `false`，直到 Jetson 回放和实车验证确认噪声参数。
- `topic_buffers.hpp/.cpp`: 提供 ROS-free timestamped sample buffer，用于按时间查找传感器样本。
- `rtk_fgo_node.cpp`: 实现 ROS shadow node，订阅 FAST-LIO odom、IMU、wheel odom、`/fix`、`/heading`、`/rtk/status`、`/rtk/nmea_sentence`，发布 `/rtk_fgo/odom`、`/rtk_fgo/path`、`/rtk_fgo/status`、`/rtk_fgo/rtk_gate`、`/rtk_fgo/correction_status`、`/rtk_fgo/factor_diagnostics`。
- `publish_tf` 默认必须保持 `false`。即使手动开启，节点也只能广播实验 `map -> odom_fgo`，不能广播生产 `map -> odom`。
- 合并到 `frc` 后的 TF 布局中，FAST-LIO2 发布 `odom -> base_footprint`，URDF 发布 `base_footprint -> base_link`；因此 `rtk_fgo.yaml` 将 `frames.base_link` 设为 `base_footprint`，用于 `/rtk_fgo/odom.child_frame_id`。这样未来可选的 FGO TF 实验不会破坏单父 TF 树；heading 语义仍表示车辆前向轴，因为 `base_footprint` 和 `base_link` 的 yaw 相同。
- `scripts/evaluate_rtk_fgo_bag.py`: 将录制的 `/rtk_fgo/status` 和 `/rtk_fgo/correction_status` 汇总成 JSON 指标，用于回放判断接受率、校正量和拒绝原因。
- `system_tightly_coupled.launch.py` + `make launch-tightly-coupled`: 启动 Explore 基线、UM982 RTK 和 `rtk_fgo_node`，并录制源传感器 topic、`/rtk_fgo/*` 和两路 Livox 点云。实验 TF/Nav2 只能通过显式 launch 参数开启，开启时节点会打印运行时告警。
- 当前实现不 remap Nav2，不改变任何现有生产导航模式。

## 2. 现有系统边界

当前系统的定位职责是分开的：

- FAST-LIO2 发布高频局部里程计。合并后的 `frc` 布局中是 `odom -> base_footprint`；旧 bag 和 URDF 前的说明可能仍写作 `odom -> base_link`。
- PGO 发布全局修正：`map -> odom`
- Corridor 模式关闭 PGO GPS 因子，使用 `gps_global_aligner_node` 发布平滑后的 `ENU->map` 路线投影变换
- UM982 RTK 已经是 GPS 模式的 GNSS 来源，发布 `/fix`、`/heading`、`/rtk/status`、`/rtk/nmea_sentence`

新的紧耦合设计第一阶段必须和这套链路并行运行。它可以发布 `/rtk_fgo/odom` 和诊断信息，但不能在第一阶段发布生产链使用的 `map -> odom`。

一个现有 topic 不一致必须显式处理：文档常写 `odom_CBoard`，但当前 `serial_reader_node.cpp` 实际发布的是 `odom_CBoar`。第一版应参数化 wheel/chassis odom topic，并在未有意修正 topic 名之前，以真实可执行 topic 作为默认值。
如果 `odom_CBoar` 在 bag 里为 0，优先检查 `serial_reader.log` 和 `/cmd_vel` 是否有数据；这通常是底盘反馈链路问题，不是 FGO 订阅名问题。

## 3. 目标数据流

计划输入：

```text
/fastlio2/lio_odom        nav_msgs/Odometry
/livox/imu                sensor_msgs/Imu
/odom_CBoar               nav_msgs/Odometry
/fix                      sensor_msgs/NavSatFix
/heading                  geometry_msgs/QuaternionStamped
/rtk/status               std_msgs/String
/rtk/nmea_sentence         nmea_msgs/Sentence
```

计划输出：

```text
/rtk_fgo/odom                 nav_msgs/Odometry
/rtk_fgo/path                 nav_msgs/Path
/rtk_fgo/status               std_msgs/String
/rtk_fgo/factor_diagnostics   diagnostic_msgs/DiagnosticArray 或 Float32MultiArray
/rtk_fgo/rtk_gate             std_msgs/String
/rtk_fgo/correction_status    std_msgs/Float32MultiArray
```

可选实验 TF：

```text
map -> odom_fgo
```

该 TF 默认必须关闭。如果后续 Nav2 使用 FGO 链路，只能在实验 launch 中显式启用，并且 frame 与参数必须和生产模式隔离。

## 4. 状态量与因子设计

每个滑动窗口状态建议包含：

```text
X_k = {
  pose: T_map_base,
  velocity: v_map,
  imu_bias: b_acc, b_gyro
}
```

未来可选状态包括 wheel scale、wheel yaw bias、慢变化 RTK map offset。第一版不建议直接估计这些状态，除非 rosbag 回放证明基础图已经稳定。

计划因子集合：

```text
X_k -- IMU preintegration factor ---------- X_{k+1}
X_k -- FAST-LIO relative pose factor ------ X_{k+1}
X_k -- wheel planar odom / yaw-rate factor- X_{k+1}
X_k -- RTK position factor
X_k -- RTK heading factor
X_k -- marginal prior from old window
```

传感器职责：

- IMU 预积分在 RTK 缺失或退化时保持短时连续性。
- FAST-LIO2 relative pose 是主要局部几何约束，承接当前系统较稳定的局部定位能力。
- wheel/chassis odom 提供平面运动和 yaw-rate 约束，尤其适合低速、原地转向和 recovery 行为。
- RTK position 提供全局绝对位置候选，但必须先经过质量和残差门控。
- 双天线 RTK heading 在稳定时提供绝对 yaw 因子。
- marginal prior 在旧状态滑出固定窗口时保留历史信息。
- 有界窗口重建必须为保留下来的每个状态恢复 velocity 与 IMU bias prior；否则 `Values` 中的 V/B 变量会在 GTSAM 中欠约束，诊断会出现 `graph_last_optimization_ok=false`。

## 5. RTK 门控与恢复

当前 gate 已参数化在 `rtk_fgo.yaml` 中：

```yaml
rtk_gating:
  fixed_quality_code: 4
  float_quality_code: 5
  min_satellites: 10
  max_hdop: 2.0
  max_strong_position_innovation_m: 3.0
  max_weak_position_innovation_m: 5.0
  max_heading_innovation_rad: 0.5
  max_implied_speed_mps: 2.0
```

这对树荫遮挡很重要：接收机可能仍持续输出 fix，但 quality、HDOP 和 implied motion 已经不一致。节点现在会在样本进入 shadow graph 前拒绝这些异常点，保留上一轮 FAST-LIO/wheel/IMU 的局部解，并在 `/rtk_fgo/status` 和 `/rtk_fgo/factor_diagnostics` 中报告明确拒绝原因。

RTK Fixed 不能被盲目当成真值。它是最高优先级的绝对位置候选，但只有通过一致性检查后才能成为强因子。

RTK Fixed/Float 分类不能只依赖 `NavSatStatus`。GGA quality `4` 和 `5` 都会映射成 `STATUS_GBAS_FIX`，因此 FGO gate 必须读取 `/rtk/nmea_sentence` 中的 raw GGA，或读取未来带解析测试的结构化 RTK 质量 topic。

质量策略：

```text
GGA q=4 RTK Fixed  -> strong position factor candidate
GGA q=5 RTK Float  -> weak factor or diagnostic-only observation
GGA q=1/2/9        -> 默认只做诊断
q=0 / invalid      -> 不进入图

Stable /heading    -> yaw factor candidate
Invalid heading    -> 不进入图
```

大跳变处理：

1. 质量门控：GGA quality、covariance/HDOP、satellite count、heading source、heading spread、NTRIP/RTCM 状态。
2. 物理连续性门控：RTK implied speed、heading innovation 和方向必须在配置限制内与 FAST-LIO2、IMU、wheel 预测一致。
3. 图残差门控：接收 RTK 因子前，先检查 innovation 或 Mahalanobis residual。
4. 持续性门控：单个好点不能让系统恢复；必须有连续一致样本。

推荐提交策略：

```text
1. 复制当前窗口到 shadow graph。
2. 加入候选 RTK position 与 heading 因子。
3. 优化 shadow graph。
4. 检查候选校正量、yaw 变化、局部速度连续性和非 RTK 因子残差。
5. 只有全部通过才 commit。
6. 否则丢弃候选 RTK 因子，保持当前局部解。
```

这样能在 RTK 恢复可信时把轨迹拉回，同时避免多路径或接收机状态抖动导致突然跳变。

当前实现中，shadow graph 拒绝一次 RTK Fixed 候选不会把系统永久锁进 `FAULT_HOLD`。`RTK_RECOVERY` 阶段的拒绝会回到 `LOCAL_ONLY` 并等待下一轮连续稳定样本；`RTK_LOCKED` 阶段的拒绝会降级到 `RTK_DEGRADED`。`FAULT_HOLD` 只保留给强候选下图残差明确异常的情况。

## 6. 室内外切换状态机

RTK position 在 frame anchor bootstrap 前不会被使用。anchor bootstrap 是 quality-only：有限的 `/fix`、配置的 GGA quality、卫星数、HDOP，以及可选 `/heading` 必须连续通过。这里刻意不使用普通 position-innovation gate，因为 innovation 只有在 RTK fix 已经映射到 FGO `map` 坐标系后才能计算。

RTK heading yaw 约定：`/heading` 先被解码为车辆前向轴在 ENU 中的 yaw，历史文档常写作 `base_link +X`，其中 `0` 指向东，正方向逆时针转向北。在 `frc` URDF 树中，`base_footprint` 与 `base_link` 的 yaw 相同，所以 FGO 输出 frame 可以是 `base_footprint`，不会改变 heading 语义。有 heading 时 anchor 使用 `map_R_enu = Rz(reference_fgo_yaw - rtk_heading_enu_yaw)`；没有 heading 时回退到 identity yaw，并在诊断中报告低置信 bootstrap。anchor 建立之后，所有 RTK heading factor 都必须经过同一个 `ENU -> map` yaw 旋转后，才能和 FGO pose yaw 比较。

当 `frame_anchor.use_rtk_heading_for_yaw=true` 时，anchor bootstrap 还要求 heading 窗口稳定：最近连续通过质量检查的 heading 样本在 wraparound 归一化后的离散度必须小于 `frame_anchor.max_heading_spread_deg`，并用这些样本的 circular mean 初始化 anchor yaw。如果检测到 heading 跳变，bootstrap 会重新开始，而不是用瞬时天线解初始化全局 yaw。

计划状态：

```text
LOCAL_ONLY
RTK_CANDIDATE
RTK_LOCKED
RTK_DEGRADED
RTK_RECOVERY
FAULT_HOLD
```

室内到室外：

1. RTK 缺失或无效时，从 `LOCAL_ONLY` 开始。
2. 有有效 RTK 观测时进入 `RTK_CANDIDATE`。
3. 要求连续满足质量、heading、残差和速度一致性检查。
4. 进入 `RTK_RECOVERY`，把可信 RTK 因子加入最近窗口并优化。
5. 通过 smoother 释放校正，而不是瞬间改变输出位姿。
6. 残差持续稳定后进入 `RTK_LOCKED`。
7. 如果 shadow graph 拒绝候选校正，回到 `LOCAL_ONLY`，重新要求连续稳定样本，而不是永久 fault。

室外到室内：

1. 质量退化或残差异常时降低 RTK 因子强度。
2. 经过 `RTK_DEGRADED` 回到 `LOCAL_ONLY`。
3. 保留最后可信 global anchor，但降低 global confidence。
4. 继续依靠 FAST-LIO2、IMU 和 wheel 约束运行。
5. 如果 `RTK_LOCKED` 状态下的新 Fixed 候选被 shadow graph 拒绝，先降级到 `RTK_DEGRADED`，保留后续重新拉回的机会。

校正平滑建议限制单周期输出变化，例如：

```text
translation <= 0.10-0.20 m per update
yaw <= 0.2-0.5 deg per update
```

具体数值需要通过回放和实车验证确定。

## 7. 计划参数

初始参数分组：

```yaml
/rtk_fgo_localizer:
  ros__parameters:
    heading_quaternion_yaw_is_compass: true

    topics:
      fastlio_odom: /fastlio2/lio_odom
      imu: /livox/imu
      wheel_odom: /odom_CBoar
      fix: /fix
      heading: /heading
      rtk_status: /rtk/status

    window:
      duration_s: 15.0
      keyframe_rate_hz: 10.0
      max_states: 120

    factors:
      imu_enabled: false
      fastlio_enabled: true
      wheel_enabled: true
      rtk_position_enabled: true
      rtk_heading_enabled: true

    rtk_gating:
      require_fixed_for_strong_factor: true
      fixed_quality_code: 4
      float_quality_code: 5
      min_satellites: 10
      max_hdop: 2.0
      max_strong_position_innovation_m: 3.0
      max_weak_position_innovation_m: 5.0
      max_heading_innovation_rad: 0.5
      recovery_min_samples: 8
      max_implied_speed_mps: 2.0
      max_position_jump_m: 3.0

    frame_anchor:
      max_heading_spread_deg: 3.0
      require_fixed_for_anchor: true
      use_rtk_heading_for_yaw: true
      max_anchor_position_innovation_m: 2.0
      bootstrap_min_satellites: 10
      bootstrap_max_hdop: 2.0
      bootstrap_required_consecutive_samples: 5

    imu:
      accelerometer_noise_sigma: 0.1
      gyroscope_noise_sigma: 0.01
      accelerometer_bias_rw_sigma: 0.001
      gyroscope_bias_rw_sigma: 0.0001
      integration_error_sigma: 1.0e-8
      gravity_mps2: 9.81

    correction_smoother:
      max_translation_step_m: 0.15
      max_yaw_step_deg: 0.3
```

这些值只是初始建议，不是已调实车参数。后续任何 tuned YAML 变更都必须在知识文档和 devlog 中说明原因。

## 8. 验证计划

阶段 1：无 ROS 核心测试

- RTK quality gate
- residual gate
- correction smoother
- sliding-window graph diagnostics 和有界重建行为
- state-machine transitions

阶段 2：rosbag replay shadow mode

- 并行运行现有定位链和 `rtk_fgo_localizer`。
- 记录 `/rtk_fgo/*`、源传感器 topic、`/tf`、`/cmd_vel`、RTK raw/status topic，以及 `/pgo/optimized_odom` 和 `/pgo/loop_markers`，方便和主 PGO 链对照。
- 确认生产 TF 没有被改变。
- 生成 replay 指标：

```bash
python3 scripts/evaluate_rtk_fgo_bag.py \
  --bag runtime-data/logs/latest/bag \
  --out runtime-data/logs/latest/system/rtk_fgo_metrics.json
```

阶段 3：实车 shadow mode

- 用 `publish_tf: false`、`nav2_use_fgo: false` 启动实验栈。
- 检查 CPU、内存、topic 频率、状态切换和 RTK gate 决策。
- session 必须保存到 `runtime-data/logs/<timestamp>/`。

阶段 4：受保护实验闭环导航

- 只有 shadow 验证通过后，才在新实验模式中启用实验 TF 和 Nav2 链路。
- 保持现有生产 `corridor`、`explore-gps`、`nav-gps` 模式不变。
- 按安全规则使用 PS2 `X` 失能电机和红色物理急停，不使用 `B` 作为急停。

## 9. 开发规则

- 禁止直接在 Jetson 上改代码。
- 禁止直接推送到 `main`。
- Jetson 构建必须使用 `--parallel-workers 1`。
- 每次构建后必须重新 `source install/setup.bash`。
- 必须显式 stage 文件，禁止 `git add -A` 或 `git add .`。
- 如果修改 RTK parser 或新增结构化 RTK 质量输出，必须给 `um982_rtk_driver` 添加原始 NMEA 样例测试。
- 永远不要提交 CORS/NTRIP 凭据。
- 新增包、launch、参数、工作流和 GPS/GNSS 行为变更时，中英文文档必须同步。
