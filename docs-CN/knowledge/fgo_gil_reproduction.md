# FGO-GIL 复现设计与开发实施文档

## 1. 文档状态

- 状态：设计冻结前草案，可用于开始编码
- 日期：2026-07-11
- 目标论文：*FGO-GIL: Factor Graph Optimization-Based GNSS RTK/INS/LiDAR Tightly Coupled Integration for Precise and Continuous Navigation*，IEEE Sensors Journal，2023
- 运行原则：只做 shadow 输出；完成回放和实车验收前，不发布生产 `map -> odom`，不向 Nav2 remap
- 现有基线：`rtk_fgo_localizer` 继续作为 `/fix`、双天线 heading、FAST-LIO odom 级融合的对照组，不将其改名为论文复现

本文定义从 UM982 原始观测采集到论文级 GNSS RTK/INS/LiDAR 因子图的完整实施路径。它不是对现有 `/fix` 型 FGO 的增量包装；论文复现必须直接使用伪距、载波相位、原始 IMU 和 LiDAR 特征残差。

## 2. 已知事实与约束

### 2.1 车辆与传感器

- GNSS：UM982 双天线，右侧为 master，左侧为 secondary。
- 天线坐标使用 ROS 车辆坐标：`base_link +X` 向前，`+Y` 向左，`+Z` 向上。
- 暂定右 master 相对 `base_link`：`[0.000, -0.184, 0.154] m`。
- 暂定左 secondary 相对 `base_link`：`[0.000, 0.186, 0.154] m`。
- 横向基线长度：`0.370 m`。
- 天线前后安装误差接近零但不为零。当前 `heading_offset_deg=88.5` 来自实车直线段，和理想横装的 `90 deg` 相差 `1.5 deg`；按 0.370 m 基线估算，两相位中心前后差的量级约为 `9.7 mm`，符号尚未实测。
- LiDAR/IMU 运行输入：Livox MID360 点云与 `/livox/imu`。
- URDF 声明 `base_link -> imu_link=[0,0,0.02] m`、`base_link -> laser_link=[0.10,-0.10,0.07] m`。
- FAST-LIO2 使用 `t_il=[-0.011,-0.02329,0.04412] m`、`r_il=I`。它和 URDF 推导值不一致，因此 URDF 数值只能作为模型初值，不能作为论文级外参真值。

所有未确认外参必须携带 `calibrated=false` 和不确定度。未标定不阻止解析、录包和纯 shadow 优化，但阻止结果接管 TF/Nav2。

### 2.2 现有 bag 能力

远端 `badger@100.88.131.52` 的 `2026-07-10-13-48-43` bag 长约 `73.38 s`，包含：

| Topic | 条数 | 约频率 | 用途 |
|---|---:|---:|---|
| `/fix` | 367 | 5 Hz | 现有解级 GNSS 对照 |
| `/heading` | 734 | 10 Hz | 双天线 heading 对照 |
| `/rtk/nmea_sentence` | 1468 | 20 Hz | GGA/THS 质量与 epoch 辅助 |
| `/fastlio2/lio_odom` | 669 | 9.1 Hz | 现有 LIO 对照 |
| `/livox/imu` | 14200 | 193.5 Hz | IMU 预积分开发与回放 |
| `/odom_CBoar` | 1090 | 14.9 Hz | 可选轮速对照，默认不入图 |
| `/rtk_fgo/*` | 约 5 Hz | - | 旧 shadow FGO 对照 |

该 bag 的 GGA 为 RTK Fixed（quality 4，样例为 33 星、HDOP 0.5），THS 有效。它可以支持时间缓冲、IMU 预积分、状态机、对照评估和故障注入，但不含 `OBSVM/OBSVH/OBSVBASE`、星历或载波相位，不能验证双差 GNSS 因子。

### 2.3 UM982 已确认能力

UM982 官方协议提供以下消息：

- `OBSVM`（ID 12）：master 原始观测。
- `OBSVH`（ID 13）：secondary 原始观测。
- `OBSVMCMP`（ID 138）和 `OBSVHCMP`（ID 139）：压缩原始观测，要求相应固件版本。
- `OBSVBASE`（ID 284）：基站原始观测。
- GPS、BDS、GLONASS、Galileo 等星历消息。
- 二进制 header 中的 week、milliseconds-of-week、time status、output delay 和 CRC。
- PPS 配置与最高 921600 baud 串口。

原始观测字段覆盖伪距、累积载波相位、Doppler、C/N0、标准差、signal type、lock time 和 tracking/validity flags，满足构建短基线 RTK 双差观测的基本输入要求。

## 3. 复现边界

### 3.1 必须与论文一致

1. 图中状态包含位置、速度、姿态、加速度计 bias、陀螺仪 bias，以及按星座/频点组织的双差模糊度。
2. GNSS 使用 rover/base 原始伪距和载波相位构造单差、双差残差。
3. GNSS 几何距离必须包含 master 天线相对 IMU 中心的杆臂。
4. IMU 使用原始加速度和角速度预积分，并反馈 bias。
5. LiDAR 非关键帧参与帧间运动估计；关键帧以 scan-to-submap 点到线/点到面残差进入滑窗。
6. 周跳、异常值、参考星选择和整数模糊度固定是显式模块。
7. GNSS 缺失时系统自动退化为 INS/LiDAR，不得重复使用旧 GNSS epoch。
8. 使用真正的 fixed-lag marginalization，不能通过清空图后添加新先验冒充滑窗。

### 3.2 车辆工程适配

- 论文使用 Septentrio 接收机、ADIS-16470 IMU 和 VLP-16；本车使用 UM982、MID360 及其 IMU。这是硬件复现差异，必须在实验报告中单列。
- 内部导航状态按论文保留 ECEF 语义。为改善数值条件，位置变量存储相对固定 ECEF 原点的增量，残差计算时恢复完整 ECEF 坐标。
- LiDAR 子图使用局部 `map_fgo` 坐标，通过固定的 `T_ecef_map_fgo` 和 ECEF 导航状态关联。
- secondary 天线可提供可选 baseline attitude factor；该因子属于车辆增强实验，不计入论文基线结果。
- wheel factor 默认关闭，只能作为单独 ablation 配置，不能混入论文主结果。

### 3.3 当前不承诺

- 不承诺仅凭现有 NMEA bag 达到论文精度。
- 不承诺 CORS 基站距离小于论文忽略电离层/对流层误差的 `20 km` 条件；必须从 RTCM/服务信息确认。
- 不承诺未测外参、未验证 PPS 或默认 IMU noise 下的绝对精度。
- 不直接复用 UM982 已输出的 RTK Fixed 坐标作为载波相位因子结果。

## 4. 总体架构

```text
UM982 COM1 (现有生产链 115200)
  -> GGA/THS/HPR + NTRIP
  -> /fix, /heading, /rtk/status

UM982 独立 raw COM (建议 921600 binary)
  -> um982_raw_driver
  -> CRC/framing -> GNSS week/TOW -> canonical observation epochs
  -> /gnss/raw/master, /gnss/raw/secondary, /gnss/raw/base, /gnss/raw/ephemeris

MID360 + /livox/imu
  -> fgo_gil_lidar_frontend: de-skew, features, keyframes, submap matches
  -> fgo_gil_imu_frontend: timestamp normalization, preintegration

GNSS frontend
  -> ephemeris/satellite state
  -> cycle-slip and outlier detection
  -> per-constellation reference satellite
  -> SD/DD pseudorange and carrier-phase observations

fgo_gil_estimator
  -> IMU + LiDAR + GNSS DD factors
  -> fixed-lag optimization
  -> float state/covariance
  -> LAMBDA ambiguity resolution
  -> fixed candidate validation
  -> /fgo_gil/* shadow outputs
```

现有 `rtk_fgo_localizer` 不删除，用作解级融合 comparator。新研究链路使用独立 namespace、独立参数和独立 launch，避免共享 `/rtk_fgo/*` 造成结果混淆。

## 5. 包与文件设计

建议新增：

```text
src/sensor_drivers/gnss/gnss_raw_msgs/
  msg/RawFrame.msg
  msg/Observation.msg
  msg/ObservationEpoch.msg
  msg/Ephemeris.msg

src/sensor_drivers/gnss/um982_raw_driver/
  include/.../binary_framer.hpp
  include/.../crc32.hpp
  include/.../observation_decoder.hpp
  include/.../time_converter.hpp
  src/...
  test/fixtures/

src/perception/fgo_gil_localizer/
  include/fgo_gil_localizer/gnss/
  include/fgo_gil_localizer/imu/
  include/fgo_gil_localizer/lidar/
  include/fgo_gil_localizer/graph/
  include/fgo_gil_localizer/ambiguity/
  include/fgo_gil_localizer/calibration/
  src/...
  test/...

src/bringup/config/fgo_gil.yaml
src/bringup/launch/system_fgo_gil_shadow.launch.py
scripts/evaluate_fgo_gil_bag.py
```

`gnss_raw_msgs` 只表达协议无关的观测，不携带 FGO 策略。`um982_raw_driver` 只负责串口、协议和观测归一化，不依赖 GTSAM。`fgo_gil_localizer` 不直接读取串口。

## 6. 数据契约

### 6.1 原始帧

每个 `RawFrame` 至少包含：接收 monotonic/ROS 时间、端口、原始 bytes、message ID、sequence、GNSS week/TOW（若 header 有效）、time status、CRC 状态。即使 decoder 尚不支持该 message ID，也允许录制 checksum-valid 原始帧。

### 6.2 观测 epoch

每个 epoch 必须唯一标识：

- receiver：master、secondary 或 base；
- GNSS week 和 TOW；
- clock/time status；
- satellite system、PRN、signal/frequency；
- pseudorange、carrier phase cycles、Doppler、C/N0；
- pseudorange/carrier standard deviation；
- lock time、tracking status、validity 和 half-cycle flags。

下游按 `(receiver, week, tow)` 去重。ROS timer 不得让同一 GNSS epoch多次进入重捕获计数或图优化。

### 6.3 时间规则

- GNSS 因子时间以 UM982 week/TOW 为真值，不使用 callback 的 `now()`。
- LiDAR/IMU 优先使用设备时间；记录 ROS reception time 仅用于诊断。
- 时间同步状态必须是 `UNSYNCED`、`COARSE`、`PPS_LOCKED` 之一，并带估计 offset/jitter。
- `UNSYNCED` 可录包和解析，不允许生成高权重 GNSS/LiDAR 联合因子。
- 发布 odometry 的 stamp 必须是估计状态时间，不是发布时刻。

## 7. 估计器数学设计

### 7.1 状态

关键帧 `k`：

```text
X_k = {delta_p_e_b, v_e_b, q_e_b, b_a, b_g}
A_k = {DD ambiguity per constellation, reference satellite, signal}
```

另维护固定 ECEF origin、`T_ecef_map_fgo`、重力/地球自转模型、GNSS master 杆臂、LiDAR-IMU 外参和各时间偏移。模糊度不是永久全局变量：失锁、周跳、reference satellite 切换或频点变化时按 arc ID 重建。

### 7.2 IMU 因子

- 使用 bias-corrected preintegration。
- 明确 ECEF mechanization、重力与地球自转；若第一版暂用局部 ENU GTSAM preintegration，必须标记为 `engineering_baseline`，不能标记为论文等价结果。
- 任意非有限 IMU、负时间差、超大 gap 均 fail-closed，并断开对应 preintegration interval。
- IMU noise、bias random walk 由参数载入；当前 YAML 数值只作为占位，不能用于最终实验结论。

### 7.3 LiDAR 因子

- 使用每点 offset time 和 IMU 预测去畸变。
- 非关键帧进行 scan-to-scan/帧间优化，输出下一关键帧的初值，但不直接全部加入全局滑窗。
- 关键帧条件至少包括平移、旋转、时间和 GNSS 可用性触发。
- 关键帧 edge feature 构造 point-to-line residual；plane feature 构造 point-to-plane residual。
- 对应点来自局部 keyframe submap，并使用鲁棒核、最小特征数、法向/线拟合退化检查。
- FAST-LIO odom 只作为 comparator 或初始化回退；论文主配置不能把 FAST-LIO BetweenFactor 当作 raw LiDAR factor。

### 7.4 GNSS 双差因子

处理顺序：

1. 由星历计算卫星位置、钟差和信号波长。
2. 质量筛选、elevation/CN0 mask、异常值预筛。
3. rover/base 同 epoch 对齐。
4. 每星座、每频点选择参考星，默认最高 elevation，但加入 hysteresis，避免频繁切换。
5. 构造 rover-base 单差，再构造 reference-nonreference 双差。
6. 使用 `p_ant^e = p_imu^e + R_b^e l_ant^b` 修正 master 杆臂。
7. 分别建立 DD pseudorange 和 DD carrier-phase factor。

如果 CORS 基线超过 20 km，必须启用差分电离层/对流层状态或改用更近基站；不能沿用论文短基线忽略项。

### 7.5 周跳与异常值

- 使用 receiver tracking flags、lock time、half-cycle、Doppler/phase consistency、geometry-free 或 Melbourne-Wubbena（信号可用时）联合检测。
- 检测到周跳只重置受影响的 `(receiver, constellation, satellite, signal, arc)` 模糊度。
- GNSS residual 使用 innovation gate 和 robust loss，但不得用 robust loss 掩盖 CRC、时间错配或错误星历。
- 所有 reject 必须输出结构化 reason 和计数。

### 7.6 整数模糊度固定

- 不手写整数搜索；使用经过验证的 LAMBDA 实现，优先评估 RTKLIB 的 LAMBDA 模块及许可证兼容性。
- 输入是 float ambiguity 与协方差；支持 partial ambiguity resolution。
- ratio test、success-rate/残差检验和 fixed-solution 回代验证均参数化。
- fixed candidate 失败时保留 float 解，不污染滑窗。
- 参考星改变时必须正确变换 ambiguity basis，而不是全部静默复用。

### 7.7 滑窗与边缘化

- 使用 GTSAM fixed-lag smoother 或等价 Schur-complement marginalization。
- window 同时受时间和最大状态数限制；`duration_s` 必须真实生效。
- 被边缘化变量的信息通过 prior 保留，不允许清空图后用当前估计加人为强 prior。
- 记录每轮变量数、因子数、线性化/优化耗时、condition/degeneracy 指标和 marginalization 次数。

## 8. 标定参数

首版参数建议：

```yaml
calibration:
  vehicle_axes: FLU
  gnss:
    master_is_right: true
    master_in_base_m: [0.000, -0.184, 0.154]
    secondary_in_base_m: [0.000, 0.186, 0.154]
    baseline_length_m: 0.370
    heading_offset_deg: 88.5
    longitudinal_uncertainty_m: 0.010
    calibrated: false
  imu_in_base:
    translation_m: [0.0, 0.0, 0.02]
    rotation_rpy_rad: [0.0, 0.0, 0.0]
    calibrated: false
  lidar_in_imu:
    translation_m: [-0.011, -0.02329, 0.04412]
    rotation_matrix: [1,0,0, 0,1,0, 0,0,1]
    source: fastlio2_current
    calibrated: false
time_sync:
  pps_enabled: false
  status: UNSYNCED
```

代码必须通过 TF 链把 `master_in_base` 转成论文 GNSS 残差需要的 `master_in_imu`，并检查 URDF、FAST-LIO 参数和 FGO 参数的方向约定。禁止在 factor 内硬编码这些数值。

## 9. 开发阶段与完成定义

### Phase 0：基线收口与接口冻结

- [ ] 从包含 corridor Task 2 和串口完整性修复的共同基线创建独立开发分支。
- [ ] 冻结消息 schema、坐标系、时间尺度、单位和 calibration schema。
- [ ] 明确 UM982 raw 使用独立 COM，现有 115200 NMEA/NTRIP 链保持不变。
- [ ] 用 `VERSIONA` 记录固件，确认 compressed observation 支持情况。

完成条件：接口设计评审通过，生产 GNSS 行为无修改。

### Phase 1：UM982 原始帧采集

- [ ] 实现 `AA 44 B5` 有界流式 framing、长度检查、CRC 和 resync。
- [ ] 支持任意分片、合并、多帧、噪声前缀、截断和未知 ID。
- [ ] 发布并录制 raw frame，增加 framing/CRC/丢帧诊断。
- [ ] 默认 921600、独立设备名和 udev 规则；不写入 CORS 凭证。

完成条件：合成/官方 fixture 全通过；fuzz 输入不崩溃、不越界；现有 NMEA 驱动测试不回归。

### Phase 2：观测与星历归一化

- [ ] 解码 OBSVM/OBSVH、可用时解码 compressed 版本。
- [ ] 解码 OBSVBASE；若 CORS 不提供该消息，增加 RTCM MSM 到 canonical epoch 的适配层。
- [ ] 解码所用星座星历并计算 satellite state。
- [ ] 实现 week rollover、TOW、time status 和 epoch 去重。

完成条件：零噪声合成观测可恢复已知几何距离；坏时间/坏星历/非有限字段被拒绝。

### Phase 3：时间同步和 IMU 预积分

- [ ] 建立 GNSS、LiDAR、IMU 时间域映射及状态诊断。
- [ ] 支持 PPS 状态输入；PPS 未锁定时传播时间不确定度。
- [ ] 完成 ECEF IMU propagation/preintegration 和 bias Jacobian 测试。
- [ ] 用现有 2026-07-10 bag 验证约 193.5 Hz IMU buffer、gap 和重复时间处理。

完成条件：合成轨迹的 propagation 误差在预设容差内；时间倒退、gap、NaN/Inf 均 fail-closed。

### Phase 4：LiDAR 原始因子前端

- [ ] 复用或抽取 MID360 point preprocessing，不复制不可维护的 FAST-LIO 私有状态。
- [ ] 完成 de-skew、edge/plane extraction、keyframe 和 KF-map 管理。
- [ ] 实现 point-to-line、point-to-plane factor 与 Jacobian 数值检查。
- [ ] 输出 feature count、match residual、degeneracy 和耗时。

完成条件：合成平面/直线残差零点正确；有限差分 Jacobian 通过；退化场景不输出虚假高置信约束。

### Phase 5：GNSS DD 与 float FGO

- [ ] 完成参考星选择、SD/DD builder、杆臂修正和 DD factors。
- [ ] 完成 cycle-slip/outlier/arc 管理。
- [ ] 联合 IMU、LiDAR、GNSS float ambiguity 进入 fixed-lag graph。
- [ ] GNSS outage 时无重复因子，系统连续退化为 LIO。

完成条件：合成 rover/base 数据中 float state 与 ambiguity 收敛；reference switch 和单星周跳测试通过。

### Phase 6：整数固定

- [ ] 集成 LAMBDA，完成 covariance ordering 和单位测试。
- [ ] 实现 ratio test、partial fix、回代和 fixed rejection。
- [ ] 发布 FLOAT/FIXED 状态、fix ratio、固定卫星数和 rejection reason。

完成条件：已知整数 fixture 正确固定；错误候选不会污染下一窗口；周跳后只重置相关 ambiguity。

### Phase 7：ROS shadow 集成和回放

- [ ] 增加 `system_fgo_gil_shadow.launch.py` 和独立 bag profile。
- [ ] 发布 `/fgo_gil/odom`、path、factor diagnostics、ambiguity status、timing status 和 performance。
- [ ] 将新节点加入 `make kill-runtime`，但保持 `publish_tf=false`、`nav2_use_fgo=false`。
- [ ] 扩展 evaluator，报告 APE/RPE、availability、fixing rate、outage drift、CPU/RAM 和实时因子。

完成条件：桌面测试和 Jetson clean build 通过；现有 bag 可验证非 GNSS-raw 路径和 comparator，缺少 raw topic 时明确报告 `RAW_GNSS_UNAVAILABLE`。

### Phase 8：天气允许后的采集与验收

- [ ] 采集 VERSION、raw master/secondary/base、星历、PPS、LiDAR、IMU、TF 和温度。
- [ ] 测量两天线相位中心的 X/Z 和 LiDAR-IMU 真外参。
- [ ] 确认 CORS station 坐标、station ID、基线距离和 RTCM 内容。
- [ ] 完成静止、直线、转弯、开阔、树荫、短时遮挡和重捕获数据集。
- [ ] 与 UM982 RTK、FAST-LIO2、现有 `rtk_fgo_localizer` 做同 bag 对照。

完成条件：达到预先冻结的精度、连续性、fixing rate 和资源预算后，才允许提出受控 TF A/B 实验；默认仍保持 shadow。

## 10. 测试矩阵

| 层级 | 必测内容 |
|---|---|
| Parser unit | CRC、长度、endianness、分片/合并、未知 ID、week/TOW、NaN/Inf |
| GNSS unit | satellite state、SD/DD 符号、频率/波长、杆臂、reference switch、周跳 |
| IMU unit | 静止、匀速、恒定角速度、bias Jacobian、gap/乱序 |
| LiDAR unit | de-skew、点到线/面 residual、Jacobian、退化 |
| Graph unit | marginalization、outage、re-entry、float ambiguity、fixed rollback |
| Synthetic integration | 已知轨迹 + 可控 GNSS 遮挡/多路径/周跳 + LiDAR 退化 |
| Existing bag | 时间关联、IMU/LIO comparator、状态机、缺 raw GNSS fail-closed |
| New raw bag | full observation-level replay、fixing rate、APE/RPE、资源占用 |
| Vehicle | 电机失能 bench 后低速 shadow；最后才考虑受控 TF A/B |

所有测试命令在 Jetson 上使用 `--parallel-workers 1`，build 后必须重新 source。parser 测试应保持 ROS-free，便于工作站快速运行。

## 11. 诊断与安全

必须至少暴露：

- raw frame rate、CRC failure、framing resync、unknown ID；
- 每接收机 epoch rate、week/TOW、time status、age 和 duplicate count；
- 每星座可用卫星、reference satellite、DD observation count；
- cycle slip、outlier、arc reset 和 ambiguity dimension；
- IMU/LiDAR/GNSS factor count、residual 和 reject reason；
- float/fixed、ratio、fixed satellite count、rollback count；
- window span、states、factors、marginalization 和 optimization latency；
- calibration validity、PPS lock、time offset/jitter；
- 输出 stamp age、非有限检测和 shadow/control ownership。

以下任一条件必须阻止接管：外参未标定、时间未锁定、非有限状态、图发散、输出 stale、错误 ECEF origin、GNSS epoch 重复、TF owner 冲突。安全停机仍遵循 PS2 `X` 和物理急停规则，禁止把 `B` 当急停。

## 12. 开发决策记录

1. 先实现 raw acquisition，再写依赖真实观测的 GNSS 因子；不继续堆叠 `/fix` GPSFactor。
2. NMEA 生产链和 raw binary 研究链物理/逻辑隔离，避免 115200 line parser 被二进制污染。
3. 当前左右天线 Y/Z 和 88.5 deg 可作为带协方差初值；未知毫米级 X 偏移不硬猜。
4. FAST-LIO 外参优先作为当前算法初值，但因其与 URDF 不一致，保持 `calibrated=false`。
5. 现有 7 月 bag 是软件回归资产，不是假装包含载波相位的论文验证数据。
6. 论文基线与车辆增强项分别出结果：secondary heading、wheel、现有 RTK solution factor 均属于 ablation/extension。

## 13. 近期第一批提交

第一批开发只覆盖 Phase 0-1：

1. 新增 `gnss_raw_msgs` 最小消息集。
2. 新增 ROS-free UM982 binary framer、CRC 和 fixture tests。
3. 新增 raw driver shadow node，只连接独立 raw port。
4. 新增 raw frame bag topic 与诊断。
5. 新增默认关闭的 bringup 参数，不修改现有 UM982 NMEA/NTRIP 启动行为。
6. 同步中英文 commands、architecture、knowledge 和 devlog。

该批次不引入 GTSAM 因子、不修改 Nav2、不修改生产 TF，也不要求天气允许外出采集后才能合并 parser 代码。
