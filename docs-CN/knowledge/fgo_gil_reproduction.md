# FGO-GIL 复现设计与开发实施文档

## 1. 文档状态

- 状态：代码实现与 shadow 验证阶段
- 日期：2026-07-21
- 目标论文：*FGO-GIL: Factor Graph Optimization-Based GNSS RTK/INS/LiDAR Tightly Coupled Integration for Precise and Continuous Navigation*，IEEE Sensors Journal，2023
- 运行原则：只做 shadow 输出；完成回放和实车验收前，不发布生产 `map -> odom`，不向 Nav2 remap
- 现有基线：`rtk_fgo_localizer` 继续作为 `/fix`、双天线 heading、FAST-LIO odom 级融合的对照组，不将其改名为论文复现

当前代码进度：Phase 1、3、4、5、6 已完成代码实现；Phase 2 已包含非压缩 observation、broadcast ephemeris、week rollover/reset、UM982 官方 signal-frequency 映射和 satellite-state 传播。Phase 5 已增加 rover/base 有界对时、参考星滞回、DD 码/载波因子、逐信号 ambiguity arc、IMU/LiDAR/GNSS float 联合优化和 Schur complement fixed-lag 边缘化；Phase 6 已增加固定版本 RTKLIB MLAMBDA、partial ambiguity resolution、固定候选回代验证与独立 fixed shadow 输出。单 Type-C 的统一 `um982_rtk_driver` 已完成 115200 生产回归和临时 921600 mixed capture：同一有界流分离 ASCII 与 `AA 44 B5` binary，并保留同口 NTRIP/RTCM 写入。NTRIP 链路现在会解码 CRC 正确的 RTCM 1005/1006 基准站 ECEF，FGO 可以使用该坐标并在基站变化时受控重置图和 ambiguity。硬件 PPS、compressed observation、现场 1005/1006 确认、`T_ecef_lidar_world` 和完整 float/fixed bag 验收仍未完成，因此默认保持 shadow-only 并 fail closed。

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
UM982 Type-C -> CP210x -> /dev/rtk_um982 (当前载板只暴露一个 UART)
  -> um982_rtk_driver unified serial owner (默认 115200 NMEA-only；现场 mixed 目标 921600)
     -> ASCII GGA/THS/HPR -> /fix, /heading, /rtk/status
     -> binary AA 44 B5 -> canonical raw observation/ephemeris
     <- NTRIP/RTCM write on the same full-duplex port

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
- lock time、tracking status、validity，以及接收机能提供时的 half-cycle 状态。UM982 OBSVM tracking status 没有 half-cycle 位，因此 canonical stream 会明确把该能力记为不可用，而不是伪造 flag。

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
- [x] 实现单 Type-C 的唯一串口 owner/mux；在同一有界流中优先识别完整 binary frame 和 ASCII line，并保留同口 NTRIP/RTCM 写入。
- [ ] 用 `VERSIONA` 记录固件，确认 compressed observation 支持情况。

完成条件：接口设计评审通过，生产 GNSS 行为无修改。

### Phase 1：UM982 原始帧采集

- [x] 实现 ASCII/`AA 44 B5` 有界 mixed-stream framing、长度检查、CRC 和 resync。
- [x] 支持任意分片、粘连、多帧、噪声前缀、截断、binary 内换行/ASCII marker 和未知 ID。
- [x] 发布并录制 raw frame，增加 framing/CRC/丢帧诊断。
- [x] 在 Jetson 用统一 driver 完成 115200 NMEA-only 回归，再将唯一 `/dev/rtk_um982` 临时迁移到 921600 mixed stream，且未持久化接收机状态、未把 CORS 凭证写入仓库。

完成条件：合成/官方 fixture 全通过；fuzz 输入不崩溃、不越界；现有 NMEA 驱动测试不回归。

### Phase 2：观测与星历归一化

- [x] 解码非压缩 OBSVM/OBSVH 及 tracking-status 语义。
- [ ] 固件支持时解码 OBSVMCMP/OBSVHCMP。
- [x] 解码非压缩 OBSVBASE。
- [ ] 若 CORS 不提供 OBSVBASE，增加 RTCM MSM 到 canonical epoch 的适配层。
- [x] 解码并归一化 GPS、GLONASS、BDS、Galileo、QZSS broadcast ephemeris。
- [x] 由 broadcast ephemeris 计算 satellite state。
- [x] 保留 week/TOW/time status，并按 receiver + week/TOW 有界去重。
- [x] 增加显式 week rollover 和接收机时间重置策略。

完成条件：零噪声合成观测可恢复已知几何距离；坏时间/坏星历/非有限字段被拒绝。

### Phase 3：时间同步和 IMU 预积分

- [x] 建立 GNSS、LiDAR、IMU 时间域映射及状态诊断。
- [x] 支持 PPS `TimeReference` 输入；PPS 未锁定时传播时间不确定度。
- [x] 完成 ECEF IMU propagation/preintegration 和 bias Jacobian 测试。
- [ ] 用现有 2026-07-10 bag 验证约 193.5 Hz IMU buffer、gap 和重复时间处理。

完成条件：合成轨迹的 propagation 误差在预设容差内；时间倒退、gap、NaN/Inf 均 fail-closed。

实现说明：当前 Livox production stamp 是 ROS `now()`，所以默认只能达到 `COARSE`；真实 PPS 连续输入后才允许进入 `PPS_LOCKED`。本机 2026-06-24 bag 已验证审计工具与 fail-closed segmentation：26,004 条可解码 IMU 无倒退/重复/非有限值，但 83.47 Hz 有效频率和 1,109 个 `>50 ms` gap 不满足连续预积分验收。

### Phase 4：LiDAR 原始因子前端

- [x] 复用或抽取 MID360 point preprocessing，不复制不可维护的 FAST-LIO 私有状态。
- [x] 完成 de-skew、edge/plane extraction、keyframe 和 KF-map 管理。
- [x] 实现 point-to-line、point-to-plane factor 与 Jacobian 数值检查。
- [x] 输出 feature count、match residual、degeneracy 和耗时。

完成条件：合成平面/直线残差零点正确；有限差分 Jacobian 通过；退化场景不输出虚假高置信约束。

实现说明：Phase 4 核心不包含 FAST-LIO 的 IESKF、ikd-tree 或私有状态，只读取 Livox `offset_time/tag/line` 公共字段。FAST-LIO odom 在 shadow 节点中只作为 IMU 轨迹和因子线性化点的初始化回退，不会变成论文 LiDAR factor。所有扫描必须通过 Phase 3 时间状态、连续 IMU 覆盖、最小线面特征数和 Hessian 可观性门控；单平面合成场景明确得到 `constraint_valid=false`。当前 `acceleration_scale=9.80665`、`t_il=[-0.011,-0.02329,0.04412] m` 和曲率/匹配阈值来自现有驱动/FAST-LIO 初值，只允许 shadow 使用。本机 bag 没有 `/livox/lidar`，真实特征数量、运行耗时和阈值仍需数据机上线后验收。

### Phase 5：GNSS DD 与 float FGO

- [x] 完成参考星选择、SD/DD builder、杆臂修正和 DD factors。
- [x] 完成 cycle-slip/outlier/arc 管理。
- [x] 联合 IMU、LiDAR、GNSS float ambiguity 进入 fixed-lag graph。
- [x] GNSS outage 时无重复因子，系统连续退化为 LIO。

完成条件：合成 rover/base 数据中 float state 与 ambiguity 收敛；reference switch 和单星周跳测试通过。

实现说明：载波频率严格按 UM982 分星座 signal-ID 表解释，未知 ID 和非法 GLONASS 频点直接 fail closed。GPS/QZSS/Galileo/BDS Kepler、BDS GEO 旋转、GLONASS RK4、发射时刻和 Sagnac 修正均有单位测试。DD 载波模糊度统一存为米，key 同时包含 target/reference satellite 和 rover/base 四条 arc ID，因此参考星切换或单星周跳不会静默复用旧变量。Eigen smoother 把 Phase 3 ECEF IMU 重传播、Phase 4 原始线面因子和 DD 码/载波因子共同重线性化；窗口同时受时间和 state 数限制，旧 state 与失活 ambiguity 通过 Schur 补进入带锚点的稠密先验，禁止清图后补虚假强 prior。

Phase 5 ROS 链使用 `fgo_gil_msgs/LidarConstraintBatch`，Phase 4 前端传递真实点线/点面因子，而不是把 FAST-LIO pose 伪装成 LiDAR factor。`system_fgo_gil_float.launch.py` 保留 `/fgo_gil/float_odom_ecef`，Phase 6 仅增加独立 `/fgo_gil/fixed_odom_ecef`，始终不发布 TF 或控制命令。`calibration.ecef_from_lidar_world.calibrated` 与静态 `calibration.gnss.base_ecef_calibrated` 覆盖默认均为 `false`；base gate 现在可由CRC正确的RTCM 1005/1006满足，ECEF/world仍必须通过现场拟合验收。master杆臂占位值 `[0.0,-0.184,0.134] m` 由已测右天线在 `base_link` 中的 `[0.0,-0.184,0.154] m` 减去当前尚未精标的IMU Z占位 `0.02 m` 得到。

### Phase 6：整数固定

- [x] 集成 LAMBDA，完成 covariance ordering 和单位测试。
- [x] 实现 ratio test、partial fix、回代和 fixed rejection。
- [x] 发布 FLOAT/FIXED 状态、fix ratio、固定 ambiguity 数和 rejection reason。

完成条件：已知整数 fixture 正确固定；错误候选不会污染下一窗口；周跳后只重置相关 ambiguity。

实现说明：MLAMBDA 核心固定到 RTKLIB commit `71db0ffa0d9735697c6adfd06fdf766d0e5ce807` 的 `lambda.c`，Eigen 只替换内存管理与最终线性求解；上游版权、BSD-2-Clause 条款与附加条款完整保存在 `third_party/rtklib/LICENSE.txt`。smoother 从完整联合 Hessian 计算边缘协方差，并用与 ambiguity key 相同的确定顺序输出；DD ambiguity 从米按 signal wavelength 转成周后才进入 LAMBDA。

整数解析先锁定全图最新 GNSS state，并要求每个 signal group 在该 state 只有唯一 reference satellite 与 reference rover/base arc 基底；不同星座/信号的当前变量在分别换算为 cycles 后使用完整交叉协方差联合进入 LAMBDA。参考星变化时，新变量通过 `N_i^q=N_i^r-N_q^r` 精确变换初始化；任一相关 arc 改变则无法匹配旧基底，自动回到新变量初始化。由于 GLONASS FDMA 的 target/reference wavelength 不同，本阶段明确排除 GLONASS 整数固定，避免把米制组合错误解释为单一整数周。

默认门限为 `ratio>=3.0`、bootstrap success rate `>=0.99`、候选归一化平方残差 `<=25`，不足时按最大方差逐个剔除并尝试 partial fix，最少保留 4 个 ambiguity。候选通过后只计算条件回代预览 `delta_x=P_xa P_aa^-1(a_fixed-a_float)`；位置、姿态、速度修正或图代价增量超限即拒绝。无论接受或拒绝，回代都不写入 float graph；float topic 始终发布原解，fixed topic 只在全部门通过且最新 keyframe 有本历元有效 DD factor 时发布，GNSS outage 不会复用旧 ambiguity 发布 stale fixed。

### Phase 7：ROS shadow 集成和回放

- [x] 增加 `system_fgo_gil_shadow.launch.py` 和独立 bag profile。
- [x] 发布 `/fgo_gil/odom`、path、factor diagnostics、ambiguity status、timing status 和 performance。
- [x] 将完整 shadow 进程集加入 `make kill-runtime`，但保持 `publish_tf=false`、`nav2_use_fgo=false`。
- [x] 扩展 evaluator，报告 APE/RPE、availability、fixing rate、outage drift、CPU/RAM 和实时因子。

完成条件：桌面测试和 Jetson clean build 通过；现有 bag 可验证非 GNSS-raw 路径和 comparator，缺少 raw topic 时明确报告 `RAW_GNSS_UNAVAILABLE`。

实现说明：完整 launch 的 live 默认启动 Livox、FAST-LIO2 initializer/comparator、统一 UM982 driver 的 mixed profile 和 Phase 3-7 estimator；replay 可逐项关闭硬件节点并启用 ROS clock。`full` profile 保存 raw frame、原始 LiDAR、TF 和完整诊断，`minimal` profile 保存重放算法与评价所需的最小输入/输出。该 launch 与 estimator 各自执行 shadow ownership 检查，任一 `publish_tf=true` 或 `nav2_use_fgo=true` 都直接拒绝启动；本阶段没有新增控制节点，`make kill-runtime` 已覆盖所有现有 FGO executable、sensor/comparator 和 rosbag 进程。

2026-07-15 Jetson 算法链 smoke 和 921600 单串口 mixed 采集已完成。首个 287 s raw bag 在独立 x86 ROS 2 主机回放时发现：master/secondary 10 Hz 与 base 1 Hz 的正常交错产生 288 次 `secondary -> base` 的 200--300 ms TOW 回退，旧时间节点把三路 receiver 共用一个 `GnssTimeTracker`，因此几乎每秒误 reset 一次。时间映射现在只接受 master receiver；secondary/base 仍完整进入 GNSS 双差，不参与接收机时钟拟合。交错回归测试冻结该行为，不通过放宽同步门限掩盖问题。同一 bag 修复后回放在实际播放期间连续记录 287 次 `COARSE_NO_PPS`，处理 2795/2867 帧 LiDAR（97.5%），pending 丢帧降至 71，并产生 1259 个有效约束和 200 个 keyframe batch，确认时间同步阻塞已解除。该 bag 仍只有 4 条星历，且 ECEF/world 与 CORS base ECEF 未标定，因此尚不构成 float/fixed 验收。现有 `/dev/pps0` 的 source 名称为 `ktimer`，只是虚拟测试时钟，禁止将其标记为 GNSS `PPS_LOCKED`；早期 shadow 继续使用带 20 ms 不确定度下限的 `COARSE_NO_PPS`。

2026-07-16 对同一 bag 的进一步干净回放发现并修复两个独立的软件问题。首先，LiDAR 旋转 Jacobian 曾使用约 6378 km 的绝对 ECEF 点，而 smoother 的位置和姿态是分离扰动，导致条件数达到 `1.86e18`；现在 Jacobian 只使用旋转后的 body-frame 点，并有 Earth-radius 回归测试。其次，单线程 executor 在优化期间阻塞 IMU/raw 回调；现在 IMU、raw 输入和 estimator 使用独立 callback group，节点使用三线程 executor、有界待处理队列，并在 IMU 历史不可用或优化失败时 fail-closed reset/reseed。raw 历元在关键帧前批量排空，不再为每条 raw 消息重复优化；每关键帧以确定性均匀采样保留最多 48 个 line 和 96 个 plane 因子。重复有限差分只传播 nominal IMU 状态，package 在未指定 build type 时使用 `RelWithDebInfo`。非有限或超过 `1e12` 的图条件估计会拒绝本轮结果并 reset，不能继续发布坏状态。

上述队列和门限是受回放约束的安全默认值：`imu_qos_depth=512` 在约 200 Hz 下保留约 2.56 s；raw DDS depth 为 512，节点内 raw/ephemeris 队列为 1024/64；LiDAR 最多等待 16 批、0.5 s。因子上限只减少同类重复约束，前端几何质量门限不变。不要仅为降低延迟继续缩小门限，后续必须用新的 GNSS-complete bag 做精度/退化对照后再调。

最终的 287 s input-only 回放使用 `scripts/replay_fgo_gil_bag.sh`，明确排除 bag 内旧 `/fgo_gil/*` 输出，避免旧约束和诊断污染当前节点。回放产生 496 个 LiDAR keyframe batch、496 个 float ECEF 输出和 496 个统一 odometry/path 输出；末尾图为 10 个状态、1929 个因子、9.40 s 窗口和 486 次 marginalization，条件估计约 `7.85e4`。优化失败、rollback、数值拒绝、IMU reseed、FGO pending/drop 和 raw queue drop 均为 0。实际播放窗口的 326 个 performance 样本中，优化延迟 mean/p95/max 为 `120.2/141.2/151.2 ms`，有限 output age 的 p95/max 为 `0.744/1.046 s`。输出 bag 有 4364 条消息且 SQLite `integrity_check=ok`。

这仍只是 LiDAR/IMU 软件连续性回归，不是论文算法的定位精度验收。输入只有 4 条 Galileo 星历，`calibration.gnss.base_ecef_calibrated=false`，因此 287 个已对齐 GNSS 历元均未进入 GNSS 融合，稳定阶段状态为 `LIO_ONLY_WAITING_BASE`；没有形成 GNSS code/carrier factor，也不能计算有意义的 ECEF APE/RPE 或 fixing rate。FGO 继续保持 shadow-only，不发布生产 TF，不影响 FAST-LIO2/Nav2。

2026-07-21户外动态bag是本分支第一份五系统raw数据。经SHA-256核对的338.17 s副本包含98,996条消息：67,604条IMU（199.91 Hz）、3,382帧原始LiDAR、3,364个FAST-LIO pose、458批LiDAR约束、master/secondary各3,378个历元、base 338个历元和284条星历。GPS/GLONASS/Galileo/BDS/QZSS星历分别覆盖11/8/10/14/3颗卫星；338个base历元按GNSS week/TOW全部与master精确重合。旧evaluator因错误使用波动的接收时间只匹配153对；现在按week/TOW配对，接收时间只用于ROS轨迹关联。

bag开始后47.78 s出现一次孤立系统事件：IMU间断313.7 ms、master缺3个历元、secondary缺4个历元、LIO间隔约405.6 ms，并在48.35 s报告唯一一次UM982 mixed-stream CRC失败。之后全部流恢复，无overflow或后续decode failure；事件附近1 s粒度 `tegrastats` 未显示内存、swap、温度或持续CPU上限。该bag可用于标定几何和连续性排查，但不通过严格连续IMU验收门。

统一driver现在用有界RTCM3 framer旁路观察其原样转发给UM982的NTRIP字节，CRC24Q正确的1005/1006发布 `/gnss/rtcm/reference_station`。显式标定的静态base参数优先，否则首个动态坐标打开base gate；station ID、ITRF realization、NTRIP source或ECEF变化超过1 cm时，epoch aligner、ambiguity arc、pending GNSS和shadow graph全部重置。`calibrate_fgo_gil_ecef_world.py` 在应用现有IMU-LiDAR外参与天线杆臂后，用GGA质量4的 `/fix` 和FAST-LIO pose拟合剩余无尺度SE(3)；缺少质量证据、轨迹过短/近似直线、样本不足或残差超限均拒绝输出。

统一 `/fgo_gil/odom` 优先选用当前历元已验证 fixed candidate，否则使用 float，并同步发布有界 ECEF path。factor diagnostics 分层输出 IMU、LiDAR line/plane、GNSS code/carrier 数量、residual RMS、DD reject reason、arc reset 和 optimizer rollback；ambiguity/timing/performance 分别输出整数状态、时钟状态、窗口/延迟/实时因子、stale/non-finite 与 control ownership。raw observation 使用 2 s stale 阈值，低频 broadcast ephemeris 使用独立 300 s 阈值，避免把正常星历刷新周期误判为断流。

`evaluate_fgo_gil_bag.py` 先按时间匹配FGO与FAST-LIO comparator，再做无尺度SE(3)刚体对齐，避免直接相减ECEF与局部坐标；outage按GNSS week/TOW在50 ms内一对一匹配master/base，接收时间只作诊断，单边raw流标记为 `RAW_GNSS_INCOMPLETE`。结果包含APE/RPE、availability、fixing rate、outage drift、optimization latency/RTF和 `tegrastats` CPU/RAM。metadata-only模式不依赖ROS解码；raw topic缺失或消息数为零时明确输出 `RAW_GNSS_UNAVAILABLE`。

### Phase 8：天气允许后的采集与验收

- [x] 实现有界 RTCM3 分帧、CRC24Q 校验和 1005/1006 基准站发布；写入 UM982 的 NTRIP 字节保持原样。
- [ ] 采集 VERSION、raw master/secondary/base、星历、PPS、LiDAR、IMU、TF 和温度。
- [ ] 测量两天线相位中心的 X/Z，并用实车 bag 复核当前 FAST-LIO2/FGO 共用的 LiDAR-IMU 六自由度外参。
- [ ] 在新 bag 中通过 `/gnss/rtcm/reference_station` 确认现场 CORS station 坐标、station ID、基线距离和 RTCM 内容。
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
