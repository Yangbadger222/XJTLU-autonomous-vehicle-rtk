# FASTLIO2 算法详解

## 当前项目中的位置

- 包名: `fastlio2`
- 当前主输出（启用高度过滤时，点云输出经过发布前高度裁剪，详见下文）:
  - `/fastlio2/lio_odom`
  - `/fastlio2/body_cloud`
- 当前主入口:

```bash
ros2 launch fastlio2 lio_no_rviz.py params_file:=~/XJTLU-autonomous-vehicle/src/bringup/config/master_params.yaml
```

- 当前参数来源以 `src/bringup/config/master_params.yaml` 为主；`lio_no_rviz.py` 保留 legacy `fastlio2/config/lio.yaml` 回退能力。
- 下游的 `pgo` 节点直接消费 `/fastlio2/lio_odom` 和 `/fastlio2/body_cloud`。
- 这份文档主要解释算法原理；整车链路请同时参考 `docs/architecture.md` 和 `docs/knowledge/pgo.md`。

## 发布点云前置高度过滤（2026-04-01）

FAST-LIO2 在 `lio_node.cpp` 中新增了发布前点云高度过滤功能（commit `f619fa6`）：

- 在 `body_cloud` 和 `world_cloud` 发布给下游（PGO、Nav2 STVL）之前，按重力方向相对车体原点的高度进行裁剪
- 过滤窗口由 `master_params.yaml` 控制：
  - `publish_cloud_height_filter_enabled: true`（开关）
  - `publish_cloud_min_z: -0.33`（最低相对高度）
  - `publish_cloud_max_z: 0.30`（最高相对高度）
- 相对高度计算：`relative_height = world_cloud.z - body_origin_z_in_world`
- 效果：地面低点和车体高度以上的杂点（顶棚/高处结构）在源头被剔除，下游 costmap 不再需要重复过滤
- 该过滤不影响 FAST-LIO2 内部的 SLAM 建图和状态估计，仅影响发布给外部的点云

## LiDAR / IMU 同步保护（2026-07-06）

FAST-LIO2 现在会在进入 IESKF 更新前检查同步包中的 IMU 样本数；若少于 `min_imu_samples_per_lidar`，直接丢弃该 LiDAR 帧。当前 corridor 验收值为：

- `min_imu_samples_per_lidar: 3`

原因：2026-07-06 21:02 corridor bag 显示车辆静止时，FAST-LIO2 仍处理了 `0 IMU samples, 925 LIDAR points`、`1 IMU samples, 1086 LIDAR points` 等同步包，随后 odom 在 Nav2 收到路线目标前跳出几十米。丢弃 IMU 约束不足的 LiDAR 帧，比让坏同步窗口推进滤波器更安全。

这个保护还有第二层，用于低特征户外启动：如果 IESKF 更新没有有效 LiDAR 修正，FAST-LIO2 会恢复到预测前状态，跳过增量建图，并且不发布该帧 TF/odom。这样当 LiDAR 匹配报告 `NO Effective Points` 时，IMU-only 预测不会被下游当作有效定位。

## 1. odom 里程计数据解读

FASTLIO2 输出的 odom（`nav_msgs/Odometry`）包含两部分核心数据：

- **`pose.pose.position`**：`(x, y, z)` — 载体在世界坐标系下的位置
- **`pose.pose.orientation`**：`(x, y, z, w)` — 载体在世界坐标系下的姿态（四元数表示）

这些数据并非直接来自某个传感器的原始测量，而是通过 **IESKF（迭代误差状态卡尔曼滤波）** 融合 IMU 前向积分与 LiDAR 点云匹配后得到的最优估计。具体来说，IMU 提供高频（200Hz）的加速度和角速度数据用于短时预测，LiDAR 提供每帧（10Hz）的点云数据用于修正累计误差。

---

## 2. 位置数据计算

### 2.1 初始状态

系统启动时，初始状态为：

```
位置: t_wi = [0, 0, 0]^T
速度: v = [0, 0, 0]^T
旋转: R_wi = I (单位矩阵)
```

### 2.2 IMU 数据到达

第一帧 IMU 数据到达，包含：

- **acc**：加速度计测量值（body 坐标系下）
- **gyro**：陀螺仪测量值（body 坐标系下）
- **dt**：采样间隔，约 **5ms**（200Hz）

### 2.3 世界坐标系下的加速度计算

IMU 测量的加速度 `acc` 是在载体坐标系（body frame）下的，需要转换到世界坐标系：

```
a_world = R_wi × (acc - ba) + g
```

其中：
- **`R_wi`**：当前时刻的旋转矩阵（body → world 的旋转）
- **`acc`**：加速度计原始测量值
- **`ba`**：加速度计偏置（bias），随时间缓慢漂移，由滤波器在线估计
- **`g`**：重力加速度向量 `[0, 0, -9.81]^T`

### 2.4 速度和位置更新

得到世界坐标系下的加速度后，通过数值积分更新速度和位置：

```
v_new = v + a_world × dt
t_wi_new = t_wi + v × dt + 0.5 × a_world × dt²
```

每到一帧 IMU 数据（每 5ms），就执行一次上述积分，不断累积得到当前位置估计。

---

## 3. 旋转矩阵 R_wi 的计算

旋转矩阵 `R_wi` 的更新是 FASTLIO2 的核心之一，分为以下 7 个步骤：

### Step 1: 去除陀螺仪偏置

```
ω_true = gyro - bg
```

- **`gyro`**：陀螺仪原始测量值（rad/s）
- **`bg`**：陀螺仪偏置，由 IESKF 在线估计
- **`ω_true`**：真实角速度

### Step 2: 计算角度增量

```
Δθ = ω_true × dt
```

- **`dt`**：IMU 采样间隔（约 5ms）
- **`Δθ`**：该时间段内的角度变化量（三维向量，单位 rad）

### Step 3: 指数映射（旋转向量 → 旋转矩阵）

```
ΔR = Sophus::SO3d::exp(Δθ).matrix()
```

使用 Sophus 库的 `SO3::exp()` 函数，将角度增量向量映射为旋转矩阵。这是 **李群 SO(3)** 的指数映射操作：
- 输入：三维旋转向量 `Δθ`
- 输出：3×3 旋转矩阵 `ΔR`
- 内部实现等价于 Rodrigues 公式

### Step 4: 更新旋转矩阵

```
R_wi = R_wi × ΔR
```

将增量旋转右乘到当前旋转矩阵上，实现旋转的累积更新。

**误差分析**：每次积分都会引入微小误差（陀螺仪噪声、离散化误差），这些误差会随时间累积。如果只靠 IMU 积分，旋转估计会很快发散。因此需要 LiDAR 数据进行校正（Step 5-6）。

### Step 5: LiDAR 数据残差计算

当一帧 LiDAR 点云到达时，利用点云与已有地图的匹配来计算残差，用于修正 IMU 积分的累积误差。

**点云坐标变换**：

LiDAR 采集的点在 LiDAR 坐标系下，需要变换到世界坐标系：

```
p_world = R_wi × p_lidar + t_wi
```

- **`p_lidar`**：LiDAR 坐标系下的点坐标
- **`R_wi`**：当前估计的旋转矩阵
- **`t_wi`**：当前估计的位置
- **`p_world`**：变换后的世界坐标

**ikd-Tree 最近邻搜索**：

变换后的每个点在 ikd-Tree（增量 KD 树）中搜索最近的 5 个邻近点，用于拟合局部平面。

**平面拟合**：

对 5 个近邻点进行最小二乘平面拟合，得到平面方程：

```
n^T × p + d = 0
```

其中 `n` 为平面法向量，`d` 为距离参数。

**残差计算**：

将变换后的点代入平面方程，得到点到平面的距离作为残差：

```
residual = n^T × p_world + d
```

如果当前的 `R_wi` 和 `t_wi` 非常准确，残差应该接近 0。残差越大，说明当前位姿估计越不准确。

### Step 6: IESKF 位姿修正

收集所有有效点的残差后，通过 **迭代误差状态卡尔曼滤波（IESKF）** 求解最优修正量。

**雅可比矩阵 H**：

对每个残差关于状态量（位置、旋转、速度、偏置等）求偏导，组成观测雅可比矩阵 `H`。

**法方程**：

```
(H^T × H) × Δx = H^T × residuals
```

**伪逆求解**：

```
Δx = (H^T × H)^(-1) × H^T × residuals
```

其中 `Δx` 包含对旋转、位置、速度、偏置等状态量的修正。

**旋转修正**：

```
R_wi = R_wi × Exp(Δθ_correction)
```

- **`Δθ_correction`**：从 `Δx` 中提取的旋转修正向量
- **`Exp()`**：SO(3) 指数映射

### Step 7: 最终旋转矩阵更新

经过多次迭代（通常 3-5 次）后收敛，最终的 `R_wi` 即为当前时刻的最优旋转估计，同时 `t_wi` 也得到了修正。这些结果被封装进 odom 消息发布。

---

## 4. 四元数数据解读

### 4.1 四元数的含义

odom 中的 `orientation` 以四元数 `(x, y, z, w)` 表示旋转，其中：

- **`w`**：标量部分，等于 `cos(θ/2)`，其中 `θ` 是旋转角度
- **`(x, y, z)`**：矢量部分，等于 `sin(θ/2) × (nx, ny, nz)`，其中 `(nx, ny, nz)` 是旋转轴的单位向量

四元数满足约束：`x² + y² + z² + w² = 1`

### 4.2 Rodrigues 旋转公式

给定旋转轴 `n`（单位向量）和旋转角 `θ`，旋转一个向量 `v` 的公式：

```
v_rotated = v × cos(θ) + (n × v) × sin(θ) + n × (n · v) × (1 - cos(θ))
```

这是理解旋转的基础公式，四元数和旋转矩阵都可以从此推导。

### 4.3 四元数 → 旋转矩阵

```
R = | 1-2(y²+z²)   2(xy-wz)     2(xz+wy)   |
    | 2(xy+wz)     1-2(x²+z²)   2(yz-wx)   |
    | 2(xz-wy)     2(yz+wx)     1-2(x²+y²) |
```

### 4.4 旋转矩阵 → 四元数

反向转换公式：

```
w = 0.5 × √(1 + R[0][0] + R[1][1] + R[2][2])
x = (R[2][1] - R[1][2]) / (4w)
y = (R[0][2] - R[2][0]) / (4w)
z = (R[1][0] - R[0][1]) / (4w)
```

注意：当 `w` 接近 0 时需要使用其他公式分支以避免数值不稳定。

### 4.5 欧拉角表示（ZYX 顺序）

四元数也可以转换为欧拉角，FASTLIO2 使用 ZYX 顺序（即先绕 Z 轴旋转 yaw，再绕 Y 轴旋转 pitch，最后绕 X 轴旋转 roll）：

- **Yaw（偏航角）**：绕 Z 轴旋转，表示车辆朝向
- **Pitch（俯仰角）**：绕 Y 轴旋转，表示前后倾斜
- **Roll（横滚角）**：绕 X 轴旋转，表示左右倾斜

**万向锁（Gimbal Lock）**：

当 Pitch 接近 ±90 度时，Yaw 和 Roll 的旋转轴重合，导致一个自由度丢失，这就是万向锁问题。这也是为什么 FASTLIO2 内部使用旋转矩阵或四元数而非欧拉角来表示旋转的原因 —— 四元数和旋转矩阵不存在万向锁问题。

---

## 5. FASTLIO2 算法流程详细解读

### Stage 0: 系统初始化

系统启动后不会立即开始定位，而是先收集 **20 帧 IMU 数据** 进行初始化：

1. **收集静止 IMU 数据**：假设载体静止，收集约 20 帧加速度和角速度数据
2. **重力估计**：对加速度取均值，得到重力方向估计 `g_est`
3. **初始偏置估计**：角速度均值作为陀螺仪初始偏置 `bg_init`
4. **初始协方差设置**：设置状态量的初始不确定性
5. **状态初始化**：`R_wi = I`，`t_wi = 0`，`v = 0`

初始化完成后，系统进入正常工作状态。

### Stage 1: 数据接收与同步（SyncPackage）

```
SyncPackage() → 将 IMU 数据和 LiDAR 点云按时间戳对齐
```

1. 接收 LiDAR 点云（10Hz），记录其起止时间戳
2. 收集该时间段内所有 IMU 数据（200Hz，约 20 帧）
3. 将配对的 {IMU 序列 + LiDAR 点云} 打包为一个处理单元
4. 确保 IMU 数据的时间覆盖范围包含整个 LiDAR 扫描周期

### Stage 2: IMU 前向积分 + 位姿序列生成

对 Stage 1 中收集的每帧 IMU 数据，逐帧执行前向积分：

1. **去偏置**：`ω_true = gyro - bg`，`a_true = acc - ba`
2. **旋转更新**：`R_wi = R_wi × Exp(ω_true × dt)`
3. **加速度转换**：`a_world = R_wi × a_true + g`
4. **速度更新**：`v = v + a_world × dt`
5. **位置更新**：`t_wi = t_wi + v × dt + 0.5 × a_world × dt²`
6. **保存中间位姿**：每帧 IMU 对应的 `{R_wi, t_wi}` 保存为位姿序列

这一步生成的位姿序列有两个用途：
- 最后一个位姿作为 IESKF 的预测值
- 中间位姿用于下一步的点云去畸变

### Stage 3: 点云去畸变（De-distortion）

**为什么需要去畸变？**

LiDAR 扫描一帧需要约 100ms，在此期间载体在持续运动。因此同一帧点云中不同时刻采集的点，实际上对应不同的载体位姿。如果不做修正，点云会出现"拖影"失真。

**去畸变步骤**：

1. **获取点云时间戳**：每个点带有相对于帧起始的时间偏移 `t_offset`
2. **查找对应位姿**：根据 `t_offset` 在 Stage 2 生成的位姿序列中插值，得到该点采集时的位姿 `{R_i, t_i}`
3. **变换到统一坐标系**：将每个点从其采集时刻的坐标系变换到帧结束时刻的坐标系
4. **公式**：`p_corrected = R_end^(-1) × (R_i × p_lidar_i + t_i - t_end)`
5. **输出**：所有点统一到同一时刻的坐标系，消除运动畸变

### Stage 4: IESKF 迭代优化（核心）

这是 FASTLIO2 最核心的步骤，通过迭代优化修正 IMU 积分的累积误差：

#### Sub-step 1: 点云变换到世界坐标系

```
p_world = R_wi × p_lidar + t_wi
```

使用当前最优位姿估计（第一次迭代使用 IMU 积分的预测值）将去畸变后的点云变换到世界坐标系。

#### Sub-step 2: KNN 最近邻搜索

对每个变换后的点，在 ikd-Tree 中搜索 **5 个最近邻点**。ikd-Tree 是一种增量式 KD 树，支持高效的动态插入和删除操作。

#### Sub-step 3: 平面拟合

对 5 个近邻点进行最小二乘平面拟合：

```
n^T × p + d = 0
```

如果拟合质量不够好（如近邻点分布太散），该点被标记为无效，不参与后续优化。

#### Sub-step 4: 计算雅可比矩阵

对残差 `r = n^T × (R_wi × p_lidar + t_wi) + d` 关于状态量求偏导：

- 对位置 `t_wi` 的偏导：`∂r/∂t = n^T`
- 对旋转 `R_wi` 的偏导：`∂r/∂θ = -n^T × R_wi × [p_lidar]×`（其中 `[p_lidar]×` 是反对称矩阵）

组成观测雅可比矩阵 `H`。

#### Sub-step 5: 求解修正量

```
K = P × H^T × (H × P × H^T + R_noise)^(-1)
Δx = K × residuals
```

或等价地通过法方程求解伪逆：

```
Δx = (H^T × R_noise^(-1) × H + P^(-1))^(-1) × H^T × R_noise^(-1) × residuals
```

#### Sub-step 6: 状态更新

```
R_wi = R_wi × Exp(Δθ)
t_wi = t_wi + Δt
v = v + Δv
bg = bg + Δbg
ba = ba + Δba
```

#### Sub-step 7: 收敛检查

检查修正量 `Δx` 的范数是否小于阈值：
- 如果 `||Δx|| < threshold`，收敛，退出迭代
- 否则回到 Sub-step 1，使用更新后的位姿重新进行点云变换和残差计算
- 通常 **3-5 次迭代**即可收敛

**关键理解：为什么只优化最终位姿？**

FASTLIO2 不会回头优化中间的 IMU 积分位姿，只优化 LiDAR 帧结束时刻的位姿。原因是：
- IMU 积分的相对旋转精度很高（短时间内陀螺仪漂移很小）
- 主要误差来自长期累积，通过修正最终位姿就能有效消除
- 逐帧修正效率远高于全局优化
- 修正后的偏置估计 `ba`、`bg` 会传递给下一帧的 IMU 积分，间接提高后续积分精度

### Stage 5: 地图更新

优化收敛后，将当前帧的点云加入全局地图：

1. **ikd-Tree 修剪**：删除距离当前位置过远的点（维护局部地图大小）
2. **增量添加**：将当前帧去畸变后的点（变换到世界坐标系）增量式插入 ikd-Tree
3. ikd-Tree 自动平衡，保持查询效率

### Stage 6: 结果发布

1. **TF 发布**：发布 `odom → base_link` 的坐标变换
2. **odom 发布**：发布 `nav_msgs/Odometry` 消息，包含位置和姿态
3. **点云发布**：发布当前帧点云（世界坐标系下）和全局地图点云
4. **路径发布**：发布历史轨迹（`nav_msgs/Path`）

---

## 6. 算法核心总结

```
FASTLIO2 = IMU高频积分预测 + LiDAR每帧校正 + 迭代优化 + 增量建图
```

- **IMU 高频积分预测**：200Hz 提供连续的位姿估计，填补 LiDAR 帧间空白
- **LiDAR 每帧校正**：10Hz 通过点云匹配修正 IMU 积分累积误差
- **迭代优化（IESKF）**：多次迭代使位姿估计收敛到最优值
- **增量建图（ikd-Tree）**：实时维护局部地图，支持高效最近邻查询

---

## 7. Corridor 模式 ESKF 安全防护（commit `308fe77`）

在 GPS corridor 实车测试中发现 FAST-LIO2 本地 odom 会在持续 recovery/backup 后发散（`odom→base_link` 单步跳变达 2.43m/0.11s）。为此部署了以下 ESKF 层面的安全防护：

## 7. Corridor 模式 ESKF 安全防护与 Jacobian Bug 修复

### 7.0 致命 Jacobian Bug 修复（commit `e4945f4`）

**这是 odom 发散的根因**，优先级高于以下所有防护措施。

**Bug 位置**: `lidar_processor.cpp:245`

```cpp
// 修复前（错误）:
hat(state.r_il * laser_p_vec + state.t_wi)   // t_wi = 世界位置，~50m
// 修复后（正确）:
hat(state.r_il * laser_p_vec + state.t_il)   // t_il = 外参偏移，~0.04m
```

**根因**: fork 将原始 HKU-Mars FAST-LIO2 的 `SKEW_SYM_MATRX` 宏重构为 `Sophus::SO3d::hat()` 时，错误引用了 `state.t_wi`（IMU 在世界坐标系的位置）而非 `state.t_il`（LiDAR 到 IMU 的外参平移）。

**数学推导**: 点到平面残差对旋转的 Jacobian（右扰动）：
```
∂h/∂δθ = -n^T * R_wi * hat(R_il * p_lidar + t_il)
```
`hat()` 参数是点在 IMU 坐标系下的位置（旋转的"杠杆臂"），与世界位置 `t_wi` 无关。使用 `t_wi` 导致杠杆臂随行驶距离线性增长，旋转 Jacobian 被放大数百倍。

**影响链**: 错误 Jacobian → IESKF 旋转修正错误 → `r_wi` 估计漂移 → 累积点云地图随车旋转 → odom 发散

### 7.1 有效特征点阈值检查

**文件**: `lidar_processor.cpp`, `ieskf.cpp`

当一帧中有效匹配特征点数 `effect_feat_num < 50` 时，跳过整个 IESKF 更新步骤。此时 IMU 积分继续提供位姿预测，但不用低质量的点云观测去修正状态。

### 7.2 Invalid Measurement 保护

**文件**: `ieskf.cpp`

原代码中，即使观测结果为无效值（NaN / Inf），仍会覆盖协方差矩阵 `m_P`。修改后，invalid measurement 不再更新 `m_P`，防止协方差被污染导致后续帧快速发散。

### 7.3 协方差矩阵对角线 Clamp

**文件**: `ieskf.cpp`

在每次 IESKF 更新后，对 `m_P` 的对角线元素做 clamp：
- 下界：防止协方差过小导致滤波器过于自信，失去对新观测的响应能力
- 上界：防止协方差爆炸（例如在退化场景中）

### 7.4 退化方向正则化

**文件**: `ieskf.cpp`

对 `shared_data.H`（观测雅可比矩阵的信息矩阵）进行特征值分析。当某方向特征值过小（退化方向），在该方向上注入正则化项，防止该方向上的修正量过大导致位姿跳变。

### 7.5 探测距离修正

**文件**: `lio.yaml`

`det_range: 60 → 30`。缩小有效探测范围，减少远距离低质量点参与匹配，提高近距离匹配质量。

### 7.6 Odom 发散 Watchdog

**文件**: `gps_route_runner_node.py`

在 runner 侧增加 odom 发散检测：
- 每个控制周期查询 `odom→base_link` TF 与上一帧比较
- 单步位移 > 0.5m：`WARNING` 日志
- 单步位移 > 1.0m：执行 `ODOM_DIVERGENCE_ABORT`，安全终止导航

实车验证（session `2026-03-27-18-43-20`）：watchdog 在 22ms 内检测到 2.45m 跳变，正确触发安全终止。
