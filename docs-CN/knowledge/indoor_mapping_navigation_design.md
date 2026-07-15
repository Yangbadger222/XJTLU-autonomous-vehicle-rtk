# SLAM 建图与室内导航设计开发方案

> 状态：阶段 1-5 代码已实现；阶段 0 实车基线证据、Jetson 构建、参数标定与实车验收待完成
> 日期：2026-07-11
> 范围：`slam` 建图模式与 `travel` 室内先验地图导航模式
> 暂不包含：室内外自动切换、楼层切换、电梯联动和室外 RTK 导航算法

## 1. 目标与边界

本方案把当前实验性链路收敛为两个可以独立验收的产品能力：

1. `slam` 生成同一次建图会话的 2D 导航地图、3D 定位地图、关键帧和可验证的地图包。
2. `travel` 在地图包上完成初始化、重定位、静态全局规划、动态局部避障、平滑控制和地点名导航。

当前阶段继续保留 RViz `2D Pose Estimate` 和 `Nav2 Goal` 作为调试入口。最终用户入口是地点名导航，但不在本阶段实现室内外模式切换。

设计原则：

- Slam Toolbox 的 2D OccupancyGrid 是 Nav2 的权威导航地图。
- PGO 优化后的 3D PCD 和关键帧是三维全局重定位的权威地图。
- 两张地图不假设天然同坐标系，地图包必须保存并验证 `T_map_2d_map_3d`。
- Travel 运行时只有 `prior_map_tf_authority` 可以发布室内 `map -> odom`；localizer 和 AMCL 只提供候选，FAST-LIO2 发布 `odom -> base_footprint`。
- 定位质量未通过、定位丢失或 TF 所有权冲突时，车辆不得输出运动命令。
- 车端只用于构建、运行和实车验证，不直接修改代码。

## 2. 当前基线

### 2.1 已实现的软件基线

- `system_slam.launch.py` 同时运行 FAST-LIO2、PGO、点云转 LaserScan、Slam Toolbox 和地图保存服务。
- `slam_toolbox_mapping.yaml` 已将 Humble async mapper 基线纳入仓库，并统一使用 `base_footprint`。
- FAST-LIO2 独立发布 `/fastlio2/body_cloud_localization`，与低窗 LaserScan 云和 Nav2 障碍云分离；PGO 与 localizer 使用同一定位结构云。
- `save_mapping_session.py` 生成版本化室内地图包：2D map、pose graph、原始/降采样 3D PCD、关键帧、Scan Context 索引、区域、地点、2D↔3D 标定报告和叠加图。
- 保存流程具有连续静止、底盘反馈、frame、patch/pose、配准 RMSE/p95/重叠率和文件完整性门槛；失败返回非零且 `consistency_ok=false`。
- `system_travel.launch.py` 以 `map_bundle` 为主入口，拒绝 schema 不支持、整体门槛未通过、标定未接受或文件缺失的地图包。
- localizer 支持 RViz 初值、区域辅助和 Scan Context 多候选自动定位，并把锁存的初始 `map -> odom` 候选交给 authority；AMCL 根据 2D 静态地图提供运行中校正候选。
- Travel 的全局代价地图只使用静态地图；局部代价地图使用实时点云。
- Travel 使用真实矩形 footprint、MPPI、Collision Monitor 和有限恢复行为树；正常 MPPI 不倒车，恢复树允许一次经过 footprint 碰撞检查的 `0.20m / 0.08m/s` 低速后退来腾出转向空间，定位/传感器/authority 异常仍会在串口前切断速度。
- Travel 控制周期使用 `15Hz`，与 MPPI `model_dt=0.0666667s` 匹配。
- `indoor_navigation_manager` 提供 `NavigateNamedDestination` Action，支持地图/backend 校验、别名、反馈、取消、并发拒绝和定位降级取消。

### 2.2 尚待实车完成

- 在 Jetson Orin NX 使用 `--parallel-workers 1` 完成 ROS 接口/C++ 全量构建和 30 分钟资源测试。
- 用真实 PCD 收口定位高度窗、voxel、ICP score/重叠率、候选差距和 2D↔3D 配准门槛。
- 测量车体最外沿和 STM32 最小有效速度，确认当前 `650x500mm + 25mm` footprint 与 Collision Monitor 区域。
- 采集区域/任意起点数据集并统计 top-k recall、误定位率、启动耗时和重复走廊拒绝率。
- 完成窄门、原地转向、人员横穿、持续遮挡、定位丢失停车与十次路线往返验收。

## 3. 目标架构

```text
建图：
Livox + IMU
    -> FAST-LIO2 -> odom -> base_footprint
                  -> navigation slice -> LaserScan -> Slam Toolbox -> 2D map
                  -> localization cloud -> PGO -> 3D map + patches + poses

保存与校准：
2D map + 3D map
    -> offline SE(2) registration
    -> alignment metrics + T_map_2d_map_3d
    -> versioned indoor map bundle

导航：
localization cloud + FAST-LIO2 odom
    -> place candidate retrieval
    -> coarse registration
    -> fine ICP/GICP
    -> compose T_map_2d_odom
    -> localization quality gate
    -> prior-map TF authority <- bounded AMCL correction
    -> Nav2 static planner + MPPI + local obstacle costmap
    -> velocity smoother -> serial_twistctl -> STM32
```

## 4. 坐标系与 TF 所有权

地图资产中使用两个逻辑坐标系：

- `map_2d`：Slam Toolbox 2D 地图坐标系；运行时对 Nav2 暴露为 `map`。
- `map_3d`：PGO 3D 定位地图坐标系；只用于地图资产和定位计算，不直接成为第二个运行时 TF 根。

地图包保存：

```text
T_map_2d_map_3d
```

3D localizer 求得 `T_map_3d_odom` 后，运行时输出：

```text
T_map_2d_odom = T_map_2d_map_3d * T_map_3d_odom
```

生产 TF 链必须保持：

```text
map -> odom -> base_footprint -> base_link -> sensor frames
```

所有权：

| TF | 唯一发布者 |
|---|---|
| `map -> odom` | `prior_map_tf_authority` |
| `odom -> base_footprint` | FAST-LIO2 |
| `base_footprint -> base_link` 与传感器静态 TF | robot_state_publisher |

localizer、AMCL 和 PGO 在 Travel 中不得直接广播 `map -> odom`。localizer 通过 `/localizer/map_to_odom` 提供锁存种子，AMCL 通过 `/amcl_pose` 提供候选；启动测试和运行时健康检查都要验证 authority 是唯一 TF 发布者。

## 5. 室内地图包契约

建议目录：

```text
runtime-data/maps/indoor/<map_id>/
  manifest.yaml
  navigation/
    map.yaml
    map.pgm
    slam_toolbox.posegraph
    slam_toolbox.data
  localization/
    map_raw.pcd
    map_localization.pcd
    poses.txt
    patches/
    descriptor_index/
  calibration/
    map_3d_to_map_2d.yaml
    alignment_report.yaml
    alignment_overlay.png
  destinations.yaml
  regions.yaml
```

manifest 最少记录：

```yaml
schema_version: 1
map_id: building_a_floor_1
created_at: "..."
git:
  branch: "..."
  commit: "..."
frames:
  navigation: map_2d
  localization: map_3d
artifacts:
  navigation_map: navigation/map.yaml
  localization_map: localization/map_localization.pcd
  raw_map: localization/map_raw.pcd
calibration:
  file: calibration/map_3d_to_map_2d.yaml
  accepted: true
quality:
  frame_check_ok: true
  patch_pose_integrity_ok: true
  alignment_wall_rmse_m: 0.0
  alignment_wall_p95_m: 0.0
  map_coverage_ok: true
```

Travel 必须拒绝缺文件、schema 不支持或 `calibration.accepted=false` 的地图包。运行时参数不再要求操作员分别手写不相关的 `map_yaml` 和 `pcd_map`；最终入口应只接收 `map_bundle` 或 `map_id`。

## 6. SLAM 建图模式设计

### 6.1 点云职责拆分

不得继续用同一个低高度窗口同时承担建图、重定位和避障。目标话题职责：

| 点云 | 内容 | 消费者 |
|---|---|---|
| navigation slice | 接近车体碰撞高度的稳定结构切片 | pointcloud_to_laserscan、Slam Toolbox |
| localization cloud | 保留墙面、门框、立柱等垂直结构的较宽高度窗 | PGO、localizer、描述子构建 |
| Nav2 obstacle cloud | 当前 `[-0.20, 1.20]m` 近场障碍语义 | local costmap、Collision Monitor |

具体高度窗必须通过实车 PCD 统计确定，不能仅凭文档推测。建图和导航使用的 localization cloud 过滤规则必须相同。

建图投影在 `base_footprint` 中启用矩形自滤波：实测车体 `650x500mm` 每侧增加 `25mm` 余量，因此排除 `x=[-0.35,0.35]m, y=[-0.275,0.275]m` 内的点。矩形优于单一 `range_min`：它不会为了覆盖车体四角而同时丢掉车头/车侧矩形外的有效近场墙面。过滤只作用于建图 `/scan`，不改 FAST-LIO2 内部匹配、PGO 定位结构云或 Travel/Corridor 活障碍云。风险是落入车体外廓内的真实悬垂物也会被排除，因此外廓只能来自实测尺寸和小余量，不能用 Nav2 膨胀半径代替；重建后仍需检查窄门、桌腿和低矮障碍。

### 6.2 Slam Toolbox 参数归档

新增仓库内 `slam_toolbox_mapping.yaml`，不再直接依赖系统安装目录的默认 YAML。至少固定：

- `map_frame/odom_frame/base_frame`，其中 `base_frame=base_footprint`。
- 地图分辨率、最大测距和更新周期。
- 最小位移/转角建图阈值。
- scan matching、回环检测、scan queue 和 TF 发布周期。
- 序列化 pose graph 的输出约定。

参数改变必须同时更新本设计文档、Nav2/SLAM 知识文档和双语 devlog。

### 6.3 建图过程门控

启动阶段：

1. 检查 Livox、IMU、FAST-LIO2 odom 与 TF。
2. 等待 IMU 初始化和 LIO 状态稳定。
3. 确认只有 Slam Toolbox 发布建图时的 `map -> odom`。
4. 再允许操作员移动小车。

采集阶段：

- 低速、匀速覆盖主要走廊、房间入口、门框和回环区域。
- 同一区域至少从不同方向经过，避免只采到单侧结构。
- 记录 LIO 退化、PGO 回环、CPU/GPU、温度和丢帧状态。
- 动态人群严重时暂停建图，避免把临时障碍写入长期地图。

保存阶段：

1. 车辆连续静止至少 2 秒。
2. `/cmd_vel` 为零，底盘反馈速度在阈值内。
3. FAST-LIO2 和 TF 时间戳新鲜。
4. PGO 不在提交回环优化，关键帧数量与覆盖范围达到门槛。
5. 保存 2D、3D、pose graph、patches 和会话元数据。
6. 生成降采样 `map_localization.pcd`。
7. 执行 2D↔3D 配准和质量检查。
8. 只有全部硬门槛通过才标记地图包 `accepted=true`。

### 6.4 2D↔3D 配准

2D 地图继续使用 Slam Toolbox 的干净 OccupancyGrid，不使用原始 PGO 点云的简单投影替代。

配准工具应：

1. 从 PGM/YAML 提取占据墙面点。
2. 在定位 PCD 的 XY 栅格中按垂直跨度提取墙面：默认 `0.06m` 栅格、至少 3 点、z 跨度 `>=0.50m`，以排除单高度地面/家具投影。
3. 使用多初值平面配准求 `SE(2)`；由于两套地图在同一会话从共同原点启动，必须显式包含 identity 初值，同时保留主轴/质心候选处理非零偏移。
4. 输出第一、第二候选分数和最终变换。
5. 计算全图以及分区 RMSE、p95 墙面距离和重叠率。
6. 生成可视化叠加图供人工复核。

初始建议门槛，必须由首批实车地图校准后收口：

- 全图墙面 RMSE `<= 0.10m`。
- p95 墙面距离 `<= 0.15m`。
- 主要门框/拐角不得出现局部明显错位。
- 第一候选必须明显优于第二候选，否则要求人工确认。

单个刚体变换无法修复两套图优化器产生的非线性形变。若分区误差无法通过，应缩小单张地图范围、重建地图或改用共享轨迹优化方案，不能仅放宽阈值。

质量计算使用完整墙面样本（上限 12000 点）和 0.50m 半径的精确空间桶最近邻；`alignment_report.yaml` 必须记录墙面提取方法、输入点数、输出单元数和参数，叠加图必须使用同一批墙面点。

## 7. Travel 室内导航设计

### 7.1 启动与定位状态机

```text
UNINITIALIZED
  -> MAP_LOADING
  -> WAITING_FOR_SENSORS
  -> GLOBAL_SEARCHING or WAITING_FOR_INITIAL_POSE
  -> LOCALIZED
  -> DEGRADED
  -> RELOCALIZING
  -> LOST
```

只有 `LOCALIZED` 允许导航。`DEGRADED/RELOCALIZING/LOST` 必须取消当前目标并持续输出零速度。

定位状态至少发布：

- 当前状态与原因。
- 地图 ID。
- 输入点云/odom 新鲜度。
- 粗配准和精配准 score。
- 点云重叠率。
- 当前修正量及其变化率。
- 第一、第二全局候选差距。
- 最后一次可信定位时间。

### 7.2 分阶段初始化

阶段 A：保留 RViz `/initialpose`，用于当前调试和基线验收。

阶段 B：区域辅助定位。用户只选择“大厅/东走廊/实验室区域”等粗区域，系统在该子地图内做多初值配准。

阶段 C：自动全局定位。使用 `patches + poses` 构建 Scan Context 或等价描述子索引，检索多个候选，然后执行粗配准和 GICP/ICP 精配准。

自动定位不得只取最高分候选；重复走廊中候选不唯一时必须停车并请求区域提示。

### 7.3 运行中防漂移

当前实现使用 3D 初始种子加受限 AMCL 运行时校正：

- FAST-LIO2 提供高频连续局部运动。
- localizer 负责初始 3D 先验地图匹配，不启用会在运动中撤掉 TF 的 continuous ICP。
- AMCL 使用 `/scan`、2D 静态地图和 FAST-LIO2 odom 生成低频先验地图候选；`/scan` 的 `scan_time=0.1s` 与 MID360/FAST-LIO2 实测 10Hz 输出一致。
- authority 对 AMCL 候选检查协方差、odom 时间差和 `0.75m/0.45rad` 目标跳变，再用 5 帧窗口要求至少 3 帧一致并取 SE(2) 中值；窗口平移/yaw 分散度上限为 `0.05m/0.04rad`。
- 稳定候选最多每秒接受一次，平移和 yaw 分别使用 `0.05m/0.035rad` 死区；AMCL 回调只更新目标，20Hz TF 发布周期再以 `alpha=0.15` 和 `0.015m/s` 平移、`0.006rad/s` yaw、`0.02m/s` 车体等效位移速度上限连续靠近，原有 `0.03m/0.01rad/0.04m` 单步上限保留为第二层硬限制。
- authority 额外限制当前车体在 `map` 中的单步等效位移不超过 `0.04m`，避免车辆远离 odom 原点后 yaw 修正被杠杆臂放大。
- 大跳变、候选切换或低重叠结果不得直接写入 TF。
- 室内 `map -> odom` 必须投影为平面 `SE(2)`：只保留 XY 和 yaw，禁止 ICP 把 z、roll 或 pitch 写入 Nav2 全局 TF。
- `/initialpose` 会显式进入手动重定位 pending，保持 TF 可视但令 authority `tf_active=false`，直到 localizer 接受新种子。已有可信 TF 后，后台 localizer 的 `ambiguous_global_candidates` 只降级保持，不得再次直接播种；AMCL 仍可继续提供运行中校正。
- 大修正需要停车、多帧一致后重新定位。
- 所有门控基于 `map -> base_footprint` 的实际影响，而不只看原始 `map -> odom` 数值。

### 7.4 规划与控制

当前路线继续使用：

```text
NavFn/A* -> Savitzky-Golay SmoothPath -> MPPI -> velocity_smoother
```

当前实现：

- `controller_frequency=15Hz`，与 `model_dt=0.0666667s` 匹配。
- MPPI `batch_size=128`、`time_steps=24`，保持约 1.6 秒预测时域，把每周期候选点降到 3072，降低 Orin NX 单线程控制周期超时。
- 根据 `650 x 500mm` 整车尺寸和实测最外沿配置 polygon footprint，并增加安全余量。
- MPPI `CostCritic.consider_footprint=true`。
- 平滑路径执行碰撞检查，避免直角墙角切角。
- 标定 STM32 最小有效 `vx/wz`，再配置速度 deadband。

小车具备原地转向能力，因此控制器保持 `DiffDrive` 运动模型和零线速度转向采样。Rotation Shim 只处理 `>0.65rad` 的大初始偏差，并用开环命令爬升越过静摩擦区；普通转弯和终点朝向由 MPPI 处理。VelocityDeadbandCritic 降到权重 `15` 和 `[0.12,0,0.08]`，防止与 Shim 抢控制。速度后处理不再放大纯旋转角速度，只在确认碰撞减速后移除无法越过线速度死区的平移分量。进度检查仍接受 `0.15rad` 角度变化。

### 7.5 动态障碍与安全

- 全局 costmap 只包含静态地图和 `0.30m` 静态膨胀，不把临时人员写入全局地图；该值仍高于 `0.285m` 内切半径。
- local costmap 使用 Nav2 专用障碍点云，负责人员、椅子和临时障碍。
- 增加 Collision Monitor 独立实现减速区和停车区。
- Travel 使用有界恢复：控制失败先清 local costmap 并立即重规划；仍失败时先尝试一次经 local footprint 碰撞预判的 `0.20m / 0.08m/s` 短后退并再次规划，之后才允许 `IsStuck` 条件下的 `0.52rad` 短 Spin，最后等待或清双图。global static layer 只清机器人当前 footprint 下的旧静态残留，现场障碍仍由 local costmap、MPPI 和 Collision Monitor 阻挡。
- PS2 `X` 是最高优先级软件失能；红色物理急停覆盖全部软件。
- 定位状态必须进入速度输出安全链，不能只在界面提示。

### 7.6 地点名导航

地点数据与算法解耦：

```yaml
schema_version: 1
map_id: building_a_floor_1
destinations:
  lab_101:
    display_name: 实验室 101
    aliases: [实验室, lab 101]
    pose: {x: 12.4, y: 6.8, yaw: 1.57}
    approach: forward
```

新增 ROS 2 Action，而不是只使用 String topic：

```text
NavigateNamedDestination
  goal: map_id, destination_name
  feedback: state, resolved_name, remaining_distance, localization_state
  result: success, error_code, message
```

第一版只调用室内 `NavigateToPose`。接口中保留 `map_id` 和 backend 字段，未来室外算法完成后再增加切换，不在本阶段实现。

Foxglove 操作面通过独立适配器接入，不绕过 Action 和安全门：`PoseStamped` 点击目标转 `NavigateToPose`，地点字符串转 `NavigateNamedDestination`，Trigger Service 取消当前目标。适配器只在结构化定位状态健康时发目标，发布地点 MarkerArray/目录/状态，不得订阅或发布 `/cmd_vel*`。

命名导航的取消必须覆盖底层 Nav2 goal 尚未返回 handle 的窗口；取消请求需要锁存，并在 handle 接受后立即下发。异常、拒绝和取消路径都必须释放并发占用。

速度链采用三层断流保护：定位速度门要求定位/障碍点云/速度命令均新鲜，主机串口节点 300ms 断流重发零速，STM32 500ms 未收到合法串口命令时独立把 `Vcx/Wc` 清零。Collision Monitor 的传感器 source timeout 本身不是停车保证，因此障碍点云 freshness 必须在其上游 fail-closed 门中检查。

## 8. 开发阶段与文件范围

### 阶段 0：固定基线与实车证据

- 已为 `system_slam.launch.py` 增加 launch contract 测试。
- 已固定 Slam Toolbox 参数文件并统一 `base_frame=base_footprint`。
- 记录一份当前 SLAM/Travel 基准 bag 和地图包。
- 已修复 Travel 20Hz 控制周期问题。

验收：现有手动初始化流程不回归，SLAM/Travel 均能在 Jetson 使用 `--parallel-workers 1` 构建运行。

### 阶段 1：正式地图包（代码已完成）

实现范围：

- `src/bringup/launch/system_slam.launch.py`
- `src/bringup/config/slam_toolbox_mapping.yaml`
- `src/perception/fastlio2/`
- `scripts/save_mapping_session.py`
- 新增地图配准/验收工具和测试

验收：单个命令生成完整地图包；失败门槛会返回非零；Travel 可通过 `map_bundle` 启动。

### 阶段 2：Travel 安全与控制闭环（代码已完成）

实现范围：

- `src/bringup/config/nav2_travel.yaml`
- `src/bringup/behavior_trees/`
- `src/bringup/launch/system_travel.launch.py`
- Collision Monitor 配置与 launch 测试

验收：直线、90 度弯、原地转向、窄门、人员横穿和 U 型障碍测试通过；定位失败时可靠停车。

### 阶段 3：定位状态与低频防漂移（代码已完成）

实现范围：

- `src/perception/localizer/`
- 新增定位状态消息/诊断
- 新增 TF 平滑与跳变门控测试

验收：长走廊往返后静态地图对齐误差保持在约定范围；错误配准不会直接写入 TF。

### 阶段 4：任意起点全局定位（代码已完成）

- 构建描述子索引。
- 多候选检索、粗配准、精配准和歧义拒绝。
- 支持区域提示降级。

验收：在预定义未知起点集合上统计 top-k recall、成功率、误定位率和启动耗时；误定位率必须优先于成功率优化。

### 阶段 5：地点名导航（代码已完成）

- 增加地点 schema、加载器和 Action server。
- 增加取消、重复请求、未知地点和定位降级测试。

验收：不使用 RViz 目标工具，仅输入地点名即可完成多地点往返。

## 9. 测试矩阵与指标

| 类别 | 场景 | 核心指标 |
|---|---|---|
| 地图 | 闭环走廊、房间入口、玻璃区域 | 2D/3D 配准误差、覆盖率、重影 |
| 初始化 | 已知初值、粗区域、任意起点 | 成功率、误定位率、耗时、候选差距 |
| 定位 | 直线往返、长时间运行、临时遮挡 | 漂移、TF 单步变化、丢失恢复时间 |
| 规划 | 直线、90 度、窄门、U 型障碍 | 规划成功率、切角、重规划次数 |
| 控制 | 原地转向、贴线、终点停车 | 横向误差、角速度换向频率、停车误差 |
| 动态障碍 | 人员横穿、持续阻挡、障碍移除 | 停车距离、恢复时间、错误清障率 |
| 资源 | 30 分钟连续运行 | CPU/GPU/RAM、温度、控制周期超时 |

正式验收至少包含 10 次相同路线往返，失败必须保留 session 日志和对应地图版本，禁止只记录成功视频。

## 10. 主要风险与决策门

1. **2D/3D 非刚性错位**：如果单个 `SE(2)` 无法通过分区误差门槛，必须分图或统一优化后端。
2. **重复走廊误定位**：全局定位必须支持候选歧义拒绝和区域提示。
3. **动态物体污染地图**：地图保存前需要人工巡视和自动离群/时序过滤。
4. **PGO 地图过大**：原始归档与定位降采样地图分离，Orin NX 只加载定位版本。
5. **底盘低速死区**：MPPI 调参前先完成命令与实际速度辨识。
6. **TF 跳变**：大修正必须停车、多帧一致并平滑释放。
7. **过早引入室内外切换**：本方案完成前不扩展模式切换，避免同时调试两个定位权威源。

## 11. 完成定义

代码侧阶段 1-5 已完成；室内链路达到以下实车条件后，才进入室内外切换设计：

- 建图命令可重复生成版本化且验收通过的地图包。
- 2D 导航地图和 3D 定位地图的关系被显式记录并严格校验。
- 任意位置或区域辅助定位可用，歧义时能够拒绝而不是误定位。
- 运行中漂移有低频校正，定位异常会可靠停车。
- 真实 footprint、MPPI、局部障碍和 Collision Monitor 通过实车测试。
- 地点名导航不依赖 RViz，可以取消、反馈状态并正确到达。
- 双语文档、测试、实车日志和 Jetson 资源指标齐全。
