# 已知问题追踪

## 当前阻塞

34. **[重要] GPS Corridor v3 普通 GNSS 坐标精度不足，Nav2 目标物理点不准**
   - 描述: 2026-05-08 v3 路线实车测试中，系统能通过 startup guard 并进入 `RUNNING_ROUTE`，但用户实车观察到 GPS 转换后的坐标在 Nav2 中不对应正确物理点。
   - 直接证据:
     - `2026-05-08-17-43-03`: startup `distance_to_start_ref=1.83m`、`spread=0.06m`，进入 `RUNNING_ROUTE`，后续持续 alignment preempt，并出现 `Failed to make progress`
     - `2026-05-08-17-50-10`: startup `distance_to_start_ref=3.25m`、`spread=0.36m`，进入 `RUNNING_ROUTE`，第一段 subgoal 从 `x=33.03 y=-0.67` 经 alignment shift 重算到 `x=46.69/-0.96`、`x=48.64/-1.46`、`x=48.35/-1.92`
     - PGO/FAST-LIO2 持续收到 cloud + odom，未见点云地图、里程计或 `map -> odom` 发散证据
     - 100x100 global costmap 已部署，v2 的 `goal off the global costmap` / `worldToMap failed` 问题未在 v3 复现
   - 状态: 2026-05-08 记录。当前 runner、costmap、RViz 部署修复已收口；剩余问题归因到普通 `/fix` 绝对坐标与 ENU -> map 锚定/换算精度。
   - 影响: 不能把当前普通 GNSS corridor 宣称为物理路径精确可用；后续应进入 RTK / robot_localization / 多点配准 / map 物理点路线等定位精度方案阶段。

1. **[致命] `feature/gps-route-ready-v2` 首轮实车 `nav-gps` 导航会在执行中失效**
   - 描述: 用户已采真实 `ls-building` scene，并完成 `current_scene/` 编译；`nav_gps_menu.py` 可正常进入 `GOAL_REQUESTED -> COMPUTING_ROUTE -> FOLLOWING_ROUTE`，但车辆只短暂动作后即停止。
   - 直接证据:
     - `goal_manager`: `FAILED; follow_path_status=6`
     - `controller_server`: `Controller patience exceeded`
     - launch 日志: `pgo_node ... process has died ... exit code -11`
   - 已确认影响链:
     - `pgo_node` 段错误退出
     - `map -> odom` TF 消失，只剩 `odom -> base_link`
     - `/gps_system/status` 从 `NAV_READY` 退回 `GNSS_READY`
     - Nav2 `follow_path` 被 abort
   - 状态: 2026-03-20 已复现并锁定为当前最高优先级 blocker
   - 影响: 新 GPS 路网导航模式当前不能通过实车验收

2. **[已验证] 室外 GNSS RF / fix 质量**
   - 描述: GPS 天线馈线已更换，设备枚举正常。
   - 状态: 2026-03-22 多轮 corridor v2 室外实车中 GPS fix 稳定工作，启动定位和 PGO 对齐均正常使用 `/fix`
   - 影响: 不再是 blocker

3. **[重要] `nav-gps` 已软件落地，但室外实车验证未完成**
   - 描述: `feature/gps-navigation-v4` 已完成 `gps_waypoint_dispatcher`、`nav2_gps.yaml`、固定 ENU 原点和 `system_nav_gps.launch.py`，室内 smoke 已通过；但真正的室外运行、路网扩充和调优还没完成。
   - 状态: 软件已完成，等待室外验证
   - 影响: 不能把 GPS 目标导航能力宣称为最终实车稳定功能

4. **[致命] 速度平滑器闭环控制失败**
   - 描述: C 板里程计闭环会导致车辆异常自转；FAST-LIO2 里程计接入闭环后也未得到稳定循迹效果。
   - 状态: 仍使用 `OPEN_LOOP`
   - 来源: 2025-11 系列实车测试

## 重要问题

5. **[重要] 长时间运行后车辆停止**
   - 描述: Nav2 路径和 costmap 仍在更新，但底盘不再运动，疑似串口输出或控制器链路问题。
   - 状态: 未排查完成

6. **[重要] SLAM Toolbox 长走廊内存压力高**
   - 描述: 长时间建图时存在明显内存增长风险。
   - 状态: 未根治

7. **[重要] LiDAR 贴墙退化**
   - 描述: MID360 贴墙时点云退化，对狭窄过道避障不友好。
   - 状态: 仅有经验性规避，暂无彻底方案

8. **[重要] FAST-LIO2 + PGO 内存增长**
   - 描述: 长时间运行时 FAST-LIO2、PGO 关键帧和相关缓存会推高内存占用。
   - 状态: 系统层面已做服务裁剪，但算法层未处理

31. **[已解决] MPPI 直线路径跟踪蛇形修正**
   - 描述: 2026-04-01 晚间高速测试中发现，无障碍直线路径跟踪会出现左-右-左-右蛇形修正
   - 解决方案: 2026-04-02 引入 Savitzky-Golay 路径平滑（BT 中插入 SmoothPath）+ MPPI critic 调优（PathAlignCritic 更早介入、PathFollowCritic 延后退出）
   - 状态: 已解决，用户实测评价"完美成功"（commit `15d4cac`）

9. **[已修复] 代价地图障碍残留 / 消散慢**
   - 描述: 移除障碍后，代价地图上的代价值清除不够快。
   - 状态: 2026-03-26 STVL `clear_after_reading` 已从 `true` 改为 `false`（local + global），障碍由 `voxel_decay` 自然管理，不再每周期清空
   - 2026-03-31 更新（`gps-mppi`）：高度窗口收窄到 `-0.33~0.30m`（车体高度范围），膨胀半径 local `0.43` / global `0.63`，障碍地图扩展到 15m。室内实测假障碍已消除
   - 影响: 已不再是问题

## 中等问题

10. **[中等] Travel 模式开发暂停**
   - 描述: `system_travel.launch.py` 与 `nav2_travel.yaml` 已在仓库内，但该模式当前不作为主开发方向。
   - 状态: 暂停

11. **[中等] GPS 路网仍需继续采集和精修**
   - 描述: 固定 ENU 原点与 dispatcher 路网规划已经软件实现，但 `campus_road_network.yaml` 仍只是 bootstrap 图，需要继续用车载 `/gnss` 采点扩图。
   - 状态: 待继续采集

12. **[中等] GPS 漂移仍会影响全局位姿与终点收敛**
   - 描述: 长距离单向运行时，单靠回环不足以约束全局漂移，因此仍依赖 GPS 因子质量、有效 offset 和户外数据质量。
   - 状态: GPS 因子已上线，现场数据不足

13. **[中等] 无 URDF**
   - 描述: 当前缺少完整 URDF / 仿真链，因此参数验证主要依赖实车。
   - 状态: 未开始

14. **[中等] 原地旋转轨迹不圆**
   - 描述: 机械侧左右轮输出不一致，狭窄区域旋转时风险较���。
   - 状态: 硬件限制

29. **[中等] Rosbag 录制提前终止**
   - 描述: 2026-04-01 GPS corridor 测试中 rosbag 在 11:22:15 停录，但系统运行到 11:31:13，后半段无法回放
   - 状态: 已记录，根因待查

## 低优先级 / 工具链问题

15. **[低] Jetson 上 `gh` token 失效风险**
   - 描述: 已知 Jetson 侧 `gh` 曾出现登录失效；如果复发，可在已登录的 Windows 工作站执行 `gh pr create` / `gh pr merge` 规避。
   - 状态: 目前可用，但需留意

16. **[低] PGO 全局点云 RViz 可视化仍不完整**
   - 描述: 当前主链强调 `map -> odom` 与 `/pgo/optimized_odom`，但全局点云展示体验仍不是主维护目标。
   - 状态: 低优先级

17. **[低] FAST-LIO2 `world_cloud` 实用性有限**
   - 描述: 该输出在当前主链中不是核心依赖，可在后续资源优化时再评估。
   - 状态: 低优先级

18. **[低] 下位机 PID 急停倒转**
   - 描述: 旧固件的 KEY/B/X 停止路径可能停发 CAN 或走 PID 锁零，导致车轮明显反转。
   - 状态: 2026-06-14 已进一步区分停止语义：KEY/X/手柄断连保持持续零电流撤力，B 键改为阻尼主动刹车，高速段保持较高刹车上限，低速段降流并限制电流变化率，接近停止后零电流释放；B 键锁存只在首次触发时初始化，持续按住不会反复清空电流爬坡，且急停提示改为非阻塞粉色常亮；仍需抬轮台架和车测验证。

19. **[低] USB 2.0 接口限制**
   - 描述: 对部分高带宽外设扩展不友好。
   - 状态: 硬件限制

## 最近已修复

32. **[已修复] runtime-data Hugging Face 远程同步受阻**
   - 描述：尝试归档并将车载测试的 runtime-data 推送到 Hugging Face 数据集时失败。
   - 现象：
     - Git/Xet push 报错 `gnutls_handshake() failed`
     - 在代理环境下由于缺少 token，导致 HF API 备用方案因 `SSL EOF` 报错失败
     - 由于未配置密钥，SSH 备用方案失败并报错 `Permission denied (publickey)`
   - 根本原因：未配置访问密钥，且由于本地防火墙问题，Jetson 无法直接连接到 Hugging Face 服务器
   - 修复方案：
     - 使用 `pip install -U "huggingface_hub[cli]"` 安装了 Hugging Face CLI
     - 使用 `export HF_ENDPOINT=https://hf-mirror.com` 将端点修改为镜像源
     - 为团队创建了一个组织（Organization）并使用共享访问 Token，通过 `hf auth login` 重新进行了身份验证
     - 使用 `hf repos create my-rtk-data --type dataset` 创建了一个组织仓库
     - 现在可以使用 `hf upload frogcar/rtk-data-2026-surf ./runtime-data/ --repo-type dataset` 正常上传 rosbag，并可在共享数据库中访问数据：https://huggingface.co/datasets/frogcar/rtk-data-2026-surf/tree/main （仅限团队成员）
   - 状态：已修复并在 Jetson 上完成验证 (2026-06-25)

33. **[已修复] 编译环境缺失 optional 目录导致 bringup 构建失败**
   - 现象: 在 Jetson 上执行全量构建时，`bringup` 包构建失败，报错提示找不到 `maps/` 或 `urdf/` 目录。
   - 根因: `src/bringup/CMakeLists.txt` 中对这些目录的 `install` 指令未做存在性检查。
   - 修复: commit `fe7e546` — 在 `install` 前包裹 `if(EXISTS ...)` 判断，确保目录存在才安装。
   - 状态: 已修复并在 Jetson 上验证通过（2026-04-15）

30. **[已修复] GPS corridor 对齐机制问题**
   - 现象: 2026-04-01 上午测试暴露三个核心问题：(1) startup stable GPS offset (~4.11m) 未被 bootstrap 吸收 (2) waypoint calibration 旋转翻转 (176.95deg) (3) per-waypoint frozen alignment 导致 live alignment 被 guard 拒绝
   - 修复: 2026-04-01 晚间完成四项改动（commit `ebc26e2` + `fe3933e` + `e73c2bf` + `c0ea847`）：
     - Calibration 改为 translation-only，避免旋转翻转
     - 启动时直接吸收 stable GPS offset 到 bootstrap alignment
     - Runner 使用实时 alignment 重新计算 subgoal，移除 per-waypoint frozen 机制
     - 速度上限提升到 1.5 m/s
   - 验证: 晚间室内高速测试通过，高速避障正常
   - 状态: 已修复并部署（2026-04-01）

27. **[已修复] syncPackage 空点云段错误 (Issue #4)**
   - 现象: 雷达被遮挡或所有点超出范围时，`livox2PCL()` 返回空 cloud，`syncPackage()` 对空 `points` 调用 `.back()` 触发段错误
   - 影响链: FAST-LIO2 崩溃 → `odom → base_link` TF 丢失 → 系统定位失效
   - 修复: commit `9a193af` — `syncPackage()` 中增加空点云守卫，空帧直接丢弃
   - 状态: 已修复，已部署，GitHub Issue #4 已关闭（2026-03-31）

28. **[已修复] Startup GPS spread 门槛过严导致 corridor 无法启动**
   - 现象: 首轮实车测试中 `gps_global_aligner` / `gps_route_runner` 停在 `WAITING_FOR_STABLE_FIX`，系统其余部分正常
   - 根因: route 默认 `startup_fix_spread_max_m: 3.0`，但当前 GPS 设备 60 点窗口 spread 典型值 ~4.8m，无法满足
   - 修复: commit `d9b63dc` — 路线采集脚本默认从 `2.0` 放宽到 `5.0`，当前运行 route 从 `3.0` 改为 `5.0`
   - 状态: 已修正，待下次晴天复测验证（2026-03-31）
     - 2026-04-15 更新: 室外 corridor 烟测中代码链路正常启动，但由于现场无有效 GNSS fix（持续输出空 NMEA `$GNGGA,,,,,,0,00,25.5,,,,,,*64`），依旧卡在 `WAITING_FOR_STABLE_FIX`。这是现场环境信号限制，非代码阻塞。

1. **[已修复] PGO 启动后 `map -> odom` 不建立**
   - 现象: 启动后持续刷 `Received out of order message`，没有关键帧，没有稳定 `map -> odom`，RViz 在 `map` fixed frame 下看起来像空白。
   - 根因: `pgo_node.cpp` 中同步状态 `last_message_time` 未初始化，第一对同步消息会被错误判定为 out-of-order。
   - 修复: 将 `last_message_time` 初始化为 `0.0`，确保第一对匹配消息被接受。
   - 状态: 已于 2026-03-18 合并到 `main`

2. **[已修复] GPS 导航主软件链缺失**
   - 现象: 历史版本只有 `GNSS -> PGO GPS factor`，没有把 GPS 目标交给 Nav2 的正式执行链。
   - 修复: 在 `feature/gps-navigation-v4` 上新增 `gps_waypoint_dispatcher`、`nav2_gps.yaml`、固定 ENU 原点、`system_nav_gps.launch.py` 与 `nav-gps` 模式。
   - 状态: 室内软件 smoke 已通过，等待室外最终验证

20. **[已升级] Corridor v1 终点几米级残差 → v2 独立 aligner 已部署**
   - 描述: v1 使用 body_vector 直线导航，受 yaw0 不确定性影响终点偏差 ~4m。
   - 状态: 已升级到 corridor v2（独立 global aligner 架构），v1 作为 baseline 保留
   - v2 当前状态（2026-03-26 收口）:
     - waypoint 1 已稳定到达
     - waypoint 边界 alignment 守卫已生效，不再换坏 alignment
     - **主问题已收敛为 GPS 路线采集/锚定方法**: startup GPS 带 ~2.5m 误差，单点锚定无法保证稳定到预定物理位置
     - 继续只调 Nav2 YAML 或小修 runner 已触及天花板，需重新设计锚定方案

21. **[已解决] Corridor v2 PGO handoff 门槛与现场频率不匹配**
   - 描述: v2 初版使用 PGO live handoff，但 PGO ~1Hz 更新频率达不到切换门槛。
   - 状态: **已通过架构变更解决** — 独立 global aligner 替代了 PGO handoff，不再需要切换门槛
   - 影响: 不再适用

22. **[已修复] Controller / Planner 循环频率不达标**
   - 描述: Controller 反复报 `Control loop missed its desired rate`，Planner 降至 `~2Hz`。
   - 状态: `gps-mppi` 上 MPPI 以 20Hz 运行，室内实测通过，未再报频率不达标
   - 影响: 不再是问题

23. **[已修复] Corridor BT 文件已移除 Spin 但 runtime 仍执行 spin**
   - 描述: 本地和 Jetson 的 corridor BT XML 都不含 `Spin`，但 `behavior_server` 日志仍打印 `Running spin`
   - 根因: `default_nav_to_pose_bt_xml` 参数写在 `bt_navigator_navigate_to_pose_rclcpp_node` 下，不是 `bt_navigator` 下，导致 launch 注入无效
   - 修复: commit `d075c6b` 将参数移到正确的 `bt_navigator` 节点下
   - 状态: 已修复（2026-03-26）

24. **[根因已修复] 后段 `lio_odom` / `odom→base_link` 发散**
   - 描述: 多轮实车中第二段后半程 `odom→base_link` 开始连续大跳，导致位姿发散、导航失败
   - **根因已确认（2026-03-30）**: `lidar_processor.cpp:245` 点到平面 Jacobian 中 `hat()` 参数错误使用 `state.t_wi`（世界位置，~50m）而非 `state.t_il`（外参偏移，~0.04m）
     - 来源：fork 将原始 FAST-LIO2 的 `SKEW_SYM_MATRX` 宏改为 `Sophus::SO3d::hat()` 时变量名写错
     - 影响：旋转 Jacobian 随距离从原点增长被放大数百倍 → IESKF 旋转修正错误 → 地图旋转 → odom 发散
   - 修复: commit `e4945f4` — 一行修复 `state.t_wi` → `state.t_il`
   - 缓解措施保留（commit `308fe77`）: odom watchdog + ESKF 退化保护仍作为通用安全机制
   - 2026-03-31 更新: `gps-mppi` 室内整圈巡航验证中 FAST-LIO2 全程稳定，无 odom 发散、无 IMU 漂移、无点云漂移。Jacobian 修复在当前基线上未引入回归
   - 状态: 根因已修复，室内验证通过，待户外长距离验证

25. **[重要] GPS 路线采集/锚定方法不足以保证物理精度**
   - 描述: 当前 corridor GPS 路线依赖单个 `start_ref` + `launch_yaw_deg` 锚定。startup GPS 本身带 ~2.5m 误差，加上路线几何只在 ENU 域定义，导致 map 中生成的目标线系统性偏离用户期望的物理路径
   - 最新证据（session `2026-03-27-13-43-46`）:
     - `distance_to_start_ref=4.75m`（tolerance 15.0m 允许了过大偏差起跑）
     - frozen alignment 下第二段 map 投影 dx=+3.28m 侧偏
     - translation-only aligner 持续拒绝修正（delta 8.88~9.50m > 8.0m 阈值）
   - Calibration handshake 尝试（commit `308fe77`）:
     - 部署航点渐进标定机制：runner 到达 waypoint 后请求 aligner 用静止 GPS 样本重新标定
     - session `2026-03-27-18-22-31` 结果：wp1 标定失败，GPS 均值与记录航点差 30.35m，执行 `CALIBRATION_FALLBACK`
     - 结论：当前路线中 wp1 不具备可靠标定锚点价值
   - 状态: 2026-03-27 再次确认为当前主瓶颈，需回 Step 8 重新复审锚定方案
   - 候选方向: 多点刚体配准 / map 物理点位路线 / 连续轨迹采集
   - 2026-04-01 GPS corridor 回归测试再次确认：startup offset 未吸收（~4.11m）、calibration theta 翻转（176.95deg）、live alignment 被 guard 拒绝（4.96m > 3.0m）。用户决定当前轮次按通过收口，问题转入下一轮
   - 2026-06-18 缓解: UM982 双天线 `/heading` 已接入 `gps_global_aligner_node` 的 bootstrap，对齐旋转优先由实时 RTK heading 计算；`launch_yaw_deg` 保留为 fallback 和一致性告警。剩余更强方案仍是多点刚体配准或 map 物理点位路线。

26. **[已知] Translation-only aligner 未能纠回启动锚定误差**
   - 描述: commit `94862d7` �� global aligner 改为固定 bootstrap 旋转、只估计平移的模式，意图在运行中逐步修正启动锚定偏差
   - 实车结果（session `2026-03-27-13-43-46`）:
     - 运行期持续打印 `Rejecting raw GPS alignment: bootstrap translation delta 8.88m > 8.00m`
     - 没有成功发布新的可信 runtime 平移修正
   - 根因: 启动锚定本身偏差已超过 aligner 的 `max_bootstrap_translation_delta_m: 8.0` 阈值
   - 状���: 已记录，不单独修 — 属于 #25 GPS 锚定主问题的下游表现
