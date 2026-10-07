# Super-LIO / EGO2D 与人车交互研究交付（v2）

车辆基线严格为 `corridor-authority-stability @ e54c6afbcb5a58db22d7c468085a87d658b0b932`；开发分支 `codex/superlio-ego-active-road`。当前完整编译源码 `ce4641836c265432c74086669fca8dbd6362b7d7`，后续测试脚本与报告不改变已安装运行代码。研究分支已正常推送，远端生产基线仍为 e54c6af。研究 [PR #22](https://github.com/Yangbadger222/XJTLU-autonomous-vehicle-rtk/pull/22) 保持草稿，不合并生产分支。原分支、生产工作区、固件、标定、串口协议和已测限值保留。v1 原报告及结果原样保存在 `audit/delivery_v1/`。

## 软件与交互

在授权的非 Jetson Ubuntu22.04.5 x86_64 / Humble 电脑实际完成干净 SDK 与全部 14 包构建（2 分 51 秒）；9 包有保留的 stderr 警告，构建退出 0。固定 Super-LIO `ros2@f89f48dc7aea6cfa262f18e4d03b319e04e0dbd2`、EGO `develop@7f5be6d4cee34871e85aa1f15285cfaf17b23877`。四个 EGO 补丁及 15 个已修改上游文件哈希与 v1 一致；没有 ROS1/3D 重复移植或 MPPI 调用。

| 实际验证 | 结果与范围 |
|---|---|
| 参数三层 | 1011 原文件字节保护；当前源/解析/本机运行时 122 检查；7 项实际覆盖探针；Jetson 覆盖仍 PENDING |
| 原样 EGO 与车辆接口 | v1 原样 planner/fake simulator 和 10 ROS / 5 native 核心证据保留，未伪装成重新运行；本车接口、动态 feasibility、连续 footprint/曲率/yaw-rate 验证均保留 |
| 模拟跟踪 | 1f110dc 实际 EGO 直线、R=3 m 弧线到达并停车；明确标记来源，不把后续界面修改叫作重新执行这两项 |
| 当前安全 | ce46418 的 35 项原 guard→原串口→PTY 故障、测量时钟暂停停车、7 项实际 HTTP 故障全部通过 |
| 界面实际操作 | 浏览器接管→复位→真实任务服务确认→启动，观察到 TRACKING 和非零最终串口；GET 单向断联后不续租，过期里程计不再显示静止确认，原 PTY 尾部五帧全零；恢复连接保持 STOP_LATCHED |
| 回放控制 | 清单身份全量重验后实际发布原始 Livox/IMU/clock；暂停计数与时钟冻结、恢复增长、结束仅退出拥有的播放器；零运动许可 |
| 默认入口 | 当前实际 15 子进程，无旧导航任务栈/实验旁路；唯一末端命令发布者；默认 replay、物理执行器与任务执行均关闭 |
| 可移植检查 | 当前 128 项研究运行时/数据合同测试通过，生成物理锁一致；保留原基线已有失败，未改原文件换取通过 |

驾驶舱使用真实 ROS 状态、已注册道路任务与有序 typed `OperatorPermit`。浏览器不能启用串口、改变环境/参数或直接写速度。暂停、软件停止、等待现场接管撤销自主许可；失联、竞争发布者、过期序号、后端退出都会锁存。RTK 失权仍停车，正常 LIO 不能绕过；恢复后必须重新 READY→AUTO。心跳 0.5 s 门槛与静止判据来自原参数锁。原 headerless guard 使用系统时钟，避免暂停 bag clock 后停发零命令；原串口的 wall timer 只记录超时，不能称为串口自行发送 watchdog 零命令。

当前交付界面在 `http://127.0.0.1:8765`，经 SSH 本机转发到隔离 domain100。它显示真实缺失输入与阻塞原因，未启动 bag、传感器驱动或物理串口。预览 PID、日志、环境在 `RUN_STATE.json` 与 `audit/optimization_v2/operator-preview.json`。桌面/平板 mock 截图显式带 SIM fixture；最终 replay 截图显示执行器禁用，不以假数据补绿。

## 三个独立真实原始输入

只读扫描 122 个 metadata，发现 3 个独立原始 Livox/IMU 流。另两份派生回放的原始 payload 相同，未当成新独立实验。完整存储 SHA、metadata、ordered raw CDR SHA 绑定身份。两个新增 bag 使用冻结 08d1835 的 FAST/Super 二进制，分别完整 1× 顺序回放至 EOF，期间没有其他 ROS/编译干扰 CPU 对照。

| 原始 bag | LiDAR / IMU | FAST / Super odom | FAST / Super 单核 CPU | FAST / Super RSS MiB | 一次初始对齐后的差异 RMSE / p95 / max m |
|---|---:|---:|---:|---:|---:|
| July15 18:31 | 2867 / 57008 | 2864 / 2855 | 40.34% / 16.36% | 205.8 / 84.0 | .0450 / .0830 / .1677 |
| July21 07:06 | 3382 / 67604 | 3378 / 3374 | 33.73% / 13.77% | 176.09 / 75.13 | .04760 / .07289 / .18197 |
| July21 14:34 | 4175 / 83164 | 4159 / 4153 | 36.53% / 15.18% | 214.51 / 84.36 | .07234 / .12737 / .13641 |

v1 中文报告与审计的 CPU/RSS 标签曾误写 Super/FAST；当前表按原 JSON 正确写 FAST/Super，历史文件未改写。差异不是定位真值误差；没有合格 RTK 参考、杠杆臂或静止段准确度结论。Super 源健康仍全部 UNKNOWN。两个新增对照各有初始化相邻重复 odom stamp 1 次，无倒退；输出数差不叫丢包，clock-minus-stamp 不叫计算延迟。

浅队列 Python 探针曾仅收到约 54k/65–67k IMU，不能推断估计器丢帧。当前真实 20 s morning prefix 同时接收诊断：期望 3975，depth5=3271，depth1024=3975，证明监测端背压；不证明估计器内部 IMU 消费。后续监测改为深队列，旧完整对照计数如实保留。

## 有限策略对照与负结果

本轮正确 60 s 对照在完整 ed4b7f6 14 包底座实际运行，三策略各两任务；底座/感知/安全参数一致，745 个基础文件哈希逐试验不变，原先验字节保留。策略没有 GT 输入；独立模拟器/评价器使用真值。受限深度/支持平面、IMU=base、健康、先验配准与 .03 m 不确定度均为模拟假设。操作员 fixture 前 3 s READY 后 AUTO，额外许可不能绕过原 RTK。ce46418 后续仅同值锁定超时引用、过期界面文案与监测脚本，未重标此次实验为 ce 执行。

| 策略 | 任务 | 路程 m | 非零串口 | 新观察 | 复用合格历史 | 总目标 |
|---|---:|---:|---:|---:|---:|---|
| PASSIVE | 1 | .607 | 140 | 0 | 0 | 未到达 |
| PASSIVE | 2 | 1.530 | 430 | 0 | 1 | 未到达 |
| PERIODIC_LOOK | 1 | .496 | 164 | 1 | 0 | 未到达 |
| PERIODIC_LOOK | 2 | 1.613 | 598 | 3 | 1 | 未到达 |
| TASK_AWARE_LOOK | 1 | 1.178 | 420 | 1 | 0 | 未到达 |
| TASK_AWARE_LOOK | 2 | .983 | 390 | 0 | 1 | 未到达 |

六次协议全部 PASS，目标 0/6；`POLICY_EFFECT_SIMULATION=FAIL`，尚未建立策略收益。保留失败/未知及历史图 UUID，观察入口不称为贯通或永久安全。此前误用脚本默认 45 s 的六次试验保留在远端 `logs/optimization-v2/policy-comparison.json`，当前正确 60 s 位于 `policy-60s/policy-comparison.json`；不混合预算比较，不从单条 bag 重排序声称主动策略获胜。原型仍只枚举当前车体朝向的有限前方观察位姿，更广转向/云台搜索未实现，不主张新颖性或统计收益。

## 实车待验收

软件有限环境验证完成；研究收益未建立；`JETSON_SHADOW_VALIDATION`、`LIVE_MOTION_ACCEPTANCE` 与 LIO 物理接口等价仍 PENDING。未自动开车、未刷固件、未编辑 Jetson、未删除旧资产、未 force push、未合并生产分支。具体缺失：Jetson 生效覆盖与固件身份、命令/反馈/URDF 轮距半径冲突解释、IMU→base/GNSS 杠杆臂、source-world→odom 注册、协方差/健康等价、硬件采集同步、相机 K/内外参/深度定义、正地面/坡度/台阶判据、真实 GeoTIFF/MaGRoad datum/控制点、制动/滑移/曲率/轮速与物理 KEY/手柄/急停。未知值继续阻塞，不用模拟结果冒充实车验收。

原始回放暂停目前是对所属播放器进程组 SIGSTOP；恢复可能追赶暂停期间的播放调度时钟。此次验证证明暂停期间不发布、数据时间戳正确和拥有进程的控制范围，未将恢复后的严格墙钟匀速或 prefix 操作称为整包 EOF 性能试验。软件停止在租约过期后会被拒绝，但过期自身已撤销许可；界面断联不能代替硬件急停。

完整状态见 `RESULTS.json`、`AUDIT_REPORT.md`、`RUN_STATE.json`、`REPRODUCE.md`、CN/EN 驾驶舱文档和 `audit/optimization_v2/README.md`。发布仅含源码、补丁和轻量证据；不上传 bag、大点云、GeoTIFF、二进制或凭据。
