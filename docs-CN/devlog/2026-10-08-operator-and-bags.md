## 2026.10.08

### 操作员许可、驾驶舱与独立原始 bag 验证

#### 文件
- `src/research_runtime/`、`src/research_interfaces/`、`src/active_road_mapping/`
- `src/bringup/launch/system_active_road_research.launch.py`
- `scripts/catalog_research_bags.py`、`scripts/validate_console_*`、`scripts/validate_raw_transport_probe_ros.py`
- `audit/optimization_v2/`、研究报告、CN/EN 驾驶舱与命令文档

#### 内容
新增统一驾驶舱、typed 操作员许可和已配准任务服务，浏览器不能改变执行器/环境/车参或直接控制速度。失联、竞争许可与恢复 AUTO 均锁存；原 RTK 失权停车保留。实测发现暂停 bag clock 后原 headerless guard 不发零，f8 红数据尾帧 .216m/s 保留；只将原 guard 定时器置 SystemClock，ce 实测最终五帧全零，未改原源码/阈值。

#### 原因
需要可核对的人工任务许可和独立数据身份，避免界面断联继续续租、恢复传输自行重启、暂停 bag clock 冻结最终停车，或把派生 bag 与监测端丢帧冒充新的估计器实验。软件和策略实验必须实际执行，并保留来源与失败。

#### 影响
只读扫描122份 metadata，按原始CDR去重为3独立输入；新增两次FAST/Super全1×顺序回放，保留UNKNOWN健康。监测端浅队列漏收已用真实同时接收探针定位，深队列3975/3975，浅队列3271；未将其称为估计器丢包。实际HTTP回放启动、暂停、恢复和拥有进程的结束通过。

ce 完整SDK/14包干净编译2分51秒；128可移植检查、35最终PTY故障、暂停时钟、7HTTP病例、真实浏览器单向GET失联、当前入口/三层122检查通过。真实浏览器恢复不自动使能；桌面/平板无横向溢出。实际默认replay界面留下，执行器/任务关闭且未启动bag。

新60秒策略六试验协议PASS、目标0/6，研究收益仍FAIL。误用45秒的历史试验保留，未混用预算。三策略同底座/感知/安全，无GT策略输入；模拟假设不冒充实车。Jetson、健康/外参/相机/正地面/真实先验/轮距滑移制动与物理急停依旧具体PENDING。未删除原资产、未刷固件、未自动开车或合并生产分支。
