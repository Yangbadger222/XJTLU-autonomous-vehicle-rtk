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

新60秒策略六试验协议PASS、目标0/6，研究收益仍FAIL。误用45秒的历史试验保留，未混用预算。三策略同底座/感知/安全，无GT策略输入；模拟假设不冒充实车。Jetson部署、定位/地面/控制接口资格与新栈物理停车依旧PENDING；已有安装与底盘资料已按下文复核，不再笼统称缺失。未删除原资产、未刷固件、未自动开车或合并生产分支。


### 车辆已有资料复核与响应计算更正

#### 文件
- `scripts/audit_recorded_chassis_response.py`、`scripts/test_recorded_chassis_response.py`
- `audit/vehicle_contract_review/`、`LIVE_ACCEPTANCE_CHECKLIST.md`、`RESULTS.json`、研究报告

#### 内容
重新读取硬件安装笔记、旧FAST参考点说明和STM32源码，明确已有MID360安装值和内置IMU关系。精确计算命令RPM界限1853.7696、理想轮反馈比例1.0105263/.9296842及原速度权限对应的速度相关曲率界限；后者不套用于实际发送的gyro。实际只读分析全部41个命令/LIO bag、68732个有效区间，以及两次July7原始串口日志，前向Y非零607/571条，正常样本完成组合响应估计；1186个含旋转的零转换中15个有源header至少1秒覆盖且二维静止，异常和未确认项保留。

#### 原因
先前将已有安装/底盘资料笼统报为缺失不准确。需要区分已提供数据、旧参考点约定、研究代码尚未接入的合同和真正未完成的新栈物理验收；不能以假参数解锁，也不能跳过现有资料要求重新测量全部参数。

#### 影响
源公式与已有权限均已确定，数据质量限制有具体计数。9项独立拟合/激励/有限值/反馈轴与符号/两时钟/横移与拒绝缺口测试通过；未改变固件、标定、运动限值或运行源码。新Super健康/参考点/地面/运动适配和车端验收仍如实未完成，不自动上车。解释研究κ=0是尚未完成的配置门控，不称为仓库没有底盘资料。

### 实机验收准备：Super观测证书与原导航参考点

#### 文件
- `patches/super_lio/0002-certify-source-observations-and-covariance.patch`、补丁脚本
- `src/super_lio_vehicle_adapter/`、`src/bringup/config/super_lio_reference.yaml`、研究launch
- `scripts/validate_super_lio_information.cpp`、`audit/vehicle_ready/`

#### 内容
从原固定外参12维信息块推导保守充分下界：min(实际Super六维观测信息最小特征值,100000)。使用原75门限、50有效点和3同步IMU样本，记录最差迭代，不伪造旧degeneracy值。补全源协方差交叉块/坐标，拒绝无有效观测推进地图。接入已记录的IMU原点base_footprint约定和明确的本地world/odom gauge；按测量戳配对健康与odom，异常时健康不放行。源时钟倒退/原位姿跳变门限锁存。

#### 原因
固定UNKNOWN和未接入的参考点使新入口只能等待。已有源码可证明更严格的安全充分条件，而物理MID360安装不需再次从零标定。需要真实观测健康与导航坐标合同，不以恒healthy或改标签解锁。

#### 影响
原固件、标定、限值和RTK失权停车不变。此为本轮继续准备实机验收的源码进展；实际C++/Humble编译、回放和最终mock串口证据尚须执行。地面、EGO源约束接入与车端shadow资格仍未完成，不据此宣布goal完成或自动开车。
