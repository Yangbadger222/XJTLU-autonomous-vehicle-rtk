# 当前研究检查点（2026-10-08）

目标仍在执行。唯一车辆基线 corridor-authority-stability@e54c6afbcb5a58db22d7c468085a87d658b0b932，研究分支 codex/superlio-ego-active-road。当前已验收的运行源码为 a7dc39774c4421b40a6a0abb7d1709a81bdc72b0；报告提交不替换该运行身份。

MID360 自带 IMU。仓库的安装位置、原 URDF 和工厂 LiDAR/IMU 变换已用于独立车辆控制原点适配，不再把“缺外置 IMU 外参”作为阻塞。STM 目标/反馈方程和41个bag也已使用；原固件、标定、协议、已测运动限值及 RTK 失去 authority 后停车保护保持不变。烧录身份、实体制动和 KEY/急停仍须在实际部署核验。

本轮软件已经实际闭合：

- 修复控制台和云适配器在 SIGINT 下抢先关闭 ROS context 的退出异常，正常 RuntimeError 仍保持失败可见。
- 审计复现“关闭中合法 reset/start 复活许可”竞态。现已原子永久撤销操作席，后来的 claim/reset/start 和重复请求均被拒绝；真实旧代码反例 RED，修复后相关19项测试 GREEN。
- 13项实际 HTTP/typed许可/原guard/原串口PTY测试通过，含RTK失权、恢复不自动续行、后端崩溃和 SIGINT/SIGTERM。信号发出时许可和最终串口确实仍在运动；新STOP必须源时间在标记之后、序号递增、且退出前被实际接收。故意省略最终发布的测试反例使两项信号测试FAIL，即使串口随后超时归零也不能混过。
- native和ARM隔离入口均验证16个自有节点、12组参数、一位cmd发布者及全部退出。15个新增/基础节点exit0；原guard的SIGINT KeyboardInterrupt基线保留。223项portable、157项源文件/解析/有效值检查通过，两种架构各30个实际加载源码哈希一致。
- 编译是已合格SDK/14包底座加隔离Python覆盖重建，不能称为当前源码的“全新十四包”。ARM运行使用明确记录的QEMU网络诊断工具；44次原生实际socket设置成功，0失败，不冒充Jetson原生性能或实车验收。

此前固定Super-LIO的2个补丁、固定现成2DEGO的11个补丁及真实完整EOF bag/运动轨迹验收保留原来源身份。本轮没有重标旧证据。Standards和Spec两轴审计已关闭本次两项P2；环境准备失败、旧行为RED和未完成试验分别保存。

研究结果仍有明确失败：真实July21AM/PM前缀及July15完整回放没有形成车辆宽度的合格地面片段；0.4秒诊断最多一格，未作为运动权限。三个同底座策略目前各0/2任务到达，没有证明研究收益。本轮退出修复不改变这两项状态。

当前上车主要门槛是可执行道路证据、目标端实际部署/禁执行器shadow和实体停止验收。已有资料能算 nominal 模型及记录响应；不应把模型推算等同于当前烧录身份或已标注实体制动。原分支、车端生产工作区和旧资产保留；没有自动开车、刷固件、生产合并或force push。

本轮所有有限测试和网络诊断broker已退出；旧用户预览保持原ce46418源码且执行器禁用。Mac磁盘满导致本地源码/报告同步及临时验证pack恢复仍待空间；远端研究提交与报告正常发布。详细证据见 AUDIT_REPORT.md、RESULTS.json 和 audit/vehicle_ready/shutdown-qualified/。
