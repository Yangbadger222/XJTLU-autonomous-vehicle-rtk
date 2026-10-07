## 2026.10.07

### 隔离 Super-LIO / EGO2D 主动道路研究

#### 文件
- `dependencies.research.repos`、`patches/super_lio/`、`patches/ego_planner_2d/`
- `src/research_interfaces/`、`src/research_runtime/`、`src/active_road_mapping/`、`src/super_lio_vehicle_adapter/`、`src/ego_vehicle_adapter/`
- `src/bringup/launch/system_active_road_research.launch.py`、研究配置与 `ego_vehicle_adapter.yaml`
- `scripts/*research*`、`scripts/validate_*`、`audit/`、根目录研究报告、中英文架构与命令

#### 内容
在指定 e54c6af corridor 车辆基线上固定 Super-LIO ros2 与已二维化的 EGO develop 提交。四个可重放 EGO 补丁接入实测状态/GridMap/道路参考/TimedTrajectory，执行动态修复、主转向代价和连续非完整/footprint 可行性检查。独立研究模块负责经原 guard/串口链跟踪、有限观察查询、可回滚测量证据和按定位会话保存的已验证历史。新 replay/shadow/live 入口退出旧任务栈，保留必要传感器、定位、安全和日志基础。

#### 原因
需要真正运行指定上游估计器/规划器，保留已测车参与权限保护，并研究有限任务驱动观察，不能把未知地面或 LIO 健康当成 RTK 失权后的运动许可。新定位会话不能用自己的 TF 重挂旧 odom 证据；新未锚定测量要留在本地，历史已验证拓扑则可在新会话复用。

#### 影响
授权非 Jetson Humble 主机实际执行上游原样仿真、两个估计器对原始 bag 的完整隔离回放、干净 SDK/14 包构建、源/解析/运行时覆盖审计，以及原串口 PTY 跟踪和故障测试。三种受限感知策略各运行两次任务，底座哈希相同。audit 保留 red/green 和负结果，当前结论以 RESULTS.json/AUDIT_REPORT.md 为准。源参数、固件与安全权限保持；未编辑 Jetson、未自动开车、未刷固件、未删除资产或合并生产分支。物理标定、源健康、真实地面/相机接入和实车验收仍为具体 PENDING，软件通过不代表研究收益或实车就绪。

持久化历史加载继续逐条验证测量/锚点/图/回滚，统一在事务结束时核对完整哈希，避免二次任务因二次复杂度重算阻塞原状态/权限期限；原期限和最终停车条件不变。
