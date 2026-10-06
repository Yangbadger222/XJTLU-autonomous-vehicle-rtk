## 2026.10.07

### Super-LIO + Ego-Planner-2D-ROS2 研究入口

#### 变动文件
- `src/research_runtime/`
- `src/research_interfaces/`
- `src/super_lio_vehicle_adapter/`
- `src/bringup/launch/system_active_road_research.launch.py`
- `audit/` 与研究报告

#### 变更内容
固定两个外部提交，锁定车辆参数和 launch 覆盖，增加 TimedTrajectory2D、动态/曲率/footprint 检查、证据地图、主动观察评分和 mock 串口回放。

#### 修改原因
避免将 EGO demo 的单位起始状态、Path、关闭的 feasibility 或 MPPI 链路当作本车可执行接口。

#### 带来影响
研究默认入口保持 replay/shadow；Super-LIO 的 IMU→车体映射未验证前报告 UNKNOWN 并停车。实车、Jetson、bag 和相机验收仍待现场证据。
