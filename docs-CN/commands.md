# 操作命令手册

本文档只记录当前仓库和当前 Jetson 环境下确认可执行的命令。

## 1. 构建与 Source

```bash
cd ~/XJTLU-autonomous-vehicle

# 首次依赖初始化
make setup

# 全量构建
make build

# 分层构建
make build-sensor
make build-perception
make build-planning
make build-navigation

# 单包构建
colcon build --packages-select <pkg> --symlink-install --parallel-workers 1

# 每次构建后必须重新 source
source /opt/ros/humble/setup.bash
source ~/XJTLU-autonomous-vehicle/install/setup.bash
```

## 2. 初始化运行时数据

```bash
cd ~/XJTLU-autonomous-vehicle
bash scripts/init_runtime_data.sh

ls ~/XJTLU-autonomous-vehicle/runtime-data
```

## 3. 启动运行模式

```bash
cd ~/XJTLU-autonomous-vehicle

make launch-slam
make launch-explore
make launch-indoor-nav
make launch-corridor
make launch-explore-gps
make launch-nav-gps
make launch-tightly-coupled
make launch-travel
```

等效的 wrapper 直调方式：

```bash
bash scripts/launch_with_logs.sh slam
bash scripts/launch_with_logs.sh explore
bash scripts/launch_with_logs.sh indoor-nav
bash scripts/launch_with_logs.sh corridor
bash scripts/launch_with_logs.sh explore-gps
bash scripts/launch_with_logs.sh nav-gps
bash scripts/launch_with_logs.sh tightly-coupled
bash scripts/launch_with_logs.sh travel
```

等效的 `ros2 launch` 方式：

```bash
ros2 launch bringup system_slam.launch.py
ros2 launch bringup system_explore.launch.py
ros2 launch bringup system_gps_corridor.launch.py
ros2 launch bringup system_tightly_coupled.launch.py
ros2 launch bringup system_explore_gps.launch.py
ros2 launch bringup system_nav_gps.launch.py
ros2 launch bringup system_travel.launch.py
```

SLAM 纯建图可选 RTK 记录：

```bash
# 默认只建 2D/3D 地图，不启动 RTK
cd ~/XJTLU-autonomous-vehicle && bash scripts/launch_with_logs.sh slam

# 需要为后续室内外地理配准记录室外 Fixed RTK 样本时再打开
cd ~/XJTLU-autonomous-vehicle && ros2 launch bringup system_slam.launch.py use_rtk:=true
```

室内无 GPS 点击点导航的一整行命令：

> 兼容性说明：`FYP_*` 是当前脚本仍在读取的 legacy 运行接口变量名，本轮只更新公开项目称呼，不重命名运行接口。

```bash
cd ~/XJTLU-autonomous-vehicle && FYP_USE_RVIZ=true bash scripts/launch_with_logs.sh indoor-nav
```

说明：
- `indoor-nav` 不启动 GNSS driver、`gps_global_aligner`、`gps_route_runner`
- 会保留 Livox、FAST-LIO2、PGO、Nav2、串口控制链路
- 在 RViz 中使用 `2D Goal Pose` 向 `/goal_pose` 发目标即可做室内点击点导航

先验地图 Travel 导航的一整行命令：

```bash
cd ~/XJTLU-autonomous-vehicle && FYP_USE_RVIZ=true bash scripts/launch_with_logs.sh travel \
  map_yaml:=/home/badger/XJTLU-autonomous-vehicle/runtime-data/maps/2d/<map_name>/map.yaml \
  pcd_map:=/home/badger/XJTLU-autonomous-vehicle/runtime-data/maps/3d/<map_name>/map.pcd
```

说明：
- `travel` 使用 2D `map.yaml` 给 Nav2 做全局规划，使用 3D `map.pcd` 给 `localizer` 做 ICP 点云重定位
- `localizer` 负责发布 `map -> odom`；FAST-LIO2 负责发布 `odom -> base_link`
- PGO 默认不启动；如果用 `use_pgo:=true`，只使用不发布 TF 的 `pgo_slam.yaml`
- 启动后可用 `/localizer/relocalize` 重新加载 PCD 并给初始位姿：

```bash
ros2 service call /localizer/relocalize interface/srv/Relocalize \
  "{pcd_path: '/home/badger/XJTLU-autonomous-vehicle/runtime-data/maps/3d/<map_name>/map.pcd', x: 0.0, y: 0.0, z: 0.0, yaw: 0.0, pitch: 0.0, roll: 0.0}"
```

验证：

```bash
ros2 run tf2_ros tf2_monitor map odom
ros2 run tf2_ros tf2_monitor odom base_link
ros2 service call /localizer/relocalize_check interface/srv/IsValid "{code: 0}"
```

GPS Corridor v2 的一整行命令：

```bash
cd ~/XJTLU-autonomous-vehicle && FYP_USE_RVIZ=true bash scripts/launch_with_logs.sh corridor
```

说明：
- `corridor` 的运行细节、路线采集和 startup watchdog 见第 14 节
- wrapper 会同时维护 session 日志和前台状态监控输出

## 4. 单独启动核心组件

```bash
# Livox
ros2 launch livox_ros_driver2 msg_MID360_launch.py

# WIT IMU
ros2 run wit_ros2_imu wit_ros2_imu

# GNSS 原始驱动
ros2 launch nmea_navsat_driver nmea_serial_driver.launch.py

# GNSS 标定
ros2 launch gnss_calibration gnss_calibration_launch.py

# GNSS scene-ready localizer（新 GPS 路网架构）
ros2 run gnss_calibration gps_anchor_localizer_node \
  --ros-args --params-file ~/XJTLU-autonomous-vehicle/runtime-data/gnss/current_scene/master_params_scene.yaml

# FAST-LIO2
ros2 launch fastlio2 lio_no_rviz.py params_file:=~/XJTLU-autonomous-vehicle/src/bringup/config/master_params.yaml

# PGO + FAST-LIO2
ros2 launch pgo pgo_launch.py params_file:=~/XJTLU-autonomous-vehicle/src/bringup/config/master_params.yaml

# 兼容旧平铺 PGO 配置
ros2 launch pgo pgo_launch.py pgo_config:=pgo_no_gps.yaml

# 串口节点
ros2 run serial_reader serial_reader_node
ros2 run serial_twistctl serial_twistctl_node

# waypoint_collector
ros2 run waypoint_collector waypoint_node

# GPS goal manager CLI
ros2 run gps_waypoint_dispatcher goto_name <destination_name>
ros2 run gps_waypoint_dispatcher list_destinations
ros2 run gps_waypoint_dispatcher stop
```

## 5. 调试与状态检查

```bash
# topic / node / action
ros2 topic list
ros2 node list
ros2 action list
ros2 action info /compute_route
ros2 action info /follow_path
ros2 action info /navigate_to_pose
ros2 node info /pgo/pgo_node

# 频率与消息
ros2 topic hz /livox/lidar
ros2 topic hz /fastlio2/body_cloud
ros2 topic echo /pgo/optimized_odom --once
ros2 topic echo /fix --once
ros2 topic echo /gnss --once
ros2 topic echo /gps_system/status --once
ros2 topic echo /gps_system/nearest_anchor --once
ros2 topic echo /gps_system/nearest_anchor_id --once
ros2 topic echo /gps_goal_manager/status --once
ros2 topic echo /gps_waypoint_dispatcher/goal_map --once
ros2 topic echo /gps_waypoint_dispatcher/path_map --once

# 参数
ros2 param get /fastlio2/lio_node lidar_max_range
ros2 param get /pgo/pgo_node gps.enable
ros2 param get /pgo/pgo_node gps.topic
ros2 param get /pgo/pgo_node gps.origin_mode

# TF
ros2 run tf2_ros tf2_monitor
ros2 run tf2_ros tf2_monitor map odom
ros2 run tf2_ros tf2_monitor odom base_link
ros2 run tf2_tools tf2_echo map base_link
```

常见诊断重点：

- `map -> odom` 是否存在
- `/pgo/optimized_odom` 是否在持续发布
- `/gps_system/status` 是否已经到 `NAV_READY`
- `/gnss` 是否为 `gps_anchor_localizer` 发布的有效 scene-calibrated GNSS 数据
- `/compute_route` / `/follow_path` / `/navigate_to_pose` action 是否在线
- RViz fixed frame 是否为 `map`

## 6. 日志与运行时数据

```bash
# 查看当前 latest 指向
readlink -f ~/XJTLU-autonomous-vehicle/runtime-data/logs/latest

# 查看当前 session 元信息
cat ~/XJTLU-autonomous-vehicle/runtime-data/logs/latest/system/session_info.yaml

# 查看 tegrastats
tail -f ~/XJTLU-autonomous-vehicle/runtime-data/logs/latest/system/tegrastats.log

# 查看 console 日志目录
ls ~/XJTLU-autonomous-vehicle/runtime-data/logs/latest/console

# 查看 data 日志目录
ls ~/XJTLU-autonomous-vehicle/runtime-data/logs/latest/data
```

## 7. 数据采集与评测

```bash
# 录制 rosbag
bash scripts/data_collection/record_bag.sh
bash scripts/data_collection/record_bag.sh ~/XJTLU-autonomous-vehicle/runtime-data/bags/my_run

# 单独录 tegrastats
bash scripts/data_collection/record_perf.sh
bash scripts/data_collection/record_perf.sh ~/XJTLU-autonomous-vehicle/runtime-data/perf/my_run.log

# 导出 TUM 轨迹
python3 scripts/data_collection/bag_to_tum.py   ~/XJTLU-autonomous-vehicle/runtime-data/bags/my_run/rosbag2   /pgo/optimized_odom   ~/XJTLU-autonomous-vehicle/runtime-data/bags/my_run/pgo_optimized.tum
```

## 8. 地图保存

```bash
# 一次保存当前 slam session 的 2D + 3D 地图，并生成 manifest
cd ~/XJTLU-autonomous-vehicle && scripts/save_mapping_session.sh <map_name>
```

输出：

```text
runtime-data/maps/<map_name>/manifest.yaml
runtime-data/maps/2d/<map_name>/map.yaml
runtime-data/maps/2d/<map_name>/map.pgm
runtime-data/maps/3d/<map_name>/map.pcd
runtime-data/maps/3d/<map_name>/poses.txt
runtime-data/maps/3d/<map_name>/patches/*.pcd
```

说明：
- `manifest.yaml` 会记录 `consistency_ok`，用于提示 2D/3D 地图是否疑似错位；它由 2D/3D 对齐诊断、patch/pose 完整性和 frame 检查共同决定，是保存时检查，不会替代后续重定位验证
- `patch_pose_integrity.ok` 必须为 `true`，即 `patches/*.pcd` 与 `poses.txt` 关键帧一一对应
- `frame_check.ok` 必须为 `true`，默认要求 `/scan.header.frame_id` 与 `/fastlio2/lio_odom.child_frame_id` 都是 `base_link`；如果现场 FAST-LIO2 使用别的子坐标系，先用 `view_frames`/`tf2_echo` 确认，再用 `--expected-base-frame <frame>` 保存
- 后续室内外地理配准必须使用 RTK Fixed 样本和航向，室内 invalid/float RTK 只能记录，不能当强约束

底层故障排查命令：

```bash
# 保存前确认 TF 与 frame；不要在没确认实际 TF 树时盲改 base_frame
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo odom base_link

# 保存 3D 点云地图；file_path 必须写绝对路径，ROS service 请求里不会展开 ~
ros2 service call /pgo/save_maps interface/srv/SaveMaps "{file_path: '/home/badger/XJTLU-autonomous-vehicle/runtime-data/maps/3d/<map_name>', save_patches: true}"

# 保存 2D 栅格地图
ros2 run nav2_map_server map_saver_cli -f ~/XJTLU-autonomous-vehicle/runtime-data/maps/2d/<map_name>/map --ros-args -p map_subscribe_transient_local:=true

# 查看 PCD
pcl_viewer -bc 1,1,1 -ps 3 <map.pcd>
```

## 9. 停止系统与紧急停车

```bash
# 系统结束后做一次干净清理，确保下次从空状态启动
cd ~/XJTLU-autonomous-vehicle && make kill-runtime
```

停止与急停优先级：

1. PS2 手柄 `X` 键失能电机，作为最高优先级软件停车手段
2. 车身红色物理急停按钮，覆盖所有软件命令

PS2 `B` 键的下位机逻辑为阻尼主动刹车：保持电机参与控制，按轮速反向给电流，高速段保留较高刹车上限以缩短距离，低速段逐步降低电流并限制电流变化率，接近停止后清状态并持续发送零电流帧。B 刹车只在首次触发时初始化锁存状态，持续按住 B 不会反复清空电流爬坡；B 急停指示为粉色常亮，不再执行阻塞式闪烁。台架和车测确认前，不要用 `B` 替代 `X` 或车身红色物理急停。

## 10. Git 与 PR

```bash
# 同步 main
git checkout main
git pull --ff-only

# 创建分支
git checkout -b docs/sync-current-state

# 检查状态
git status
git branch -v
git log --oneline -5

# 推送分支
git push -u origin docs/sync-current-state
```

GitHub CLI：

```bash
gh auth status
gh pr create
gh pr merge --merge --delete-branch
```

如果 Jetson 上 `gh auth status` 返回 token 无效，可以在已登录 GitHub CLI 的本地工作站上对同一分支执行 `gh pr create` / `gh pr merge`，然后回 Jetson 执行：

```bash
git checkout main
git pull --ff-only
git fetch --prune
```

## 11. 系统维护

```bash
# 磁盘 / 内存
df -h /
free -h
htop

# JetPack / 机型
cat /etc/nv_tegra_release
cat /proc/device-tree/model

# NetworkManager 与有线网口自启动状态
systemctl is-enabled NetworkManager
systemctl is-active NetworkManager
nmcli -t -f NAME,AUTOCONNECT,AUTOCONNECT-PRIORITY,DEVICE connection show --active

# 检查当前机器是否具备无密码 sudo
sudo -n true && echo sudo_ok

# 切换 Jetson WiFi，并在 Jetson 侧重启 ToDesk（Linux 本机直接执行）
cd ~/XJTLU-autonomous-vehicle && bash scripts/switch_jetson_wifi.sh --status
cd ~/XJTLU-autonomous-vehicle && bash scripts/switch_jetson_wifi.sh
cd ~/XJTLU-autonomous-vehicle && bash scripts/switch_jetson_wifi.sh outdoor
cd ~/XJTLU-autonomous-vehicle && bash scripts/switch_jetson_wifi.sh indoor
cd ~/XJTLU-autonomous-vehicle && bash scripts/switch_jetson_wifi.sh Pixel
cd ~/XJTLU-autonomous-vehicle && bash scripts/switch_jetson_wifi.sh XJTLU

# GPS dispatcher 依赖
apt list --installed | grep ros-humble-geographic-msgs
python3 -c "import pyproj; print(pyproj.__version__)"
```

说明：
- 不带参数时默认在 `XJTLU` 和 `Pixel` 之间 toggle
- 上面这几条就是在 Jetson / Linux 本机直接运行的完整一行命令
- 脚本在 Jetson 本机执行时会自动切到本地模式；如果当前 shell 是 SSH/Tailscale，会话可能在切网过程中断开
- 每次切网都会在 Jetson 侧重启 `todeskd`，日志写入 `/tmp/wifi-switch.log`

## 12. GPS 数据采集

最短两行启动命令：

```bash
cd ~/XJTLU-autonomous-vehicle && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch nmea_navsat_driver nmea_serial_driver.launch.py params_file:=/home/jetson/XJTLU-autonomous-vehicle/src/bringup/config/master_params.yaml
cd ~/XJTLU-autonomous-vehicle && source /opt/ros/humble/setup.bash && source install/setup.bash && python3 scripts/collect_gps_scene.py
```

```bash
cd ~/XJTLU-autonomous-vehicle
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/collect_gps_scene.py
```

脚本说明：
- 坐标源: **仅使用 /fix**
- 采样: 每点 10 个样本取平均
- 质量门槛: 样本散布 < 2m，否则拒绝采集
- 输出文件: `~/XJTLU-autonomous-vehicle/runtime-data/gnss/scene_gps_bundle.yaml`
- 单文件同时维护：
  - fixed origin
  - graph nodes
  - `anchor`
  - `dest`
  - edges

交互命令：
- `Enter`：采图点
- `e`：添加两点之间的边，按双向通行处理
- `o`：从已有点里选择 fixed origin
- `u`：修改已有点的名字 / anchor / destination
- `l`：列出所有点和边，显示 anchor / dest / origin
- `d`：按 ID 删除指定点
- `q`：保存并退出

采集后编译运行时文件：

```bash
cd ~/XJTLU-autonomous-vehicle
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/build_scene_runtime.py
```

采集规范：
- 所有转弯、路口、目的地入口必须踩点
- 允许系统上电启动的区域附近必须布 `anchor`
- graph edge 按节点间直线段理解，弯道必须靠增加节点离散化
- 脚本会提示是否与上一个点自动建边

## 13. GPS 导航调试

```bash
# 启动 nav-gps
make launch-nav-gps

# 查看 scene 目标列表
ros2 run gps_waypoint_dispatcher list_destinations

# 室内软件 smoke 可用 mock /fix 驱动 gps_anchor_localizer
ros2 topic pub /fix sensor_msgs/msg/NavSatFix \
  "{header: {frame_id: 'gps'}, status: {status: 0, service: 1}, latitude: 31.274927, longitude: 120.737548, altitude: 0.0, position_covariance: [4.0, 0.0, 0.0, 0.0, 4.0, 0.0, 0.0, 0.0, 25.0], position_covariance_type: 2}" \
  --rate 5

# 观察 ready 状态
ros2 topic echo /gps_system/status
ros2 topic echo /gps_goal_manager/status

# 发送英文命名目标
ros2 run gps_waypoint_dispatcher goto_name anchor_a

# 检查 route / local planner action 是否在线
ros2 action list | grep -E 'compute_route|follow_path|navigate_to_pose'

# 停止当前任务
ros2 run gps_waypoint_dispatcher stop

# 一键拉起 nav-gps，等待 NAV_READY，并按编号选择 destination
cd ~/XJTLU-autonomous-vehicle && source /opt/ros/humble/setup.bash && source install/setup.bash && python3 scripts/nav_gps_menu.py
```

## 14. Fixed-Launch GPS Corridor

### GPS 路线采集（踩点）

```bash
cd ~/XJTLU-autonomous-vehicle && source /opt/ros/humble/setup.bash && source install/setup.bash && python3 scripts/collect_gps_route.py
```

交互流程：
1. 输入路线名称
2. 把车放在起点，按 Enter 采 `start_ref`（10 次采样，spread < 2m）
3. 依次移动到各 waypoint，按 Enter 采点
   - 每个点采完显示 ENU 坐标预览和 spread
   - `Accept / Retry? [A/r]` — 信号不好可以当场重采
   - 高度异常（> 10m 跳变）会自动告警
4. 确认 `launch_yaw_deg`（首段 > 5m 自动建议，否则手动输入）
5. 保存前显示路线摘要表格（各段距离、方位角、ENU 坐标）
6. 确认保存 → `~/XJTLU-autonomous-vehicle/runtime-data/gnss/current_route.yaml`

### 自动 corridor 导航

```bash
cd ~/XJTLU-autonomous-vehicle && source /opt/ros/humble/setup.bash && source install/setup.bash && bash scripts/launch_with_logs.sh corridor
```

带 RTK/CORS 参数的一行启动（现场测试用）：

> 不要把真实 CORS 密码写进仓库；`<CORS_PASSWORD>` 只表示运行时手动替换的占位符。

```bash
cd ~/XJTLU-autonomous-vehicle && source /opt/ros/humble/setup.bash && source install/setup.bash && FYP_RTK_PARAMS_FILE=/tmp/um982_cors.yaml NTRIP_PASSWORD='<CORS_PASSWORD>' FYP_USE_RVIZ=false FYP_CORRIDOR_CONSOLE_MODE=quiet bash scripts/launch_with_logs.sh corridor
```

启动前确认当前要跑的路线：

```bash
cd ~/XJTLU-autonomous-vehicle && sed -n '1,120p' runtime-data/gnss/current_route.yaml
```

结束后清理残留进程：

```bash
cd ~/XJTLU-autonomous-vehicle && make kill-runtime
```

Makefile 快捷启动：

```bash
cd ~/XJTLU-autonomous-vehicle && make launch-corridor
```

调试观察：

```bash
ros2 topic echo /gps_corridor/status
ros2 topic echo /gps_corridor/goal_map
ros2 topic echo /gps_corridor/path_map
ros2 topic echo /gps_corridor/enu_to_map
```

检查 corridor 自动录包里是否包含 RTK 诊断话题：

```bash
cd ~/XJTLU-autonomous-vehicle && ros2 bag info runtime-data/logs/latest/bag | grep -E '/heading|/rtk/status|/rtk/nmea_sentence'
```

说明：
- 该模式假定车辆已经摆在固定 Launch Pose，并且车头朝向摆正
- `collect_gps_route.py` 会采 `start_ref + 多个关键 waypoint`，并生成 `~/XJTLU-autonomous-vehicle/runtime-data/gnss/current_route.yaml`
- `collect_gps_route.py` 若未检测到 `/fix`，会自动后台拉起 `nmea_navsat_driver`，采完后自动收掉
- 采集时会显式确认 `launch_yaw_deg`；如果起点到第一个 waypoint 太近，会要求手工输入
- 子目标间距默认 30m（基于 global costmap 半径 35m - 5m buffer），采集时自动写入路线文件
- 运行时不会再弹出 menu，也不会等待额外命令
- wrapper 会把日志和 bag 写入 `~/XJTLU-autonomous-vehicle/runtime-data/logs/<session>/`
- corridor bag 会记录 `/heading`、`/rtk/status`、`/rtk/nmea_sentence`，用于复盘双天线航向和 RTK 质量
- 启动阶段若当前 `/fix` 与 `start_ref` 偏差超限，`gps_route_runner` 会直接 abort，不动车
- **Ctrl+C 会自动清理全部节点、ros2 daemon、串口占用**，无需手动 `make kill-runtime`

**Quiet 模式**（默认）:
- 前台只显示简化中文状态
- 完整 launch 输出写入 `~/XJTLU-autonomous-vehicle/runtime-data/logs/<session>/system/launch_stdout.log`
- 启动超时默认 45s，可通过 `FYP_CORRIDOR_STARTUP_TIMEOUT_S` 环境变量调整

**Raw 模式**（调试用）:
```bash
FYP_CORRIDOR_CONSOLE_MODE=raw bash scripts/launch_with_logs.sh corridor
```

## RTK FGO 紧耦合 shadow mode

构建：

```bash
cd ~/XJTLU-autonomous-vehicle
make build-perception
source install/setup.bash
```

启动实验旁路模式：

```bash
cd ~/XJTLU-autonomous-vehicle
make launch-tightly-coupled
cd ~/XJTLU-autonomous-vehicle && FYP_USE_RVIZ=false bash scripts/launch_with_logs.sh tightly-coupled
```

带现场 RTK/CORS 参数启动：

```bash
cd ~/XJTLU-autonomous-vehicle && FYP_RTK_PARAMS_FILE=/tmp/um982_cors.yaml FYP_USE_RVIZ=false bash scripts/launch_with_logs.sh tightly-coupled
```

观察 shadow 输出：

```bash
ros2 topic echo /rtk_fgo/status
ros2 topic echo /rtk_fgo/rtk_gate
ros2 topic echo /rtk_fgo/correction_status
ros2 topic echo /rtk_fgo/factor_diagnostics
```

检查底盘反馈和录包是否正常：

```bash
ros2 topic hz /cmd_vel
ros2 topic hz /odom_CBoar
ros2 topic echo /odom_CBoar --once
ros2 bag info runtime-data/logs/latest/bag | grep -E '/odom_CBoar|/cmd_vel|/fix|/heading|/rtk_fgo|/pgo/optimized_odom|/pgo/loop_markers|/livox/lidar|/fastlio2/body_cloud'
tail -f runtime-data/logs/latest/data/serial_reader.log
```

从最新 tightly-coupled bag 生成 replay 指标：

```bash
python3 scripts/evaluate_rtk_fgo_bag.py \
  --bag runtime-data/logs/latest/bag \
  --out runtime-data/logs/latest/system/rtk_fgo_metrics.json
```

实验 TF 必须显式开启，并且只用于受保护测试：

```bash
ros2 launch bringup system_tightly_coupled.launch.py publish_fgo_tf:=true nav2_use_fgo:=false
```

说明：
- 该模式默认 `publish_tf=false`，不广播生产 `map -> odom`
- 不 remap Nav2，不替代 `corridor`、`explore-gps`、`nav-gps`
- 自动录包包含 `/rtk_fgo/*`、`/fix`、`/heading`、`/rtk/status`、`/rtk/nmea_sentence`、`/livox/lidar`、`/livox/imu`、`/fastlio2/lio_odom`、`/fastlio2/body_cloud`、`/pgo/optimized_odom`、`/pgo/loop_markers` 和 `/tf`
- `/rtk_fgo/factor_diagnostics` 包含 frame anchor、wheel factor 和 graph window 健康状态字段

***

## Huggingface

把rosbags上转到Huggingface:

```bash
cd ~/XJTLU-autonomous-vehicle
hf upload frogcar/rtk-data-2026-surf ./runtime-data/bags /bags --repo-type dataset
```

从自己的电脑clone:

1. 初次安装
```bash
pip install -U "huggingface_hub[cli]"
export HF_ENDPOINT=https://hf-mirror.com
hf auth login
```

登录时，用我们organization的access token.

2. 从自己的电脑clone:
```bash
hf download frogcar/rtk-data-2026-surf --repo-type dataset --local-dir ./rtk-data-2026-surf
```
