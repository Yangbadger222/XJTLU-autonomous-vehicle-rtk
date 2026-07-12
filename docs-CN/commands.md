# 操作命令手册

本文档只记录当前仓库和当前 Jetson 环境下确认可执行的命令。

## 初始设置

以下所有命令均基于以下条件：

1. 机器人仓库已克隆到 `~/XJTLU-autonomous-vehicle` 文件夹中
2. 机器人已安装 ROS2 Humble
3. `~/.bashrc` 与 [/scripts/.bashrc](/scripts/.bashrc) 中的版本完全一致

要创建该目录，请运行：

```bash
cd ~/
mkdir XJTLU-autonomous-vehicle
cd ~/XJTLU-autonomous-vehicle
```

要安装 ROS2 Humble，请参考其[官方文档](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html)。

首次设置 `~/.bashrc`：

* 将 [/scripts/.bashrc](/scripts/.bashrc) 的内容复制到剪贴板
* 通过 SSH 连接到 Jetson
* 运行：
```bash
vi ~/.bashrc
```
* 然后，输入 `:%d`，按 `Enter` 键
* 接着，粘贴你剪贴板中的内容
* 之后，按 `Esc` 键，然后输入 `:wq`，按 `Enter` 键
* 此时你应该已经返回终端。运行：
```bash
source ~/.bashrc
```

## 1. 构建与 Source

```bash
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
ss
```

## 2. 初始化运行时数据

```bash
bash scripts/init_runtime_data.sh

ls ~/XJTLU-autonomous-vehicle/runtime-data
```

## 3. 启动运行模式

```bash
make launch-slam
make launch-explore
make launch-indoor-nav
make launch-corridor
make launch-travel
make launch-explore-gps
make launch-nav-gps
make launch-rtk-basic
make launch-tightly-coupled
```

等效的 wrapper 直调方式：

```bash
bash scripts/launch_with_logs.sh slam
bash scripts/launch_with_logs.sh explore
bash scripts/launch_with_logs.sh indoor-nav
bash scripts/launch_with_logs.sh corridor
bash scripts/launch_with_logs.sh travel
bash scripts/launch_with_logs.sh explore-gps
bash scripts/launch_with_logs.sh nav-gps
bash scripts/launch_with_logs.sh rtk-basic
bash scripts/launch_with_logs.sh tightly-coupled
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
bash scripts/launch_with_logs.sh slam

# 需要为后续室内外地理配准记录室外 Fixed RTK 样本时再打开
ros2 launch bringup system_slam.launch.py use_rtk:=true

# 默认同时在 session 的 slam_bag/ 录 lean 证据；需要原始 Livox/结构云回放时用 debug
FYP_SLAM_BAG_PROFILE=debug bash scripts/launch_with_logs.sh slam
# 临时禁用 bag（不建议用于正式验收）
FYP_SLAM_RECORD_BAG=false bash scripts/launch_with_logs.sh slam
```

室内无 GPS 点击点导航的一整行命令：

> 兼容性说明：`FYP_*` 是当前脚本仍在读取的 legacy 运行接口变量名，本轮只更新公开项目称呼，不重命名运行接口。

```bash
FYP_USE_RVIZ=true bash scripts/launch_with_logs.sh indoor-nav
```

说明：
- `indoor-nav` 不启动 GNSS driver、`gps_global_aligner`、`gps_route_runner`
- 会保留 Livox、FAST-LIO2、PGO、Nav2、串口控制链路
- 在 RViz 中使用 `2D Goal Pose` 向 `/goal_pose` 发目标即可做室内点击点导航

先验地图 Travel 导航的一整行命令：

```bash
FYP_USE_RVIZ=true bash scripts/launch_with_logs.sh travel \
  map_bundle:=/home/badger/XJTLU-autonomous-vehicle/runtime-data/maps/indoor/<map_id>
```

Travel 默认把每个启动会话的导航诊断 rosbag 写到
`runtime-data/logs/latest/data/travel_bag/`。lean profile 记录目标、导航状态、TF、
FAST-LIO2/底盘里程计、全局/局部路径、代价地图、激光扫描和四级速度命令，足以复盘
轨迹扭动、停车和恢复行为，同时不录原始 Livox 点云。需要检查点云障碍输入时使用：

```bash
FYP_TRAVEL_BAG_PROFILE=debug FYP_USE_RVIZ=false FYP_USE_FOXGLOVE=true \
  bash scripts/launch_with_logs.sh travel \
  map_bundle:=/home/badger/XJTLU-autonomous-vehicle/runtime-data/maps/indoor/floor_4

# 仅在明确不需要复盘时关闭自动录包
FYP_TRAVEL_RECORD_BAG=false bash scripts/launch_with_logs.sh travel \
  map_bundle:=/home/badger/XJTLU-autonomous-vehicle/runtime-data/maps/indoor/floor_4
```

说明：
- `map_bundle` 是正式入口；Travel 会检查 schema、`consistency_ok`、标定接受状态以及 2D map、定位 PCD、标定、描述子和地点文件。分开传 `map_yaml/pcd_map` 只保留给兼容调试
- `localizer` 负责发布 `map -> odom`；FAST-LIO2 负责发布 `odom -> base_footprint`，URDF 再提供 `base_footprint -> base_link`
- 默认先用 Scan Context 检索多个候选并做 ICP；重复走廊候选不唯一时进入 `LOST`，此时用区域 service 或 RViz `2D Pose Estimate` 降级，不会带着歧义开车
- 定位成功后保持已接受的 `map -> odom` 偏移，由 FAST-LIO2 高频 `odom -> base_footprint` 传播运动；当前禁用运行中连续 ICP，避免匹配失败时撤掉 TF
- Travel 也会启动 `nav2_cloud_retime.py`；local costmap 使用 `/fastlio2/body_cloud_nav2`，这是 `/fastlio2/body_cloud_nav2_obstacles` 的当前时间戳副本；global costmap 只基于静态 2D 地图做全局规划，`localizer` 和建图相关节点继续使用原始 `/fastlio2/body_cloud`
- Travel 的 `NavigateToPose` / `NavigateThroughPoses` 使用有限恢复行为树：以 `0.5Hz` 重规划，每条 NavFn 路径先经过 Savitzky-Golay 几何平滑再跟踪；MPPI 保持 `20Hz/model_dt=0.05s`、`batch_size=160` 和固定采样噪声，Rotation Shim 使用闭环里程计与迟滞。碰撞安全仍由 local costmap、MPPI footprint critic 和 Collision Monitor 检查，仍禁止自动 BackUp
- Travel 使用 `650x500mm` 外廓加 `25mm` 余量的 polygon footprint、`vx_max=0.30m/s` 的 20Hz MPPI、`0.30m` 全局静态膨胀、车头方向 Slow 区和车体近场 Stop 区；路径方向误差超过 `0.35rad` 时先以 `0.30rad/s` 原地对准。所有非零纯旋转命令至少为 `0.20rad/s`，`PoseProgressChecker` 将 `0.15rad` 转向也算作有效进展；`/localizer/status` 非 `LOCALIZED` 或超时会在串口前强制零速度
- 发目标前检查 `ros2 topic echo /chassis/status --once`：必须为 `ctrl_mode: 0`（上位机串口模式）。`ctrl_mode: 1` 是手柄模式，`ctrl_mode: 2` 是电机禁用/安全接管；适配器会拒绝目标，避免先积压目标、使能电机后突然起步
- Travel 的串口末级限制器只限制加速恢复；零速和降速立即执行。线/角加速恢复上限为 `0.30m/s2`、`0.80rad/s2`，用于消除 Collision Monitor Stop 解除后的速度跳变
- 定位速度门还要求 `/fastlio2/body_cloud_nav2` 在 0.5s 内更新、`/cmd_vel` 在 0.25s 内更新；`serial_twistctl` 300ms 断流重发零速，STM32 500ms watchdog 再独立清零。固件层保护只有重新编译并烧录 `src/firmware/rm_c_board/` 后才生效
- localizer 发布 `map->odom` 时使用 `0.10s` 未来容差；单次 ICP 失败进入 DEGRADED 后最多保留 `2.5s` 最近可信定位。窗口内点云/状态 freshness 门仍生效，超时或 LOST 立即取消目标并清零
- 自动全局定位会等待至少 200 个结构点，以 3s 间隔最多尝试 5 次；`map -> odom` 始终投影为平面 XY+yaw
- PGO 默认不启动；如果用 `use_pgo:=true`，只使用不发布 TF 的 `pgo_slam.yaml`

定位状态和区域辅助重定位：

```bash
ros2 topic echo /localizer/status
ros2 service call /localizer/global_relocalize interface/srv/GlobalRelocalize \
  "{descriptor_index: '', region: 'east_corridor', max_candidates: 5}"
```

RViz `2D Pose Estimate` 仍可作为手动降级入口；也可直接调用旧重定位 service：

```bash
ros2 service call /localizer/relocalize interface/srv/Relocalize \
  "{pcd_path: '/home/badger/XJTLU-autonomous-vehicle/runtime-data/maps/indoor/<map_id>/localization/map_localization.pcd', x: 0.0, y: 0.0, z: 0.0, yaw: 0.0, pitch: 0.0, roll: 0.0}"
```

不依赖 RViz 的地点名导航：

```bash
ros2 action send_goal /navigate_named_destination \
  interface/action/NavigateNamedDestination \
  "{map_id: '<map_id>', destination_name: 'lab 101', backend: 'indoor'}" --feedback
```

验证：

```bash
ros2 run tf2_ros tf2_monitor odom base_footprint
ros2 service call /localizer/relocalize_check interface/srv/IsValid "{code: 0}"
ros2 run tf2_ros tf2_monitor map odom
ros2 topic echo /cmd_vel_safe
```

GPS Corridor v2 的一整行命令：

```bash
FYP_USE_RVIZ=true bash scripts/launch_with_logs.sh corridor
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
ros2 run tf2_ros tf2_monitor odom base_footprint
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
# 车辆先连续静止；一次生成并验收完整室内地图包
scripts/save_mapping_session.sh <map_name>
# 可选：用区域多边形给描述子候选加标签
python3 scripts/save_mapping_session.py <map_name> --regions-file /path/to/regions.yaml
# 已保存地图只重算标定/叠加图/描述子区域标签/manifest，不调用 ROS 保存服务
scripts/save_mapping_session.sh <map_name> --recalibrate-only

# 仅清除已保存 PGM 中最多 3 格的孤立占用块；始终输出到新文件，检查后再替换
python3 scripts/clean_occupancy_map.py \
  runtime-data/maps/indoor/<map_name>/navigation/map.pgm \
  runtime-data/maps/indoor/<map_name>/navigation/map.cleaned.pgm
```

输出：

```text
runtime-data/maps/indoor/<map_name>/manifest.yaml
runtime-data/maps/indoor/<map_name>/navigation/{map.yaml,map.pgm,slam_toolbox.posegraph,slam_toolbox.data}
runtime-data/maps/indoor/<map_name>/localization/{map_raw.pcd,map_localization.pcd,poses.txt}
runtime-data/maps/indoor/<map_name>/localization/patches/*.pcd
runtime-data/maps/indoor/<map_name>/localization/descriptor_index/scan_context.yaml
runtime-data/maps/indoor/<map_name>/calibration/{map_3d_to_map_2d.yaml,alignment_report.yaml,alignment_overlay.png}
runtime-data/maps/indoor/<map_name>/{destinations.yaml,regions.yaml}
```

说明：
- 保存前连续两次检查 FAST-LIO2 和 `/odom_CBoar` 速度；存在实时 `/cmd_vel` 时也必须为零。任一硬门槛失败会返回非零，Travel 不加载 `consistency_ok=false` 的地图包
- 2D↔3D 标定不再直接投影整个 PCD：默认在 `0.06m` XY 栅格中保留至少 3 点且垂直跨度 `>=0.50m` 的强墙面单元，以排除地面、桌面和单层动态杂点；该设置对应默认 `0.10m` 定位 PCD 体素
- `--recalibrate-only` 保留原始 `created_at`、保存输出、静止/frame/patch 证据和地图资产，只原子更新 manifest，并重建标定文件、叠加图及 Scan Context 区域标签
- 2D↔3D 标定默认要求全图及已配置区域的墙面 RMSE `<=0.10m`、p95 `<=0.15m`、重叠率 `>=0.55`，且第一/第二初值候选差距 `>=0.001`；这些是首轮安全值，必须用实车地图收口
- `patch_pose_integrity.ok` 必须为 `true`，即 `patches/*.pcd` 与 `poses.txt` 关键帧一一对应
- `frame_check.ok` 必须为 `true`，默认要求 `/scan.header.frame_id` 与 `/fastlio2/lio_odom.child_frame_id` 都是 `base_footprint`；如果现场 FAST-LIO2 使用别的子坐标系，先用 `view_frames`/`tf2_echo` 确认，再用 `--expected-base-frame <frame>` 保存
- 后续室内外地理配准必须使用 RTK Fixed 样本和航向，室内 invalid/float RTK 只能记录，不能当强约束
- 孤立点清理采用 8 连通域，默认只处理 1–3 格（`0.0025~0.0075m2`）的占用块，并按边界多数恢复为自由或未知；不会把未知区统一涂成自由。大于 3 格的柱子、家具和墙体原样保留。替换 `map.pgm` 前必须保留备份并检查可视化结果

底层故障排查命令：

```bash
# 保存前确认 TF 与 frame；不要在没确认实际 TF 树时盲改 base_frame
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo odom base_footprint

# 保存 3D 点云地图；file_path 必须写绝对路径，ROS service 请求里不会展开 ~
ros2 service call /pgo/save_maps interface/srv/SaveMaps "{file_path: '/home/badger/XJTLU-autonomous-vehicle/runtime-data/maps/indoor/<map_name>/localization', save_patches: true}"

# 保存 2D 栅格地图
ros2 run nav2_map_server map_saver_cli -f ~/XJTLU-autonomous-vehicle/runtime-data/maps/indoor/<map_name>/navigation/map --ros-args -p map_subscribe_transient_local:=true

# 查看 PCD
pcl_viewer -bc 1,1,1 -ps 3 <map.pcd>
```

## 9. 停止系统与紧急停车

```bash
# 系统结束后做一次干净清理，确保下次从空状态启动
make kill
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
git checkout -b <BRANCH_NAME>

# 检查状态
git status
git branch -v
git log --oneline -5

# 推送分支
git push -u origin <BRANCH_NAME>
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

# 每个文件夹大小
sudo du -h --max-depth=1 / | sort -hr

# JetPack / 机型
cat /etc/nv_tegra_release
cat /proc/device-tree/model

# NetworkManager 与有线网口自启动状态
systemctl is-enabled NetworkManager
systemctl is-active NetworkManager
nmcli -t -f NAME,AUTOCONNECT,AUTOCONNECT-PRIORITY,DEVICE connection show --active

# 设置网络自动连接设置
sudo nmcli connection modify "WiFi-Name" connection.autoconnect yes
sudo nmcli connection modify "WiFi-Name" connection.autoconnect-priority 100
sudo nmcli connection modify "WiFi-Name" connection.autoconnect-retries 3

# 检查当前机器是否具备无密码 sudo
sudo -n true && echo sudo_ok

# 切换 Jetson WiFi，并在 Jetson 侧重启 ToDesk（Linux 本机直接执行）
bash scripts/switch_jetson_wifi.sh --status
bash scripts/switch_jetson_wifi.sh
bash scripts/switch_jetson_wifi.sh outdoor
bash scripts/switch_jetson_wifi.sh indoor
bash scripts/switch_jetson_wifi.sh Pixel
bash scripts/switch_jetson_wifi.sh XJTLU

# GPS dispatcher 依赖
apt list --installed | grep ros-humble-geographic-msgs
python3 -c "import pyproj; print(pyproj.__version__)"
```

说明：
- 不带参数时默认在 `XJTLU` 和 `Pixel` 之间 toggle
- 上面这几条就是在 Jetson / Linux 本机直接运行的完整一行命令
- `pyproj` 仍是推荐依赖；若临时缺失，QGIS scene 编译和 nav-gps scene 读取会退回本地 ENU 近似，不应直接启动失败
- 脚本在 Jetson 本机执行时会自动切到本地模式；如果当前 shell 是 SSH/Tailscale，会话可能在切网过程中断开
- 每次切网都会在 Jetson 侧重启 `todeskd`，日志写入 `/tmp/wifi-switch.log`

## 12. GPS 数据采集

最短两行启动命令：

```bash
ros2 launch nmea_navsat_driver nmea_serial_driver.launch.py params_file:=/home/jetson/XJTLU-autonomous-vehicle/src/bringup/config/master_params.yaml
python3 scripts/collect_gps_scene.py
```

```bash
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
python3 scripts/build_scene_runtime.py
```

QGIS 路网包导入流程（例如 `/Users/badger/Desktop/maps/qgis_4_package/3.geojson`）：

```bash
python3 scripts/compile_qgis_scene.py \
  --input /Users/badger/Desktop/maps/qgis_4_package/3.geojson \
  --scene-name qgis_4 \
  --densify-step-m 5.0 \
  --output ~/XJTLU-autonomous-vehicle/runtime-data/gnss/scene_gps_bundle.yaml

python3 scripts/build_scene_runtime.py
```

`compile_qgis_scene.py` 会把 `feature_type=route` 的 LineString 转成 scene graph，按 5m 最大边长补点，并把可用终点写入 `scene_gps_bundle.yaml`。对当前 `qgis_4_package/3.geojson`，默认生成约 `557` 个路网节点、`558` 条边和这些初始 destination：`math_building`、`environment_building`、`route_end_*`、`junction_*`。

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
ros2 action list | grep -E 'compute_route|follow_path'

# 停止当前任务
ros2 run gps_waypoint_dispatcher stop

# 一键拉起 nav-gps，等待 NAV_READY 或 RTK_AUTHORITATIVE，并按编号选择 destination
python3 scripts/nav_gps_menu.py
```

运行说明：
- 修改或导入新的 QGIS/scene 地图后，必须先重新执行 `python3 scripts/build_scene_runtime.py`，让 `master_params_scene.yaml` 写入 scene fixed origin、`rtk_map_odom_corrector` 的 `scene_points_file` 和 `use_scene_identity_alignment=true`。
- `nav-gps` 不要求车辆在 anchor 附近才能发目标；goal manager 会读取当前 `map→base_link` pose，并用 `ComputeRoute(use_poses=true)` 让 route server 从最近可通行路网节点开始规划。
- `route_server` 在 `nav-gps` 下开启 `enable_nn_search=true`，因此只要车辆在路网附近，起点会被吸附到最近可通行 graph node，再沿路网规划到 destination。
- `nav-gps` 现在复用 corridor RTK authoritative 链：PGO 关闭 `publish_tf` 和 GPS 因子，`rtk_map_odom_corrector` 是唯一 `map→odom` owner。
- Nav2 使用 corridor RTK MPPI profile 与 `/fastlio2/body_cloud_nav2_obstacles` 高窗障碍点云；旧 `nav2_gps.yaml` DWB profile 暂不作为实车选点导航入口。
- 默认会启动 RTK FGO shadow node，但固定 `publish_tf=false`、`nav2_use_fgo=false`；如需关闭可设置 `FYP_NAV_GPS_ENABLE_FGO_SHADOW=false`。
- 默认 lean bag 记录 RTK、FAST-LIO2 odom、Livox IMU、底盘 `/odom_CBoar`、`/rtk_fgo/*`、TF、GPS/goal 状态、costmap、`/cmd_vel` 和 `/plan`；需要原始点云回放时再设置 `FYP_NAV_GPS_BAG_PROFILE=debug`。
- 车上建议用 `FYP_USE_RVIZ=false bash scripts/launch_with_logs.sh nav-gps`，避免 RViz 消耗 Jetson 资源。

## 14. Fixed-Launch GPS Corridor

### GPS 路线采集（踩点）

```bash
python3 scripts/collect_gps_route.py
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
bash scripts/launch_with_logs.sh corridor
```

带 RTK/CORS 参数的一行启动（现场测试用）：

> 不要把真实 CORS 密码写进仓库；所有 CORS 账号信息用 `make ntrip-login` 管理。

```bash
FYP_RTK_PARAMS_FILE=/tmp/um982_cors.yaml NTRIP_PASSWORD='<CORS_PASSWORD>' FYP_USE_RVIZ=false FYP_CORRIDOR_CONSOLE_MODE=quiet bash scripts/launch_with_logs.sh corridor
```

启动前确认当前要跑的路线：

```bash
sed -n '1,120p' runtime-data/gnss/current_route.yaml
```

结束后清理残留进程：

```bash
make kill
```

Makefile 快捷启动：

```bash
make launch-corridor
```

调试观察：

```bash
ros2 topic echo /gps_corridor/status
ros2 topic echo /gps_corridor/goal_map
ros2 topic echo /gps_corridor/path_map
ros2 topic echo /gps_corridor/enu_to_map
```

检查 corridor 默认自动录包里是否包含轻量 RTK / FAST-LIO2 / FGO shadow / Nav2 诊断话题：

```bash
ros2 bag info runtime-data/logs/latest/bag | grep -E '/fix|/heading|/rtk/status|/rtk/nmea_sentence|/fastlio2/lio_odom|/livox/imu|/odom_CBoar|/rtk_fgo|/cmd_vel|/plan'
```

如果需要回放原始 Livox 数据，启动前显式切到更重的 debug bag profile：

```bash
FYP_CORRIDOR_BAG_PROFILE=debug FYP_USE_RVIZ=false FYP_CORRIDOR_CONSOLE_MODE=quiet bash scripts/launch_with_logs.sh corridor
ros2 bag info runtime-data/logs/latest/bag | grep -E '/livox/lidar|/fastlio2/body_cloud'
```

说明：
- 该模式假定车辆已经摆在固定 Launch Pose，并且车头朝向摆正
- `collect_gps_route.py` 会采 `start_ref + 多个关键 waypoint`，并生成 `~/XJTLU-autonomous-vehicle/runtime-data/gnss/current_route.yaml`
- `collect_gps_route.py` 若未检测到 `/fix`，会自动后台拉起 `nmea_navsat_driver`，采完后自动收掉
- 采集时会显式确认 `launch_yaw_deg`；如果起点到第一个 waypoint 太近，会要求手工输入
- 子目标间距默认 5m，采集时自动写入路线文件；长 RTK 路线会被拆成短子目标，减少 rolling costmap 和局部跟踪耦合风险
- 运行时不会再弹出 menu，也不会等待额外命令
- wrapper 会把日志和 bag 写入 `~/XJTLU-autonomous-vehicle/runtime-data/logs/<session>/`
- corridor 当前启动时会从 `nav2_corridor_rtk.yaml` 生成临时 Nav2 参数文件并使用 RTK authoritative corridor 档：`vx_max=0.85`、`wz_max=0.70`、`ax_max=0.85`、`ax_min=-1.2`、`az_max=1.4`、`temperature=0.45`、`regenerate_noises=true`、`failure_tolerance=1.5s`、`controller_frequency=20Hz`、`batch_size=500`；local costmap 为近场 `12m x 12m`，STVL marking `obstacle_range=5m`，`CostCritic.cost_weight=7.0`；MPPI 的 `model_dt=0.05s` 要求控制周期不能大于模型步长，因此不能降到 `15Hz`
- 到达最后一个 waypoint 后，`gps_route_runner` 会先发布 `STOPPING_BEFORE_EXIT`，以 20Hz 保持 1.2s 的零 `/cmd_vel`，然后再发布 `SUCCEEDED`；quiet 模式只会在该保持结束后退出，因此 bag 中应能看到明显的零速度尾巴。
- corridor 默认启动 RTK FGO shadow node，但固定 `publish_tf=false`、`nav2_use_fgo=false`，不会接管 `map→odom` 或 Nav2；如需关闭可设置 `FYP_CORRIDOR_ENABLE_FGO_SHADOW=false`
- corridor 默认使用 lean bag profile，记录 RTK、FAST-LIO2 odom、Livox IMU、底盘 `/odom_CBoar`、`/rtk_fgo/*`、TF、corridor 状态、目标、costmap、`/cmd_vel` 和 `/plan`；原始 Livox 点云、`/fastlio2/body_cloud` 与 `/fastlio2/body_cloud_nav2_obstacles` 仅在 `FYP_CORRIDOR_BAG_PROFILE=debug` 时记录
- Livox 逐包 console/CSV 日志默认关闭。只有短时间台架诊断时才使用 `LIVOX_VERBOSE_PACKET_LOGS=1`，因为它会逐包打印并 flush。
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
make build-perception
ss
```

启动实验旁路模式：

```bash
make launch-tightly-coupled
FYP_USE_RVIZ=false bash scripts/launch_with_logs.sh tightly-coupled
```

带现场 RTK/CORS 参数启动：

```bash
FYP_RTK_PARAMS_FILE=/tmp/um982_cors.yaml FYP_USE_RVIZ=false bash scripts/launch_with_logs.sh tightly-coupled
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
hf upload frogcar/rtk-data-2026-surf ./runtime-data --repo-type dataset
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

***

## NTRIP 账户设置

在使用 RTK 天线时，机器人必须拥有一个 NTRIP 账户才能接收到完整质量的信号。您可以在淘宝上购买这些账户，例如：[https://e.tb.cn/h.Ry4kJCGRkkS8a8n?tk=VpOEgN1OG2z](https://e.tb.cn/h.Ry4kJCGRkkS8a8n?tk=VpOEgN1OG2z)

此外，本仓库自带一个用于管理这些认证凭据的脚本。

如需登录 NTRIP 账户，请运行：
```bash
make ntrip-login
```

如需修改账户参数（例如服务器 IP 和挂载点），请运行：
```bash
make ntrip-setup
```

如需检查当前凭据并进行连接测试，请运行：
```bash
make ntrip-status
```

如需登出账户，请运行：
```bash
make ntrip-logout
```

等效的脚本直接调用方式：
```bash
@python3 scripts/setup_ntrip.py
@python3 scripts/setup_ntrip.py --setup
@python3 scripts/setup_ntrip.py --status
@python3 scripts/setup_ntrip.py --logout
```

一旦登录成功，凭据将会保存在机器人中。除非您手动登出或更改凭据，否则每次系统启动时都会自动登录。

***

## Foxglove

### 初始设置

在您的个人电脑上：
1. 在 https://app.foxglove.dev/signin 创建一个账号
2. 下载 Foxglove：https://foxglove.dev/download

在 Jetson 上下载并安装 Foxglove：
```bash
sudo apt update
sudo apt install ros-$ROS_DISTRO-foxglove-bridge
```

### Travel 室内导航实时连接

Travel 默认启动 Foxglove Bridge 和受控导航适配器，不需要再单独运行 bridge：

```bash
FYP_USE_RVIZ=false FYP_USE_FOXGLOVE=true \
  bash scripts/launch_with_logs.sh travel \
  map_bundle:=/home/badger/XJTLU-autonomous-vehicle/runtime-data/maps/indoor/<map_id>
```

如端口冲突，可直接传 `foxglove_port:=8766`。临时完全关闭 Bridge：

```bash
FYP_USE_FOXGLOVE=false bash scripts/launch_with_logs.sh travel \
  map_bundle:=/home/badger/XJTLU-autonomous-vehicle/runtime-data/maps/indoor/<map_id>
```

首次使用在 Foxglove 中导入仓库布局：

```text
src/bringup/foxglove/indoor_navigation.json
```

布局包含 3D 地图/机器人/点云/costmap/路径/安全区、定位状态、导航状态、地点目录、地点名发送、区域重定位、取消、日志和 Topic Graph。

操作约定：

- 3D 面板 `Publish -> 2D pose estimate` 发布到 `/initialpose`，用于手动定位降级。
- 仓库布局的 3D 面板 `Publish -> 2D pose` 发布到 `/foxglove/goal_pose`；Foxglove 默认 3D 布局常发布 `/move_base_simple/goal`。两者都由同一个定位门控适配器转换为 Nav2 `NavigateToPose` Action。
- `Navigate to destination` 面板把 `data` 改成地点 ID、显示名或 alias，发布到 `/foxglove/named_destination`。
- `Relocalize in region` 调 `/localizer/global_relocalize`；`region` 留空代表全图搜索。
- `Cancel navigation` 调 `/foxglove/cancel_navigation`，只取消适配器当前持有的目标。
- `/foxglove/navigation/status` 显示目标来源、目标、定位状态、剩余距离和结果；地点会以 MarkerArray 显示在 3D 地图。

安全门：适配器不订阅或发布任何 `/cmd_vel*`，Bridge 的 client-publish 白名单只允许 `/initialpose`、`/foxglove/goal_pose`、`/move_base_simple/goal` 和 `/foxglove/named_destination`，且不开放参数修改能力。两个 Pose 入口都经过相同的定位/并发/Action 门控；只有 `/localizer/status` 同时满足 `LOCALIZED`、`localized=true`、`sensors_ready=true` 才发送目标，定位降级会取消当前目标，底层速度门仍独立执行零速保护。

### 手动启动 Bridge（其它模式）

要建立连接，请通过 SSH 登录到 Jetson 并运行：

```bash
ros2 run foxglove_bridge foxglove_bridge
```

然后在您的电脑上打开 Foxglove，点击 **“Open Connection”** -> **“Foxglove WebSocket (default)”**，并输入 `ws://100.79.128.22:8765`

Foxglove Bridge 没有项目级登录认证；只应通过受控局域网或 Tailscale 访问，不要把 `8765` 暴露到公网。

输入完成后，点击中间视图的任意位置，左侧面板将开始加载许多选项。这可能需要一些时间，最多可能需要一分钟。

**注意**：每个账号的 Tailscale IP 可能会略有不同。如果您不确定正确的 IP：

1. 在 Jetson 上打开另一个终端并运行：`tailscale ip -4`
2. 从您的电脑或[浏览器](https://login.tailscale.com/admin/machines)打开 Tailscale，检查名为 “badger” 的机器人地址
3. 返回 Foxglove 并在此处输入正确的 IP 地址：`ws://<TAILSCALE_IP>:8765`

#### Foxglove 设置

为了正确渲染机器人 URDF 以及其他数据（如点云），请使用以下设置：

* Fixed frame: `<Root frame>`
* Display frame: `base_link`
* Follow mode: `Pose`（位置 + 姿态）
* Sync timestamps: `Off`
* Location topic: `Auto`
* ENU frame: `<Fixed frame>`
* Grid Frame: `base_footprint`

### 故障排查

无法连接：
- 仔细检查 Tailscale 地址是否正确，并确认您已开启 Tailscale。
- 在您的电脑上使用 `ping <TAILSCALE_IP>` 命令，对 Jetson 的 Tailscale 地址进行 Ping 测试。
- 如果使用了 VPN，请前往您的代理设置并将所有 Tailscale IP 添加到例外列表中：`100.*.*.*`（或者尝试关闭 VPN 并重新连接）。
- 如果使用 Clash Verge:
  1. 前往《设置》，然后《系统代理》点击小齿轮
  2. 找《始终使用默认绕过》，关掉 OFF
  3. 有个 text box 会出现（在《代理绕过设置》下面）。在 text box 上，写这个IP：`100.64.0.0/10`。然后点《新建》，再点《保存》。
  4. 回去设置页。找《虚拟网卡模式》点击小齿轮。
  5. 在下面的 text box （在《排除自定义网段》下面），写同样的IP：`100.64.0.0/10`。然后点《新建》，再点《保存》。
  6. 回去 Foxglove 再试一遍


连接速度过慢：
- 将 Jetson 的 WiFi 切换为您的手机热点，然后再次进行 Ping 测试。

话题（Topics）未正常渲染（URDF 或点云缺失）：
- 确保在 **Panel** -> **Topics** 中，话题 `/fastlio2/world_cloud` 和 `/robot_description` 是可见的（点击眼睛图标）。
- 关闭当前的 Foxglove 会话并重新打开一个，通常可以解决问题。

## `~/.bashrc`

每当在 Jetson 中打开终端（包括 SSH 连接）时，该脚本都会运行。我们对其进行了修改，以包含常用命令并提供机器人当前状态的概述。

该脚本正在 [/scripts/.bashrc](/scripts/.bashrc) 中进行版本追踪。如需在 Jetson 中进行设置：

1. 将 [/scripts/.bashrc](/scripts/.bashrc) 中的脚本内容复制到剪贴板中
2. 在 Jetson 中打开一个终端（SSH 或本地终端均可）
3. 输入以下命令以使用 Vim 打开 `~/.bashrc`：
```bash
rc
```
4. 打开后，输入 `:%d` 以删除文件中的所有内容
5. 使用 `Ctrl + V` 将剪贴板中的新脚本粘贴进去
6. 按 `Esc`，然后输入 `:wq` 保存并退出
7. 如需进行测试，请打开一个新终端或运行：`s1`

每当您想要更新 `~/.bashrc` 时，请先在 [/scripts/.bashrc](/scripts/.bashrc) 中进行修改，然后按照上述步骤操作，以确保我们能够追踪该文件的变化。请勿在未在本仓库中进行追踪的情况下直接在 Jetson 中修改它。
