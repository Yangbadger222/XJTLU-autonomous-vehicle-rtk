# UM982 RTK 基础链路说明

本文只覆盖第一次下楼前的 RTK 基础链路：UM982 串口接入、NMEA 解析、`/fix`、`/heading`、raw NMEA 和可选 CORS/NTRIP 注入。完整导航验收、路线重采集和参数调优不在本文范围内。

## 当前实现

- 驱动包：`um982_rtk_driver`
- 语言：C++17 / `ament_cmake`
- 默认设备：`/dev/rtk_um982`
- 默认波特率：`115200`
- 基础启动：`make launch-rtk-basic`
- GPS 模式入口已切到 C++ 驱动：`explore-gps`、`nav-gps`、`corridor`
- 旧 `nmea_navsat_driver` 仍保留在仓库中，只作为兼容/回退路径

发布话题：

| Topic | 类型 | 来源 |
|---|---|---|
| `/fix` | `sensor_msgs/msg/NavSatFix` | UM982 GGA |
| `/heading` | `geometry_msgs/msg/QuaternionStamped` | UM982 THS/HPR |
| `rtk/nmea_sentence` | `nmea_msgs/msg/Sentence` | 校验通过的 raw NMEA |
| `rtk/status` | `std_msgs/msg/String` | 人可读 RTK 状态 |

## Jetson 构建

```bash
cd ~/XJTLU-autonomous-vehicle
source /opt/ros/humble/setup.bash
make build-rtk-basic
source install/setup.bash
```

`make launch-rtk-basic` 直接启动 `um982_rtk_driver`，不依赖 `bringup` 包安装完成。进入 `explore-gps`、`nav-gps` 或 `corridor` 前仍需要按仓库正常流程构建完整整车栈。

## 基础烟测

```bash
cd ~/XJTLU-autonomous-vehicle
source /opt/ros/humble/setup.bash
source install/setup.bash
make launch-rtk-basic
```

另开一个 SSH：

```bash
ros2 topic echo /fix --once
ros2 topic echo /heading --once
ros2 topic echo /rtk/status --once
ros2 topic echo /rtk/nmea_sentence --once
```

如果在室内看到 `/fix.status.status=-1`、GGA quality 为 `0`、`THS` 为 `V` 或没有 `/heading`，这是正常的。之前 Jetson 已经能从 `/dev/rtk_um982` 读到 `$GNGGA,,,,,,0,...` 和 `INSUFFICIENT_OBS`，说明室内信号不足不是代码问题。

## Fixed / Float 判断

不要只看 `NavSatStatus`：

- GGA quality `4`：RTK Fixed，现场验收目标
- GGA quality `5`：RTK Float，只能说明差分进入但未固定
- GGA quality `1`：单点
- GGA quality `0`：无定位

`NavSatStatus` 里 quality `4` 和 `5` 都会映射到 `STATUS_GBAS_FIX`，所以 Fixed / Float 必须看 `rtk/status` 或 `rtk/nmea_sentence` 里的 raw GGA。

## CORS / NTRIP

CORS 账号密码不要写进仓库。默认配置关闭 NTRIP：

```yaml
ntrip:
  enabled: false
```

现场可新建一个不提交的临时参数文件，例如 `/tmp/um982_cors.yaml`：

```yaml
um982_rtk_driver:
  ros__parameters:
    ntrip:
      enabled: true
      host: "<caster-host>"
      port: 2101
      mountpoint: "<mountpoint>"
      username: "<cors-user>"
      password: ""
      password_env: NTRIP_PASSWORD
      connect_requires_valid_gga: true
```

启动前输入密码到环境变量：

```bash
export NTRIP_PASSWORD='不要提交到git'
ros2 launch um982_rtk_driver um982_rtk.launch.py params_file:=/tmp/um982_cors.yaml
```

使用仓库入口启动时，可复用同一个临时文件：

```bash
export NTRIP_PASSWORD='不要提交到git'
export FYP_RTK_PARAMS_FILE=/tmp/um982_cors.yaml
make launch-rtk-basic
```

`FYP_RTK_PARAMS_FILE` 也会透传给 `explore-gps`、`nav-gps` 和 `corridor` 的 UM982 driver；导航、PGO、场景等其他节点仍使用各自原来的参数文件。

当前 C++ NTRIP 客户端支持普通 TCP NTRIP，不支持 TLS caster。如果学校/CORS caster 强制 TLS，需要后续再加 TLS 库，或者由接收机/4G 模块自己处理 NTRIP。

## 下楼测试顺序

1. 确认 `/dev/rtk_um982` 存在。
2. `make launch-rtk-basic`。
3. 看 `rtk/status` 是否有 GGA、卫星数、HDOP。
4. 室外开阔位置等待 quality 从 `0/1/5` 进入 `4`。
5. 确认 `/heading` 有数据；双天线未满足观测时 heading 可能为空。
6. 再考虑启动 `make launch-explore-gps`、`make launch-nav-gps` 或 `make launch-corridor`。

旧 G60 采集的 route/scene 不能用于 RTK 验收；进入 Fixed 后需要重新采集。
