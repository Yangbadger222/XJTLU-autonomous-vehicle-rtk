# UM982 RTK 基础链路说明

本文只覆盖第一次下楼前的 RTK 基础链路：UM982 串口接入、NMEA 解析、`/fix`、`/heading`、raw NMEA 和可选 CORS/NTRIP 注入。完整导航验收、路线重采集和参数调优不在本文范围内。

## 当前实现

- 驱动包：`um982_rtk_driver`
- 语言：C++17 / `ament_cmake`
- 默认设备：`/dev/rtk_um982`
- 默认波特率：`115200`（2026-07-23 在实车 `/dev/rtk_um982` 上通过原始 NMEA 采样确认）
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

## Heading 校准

当前双天线安装为右侧天线接 master、左侧天线接 secondary。UM982 原始 heading 表示横向基线方向，不是车辆 `base_link +X` 车头方向；因此驱动通过 `heading_offset_deg: 88.5` 把 raw heading 转成车辆 heading 后发布到 `/heading`。该值来自 2026-06-24 与 2026-06-29 已有 bag 的 RTK Fixed 直线段估计，后续仍需专门直线标定确认。

`rtk/status` 中 `heading=` 为校准后的车辆朝向，`raw=` 为接收机原始横向基线 heading。若后续调换主从天线或改成前后安装，只修改 `heading_offset_deg`，不要在导航代码里硬编码补偿。

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

使用 `make ntrip-setup` 配置凭据。它会写入 gitignore 的
`runtime-data/config/`、保存前测试 caster，并创建带时间戳的备份。随后单独选择
活动的传输 profile：

```bash
make ntrip-setup
make ntrip-use-standard  # 旧 NMEA/115200 profile
make ntrip-use-mixed     # 新 raw+NMEA/921600 profile
make ntrip-status
```

当前选择会写入 `FYP_RTK_PARAMS_FILE`，并传给 `rtk-basic`、`explore-gps`、
`nav-gps`、`corridor` 与 `tightly-coupled` 的 UM982 driver。每次 launch 时
`launch_with_logs.sh` 都会重新加载 runtime 环境，因此已打开的 shell 也会使用更新后的配置。

当前 C++ NTRIP 客户端支持普通 TCP NTRIP，不支持 TLS caster。如果学校/CORS caster 强制 TLS，需要后续再加 TLS 库，或者由接收机/4G 模块自己处理 NTRIP。

## 下楼测试顺序

1. 确认 `/dev/rtk_um982` 存在。
2. `make launch-rtk-basic`。
3. 看 `rtk/status` 是否有 GGA、卫星数、HDOP。
4. 室外开阔位置等待 quality 从 `0/1/5` 进入 `4`。
5. 确认 `/heading` 有数据；双天线未满足观测时 heading 可能为空。
6. 再考虑启动 `make launch-explore-gps`、`make launch-nav-gps` 或 `make launch-corridor`。

旧 G60 采集的 route/scene 不能用于 RTK 验收；进入 Fixed 后需要重新采集。
