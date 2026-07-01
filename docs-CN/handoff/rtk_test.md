# RTK 测试命令

首先，确保机器人连接的是你的手机热点。切勿使用校园网（Wi-Fi），它非常不稳定。然后，确保你已经启用了 Tailscale。

你需要打开两个终端。

通过 SSH 登录机器人：
```bash
ssh badger@100.79.128.21
```

登录后，运行以下命令：

1. 对于第一个终端（每天只需执行一次）：

```bash
cd XJTLU-autonomous-vehicle/
git switch Tightly-coupled
git pull
colcon build --packages-select bringup --symlink-install --parallel-workers 1
source install/setup.bash
source /opt/ros/humble/setup.bash
```

等待加载完成。可以使用 `Ctrl L` 清空终端。

2. 随后打开的其他终端：

```bash
cd XJTLU-autonomous-vehicle/
source install/setup.bash
source /opt/ros/humble/setup.bash
```

## NTRIP 账号

RTK 需要一个 NTRIP 账号。购买后，运行以下命令（**只需**修改包含 `ADD_YOUR_USERNAME_HERE` 和 `ADD_YOUR_PASSWORD_HERE` 的行）：

```bash
cat << 'EOF' > /tmp/um982_cors.yaml
um982_rtk_driver:
  ros__parameters:
    port: /dev/rtk_um982
    baud: 115200
    frame_id: gps
    heading_offset_deg: 90.0
    publish_raw: true
    status_period_s: 1.0
    ntrip:
      enabled: true
      host: "120.253.239.161"
      port: 8002
      mountpoint: "RTCM33_GRCEJ"
      username: "ADD_YOUR_USERNAME_HERE"
      password: ""
      password_env: NTRIP_PASSWORD
      connect_requires_valid_gga: true
EOF
export NTRIP_PASSWORD='ADD_YOUR_PASSWORD_HERE'
export FYP_RTK_PARAMS_FILE=/tmp/um982_cors.yaml
```

> 如果你切换终端、断开连接或打开了新终端，必须重新运行此命令。

***

## 运行并收集数据

打开 2 个终端。在第一个终端中，运行上述所有命令。然后，启动 RTK：

```bash
bash scripts/launch_with_logs.sh tightly-coupled
```

在第二个终端中，查看 RTK 信号数据。如果你刚刚登录：

```bash
cd XJTLU-autonomous-vehicle/
source install/setup.bash
source /opt/ros/humble/setup.bash
ros2 topic list
ros2 topic echo /rtk/status
```

如果你已经处于项目目录中：

```bash
cd XJTLU-autonomous-vehicle/
source /opt/ros/humble/setup.bash
ros2 topic echo /rtk/status
```

你可以通过运行 `Ctrl C` 停止查看数据。

RTK 信号状态应该达到 `q=4`。

- 如果 `q=0`，说明 RTK 没有信号。请前往室外开阔区域。
- 如果你已经完全处于室外但 `q=1`，请确保你已成功登录账号。

Use the custom python script:
```bash
python3 /tmp/rtk_heading_calibrate.py
```

完成后，只需运行以下命令即可停止记录数据：

```bash
make kill
```

数据将保存在机器人的 `/runtime-data/logs/` 目录下。

当你想将数据传输到电脑时，可以使用 U 盘，也可以将机器人中的文件上传到网上。你可以运行以下命令：

```bash
cd XJTLU-autonomous-vehicle/
hf upload frogcar/rtk-data-2026-surf ./runtime-data --repo-type dataset
```

然后等待上传完成。你可以在这里查看文件：https://huggingface.co/datasets/frogcar/rtk-data-2026-surf/tree/main （联系我获取访问权限）
