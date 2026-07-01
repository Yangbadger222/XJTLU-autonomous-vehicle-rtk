# RTK test commands

First, make sure the robot uses your phone hotspot. Do NOT use our campus wifi, it is very unstable. Then, make sure you have tailscale activated.

You will need two terminals.

SSH into the robot:
```bash
ssh badger@100.79.128.21
```

After entering, run:

1. For the first terminal (only one time per day):
```bash
cd XJTLU-autonomous-vehicle/
git switch Tightly-coupled
git pull
colcon build --packages-select bringup --symlink-install --parallel-workers 1
source install/setup.bash
source /opt/ros/humble/setup.bash
```

Wait until it loads. Clear the terminal with `Ctrl L`.

2. For any other terminal afterwards:
```bash
cd XJTLU-autonomous-vehicle/
source install/setup.bash
source /opt/ros/humble/setup.bash
```

## NTRIP Account

The RTK requires an NTRIP account. After purchasing it, run this command (modify ONLY the lines that say `ADD_YOUR_USERNAME_HERE` and `ADD_YOUR_PASSWORD_HERE`):

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

> If you change terminals, get disconnected, or open a new terminal, you must run this command again.

***

## Run and collect data

Open 2 terminals. In the first one, run all the commands above. After that, start the RTK:

```bash
bash scripts/launch_with_logs.sh tightly-coupled
```

In the second terminal, see the RTK signal data. If you just logged in:
```bash
cd XJTLU-autonomous-vehicle/
source install/setup.bash
source /opt/ros/humble/setup.bash
ros2 topic list
ros2 topic echo /rtk/status
```

If you are already in:
```bash
cd XJTLU-autonomous-vehicle/
source /opt/ros/humble/setup.bash
ros2 topic echo /rtk/status
```

You can stop seeing the data by running `Ctrl C`.

The RTK signal status should achieve `q=4`.
- If it is `q=0`, the RTK has no signal. Go outdoors in a clear area.
- If you are completely outside but `q=1`, make sure you are logged in.

Use the custom python script:
```bash
python3 /tmp/rtk_heading_calibrate.py
```

When you are done, simply run this to stop recording data:
```bash
make kill
```

The data will be stored in the robot in `/runtime-data/logs/`.

When you want to transfer the data to your computer, you can use a USB driver, or you can also upload the robot files online. You can run this command:

```bash
cd XJTLU-autonomous-vehicle/
hf upload frogcar/rtk-data-2026-surf ./runtime-data --repo-type dataset
```

Then wait until it uploads. You can see the files here: https://huggingface.co/datasets/frogcar/rtk-data-2026-surf/tree/main (contact me for access)