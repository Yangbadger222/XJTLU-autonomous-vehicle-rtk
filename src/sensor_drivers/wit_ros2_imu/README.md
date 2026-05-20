# wit_ros2_imu

`wit_ros2_imu` is the ROS 2 driver package for the WIT IMU used in the XJTLU vehicle workspace. It currently publishes raw IMU data for experiments and standalone debugging.

## Current Role

- Main output: `/imu/data_raw`
- Message type: `sensor_msgs/msg/Imu`
- Default device used by the implementation: `/dev/imu_usb`
- Default baud rate in launch configuration: `9600`

This package is not currently the primary IMU source for FAST-LIO2. The main localization chain still relies on the Livox-side IMU data used by the active FAST-LIO2 integration.

## Interfaces

Published topics:

| Topic | Type | Purpose |
|------|------|---------|
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | Parsed acceleration, angular velocity, and orientation fields |

## Build And Run

```bash
cd ~/XJTLU-autonomous-vehicle
colcon build --packages-select wit_ros2_imu --symlink-install --parallel-workers 1
source install/setup.bash
ros2 run wit_ros2_imu wit_ros2_imu
```

Launch with RViz helper:

```bash
ros2 launch wit_ros2_imu rviz_and_imu.launch.py
```

## Runtime Logs

- Managed launch sessions use the active `runtime-data/logs/latest/` session.
- Standalone runs fall back to `runtime-data/logs/wit_imu_log/`.
- Log switches are controlled through `runtime-data/config/log_switch.yaml`.

## Current Limitations

- `rviz_and_imu.launch.py` declares a `port` parameter, but the main implementation still opens the device path inside the driver loop. Treat launch-level port switching as not fully wired until this is refactored.
- Magnetometer data is parsed internally but not published as a separate ROS topic.
- Use this package as an experimental/debug driver unless the active localization plan explicitly assigns it to the production chain.
