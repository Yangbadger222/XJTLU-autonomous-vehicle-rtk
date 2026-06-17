# serial_twistctl

`serial_twistctl` is the ROS 2 bridge from Nav2 velocity commands to the STM32 lower controller. It subscribes to `/cmd_vel`, formats the command expected by the lower controller, and writes it to the configured serial port.

## Role In The Vehicle

This package is part of the production runtime chain. In normal navigation modes, Nav2 publishes `/cmd_vel`, this node forwards the command over serial, and the STM32 board drives the chassis.

## Interfaces

Subscribed topics:

| Topic | Type | Purpose |
|------|------|---------|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Linear and angular velocity command from Nav2 or an upper controller |

ROS topics published:

- None. The node writes directly to the serial port.

Serial command format:

```text
vcx=<linear.x>,wc=<angular.z * angular_z_scale>
```

The actual transmitted command ends with a newline.

## Parameters

| Parameter | Default | Purpose |
|----------|---------|---------|
| `port` | `/dev/serial_twistctl` | udev-managed serial device for the lower controller |
| `baudrate` | `115200` | Serial baud rate |
| `send_attempts` | `1` | Number of send attempts per command |
| `delay_between_attempts_ms` | `0` | Delay between repeated sends |
| `angular_z_scale` | `1.0` | Scale/sign applied when mapping ROS `angular.z` to chassis `wc`; set to `-1.0` when the STM32 firmware treats positive `wc` as a right turn |

Parameters are normally supplied through `src/bringup/config/master_params.yaml` and the active launch file.

## Build And Run

```bash
cd ~/XJTLU-autonomous-vehicle
colcon build --packages-select serial_twistctl --symlink-install --parallel-workers 1
source install/setup.bash
ros2 run serial_twistctl serial_twistctl_node
```

## Runtime Logs

- In a managed launch session, logs are written under `runtime-data/logs/latest/`.
- Outside a managed session, the node falls back to `runtime-data/logs/twist_log/`.

## Troubleshooting

- Confirm `/dev/serial_twistctl` exists.
- Confirm the lower controller is powered.
- Confirm udev rules are active instead of temporarily relying on `chmod`.
- Confirm `master_params.yaml` has not changed the serial port or baud rate unexpectedly.
