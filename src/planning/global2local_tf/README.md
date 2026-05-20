# global2local_tf

`global2local_tf` is an experimental GNSS planning helper. It converts longitude/latitude route information into a local planar target frame for early global-planning experiments.

## Current Role

This package is not automatically started by the main `make launch-*` production modes. It remains in the repository for GNSS global path-planning experiments and for comparison with newer GPS Corridor work.

## Interfaces

Subscribed topics:

| Topic | Type | Purpose |
|------|------|---------|
| `/gnss` | `sensor_msgs/msg/NavSatFix` | Calibrated GNSS input |
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | IMU samples used during initialization |
| `/next_node` | `std_msgs/msg/String` | Next route node as `longitude,latitude` |
| `/path4global` | `nav_msgs/msg/Path` | Global route path |

Published topics:

| Topic | Type | Purpose |
|------|------|---------|
| `/target_pos` | `geometry_msgs/msg/PoseStamped` | Local target converted from `/next_node` |
| `/path4local` | `nav_msgs/msg/Path` | Local path converted from global route data |

## Runtime Files

- `runtime-data/planning/angle_offset.txt` provides a heading offset.
- If the file is missing, the node falls back to `0.0`.

## Build And Run

```bash
cd ~/XJTLU-autonomous-vehicle
colcon build --packages-select global2local_tf --symlink-install --parallel-workers 1
source install/setup.bash
ros2 launch global2local_tf global2local_tf.launch.py
```

The launch file also starts the WIT IMU helper launch.

## Current Limitations

- The node uses the first few GNSS and IMU samples to initialize its local frame, so unstable startup data can rotate or shift the whole result.
- The implementation still uses a fixed base heading plus `angle_offset`; it is not a general-purpose geodesy transform.
- Several historical `copy*.py` files remain in the package. The current entry point is `global2local_tf/global2local_tf.py`.
