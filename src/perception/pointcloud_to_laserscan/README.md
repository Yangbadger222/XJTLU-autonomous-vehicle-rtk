# pointcloud_to_laserscan

This ROS 2 package converts between `sensor_msgs/msg/PointCloud2` and `sensor_msgs/msg/LaserScan`. It is kept under `src/perception/pointcloud_to_laserscan` for compatibility with the XJTLU vehicle workspace.

## Role In This Repository

The package provides conversion components for experiments and perception compatibility layers. The active FAST-LIO2 + PGO + Nav2 stack primarily uses point cloud and costmap paths, so verify the launch file before assuming this converter is active in a specific runtime mode.

## Components

### `pointcloud_to_laserscan::PointCloudToLaserScanNode`

Subscribes:

| Topic | Type | Purpose |
|------|------|---------|
| `cloud_in` | `sensor_msgs/msg/PointCloud2` | Input point cloud |

Publishes:

| Topic | Type | Purpose |
|------|------|---------|
| `scan` | `sensor_msgs/msg/LaserScan` | Projected laser scan |

Key parameters include `min_height`, `max_height`, `angle_min`, `angle_max`, `angle_increment`, `range_min`, `range_max`, `target_frame`, `transform_tolerance`, and `use_inf`. An optional axis-aligned vehicle exclusion box is configured with `self_filter.enabled` and `self_filter.{min,max}_{x,y}`. The box is evaluated after transforming the cloud into `target_frame`; enabling it without a target frame is rejected at startup.

### `pointcloud_to_laserscan::LaserScanToPointCloudNode`

Subscribes:

| Topic | Type | Purpose |
|------|------|---------|
| `scan_in` | `sensor_msgs/msg/LaserScan` | Input scan |

Publishes:

| Topic | Type | Purpose |
|------|------|---------|
| `cloud` | `sensor_msgs/msg/PointCloud2` | Reconstructed point cloud |

## Build

```bash
cd ~/XJTLU-autonomous-vehicle
colcon build --packages-select pointcloud_to_laserscan --symlink-install --parallel-workers 1
source install/setup.bash
```

## Notes

- Input messages are only processed when there is at least one subscriber on the output topic.
- Use `target_frame` only when the relevant TF transform is available.
- Treat this package as a conversion utility, not as a full obstacle-processing pipeline.
