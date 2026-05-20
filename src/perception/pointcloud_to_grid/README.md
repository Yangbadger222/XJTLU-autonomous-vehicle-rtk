# pointcloud_to_grid

`pointcloud_to_grid` converts LiDAR `sensor_msgs/msg/PointCloud2` data into 2D map representations. This repository vendors and adapts the package under `src/perception/pointcloud_to_grid` for ROS 2 Humble experiments in the XJTLU vehicle workspace.

## Outputs

The node can publish both occupancy-grid and GridMap outputs:

| Topic | Type | Purpose |
|------|------|---------|
| `intensity_grid` | `nav_msgs/msg/OccupancyGrid` | Intensity-derived 2D grid |
| `height_grid` | `nav_msgs/msg/OccupancyGrid` | Height-derived 2D grid |
| `intensity_gridmap` | `grid_map_msgs/msg/GridMap` | Intensity GridMap output |
| `height_gridmap` | `grid_map_msgs/msg/GridMap` | Height GridMap output |

Subscribed input:

| Topic | Type | Default |
|------|------|---------|
| `cloud_in_topic` | `sensor_msgs/msg/PointCloud2` | `nonground` |

## Key Parameters

| Parameter | Default | Purpose |
|----------|---------|---------|
| `mapi_topic_name` | `intensity_grid` | OccupancyGrid intensity topic |
| `maph_topic_name` | `height_grid` | OccupancyGrid height topic |
| `mapi_gridmap_topic_name` | `intensity_gridmap` | GridMap intensity topic |
| `maph_gridmap_topic_name` | `height_gridmap` | GridMap height topic |
| `cloud_in_topic` | `nonground` | Input point cloud topic |
| `cell_size` | `0.5` | Grid cell size |
| `length_x` / `length_y` | `20.0` / `30.0` | Grid dimensions |

The point cloud subscription uses `BEST_EFFORT` reliability to match typical LiDAR publishers.

## Build

```bash
cd ~/XJTLU-autonomous-vehicle
colcon build --packages-select pointcloud_to_grid --symlink-install --parallel-workers 1
source install/setup.bash
```

## Run

Demo launch with the default input topic:

```bash
ros2 launch pointcloud_to_grid demo.launch.py
```

Demo launch with a custom input point cloud:

```bash
ros2 launch pointcloud_to_grid demo.launch.py topic:=my_pointcloud
```

Dual-output example:

```bash
ros2 launch pointcloud_to_grid dual_output_example.launch.py cloud_in_topic:=my_pointcloud
```

## Current Notes

- The package is an experimental perception utility, not the only source of Nav2 costmap data.
- Keep upstream attribution and license information from the vendored source tree intact.
- The old sample bag download instructions from the upstream README are not required for normal XJTLU vehicle operation.
