# gnss_global_path_planner

`gnss_global_path_planner` contains the early GeoJSON + A* GNSS route-planning experiment. It is useful for route-graph research and tooling, but it is not the current production GPS Corridor runner.

## Current Role

The active production GNSS corridor work uses dedicated route collection, alignment, and runner nodes. This package remains as a planning experiment for:

- Reading GeoJSON route maps.
- Running node-level A* planning.
- Publishing the next route node for local conversion experiments.

## Interfaces

| Interface | Type | Purpose |
|----------|------|---------|
| `/gnss` | `sensor_msgs/msg/NavSatFix` | Calibrated GNSS input |
| `/next_node` | `std_msgs/msg/String` | Published next graph node as `longitude,latitude` |

## Build

```bash
cd ~/XJTLU-autonomous-vehicle
colcon build --packages-select gnss_global_path_planner --symlink-install --parallel-workers 1
source install/setup.bash
```

## Run

Combined experimental launch:

```bash
ros2 launch gnss_global_path_planner gnss_combined_launch.py
```

Standalone planner:

```bash
ros2 run gnss_global_path_planner global_path_planner.py
```

## Map Format

The planner reads GeoJSON route data:

- `Point` features represent graph nodes.
- `LineString` features represent edges between nodes.

Example maps live under this package's `map/` directory.

## Current Limitations

- This is not the `make launch-explore-gps` production entry point.
- GPS-to-Nav2 target conversion is still experimental.
- The package should not be treated as the validated 1-2 km outdoor navigation stack without a new integration plan.
