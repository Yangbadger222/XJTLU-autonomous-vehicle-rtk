# gnss_calibration

`gnss_calibration` converts raw GNSS `/fix` messages into calibrated `/gnss` messages for PGO GPS factors and GNSS experiments.

## Runtime Chain

```text
nmea_navsat_driver -> /fix -> gnss_calibration -> /gnss
```

`make launch-explore-gps` starts this package through `system_explore_gps.launch.py`.

## Interfaces

Subscribed topics:

| Topic | Type | Purpose |
|------|------|---------|
| `/fix` | `sensor_msgs/msg/NavSatFix` | Raw GNSS fix from `nmea_navsat_driver` |

Published topics:

| Topic | Type | Purpose |
|------|------|---------|
| `/gnss` | `sensor_msgs/msg/NavSatFix` | Calibrated GNSS fix for downstream consumers |

Parameters:

| Parameter | Purpose |
|----------|---------|
| `calibration_points_file` | Optional explicit calibration-point source |

## Runtime Files

- `runtime-data/gnss/startid.txt` - selected calibration point ID, expected to be `1` through `4`.
- `runtime-data/gnss/gnss_offset.txt` - generated latitude/longitude offset after successful calibration.
- Managed-session logs are written under `runtime-data/logs/latest/`; standalone logs fall back to `runtime-data/logs/gnss_calibration/`.

## Calibration Behavior

Invalid samples are skipped when:

- GNSS status is negative.
- Latitude, longitude, or altitude is not finite.
- Latitude and longitude are both `0.0`.
- Covariance type is unknown.

The node writes a new offset only after five consecutive samples stay within the configured stable range. This protects the system from updating offsets when no valid GNSS fix is available.

## Build And Run

```bash
cd ~/XJTLU-autonomous-vehicle
colcon build --packages-select gnss_calibration --symlink-install --parallel-workers 1
source install/setup.bash
ros2 launch gnss_calibration gnss_calibration_launch.py
```

## Field Notes

- The current receiver is ordinary GNSS, not RTK.
- If the field site has no stable fix, `gnss_offset.txt` should not refresh.
- `startid.txt` must match the actual physical start point for the day.
