# UM982 RTK Basic Chain

This note covers only first-stage RTK bring-up: UM982 serial access, NMEA parsing, `/fix`, `/heading`, raw NMEA, and optional CORS/NTRIP injection. Full navigation acceptance, route recollection, and tuning are out of scope here.

## Current Implementation

- Driver package: `um982_rtk_driver`
- Language: C++17 / `ament_cmake`
- Default device: `/dev/rtk_um982`
- Default baud: `115200`
- Basic launch: `make launch-rtk-basic`
- GPS mode entry points now use the C++ driver: `explore-gps`, `nav-gps`, `corridor`
- The old `nmea_navsat_driver` remains in-tree only for compatibility/fallback

Published topics:

| Topic | Type | Source |
|---|---|---|
| `/fix` | `sensor_msgs/msg/NavSatFix` | UM982 GGA |
| `/heading` | `geometry_msgs/msg/QuaternionStamped` | UM982 THS/HPR |
| `rtk/nmea_sentence` | `nmea_msgs/msg/Sentence` | checksum-valid raw NMEA |
| `rtk/status` | `std_msgs/msg/String` | human-readable RTK state |

## Heading Calibration

The current dual-antenna installation uses the right antenna as master and the left antenna as secondary. The UM982 raw heading therefore describes the lateral antenna baseline, not the vehicle `base_link +X` forward direction. The driver applies `heading_offset_deg: 88.5` before publishing `/heading`. This value is estimated from RTK Fixed straight-line segments in the 2026-06-24 and 2026-06-29 bags and still needs a dedicated straight-line calibration run.

In `rtk/status`, `heading=` is the calibrated vehicle heading and `raw=` is the receiver's raw lateral-baseline heading. If the antenna master/secondary wiring or physical mounting changes later, adjust `heading_offset_deg` rather than hard-coding compensation in navigation code.

## Jetson Build

```bash
cd ~/XJTLU-autonomous-vehicle
source /opt/ros/humble/setup.bash
make build-rtk-basic
source install/setup.bash
```

`make launch-rtk-basic` launches `um982_rtk_driver` directly, so it does not require the `bringup` package to be installed. Before running `explore-gps`, `nav-gps`, or `corridor`, still build the full vehicle stack through the normal repository workflow.

## Basic Smoke Test

```bash
cd ~/XJTLU-autonomous-vehicle
source /opt/ros/humble/setup.bash
source install/setup.bash
make launch-rtk-basic
```

In another SSH session:

```bash
ros2 topic echo /fix --once
ros2 topic echo /heading --once
ros2 topic echo /rtk/status --once
ros2 topic echo /rtk/nmea_sentence --once
```

Indoors, `/fix.status.status=-1`, GGA quality `0`, `THS` mode `V`, or missing `/heading` is expected. The Jetson already read `$GNGGA,,,,,,0,...` and `INSUFFICIENT_OBS` from `/dev/rtk_um982`, so that indoor state is a signal/sky-view limit, not a code failure.

## Fixed / Float Check

Do not rely on `NavSatStatus` alone:

- GGA quality `4`: RTK Fixed, the acceptance target
- GGA quality `5`: RTK Float, corrections are present but not fixed
- GGA quality `1`: single-point fix
- GGA quality `0`: no fix

Both quality `4` and `5` map to `STATUS_GBAS_FIX` in `NavSatStatus`, so Fixed vs Float must be checked through `rtk/status` or raw GGA in `rtk/nmea_sentence`.

## CORS / NTRIP

Never commit CORS credentials. NTRIP is disabled by default:

```yaml
ntrip:
  enabled: false
```

For field testing, create an untracked temporary parameter file such as `/tmp/um982_cors.yaml`:

```yaml
um982_rtk_driver:
  ros__parameters:
    ntrip:
      enabled: true
      host: "<caster-host>"
      port: 2101
      mountpoint: "<mountpoint>"
      username: "<cors-user>"
      password: ""
      password_env: NTRIP_PASSWORD
      connect_requires_valid_gga: true
```

Pass the password through an environment variable:

```bash
export NTRIP_PASSWORD='do-not-commit'
ros2 launch um982_rtk_driver um982_rtk.launch.py params_file:=/tmp/um982_cors.yaml
```

When launching through repository entrypoints, reuse the same temporary file:

```bash
export NTRIP_PASSWORD='do-not-commit'
export FYP_RTK_PARAMS_FILE=/tmp/um982_cors.yaml
make launch-rtk-basic
```

`FYP_RTK_PARAMS_FILE` is also passed to the UM982 driver in `explore-gps`, `nav-gps`, and `corridor`; navigation, PGO, scene, and other nodes continue using their original parameter files.

The current C++ NTRIP client supports plain TCP NTRIP, not TLS casters. If the CORS caster requires TLS, add a TLS dependency later or let the receiver/vendor 4G module own NTRIP.

## Field Test Order

1. Confirm `/dev/rtk_um982` exists.
2. Run `make launch-rtk-basic`.
3. Check `rtk/status` for GGA, satellite count, and HDOP.
4. In open sky, wait for quality to move from `0/1/5` to `4`.
5. Confirm `/heading` publishes; dual-antenna heading may be absent until the receiver has enough observations.
6. Only then move to `make launch-explore-gps`, `make launch-nav-gps`, or `make launch-corridor`.

Routes/scenes collected with the old G60 must not be used for RTK acceptance. Recollect them under RTK Fixed.
