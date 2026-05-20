# XJTLU Autonomous Vehicle

ROS 2 Humble monorepo for the XJTLU autonomous vehicle platform. The repository contains the Jetson runtime workspace, sensor drivers, SLAM/localization, Nav2 navigation, GNSS tooling, engineering documentation, and the STM32 lower-controller firmware snapshot used by the current vehicle.

## Current State

- Main deployment target: Jetson Orin NX running Ubuntu 22.04 and ROS 2 Humble.
- Current verified indoor stack: FAST-LIO2 + PGO + Nav2 MPPI, launched through `make launch-indoor-nav`.
- Current outdoor GNSS path: GPS Corridor, launched through `make launch-corridor`.
- GPS corridor software can enter route execution, but ordinary GNSS `/fix` physical-coordinate accuracy remains the main outdoor blocker.
- Runtime parameters are centralized in `src/bringup/config/master_params.yaml`.
- Runtime data and logs live under `~/XJTLU-autonomous-vehicle/runtime-data`.
- Session logs are created by `scripts/launch_with_logs.sh`.

## Hardware And Software Stack

| Layer | Current implementation |
|------|------------------------|
| Compute | Jetson Orin NX, Ubuntu 22.04, ROS 2 Humble |
| LiDAR / IMU | Livox MID360 and WIT/BMI088-related IMU support |
| GNSS | Basic NMEA GNSS receiver, no RTK in the current stack |
| Lower controller | STM32 RM C Board over serial command bridge |
| Localization | FAST-LIO2 front end + PGO back end |
| Navigation | Nav2 MPPI with project-specific bringup and behavior trees |
| Runtime data | `runtime-data/` inside the workspace |

## Quick Start

Run these commands on the Jetson workspace:

```bash
cd ~/XJTLU-autonomous-vehicle

make setup
make build
source install/setup.bash
bash scripts/init_runtime_data.sh
```

Build commands must keep `--parallel-workers 1` on Jetson because of memory limits. The `Makefile` already follows that rule.

## Operating Modes

```bash
make launch-slam         # FAST-LIO2 + PGO + SLAM Toolbox mapping workflow
make launch-explore      # FAST-LIO2 + PGO + Nav2 local navigation
make launch-indoor-nav   # Explore stack without GNSS, for RViz click-to-go testing
make launch-corridor     # GPS Corridor runtime on the MPPI baseline
make launch-explore-gps  # Explore mode with GNSS bringup and PGO GPS factor
make launch-nav-gps      # Scene-bundle + route-graph GPS goal navigation workflow
make launch-travel       # Static-map navigation workflow, currently paused
```

All `make launch-*` targets go through `scripts/launch_with_logs.sh`, which creates a per-session directory under `runtime-data/logs/`.

Stop runtime processes with:

```bash
make kill-runtime
```

## Repository Layout

```text
src/
  bringup/          Launch files, Nav2 configs, maps, behavior trees, RViz assets
  sensor_drivers/   Livox, WIT IMU, GNSS, serial, and lower-controller interfaces
  perception/       FAST-LIO2, PGO, point cloud conversion, occupancy-grid helpers
  planning/         GNSS global planning and coordinate transforms
  navigation/       Waypoint tools and GPS route dispatch
  firmware/         STM32 RM C Board lower-controller snapshot

docs-CN/            Active Chinese engineering documentation
docs-EN/            Active English engineering documentation
scripts/            Runtime helpers, data collection, plotting, and log tooling
dependencies.repos  vcs import manifest for external dependencies
```

## Build And Test

Common build targets:

```bash
make build
make build-sensor
make build-perception
make build-planning
make build-navigation
make test
```

After every build:

```bash
source install/setup.bash
```

Package-level builds follow this form:

```bash
colcon build --packages-select <pkg> --symlink-install --parallel-workers 1
source install/setup.bash
```

## Runtime Data

The active runtime root is:

```text
~/XJTLU-autonomous-vehicle/runtime-data
```

Important subdirectories:

- `logs/` - launch session logs, rosbag output, console logs, and custom node logs.
- `gnss/` - GNSS route files, startup repeatability data, and GNSS calibration artifacts.
- `maps/` - map and route inputs used by mapping and GNSS planning tools.
- `config/` - runtime logging switches and local operational state.

Some code still uses legacy `FYP_*` environment variables as runtime interface names. They are retained for compatibility and should not be renamed without a separate compatibility migration.

## Documentation

- Chinese documentation index: [`docs-CN/index.md`](docs-CN/index.md)
- English documentation index: [`docs-EN/index.md`](docs-EN/index.md)
- System architecture: [`docs-EN/architecture.md`](docs-EN/architecture.md)
- Command reference: [`docs-EN/commands.md`](docs-EN/commands.md)
- Known issues: [`docs-EN/known_issues.md`](docs-EN/known_issues.md)
- Contributing guide (CN): [`CONTRIBUTING.md`](CONTRIBUTING.md)
- Contributing guide (EN): [`CONTRIBUTING-EN.md`](CONTRIBUTING-EN.md)
- STM32 firmware snapshot: [`src/firmware/rm_c_board/README.md`](src/firmware/rm_c_board/README.md)

The `docs-CN/` and `docs-EN/` trees are maintained as paired documentation. Update both when a user-facing engineering fact changes.

## Firmware Snapshot

The lower-controller firmware snapshot is stored at:

```text
src/firmware/rm_c_board/
```

It is kept as a repository-local snapshot for inspection and vehicle integration reference. It is not a ROS 2 package and should not be treated as part of the colcon workspace build.

## Contribution And Safety Boundaries

- Do not modify tuned YAML parameters without documenting why.
- Do not modify imported upstream/vendor code casually.
- Keep Jetson builds at `--parallel-workers 1`.
- Keep runtime interfaces such as `FYP_LOG_SESSION_DIR`, `FYP_RUNTIME_ROOT`, and `FYP_USE_RVIZ` compatible unless a separate migration plan is approved.
- Use specific-file `git add` commands; do not use `git add -A` or `git add .` in this repository.
- Follow [`CONTRIBUTING.md`](CONTRIBUTING.md) before opening or merging changes.
