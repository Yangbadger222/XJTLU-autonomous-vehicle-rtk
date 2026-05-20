# XJTLU Autonomous Vehicle

> ROS 2 Humble monorepo for the Jetson Orin NX autonomous vehicle project.

## Current State

- Main deployment target: Jetson Orin NX running Ubuntu 22.04 and ROS 2 Humble
- Common runtime entry points: `make launch-explore`, `make launch-indoor-nav`, `make launch-corridor`
- GPS-assisted mode available: `make launch-explore-gps`; GPS route-graph mode available: `make launch-nav-gps`
- Current integrated MPPI baseline uses the anti-understeering tune absorbed from the former IEEE demo line: `vx_max=1.0`, `wz_max=1.2`, `ax_max=1.2`, `PathAlignCritic.offset_from_furthest=6`, `PathFollowCritic.cost_weight=16.0`
- Global replanning remains at `5 Hz` with NavFn `A*` enabled in `src/bringup/config/nav2_explore.yaml`
- PGO now supports on-demand `/pgo/global_map` publication and custom RViz layout injection via `ros2 launch pgo pgo_launch.py rviz_config:=...`
- Runtime data lives inside the workspace under `~/XJTLU-autonomous-vehicle/runtime-data`
- Runtime parameters are centralized in `src/bringup/config/master_params.yaml`
- Session logs are created by `scripts/launch_with_logs.sh`

## Quick Start

```bash
# SSH to Jetson
ssh jetson@100.97.227.24

# Enter workspace
cd ~/XJTLU-autonomous-vehicle

# First-time setup
make setup
make build
source install/setup.bash
bash scripts/init_runtime_data.sh

# Launch one operating mode
make launch-slam
make launch-explore
make launch-indoor-nav
make launch-corridor
make launch-explore-gps
make launch-nav-gps
make launch-travel
```

## Repository Structure

```text
src/
├── sensor_drivers/   Livox LiDAR, WIT IMU, GNSS, serial drivers
├── perception/       FAST-LIO2, PGO GPS fusion, pointcloud processing
├── planning/         GNSS global planning, coordinate transforms
├── navigation/       waypoint_collector and navigation tools
├── bringup/          launch files, Nav2 configs, maps, RViz assets
└── firmware/         STM32 RM C Board lower-controller snapshot

docs-CN/              Active engineering documentation (Chinese)
docs-EN/              Active engineering documentation (English)
scripts/              Runtime helpers and data-collection tools
dependencies.repos    vcs import manifest for upstream dependencies
```

## Firmware Snapshot

- Full STM32 RM C Board firmware snapshot is checked in under `src/firmware/rm_c_board/`
- The directory currently keeps the saved source tree, project files, and companion build artifacts together as one repository-local snapshot
- Firmware-local readme: [`src/firmware/rm_c_board/README.md`](src/firmware/rm_c_board/README.md)

## Operating Modes

1. `make launch-slam`
   FAST-LIO2 + PGO + SLAM Toolbox mapping workflow
2. `make launch-explore`
   FAST-LIO2 + PGO + Nav2 local navigation
3. `make launch-indoor-nav`
   Explore stack without GNSS, optimized for RViz click-to-go testing
4. `make launch-corridor`
   GPS Corridor v2 runtime on the MPPI baseline
5. `make launch-explore-gps`
   Explore mode with GNSS bringup and PGO GPS factor enabled
6. `make launch-nav-gps`
   Scene-bundle + route-graph GPS goal navigation workflow
7. `make launch-travel`
   Static-map navigation workflow, currently paused

All seven `make launch-*` targets go through `scripts/launch_with_logs.sh`, which creates a per-session log directory under `~/XJTLU-autonomous-vehicle/runtime-data/logs/`.

## Documentation

Project docs are maintained in two languages:
- **Chinese**: [`docs-CN/`](docs-CN/index.md)
- **English**: [`docs-EN/`](docs-EN/index.md)

Both directories have identical structure. When updating docs, update both.

- [Documentation Index](docs-EN/index.md)
- [System Architecture](docs-EN/architecture.md)
- [Command Reference](docs-EN/commands.md)
- [STM32 RM C Board Firmware Snapshot](src/firmware/rm_c_board/README.md)
- [Contributing Guide (CN)](CONTRIBUTING.md)
- [Contributing Guide (EN)](CONTRIBUTING-EN.md)
- [Known Issues](docs-EN/known_issues.md)
- [PGO Notes](docs-EN/knowledge/pgo.md)
- [GPS Planning Notes](docs-EN/knowledge/gps_planning.md)

## Development Workflow

1. Sync `main` and create a branch from it.
2. Implement and verify on the branch.
3. Rebuild with `--parallel-workers 1`.
4. `source install/setup.bash` after every build.
5. Push the branch and open a PR to `main`.
6. Merge through PR, then sync Jetson back to `main`.
7. Update affected docs in **both** `docs-CN/` and `docs-EN/` before the session is considered complete.

See [`CONTRIBUTING.md`](CONTRIBUTING.md) for the full development conventions and documentation trigger rules.

## Hardware Snapshot

- Compute: Jetson Orin NX, 16 GB RAM
- LiDAR: Livox MID360
- IMU: WIT IMU
- GNSS: basic GPS receiver, about 2.5 m class accuracy, no RTK in the current stack
- Drive interface: serial link to STM32 lower controller board
- Manual override: PS2 wireless controller, highest priority

## Key Rules

1. Do not modify tuned YAML parameters without documenting why.
2. Always build with `--parallel-workers 1`.
3. Always source `install/setup.bash` after a build.
4. Do not modify imported upstream dependencies casually; this repository no longer keeps a checked-in `src/third_party/` tree.
5. Do not push directly to `main`; use PRs.
