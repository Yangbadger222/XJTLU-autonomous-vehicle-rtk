# XJTLU Autonomous Vehicle Documentation Index

> Last updated: 2026-07-09

## Current System Summary

- Common runtime entry points: `make launch-explore` / `make launch-indoor-nav` / `make launch-corridor`
- **Indoor click-to-go navigation without GPS**: `make launch-indoor-nav` (RViz goal publishing, no GNSS required)
- GPS fusion mode deployed: `make launch-explore-gps`
- GPS goal navigation mode software deployment completed on `feature/gps-navigation-v4`, passed indoor smoke test: `make launch-nav-gps`
- **GPS Corridor v2 is now integrated into the MPPI mainline baseline**: `make launch-corridor`
  - Controller switched from RotationShim + RPP to MPPI (commit `9d71823`), gaining native sampling-based obstacle avoidance
  - Costmap vehicle-height filtering tuned (commit `ce5226f`): only obstacles within vehicle body height range retained, false obstacles eliminated
  - Obstacle map coverage expanded to 15m (commit `2c2b8e6`), matching Livox MID360 range
  - **2026-04-15 closeout**: absorbed the IEEE demo anti-understeering MPPI baseline into the main line, with `vx_max=1.0`, `wz_max=1.2`, `ax_max=1.2`, `PathAlignCritic.offset_from_furthest=6`, and `PathFollowCritic.cost_weight=16.0`
  - FAST-LIO2 publish cloud pre-height-filtering (commit `f619fa6`), C++ level filtering before downstream consumption
  - **GPS live alignment deployed** (commit `ebc26e2` + `fe3933e` + `e73c2bf`):
    - Calibration switched to translation-only, avoiding rotation flips
    - Startup directly absorbs stable GPS offset
    - Runner uses real-time alignment to recompute subgoals, removed per-waypoint frozen mechanism
  - **Indoor on-vehicle verification passed**: representative full-stack session `2026-03-31-20-51-45` (`indoor-nav`, `gps-mppi@2c2b8e6`) lasted about 16 min 59 s with 1009 `tegrastats` samples; MPPI successfully navigated around a person, completed a full corridor loop with no drift, and kept RAM within `2.676-3.387 GB / 15.289 GB`
  - **GPS outdoor regression test closed out** (2026-04-01): live alignment mechanism deployed, basic alignment issues fixed
  - **Straight-line stability tuning completed** (2026-04-02): Introduced Savitzky-Golay path smoothing + MPPI critic tuning, straight-line tracking achieved "perfectly successful" standard
  - **Dynamic obstacle avoidance recovery and replanning optimized** (2026-04-05): Global replanning raised to 5Hz, Navfn A* search enabled; BT recovery upgraded to 5-level progressive escalation (partial clear -> wait -> spin -> full clear -> backup 1m). This 5Hz replanning + A* baseline remains active.
  - **PGO visualization enhancements are now part of the baseline**: on-demand `/pgo/global_map` publication and custom RViz layout injection through `pgo_launch.py rviz_config:=...`.
  - **2026-05-08 deployment fixes**: alignment shift now uses Nav2 goal preemption, global costmap expanded to `100m x 100m`, and corridor/explore defaults back to the full bringup RViz layout.
  - Current status: **Indoor navigation verified; GPS corridor software chain is deployable and can enter route execution; ordinary GNSS `/fix` physical-coordinate accuracy remains the current main blocker**
- Current navigation and mapping stack: FAST-LIO2 + PGO + Nav2 (MPPI)
- Runtime data root directory: `~/XJTLU-autonomous-vehicle/runtime-data`
- Unified parameter entry point: `src/bringup/config/master_params.yaml`
- Unified logging entry point: `scripts/launch_with_logs.sh`

## Core Documentation

| Topic | File |
|-------|------|
| System architecture and data flow | [architecture.md](architecture.md) |
| Common commands | [commands.md](commands.md) |
| Contributing guide (CN) | [`../CONTRIBUTING.md`](../CONTRIBUTING.md) |
| Contributing guide (EN) | [`../CONTRIBUTING-EN.md`](../CONTRIBUTING-EN.md) |
| Known issues and current blockers | [known_issues.md](known_issues.md) |
| Hardware specifications | [hardware_spec.md](hardware_spec.md) |

## Technical Deep-Dive Documentation

| Topic | File |
|-------|------|
| FAST-LIO2 internals | [knowledge/fastlio2.md](knowledge/fastlio2.md) |
| PGO + GPS Factor | [knowledge/pgo.md](knowledge/pgo.md) |
| Nav2 tuning and runtime constraints | [knowledge/nav2_tuning.md](knowledge/nav2_tuning.md) |
| GPS global navigation and route planning | [knowledge/gps_planning.md](knowledge/gps_planning.md) |

## Development Log

- Found in `devlog/YYYY-MM.md`

## Firmware Snapshot

- The full STM32 RM C Board lower-controller directory is now checked into this repository: [`../src/firmware/rm_c_board/`](../src/firmware/rm_c_board/)
- This path is intentionally preserved as a repository-local snapshot, including the saved source tree, project files, and the companion build artifacts already stored with it
- Firmware-local readme: [`../src/firmware/rm_c_board/README.md`](../src/firmware/rm_c_board/README.md)

## Archive Notes

- `docs-CN/devlog/legacy/` and `docs-EN/devlog/legacy/` contain archived historical documents, kept for reference only and not maintained under the current structure.
- Documentation belonging to dependencies fetched through `dependencies.repos` is outside the scope of this repository's self-maintained documentation.

## Maintenance Principles

1. When code, launch files, parameters, system environment, or workflows change, update the corresponding documentation accordingly.
2. Monthly development records are appended to `devlog/YYYY-MM.md`.
3. New issues or status changes are synced to `known_issues.md`.
4. Command and workflow documentation must reflect the current executable state of the repository, not historical conventions.
