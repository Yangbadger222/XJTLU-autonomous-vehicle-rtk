# FRC Dual-Anchor Risk Memory

## Purpose

The FRC (Failure/Risk Context) stack records, replays, and optionally injects residual risk alongside the Explore/Nav2 baseline. The default mode is `off`. Experiments should start with `shadow`, where the risk grid is published but costmap injection remains disabled, and only move to `full` after the overlay and baseline behavior are checked.

## Runtime Modes

- `off`: do not start FRC online nodes.
- `shadow`: start `frc_health_aggregator`, `frc_event_marker`, `frc_risk_pipeline`, and `frc_memory_manager`; publish `/frc/risk_grid`; keep `frc_layer.enabled=false` so Nav2 costmap behavior should match the baseline.
- `full`: start the same nodes as `shadow`, then call `/frc/enable` after a delay so `frc_costmap_layer` reads `/frc/risk_grid` and only increases local costmap costs.

Implementation note: `frc_health_aggregator`, `frc_event_marker`, and `frc_risk_pipeline` now run from the C++ `frc_nodes_cpp` package while keeping the same node names, topics, and parameters. `frc_memory_manager` still comes from the Python `frc_nodes` package. The C++ `frc_risk_pipeline` currently covers the default online path: map-anchor risk rendering, `/frc/risk_grid` publishing, watchdog degradation, and debug BEV output. Prototype feature retrieval and TensorRT model inference are not wired yet; if those files are configured, the node warns and continues with map-anchor risk.

Examples:

```bash
FRC_MODE=shadow make launch-explore
FRC_MODE=full make launch-explore
FRC_MODE=shadow FRC_EXTRA_PARAMS=/tmp/frc_overrides.yaml make launch-explore
```

`explore-gps` supports the same `FRC_MODE` / `FRC_EXTRA_PARAMS` pass-through. `trial_runner` is not started by the FRC stack; run it manually for formal experiments and provide the route file explicitly.

## Main Topics

- `/fastlio2/degeneracy`: FAST-LIO2 `Float32MultiArray[min_eig, cond, regularized]` for health aggregation and offline attribution.
- `/pgo/keyframes`: PGO keyframe array for anchor attachment and map-pose refresh after loop correction.
- `/pgo/correction_status`: PGO correction window for training filters and health aggregation.
- `/chassis/status`: STM32 control mode and PS2 key state; old 16-field firmware falls back to zeros.
- `/frc/health`: aggregated FRC health state.
- `/frc/event_marker`: online or manual failure/risk events.
- `/frc/risk_grid`: risk grid in `odom`, consumed by `frc_layer`.
- `/frc/anchor_states`: anchor state snapshots from the memory manager.

## Costmap Injection

The `nav2_explore.yaml` local costmap plugin order is:

```yaml
plugins: ["stvl_layer", "denoise_layer", "frc_layer", "inflation_layer"]
```

`frc_layer` defaults to `enabled: false`. This tuned YAML change is intentional: one configuration covers baseline, shadow, and full runs. Shadow bags stay structurally identical to full bags, while disabled costmap behavior should remain byte-for-byte equivalent to the baseline. Closed-loop injection is enabled only through `/frc/enable` or dynamic parameters.

Safety invariants:

- Never modify `LETHAL_OBSTACLE` or `NO_INFORMATION`.
- Never decrease existing cost.
- Apply a `tau` confidence gate and `max_cost` cap.
- Watchdog timeout on `/frc/risk_grid` automatically bypasses the layer and returns to the geometric baseline.

## Parameters and Overrides

FRC parameters live in `src/bringup/config/master_params.yaml`:

- `/frc_health_aggregator`
- `/frc_event_marker`
- `/frc_risk_pipeline`
- `/frc_memory_manager`
- `/frc_trial_runner`
- `"frc.*"` PGO export parameters under `/pgo.pgo_node`

For experiments, avoid editing tuned YAML directly. Use `FRC_EXTRA_PARAMS=/path/to/overrides.yaml` to append overrides. Path-like parameters use the Jetson default workspace path; override them when the username or workspace location differs.

## Data and Bags

Initialize runtime directories:

```bash
bash scripts/init_runtime_data.sh
```

The script creates:

```text
runtime-data/frc/{models,routes,trials,events,anchor_log}
```

FRC bag recording:

```bash
PROFILE=frc bash scripts/data_collection/record_bag.sh
```

`PROFILE=frc` appends FRC, costmap, PGO keyframe, degeneracy, and Nav2 status topics to the default sensor/localization topics, and records with zstd file compression.
When `FYP_LOG_SESSION_DIR` is set by `launch_with_logs.sh`, the bag defaults to the same session under `data/rosbag2`; otherwise it falls back to `runtime-data/bags/run_*`.
