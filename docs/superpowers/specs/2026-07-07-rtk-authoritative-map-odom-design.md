# RTK Authoritative Map Odom Design

## Goal

Make outdoor corridor navigation use RTK as the authoritative global pose source so a 1 m FAST-LIO local drift does not stop or mislead Nav2. Keep the interface compatible with a later indoor/outdoor transition that seeds prior-map relocalization from the last accurate outdoor RTK pose.

## Architecture

Nav2 continues to consume the same TF chain:

```text
map -> odom -> base_link
```

FAST-LIO2 still owns `odom -> base_link` for local continuity and point-cloud obstacle projection. A new localization authority node owns `map -> odom` in RTK-authoritative corridor mode. PGO must not publish `map -> odom` in this mode.

The authority node computes:

```text
T_map_odom = T_map_base_from_rtk * inverse(T_odom_base_from_fastlio)
```

`T_map_base_from_rtk` comes from `/fix` projected through `/gps_corridor/enu_to_map`, plus `/heading` for yaw. `T_odom_base_from_fastlio` comes from the current TF buffer.

## First Version

Add `rtk_map_odom_corrector_node` in `gps_waypoint_dispatcher`.

Inputs:

- `/fix`
- `/heading`
- `/gps_corridor/enu_to_map`
- TF lookup `odom -> base_link`

Outputs:

- TF `map -> odom`
- `/localization_authority/mode`
- `/localization_authority/status`
- `/localization_authority/diagnostics`

Safety rules:

- If no external `ENU -> map` has arrived yet, bootstrap a temporary alignment from current RTK fix, heading, and `odom -> base_link`; this publishes an initial `map -> odom` so `gps_global_aligner` can obtain `map -> base_link` without relying on PGO.
- Publish only when alignment is valid, fix is finite and recent, heading is recent, and `odom -> base_link` is available.
- Limit single-step `map -> odom` translation and yaw corrections.
- If RTK correction disagrees with local odom beyond the configured hard threshold, publish a status warning and stop updating rather than jumping.
- Corridor launch disables PGO `publish_tf` so there is exactly one `map -> odom` publisher.

## Indoor/Outdoor Interface

The node exposes mode/status/diagnostics as a stable authority interface:

```text
LOCAL_LIO_ONLY
RTK_BOOTSTRAP
RTK_AUTHORITATIVE
RTK_DEGRADED
PRIOR_MAP_SEED_READY
PRIOR_MAP_AUTHORITATIVE
```

The first implementation uses `RTK_BOOTSTRAP`, `RTK_AUTHORITATIVE`, and degraded/no-output states. Later, when the vehicle is still outside with good RTK, the system can export the current `map -> base_link` pose as a seed for prior-map relocalization. After the indoor prior map validates that seed, a prior-map authority can take over `map -> odom` without changing Nav2 or the route runner.

## Testing

Use ROS-free pure functions for the core transform math and gate decisions:

- `T_map_odom = T_map_base * inverse(T_odom_base)`
- step limiting for translation and yaw
- stale/missing input gate
- hard divergence gate

Launch tests verify corridor starts the new node and disables PGO TF ownership.
