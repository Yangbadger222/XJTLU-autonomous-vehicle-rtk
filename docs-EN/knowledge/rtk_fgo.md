# RTK FGO Tight-Coupled Outdoor Navigation Design

## 1. Status and Scope

This page is both a design note and an early implementation record. A minimal `ament_cmake` skeleton for `rtk_fgo_localizer` now exists, with RTK GGA quality parsing, gate-decision core logic, the indoor/outdoor RTK recovery state machine, the correction smoother, a minimal GTSAM graph core, a ROS shadow node, and matching gtests. The launch file, Make target, and Nav2 remap have not been implemented yet.

The planned system must be introduced as a new experimental mode. It must not replace or silently alter the current `corridor`, `explore-gps`, or `nav-gps` chains.

Target first implementation boundary:

- New package: `src/perception/rtk_fgo_localizer/` (minimal skeleton exists)
- Planned launch file: `src/bringup/launch/system_tightly_coupled.launch.py`
- Planned parameter file: `src/bringup/config/rtk_fgo.yaml`
- Planned runtime command: `make launch-tightly-coupled`
- Default behavior: shadow/localization output first; no ownership of the existing `map -> odom` TF until explicitly enabled in the experimental mode

Current implemented scope only covers:

- `rtk_quality.hpp/.cpp`: parses raw GGA quality, satellite count, and HDOP, then performs first-stage gate decisions from RTK Fixed/Float status, innovation, and heading residual.
- `test_rtk_quality.cpp`: covers Fixed as a strong candidate, Float as a weak candidate, and rejection of a large Fixed-position innovation.
- `state_machine.hpp/.cpp`: implements the base transition rules for `LOCAL_ONLY`, `RTK_CANDIDATE`, `RTK_RECOVERY`, `RTK_LOCKED`, `RTK_DEGRADED`, and `FAULT_HOLD`.
- `correction_smoother.hpp/.cpp`: limits per-step translation and yaw correction so trusted RTK recovery cannot create a single output jump.
- `fgo_graph.hpp/.cpp`: implements the minimal GTSAM graph API for initial states, FAST-LIO relative pose, wheel planar relative pose, RTK position shadow commit/reject, and the RTK heading yaw factor.
- `yaw_factor.hpp/.cpp`: implements a yaw-only Pose3 factor for dual-antenna RTK heading as an absolute yaw candidate constraint.
- The IMU preintegration API exists, but the first version does not add actual IMU factors yet; that should continue after confirming the Jetson GTSAM API version.
- `topic_buffers.hpp/.cpp`: provides a ROS-free timestamped sample buffer for time-based sensor lookup.
- `rtk_fgo_node.cpp`: implements the ROS shadow node. It subscribes to FAST-LIO odometry, IMU, wheel odometry, `/fix`, `/heading`, `/rtk/status`, and `/rtk/nmea_sentence`; it publishes `/rtk_fgo/odom`, `/rtk_fgo/path`, `/rtk_fgo/status`, `/rtk_fgo/rtk_gate`, `/rtk_fgo/correction_status`, and `/rtk_fgo/factor_diagnostics`.
- `publish_tf` must stay `false` by default. Even when manually enabled, the node may only broadcast the experimental `map -> odom_fgo`, never the production `map -> odom`.
- The current implementation still has no launch file or Make target and changes no existing navigation mode.

## 2. Existing System Boundary

The current system splits localization responsibilities:

- FAST-LIO2 publishes high-rate local odometry: `odom -> base_link`
- PGO publishes global correction: `map -> odom`
- Corridor mode disables PGO GPS factors and uses `gps_global_aligner_node` to publish a smoothed `ENU->map` route projection transform
- UM982 RTK is already the GNSS source for GPS modes, publishing `/fix`, `/heading`, `/rtk/status`, and `/rtk/nmea_sentence`

The new tight-coupled design must run beside this stack first. It may publish `/rtk_fgo/odom` and diagnostics, but it must not publish the production `map -> odom` transform in the first stage.

One current topic mismatch must be handled explicitly: documentation often says `odom_CBoard`, while `serial_reader_node.cpp` currently publishes `odom_CBoar`. The first implementation should parameterize the wheel/chassis odometry topic and use the real executable topic as the default until the topic name is corrected deliberately.

## 3. Target Data Flow

Planned inputs:

```text
/fastlio2/lio_odom        nav_msgs/Odometry
/livox/imu                sensor_msgs/Imu
/odom_CBoar               nav_msgs/Odometry
/fix                      sensor_msgs/NavSatFix
/heading                  geometry_msgs/QuaternionStamped
/rtk/status               std_msgs/String
/rtk/nmea_sentence         nmea_msgs/Sentence
```

Planned outputs:

```text
/rtk_fgo/odom                 nav_msgs/Odometry
/rtk_fgo/path                 nav_msgs/Path
/rtk_fgo/status               std_msgs/String
/rtk_fgo/factor_diagnostics   diagnostic_msgs/DiagnosticArray or Float32MultiArray
/rtk_fgo/rtk_gate             std_msgs/String
/rtk_fgo/correction_status    std_msgs/Float32MultiArray
```

Optional experimental TF:

```text
map -> odom_fgo
```

The optional TF must stay disabled by default. If Nav2 later uses the FGO chain, that must happen only in the experimental launch file with explicit frame and parameter separation from the production modes.

## 4. State and Factor Design

Each sliding-window state should contain:

```text
X_k = {
  pose: T_map_base,
  velocity: v_map,
  imu_bias: b_acc, b_gyro
}
```

Optional future states include wheel scale, wheel yaw bias, and slow RTK map offset. These should not be estimated in the first version unless bag replay proves the base graph is stable.

Planned factor set:

```text
X_k -- IMU preintegration factor ---------- X_{k+1}
X_k -- FAST-LIO relative pose factor ------ X_{k+1}
X_k -- wheel planar odom / yaw-rate factor- X_{k+1}
X_k -- RTK position factor
X_k -- RTK heading factor
X_k -- marginal prior from old window
```

Sensor roles:

- IMU preintegration preserves short-term continuity when RTK is missing or degraded.
- FAST-LIO2 relative pose is the main local geometric constraint and should carry the existing system's stable local behavior.
- Wheel/chassis odometry adds planar motion and yaw-rate constraints, especially for low-speed motion, in-place turns, and recovery behavior.
- RTK position provides the global absolute position candidate, but only after quality and residual gating.
- Dual-antenna RTK heading provides an absolute yaw factor when heading is stable.
- Marginal prior preserves historical information when old states leave the fixed window.

## 5. RTK Gating and Recovery

RTK Fixed is not blindly treated as truth. It is the highest-priority absolute-position candidate, but it must pass consistency checks before becoming a strong factor.

RTK Fixed/Float classification must not rely on `NavSatStatus` alone. Both GGA quality `4` and `5` map to `STATUS_GBAS_FIX`, so the FGO gate must read raw GGA from `/rtk/nmea_sentence` or a future structured RTK quality topic with parser tests.

Quality policy:

```text
GGA q=4 RTK Fixed  -> strong position factor candidate
GGA q=5 RTK Float  -> weak factor or diagnostic-only observation
GGA q=1/2/9        -> diagnostic by default
q=0 / invalid      -> ignored by the graph

Stable /heading    -> yaw factor candidate
Invalid heading    -> ignored by the graph
```

Large-jump handling:

1. Gate by quality: GGA quality, covariance/HDOP, satellite count, heading source, heading spread, NTRIP/RTCM state.
2. Gate by physical continuity: implied RTK speed, yaw rate, and direction must match FAST-LIO2, IMU, and wheel predictions within configured limits.
3. Gate by graph residual: use innovation or Mahalanobis residual thresholds before accepting an RTK factor.
4. Gate by persistence: a single good point must not recover the system. Recovery requires consecutive consistent samples.

Recommended commit strategy:

```text
1. Copy the active window into a shadow graph.
2. Add candidate RTK position and heading factors.
3. Optimize the shadow graph.
4. Check proposed correction size, yaw change, local velocity continuity, and non-RTK factor residuals.
5. Commit only if the candidate solve passes all checks.
6. Otherwise discard the candidate RTK factors and keep the current local solution.
```

This allows the system to pull the trajectory back when RTK recovers, while avoiding sudden jumps from multipath or receiver state glitches.

## 6. Indoor/Outdoor Transition State Machine

Planned states:

```text
LOCAL_ONLY
RTK_CANDIDATE
RTK_LOCKED
RTK_DEGRADED
RTK_RECOVERY
FAULT_HOLD
```

Indoor to outdoor:

1. Start in `LOCAL_ONLY` when RTK is missing or invalid.
2. Enter `RTK_CANDIDATE` when valid RTK observations appear.
3. Require consecutive quality, heading, residual, and speed-consistency checks.
4. Enter `RTK_RECOVERY`, add trusted RTK factors to the recent window, and optimize.
5. Release correction through a smoother instead of instantly changing the output pose.
6. Enter `RTK_LOCKED` after residuals remain stable.

Outdoor to indoor:

1. Drop RTK factor strength when quality degrades or residuals become abnormal.
2. Move through `RTK_DEGRADED` to `LOCAL_ONLY`.
3. Keep the last trusted global anchor, but report lower global confidence.
4. Continue with FAST-LIO2, IMU, and wheel constraints.

Correction smoothing should cap per-cycle output changes, for example:

```text
translation <= 0.10-0.20 m per update
yaw <= 0.2-0.5 deg per update
```

The exact values require replay and vehicle validation.

## 7. Planned Parameters

Initial parameter groups:

```yaml
/rtk_fgo_localizer:
  ros__parameters:
    topics:
      fastlio_odom: /fastlio2/lio_odom
      imu: /livox/imu
      wheel_odom: /odom_CBoar
      fix: /fix
      heading: /heading
      rtk_status: /rtk/status

    window:
      duration_s: 15.0
      keyframe_rate_hz: 10.0
      max_states: 120

    factors:
      imu_enabled: true
      fastlio_enabled: true
      wheel_enabled: true
      rtk_position_enabled: true
      rtk_heading_enabled: true

    rtk_gating:
      require_fixed_for_strong_factor: true
      fixed_quality_code: 4
      float_quality_code: 5
      recovery_min_samples: 8
      max_implied_speed_mps: 2.0
      max_position_jump_m: 3.0
      max_heading_spread_deg: 3.0

    correction_smoother:
      max_translation_step_m: 0.15
      max_yaw_step_deg: 0.3
```

These values are starting points, not tuned vehicle parameters. Any later tuned YAML change must include a documented reason in the knowledge docs and devlog.

## 8. Verification Plan

Stage 1: ROS-free core tests

- RTK quality gate
- residual gate
- correction smoother
- sliding-window bookkeeping
- state-machine transitions

Stage 2: rosbag replay shadow mode

- Run the existing localization stack and `rtk_fgo_localizer` in parallel.
- Record `/rtk_fgo/*`, source sensor topics, `/tf`, `/cmd_vel`, and RTK raw/status topics.
- Confirm that no production TF is changed.

Stage 3: vehicle shadow mode

- Launch the experimental stack with `publish_tf: false` and `nav2_use_fgo: false`.
- Check CPU, memory, topic rates, state transitions, and RTK gate decisions.
- Store the session under `runtime-data/logs/<timestamp>/`.

Stage 4: guarded experimental navigation

- Only after shadow validation, enable an experimental TF and Nav2 path inside the new launch mode.
- Keep the existing production `corridor`, `explore-gps`, and `nav-gps` modes unchanged.
- Use the PS2 `X` motor-disable and physical red e-stop as required safety controls. Do not use `B` as an emergency stop.

## 9. Development Rules

- Do not edit code directly on the Jetson.
- Do not push directly to `main`.
- Build on the Jetson with `--parallel-workers 1`.
- Re-source `install/setup.bash` after every build.
- Stage files explicitly; do not use `git add -A` or `git add .`.
- If RTK parsing or structured RTK quality output changes, add raw-NMEA sample tests for `um982_rtk_driver`.
- Never commit CORS/NTRIP credentials.
- Keep CN/EN documentation synchronized for package, launch, parameter, workflow, and GPS/GNSS changes.
