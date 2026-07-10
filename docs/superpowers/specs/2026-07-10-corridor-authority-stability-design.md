# Corridor Authority Stability Design

## Goal

Prevent RTK/LIO timing skew and rejected RTK heading samples from producing fast
`map -> odom` corrections that destabilize Nav2. Preserve RTK as the outdoor
global reference while keeping FAST-LIO2 responsible for continuous local
motion.

## Evidence

The four 2026-07-10 corridor sessions separate three failures:

- `13:34`: GNSS and LIO both stopped after about 5.8 m while Nav2 continued to
  request `vx=0.85`. This is a drivetrain/feedback failure, not an RTK heading
  failure.
- `13:36`: RTK heading contained about 52 degree single-sample jumps and RTK
  quality degraded. The FGO shadow gate rejected most samples.
- `13:46` and `13:48`: RTK heading and LIO yaw remained broadly consistent,
  but the authority raw/output transform gap grew to about 8 m and 13 m. The
  current 0.20 m per 50 ms transform limit saturated before the route watchdog
  aborted waypoint 4.

The current node combines the latest fix, latest heading, and latest odom pose
even though they represent different times. During a fast turn, a few degrees
of timing error at a 35 m odom radius becomes meters of `map -> odom`
translation. Component-wise transform smoothing then releases that error at up
to 4 m/s.

## Architecture

The TF ownership stays unchanged in corridor mode:

```text
rtk_map_odom_corrector: map -> odom
FAST-LIO2:              odom -> base_footprint -> base_link
```

The corrector will estimate yaw and translation corrections independently from
timestamp-aligned observations instead of constructing one pose from unrelated
latest messages.

### Heading observation

For each new heading sample at `t_heading`:

1. Transform the RTK compass heading into map yaw through the accepted
   `ENU -> map` alignment.
2. Look up `odom -> base` at `t_heading`, not at timer callback time.
3. Form a yaw-correction observation:

   ```text
   yaw_map_odom = yaw_map_base_rtk(t) - yaw_odom_base(t)
   ```

4. Gate the correction innovation against the last accepted correction.

Raw heading rate is diagnostic only. A physical turn may legitimately exceed
30 degrees per second, so the primary gate is RTK-versus-LIO innovation.

### Position observation

For each new fix at `t_fix`:

1. Project the fix through the accepted `ENU -> map` alignment.
2. Look up `odom -> base` at `t_fix`.
3. Use the accepted yaw correction to solve the corresponding translation
   correction.
4. Gate position-correction innovation separately from heading.

Each sensor sample is processed once. Repeated timer ticks only rebroadcast the
last accepted output.

## Gate State

Replace single-sample `YAW_REACQUIRE` with an explicit state machine:

- `LOCKED`: accept observations inside the innovation gates.
- `SUSPECT`: freeze the affected correction after one rejected observation.
- `REACQUIRING`: require a configured number of mutually consistent samples.
- `DEGRADED`: stale, non-fixed, or persistently inconsistent RTK; rebroadcast
  the last safe transform.

RTK GGA quality 4 is required but not sufficient. A heading observation must
also be fresh and consistent with local yaw. Reacquisition never accepts a
single 20-45 degree jump.

## Correction Release

Limits become rates multiplied by measured `dt`, expressed as m/s and deg/s.
The release is limited in `map -> base` space:

1. Compose the previous and target `map -> odom` transforms with the current
   `odom -> base` pose.
2. Bound the resulting base translation and yaw change.
3. Recompute `map -> odom` from the bounded map-base pose.

This keeps yaw and translation coupled around the vehicle rather than smoothing
transform coefficients independently. If the target/output backlog exceeds a
hard runtime threshold, freeze correction and publish `CORRECTION_BACKLOG`
instead of chasing it while moving.

## Route Watchdog

The route runner will monitor three signals separately:

- `odom -> base`: local LIO continuity. Persistent physically impossible motion
  can abort the route.
- `map -> odom`: global correction motion. A large update preempts the current
  goal and waits for authority stability; it does not report odom divergence.
- `map -> base`: Nav2-visible pose, retained for goal progress and diagnostics.

Step rates use TF header timestamps. Wall-clock callback spacing is not a valid
motion interval when TF callbacks are backlogged.

## Command Containment

Corridor mode adds a command guard after Nav2 velocity smoothing:

```text
Nav2 -> velocity_smoother -> /cmd_vel_nav -> corridor_cmd_vel_guard -> /cmd_vel
```

The guard preserves straight-line speed but caps linear velocity when angular
velocity is high using a configurable `v * abs(w)` limit. It also applies a low
degraded-mode speed cap when localization authority is not locked. Missing
input commands time out to zero.

Uniform route subgoal insertion is deferred until localization and command
stability pass replay and low-speed vehicle acceptance. It cannot repair a bad
global transform.

## Compatibility And Diagnostics

- Keep existing authority topics and the first 12 diagnostic array fields.
- Append gate state, observation innovation, and correction backlog metrics.
- Keep PGO TF disabled in corridor mode.
- Keep route files and other navigation modes unchanged.

## Testing

ROS-free tests cover correction observations, circular yaw interpolation, the
reacquisition state machine, base-space rate limiting, backlog freeze, watchdog
classification, and command limiting.

The four captured bags are replay acceptance fixtures outside the repository:

- `13:36` must freeze the heading outlier and must not reacquire from one sample.
- `13:46` and `13:48` must not sustain a maximum-rate transform chase or trigger
  local odom aborts.
- `13:34` must remain classified as local motion stopped while commands remain
  nonzero.

Jetson validation is build/run only. Initial vehicle testing remains low speed
with the PS2 `X` disable and physical e-stop continuously available.

