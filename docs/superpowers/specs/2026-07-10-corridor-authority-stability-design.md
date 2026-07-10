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

## Timestamp Contract

All RTK observations use ROS header time. Receipt monotonic time is used only
for topic-staleness and output-rate calculations.

| Condition | Required behavior |
|---|---|
| Zero stamp | Reject the observation. |
| More than 0.10 s in the future | Reject the observation. |
| Duplicate or older than the last processed stamp | Drop without changing gate state. |
| Exact TF not yet available | Keep one pending sample for at most 0.30 s and retry. |
| TF still unavailable after 0.30 s | Reject as `TF_AT_STAMP_UNAVAILABLE`. |
| Requested stamp outside the TF cache | Reject; never substitute latest TF. |
| Alignment receipt age exceeds 1.0 s | Freeze both correction targets. |

No extrapolation is allowed. A requested `odom -> base` transform must bracket
the observation time in the TF cache; tf2 performs translation and circular
quaternion interpolation. Tests use a maximum bracketing interval of 0.20 s.

GGA quality is read from timestamped raw `/rtk/nmea_sentence` GGA samples,
because `NavSatFix` cannot distinguish Fixed from Float. A fix is eligible only
when its nearest GGA sample has quality 4 and is within 0.25 s. A newer non-4
GGA immediately clears pending fix candidates and degrades both correction
gates. Malformed or stale GGA cannot authorize a fix.

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

The yaw correction used for a fix is the newest accepted heading correction at
or before `t_fix`, with maximum age 0.30 s. Position updates are frozen until a
heading correction is locked. If heading leaves `LOCKED`, translation retains
its last safe target but does not consume new fixes.

Each sensor sample is processed once. Repeated timer ticks only rebroadcast the
last accepted output.

## Gate State

Heading and translation use separate gate instances with the same states:

| State | Input | Next state and action |
|---|---|---|
| `UNINITIALIZED` | Eligible observation | Seed candidate window and enter `REACQUIRING`. |
| `LOCKED` | Innovation inside locked gate | Accept target and remain `LOCKED`. |
| `LOCKED` | One rejected innovation | Freeze that target, clear candidates, enter `SUSPECT`. |
| `SUSPECT` | First eligible observation | Seed candidate window and enter `REACQUIRING`. |
| `REACQUIRING` | Candidate consistent with window | Append; lock after 5 consecutive samples. |
| `REACQUIRING` | Candidate inconsistent with window | Replace window with this sample; remain `REACQUIRING`. |
| Any | Input stale, non-Fixed, malformed, or TF unavailable for 1.0 s | Enter `DEGRADED`, freeze target, clear candidates. |
| `DEGRADED` | Eligible observation | Seed candidate window and enter `REACQUIRING`. |

Default locked innovation gates are 15 degrees for yaw correction and 1.0 m
for translation correction. Recovery candidates must have circular yaw spread
at most 5 degrees or translation diameter at most 0.30 m. The five samples
must be consecutive and span at least 0.30 s; duplicate samples do not count.
Parameters remain configurable and are reported in startup logs.

Overall motion authority is ready only when both gates are `LOCKED`, the latest
quality/fix/heading ages are within limits, and correction release is not held.
If heading is not locked, position cannot advance even if its own last state was
locked.

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

The output timer uses monotonic time. `dt <= 0` freezes output; `dt` is capped
at 0.10 s so a stalled callback never releases accumulated correction in one
step. Defaults are 0.20 m/s base translation and 2.0 deg/s base yaw.

Backlog handling is explicit:

- Below 0.50 m and 5 degrees: normal release.
- 0.50-2.0 m or 5-20 degrees: `CORRECTION_BACKLOG`; motion authority becomes
  false and the output freezes until `odom -> base` remains below 0.05 m/s and
  2 deg/s for 1.0 s. While stopped, release resumes at the normal rate.
- Above 2.0 m or 20 degrees: `FAULT_HOLD`; no automatic release or
  reacquisition. The route aborts and a node restart/relocalization is required.
- Backlog recovery completes only after the gap stays below 0.15 m and 2
  degrees with both gates locked for 1.0 s.

Targets continue to be evaluated while held so faults and recovery can be
diagnosed, but they do not move the published transform.

Before external alignment exists, preserve the current bootstrap dependency
break: one finite, stamped fix and heading plus TF-at-stamp may initialize and
rebroadcast `map -> odom` as `RTK_BOOTSTRAP`, even without GGA quality 4. This
mode never sets motion authority true. External alignment plus five matched
Fixed observations is required to enter authoritative motion.

## Route Watchdog

The route runner will monitor three signals separately:

- `odom -> base`: local LIO continuity. Persistent physically impossible motion
  can abort the route.
- `map -> odom`: global correction motion. A large update preempts the current
  goal and waits for authority stability; it does not report odom divergence.
- `map -> base`: Nav2-visible pose, retained for goal progress and diagnostics.

Step rates use TF header timestamps. Wall-clock callback spacing is not a valid
motion interval when TF callbacks are backlogged.

Local discontinuity defaults are translation speed above 3.0 m/s or yaw rate
above 3.0 rad/s for three consecutive stamped samples. Only this condition is
reported as `ODOM_DIVERGENCE_ABORT`.

A `map -> odom` rate above 0.50 m/s or 5 deg/s, or motion authority becoming
false, returns `GLOBAL_CORRECTION_HOLD` from goal monitoring. The runner:

1. asserts the stop override;
2. requests action cancellation and waits up to 2.0 s for acknowledgement;
3. keeps the stop override asserted while waiting for authority readiness;
4. requires readiness continuously for 1.0 s;
5. recomputes and resends the same ENU subgoal; and
6. aborts the route if cancellation fails, `FAULT_HOLD` appears, or readiness
   does not return within 10.0 s.

## Command Containment

Corridor mode adds a command guard after Nav2 velocity smoothing:

```text
Nav2 -> velocity_smoother -> /cmd_vel_nav -> corridor_cmd_vel_guard -> /cmd_vel
route runner --------------------> /gps_corridor/stop_override ----^
```

The guard is the only corridor publisher to `/cmd_vel`; the route runner no
longer publishes Twist directly. It preserves straight-line speed but applies:

```text
v_limit = min(v_straight_max, turn_product_limit / max(abs(w), 0.05))
v_out = sign(v_in) * min(abs(v_in), v_limit)
```

Defaults are `v_straight_max=0.85` and `turn_product_limit=0.25 m/s^2`, so
`w=0.70` limits linear speed to about 0.36 m/s. Reverse commands use the same
magnitude rule. Angular velocity remains bounded by the existing Nav2/smoother
limits.

The guard publishes zero for non-finite input, command age above 0.25 s,
motion-authority age above 0.50 s, motion authority false, or stop override
true. Startup is fail-closed until fresh authority and command samples exist.
The stop override is a runner heartbeat; missing for 0.50 s is also treated as
stop. Guard process exit shuts down corridor launch, and the existing
`serial_twistctl` command timeout remains the final zero-command fallback.

Uniform route subgoal insertion is deferred until localization and command
stability pass replay and low-speed vehicle acceptance. It cannot repair a bad
global transform.

## Compatibility And Diagnostics

- Keep existing authority topics and the first 12 diagnostic array fields.
- Add `/localization_authority/motion_allowed` as a fresh Boolean heartbeat.
- Append diagnostic fields with this fixed mapping:

  | Index | Value |
  |---|---|
  | 12 | heading gate enum |
  | 13 | position gate enum |
  | 14 | heading correction innovation, degrees |
  | 15 | position correction innovation, meters |
  | 16 | translation backlog, meters |
  | 17 | absolute yaw backlog, degrees |
  | 18 | motion allowed, 0 or 1 |

  Gate enums are `UNINITIALIZED=0`, `LOCKED=1`, `SUSPECT=2`,
  `REACQUIRING=3`, `DEGRADED=4`, and `FAULT_HOLD=5`. Unavailable numeric
  diagnostics use NaN.
- Keep PGO TF disabled in corridor mode.
- Keep route files and other navigation modes unchanged.

## Testing

ROS-free tests cover correction observations, circular yaw interpolation, the
reacquisition state machine, base-space rate limiting, backlog freeze, watchdog
classification, and command limiting.

`scripts/evaluate_corridor_authority_replay.py` consumes a bag directory and
emits JSON plus a nonzero exit status on failed assertions. A manifest in the
script identifies the four external fixtures under `FYP_CORRIDOR_BAG_ROOT`.
Replay uses bag timestamps and offline interpolation, not wall time.

The four captured bags are replay acceptance fixtures outside the repository:

- `13:36`: the approximately 52 degree correction innovation makes motion false
  within one sample; no reacquisition occurs before five consistent samples.
- `13:46` and `13:48`: every released base correction respects 0.20 m/s and
  2 deg/s plus numeric tolerance; no `ODOM_DIVERGENCE_ABORT` is generated from
  global correction; no more than 10 consecutive release samples are rate
  saturated.
- `13:34`: the evaluator reports `LOCAL_NO_PROGRESS` after GNSS and LIO remain
  below 0.05 m/s for 15 s while command speed exceeds 0.5 m/s.

Launch/integration tests verify historical TF lookup, bootstrap TF publication,
quality matching, action-cancel hold behavior, exactly one `/cmd_vel` publisher
in corridor topology, guard fail-closed timeouts, and PGO TF ownership.

Jetson validation is build/run only. Initial vehicle testing remains low speed
with the PS2 `X` disable and physical e-stop continuously available.
