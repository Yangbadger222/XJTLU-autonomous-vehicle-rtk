# FGO-GIL Reproduction Design and Implementation Plan

## 1. Document Status

- Status: pre-design-freeze draft, sufficient to begin implementation
- Date: 2026-07-11
- Target paper: *FGO-GIL: Factor Graph Optimization-Based GNSS RTK/INS/LiDAR Tightly Coupled Integration for Precise and Continuous Navigation*, IEEE Sensors Journal, 2023
- Runtime rule: shadow outputs only; do not publish production `map -> odom` or remap Nav2 before replay and vehicle acceptance
- Existing baseline: keep `rtk_fgo_localizer` as the solution-level comparator that fuses `/fix`, dual-antenna heading, and FAST-LIO odometry; do not relabel it as the paper reproduction

Current implementation status: Phase 1 is complete. Phase 2 now includes strict 40-byte record decoding for uncompressed OBSVM/OBSVH/OBSVBASE, physical-unit scaling, tracking-status semantics, constellation/PRN validation, non-finite rejection, and bounded receiver + week/TOW deduplication. A real raw-UART fixture, compressed observations, RTCM fallback, ephemerides, and satellite-state calculation remain pending.

This document defines the complete implementation path from UM982 raw observation acquisition to an observation-level GNSS RTK/INS/LiDAR factor graph. It is not another wrapper around the existing `/fix` FGO. A paper-level reproduction must consume pseudorange, carrier phase, raw IMU, and LiDAR feature residuals directly.

## 2. Known Facts and Constraints

### 2.1 Vehicle and sensors

- GNSS: UM982 dual antenna, right antenna is master and left antenna is secondary.
- Antenna coordinates use ROS vehicle axes: `base_link +X` forward, `+Y` left, and `+Z` up.
- Provisional right master position relative to `base_link`: `[0.000, -0.184, 0.154] m`.
- Provisional left secondary position relative to `base_link`: `[0.000, 0.186, 0.154] m`.
- Lateral baseline length: `0.370 m`.
- The longitudinal installation error is close to zero but nonzero. The current `heading_offset_deg=88.5` came from straight vehicle runs and differs from the ideal transverse mounting by `1.5 deg`. For a 0.370 m baseline this implies a longitudinal phase-center difference on the order of `9.7 mm`; its sign has not been measured.
- LiDAR/IMU runtime inputs are Livox MID360 point clouds and `/livox/imu`.
- The URDF declares `base_link -> imu_link=[0,0,0.02] m` and `base_link -> laser_link=[0.10,-0.10,0.07] m`.
- FAST-LIO2 uses `t_il=[-0.011,-0.02329,0.04412] m` and `r_il=I`. This disagrees with the transform inferred from the URDF, so the URDF values are model initial values rather than paper-grade extrinsic truth.

Every unconfirmed extrinsic must carry `calibrated=false` and an uncertainty. An uncalibrated configuration may parse, record, and run pure shadow optimization, but it must block TF/Nav2 ownership.

### 2.2 Existing bag capability

The `2026-07-10-13-48-43` bag on `badger@100.88.131.52` is approximately `73.38 s` long and contains:

| Topic | Messages | Approx. rate | Use |
|---|---:|---:|---|
| `/fix` | 367 | 5 Hz | Existing solution-level GNSS comparator |
| `/heading` | 734 | 10 Hz | Dual-antenna heading comparator |
| `/rtk/nmea_sentence` | 1468 | 20 Hz | GGA/THS quality and epoch aid |
| `/fastlio2/lio_odom` | 669 | 9.1 Hz | Existing LIO comparator |
| `/livox/imu` | 14200 | 193.5 Hz | IMU preintegration development and replay |
| `/odom_CBoar` | 1090 | 14.9 Hz | Optional wheel comparator, disabled as a factor by default |
| `/rtk_fgo/*` | about 5 Hz | - | Legacy shadow FGO comparator |

The bag contains RTK Fixed GGA (quality 4; an observed sample has 33 satellites and HDOP 0.5) and valid THS. It supports time buffering, IMU preintegration, state-machine, comparator, and fault-injection work. It does not contain `OBSVM/OBSVH/OBSVBASE`, ephemerides, or carrier phase and therefore cannot validate GNSS DD factors.

### 2.3 Confirmed UM982 capability

The official UM982 protocol provides:

- `OBSVM` (ID 12): master raw observations.
- `OBSVH` (ID 13): secondary raw observations.
- `OBSVMCMP` (ID 138) and `OBSVHCMP` (ID 139): compressed observations on supported firmware.
- `OBSVBASE` (ID 284): base-station observations.
- Ephemeris messages for GPS, BDS, GLONASS, Galileo, and other supported systems.
- GNSS week, milliseconds-of-week, time status, output delay, and CRC in binary headers.
- Configurable PPS and serial rates up to 921600 baud.

The raw fields include pseudorange, accumulated carrier phase, Doppler, C/N0, standard deviations, signal type, lock time, and tracking/validity flags. These cover the fundamental inputs required for short-baseline RTK double differences.

## 3. Reproduction Boundary

### 3.1 Required paper fidelity

1. Graph states contain position, velocity, attitude, accelerometer bias, gyroscope bias, and double-difference ambiguities organized by constellation and signal.
2. GNSS factors are constructed from rover/base raw pseudorange and carrier phase through single and double differences.
3. GNSS geometric range includes the master antenna lever arm relative to the IMU center.
4. Raw acceleration and angular velocity are preintegrated and estimated biases are fed back.
5. LiDAR non-keyframes contribute to inter-frame motion; keyframes enter the window as scan-to-submap point-to-line and point-to-plane residuals.
6. Cycle-slip detection, outlier rejection, reference-satellite selection, and integer ambiguity resolution are explicit modules.
7. When GNSS is unavailable, the graph degrades to INS/LiDAR without reusing an old GNSS epoch.
8. The window uses real fixed-lag marginalization. Clearing a graph and adding a fresh prior is not a sliding-window implementation.

### 3.2 Vehicle adaptations

- The paper used Septentrio receivers, an ADIS-16470 IMU, and a VLP-16. This vehicle uses UM982 and MID360 with its IMU. The hardware difference must be reported separately.
- Preserve the paper's ECEF navigation-state semantics. Store position as an increment from a fixed ECEF origin for numerical conditioning and reconstruct full ECEF coordinates inside residuals.
- Maintain the LiDAR submap in local `map_fgo`, related to the ECEF navigation state by a fixed `T_ecef_map_fgo`.
- The secondary antenna may provide an optional baseline-attitude factor. This is a vehicle extension and must not be included in the paper-baseline result.
- The wheel factor remains disabled by default and is only enabled in a separately reported ablation configuration.

### 3.3 Non-commitments

- Existing NMEA bags cannot demonstrate the paper's accuracy.
- The CORS baseline has not been shown to satisfy the paper's `<20 km` assumption for neglecting double-difference atmospheric residuals.
- Absolute accuracy is not claimed with unmeasured extrinsics, unverified PPS, or placeholder IMU noise.
- The UM982's already-fixed RTK coordinate is not reused as if it were a carrier-phase observation factor.

## 4. System Architecture

```text
UM982 COM1 (existing production path, 115200)
  -> GGA/THS/HPR + NTRIP
  -> /fix, /heading, /rtk/status

UM982 dedicated raw COM (recommended 921600 binary)
  -> um982_raw_driver
  -> CRC/framing -> GNSS week/TOW -> canonical observation epochs
  -> /gnss/raw/master, /gnss/raw/secondary, /gnss/raw/base, /gnss/raw/ephemeris

MID360 + /livox/imu
  -> fgo_gil_lidar_frontend: de-skew, features, keyframes, submap matches
  -> fgo_gil_imu_frontend: timestamp normalization, preintegration

GNSS frontend
  -> ephemeris/satellite state
  -> cycle-slip and outlier detection
  -> per-constellation reference satellite
  -> SD/DD pseudorange and carrier-phase observations

fgo_gil_estimator
  -> IMU + LiDAR + GNSS DD factors
  -> fixed-lag optimization
  -> float state/covariance
  -> LAMBDA ambiguity resolution
  -> fixed-candidate validation
  -> /fgo_gil/* shadow outputs
```

Keep the existing `rtk_fgo_localizer` as the solution-level comparator. The new research path uses a separate namespace, parameter file, and launch so results cannot be confused through shared `/rtk_fgo/*` topics.

## 5. Package and File Design

Proposed additions:

```text
src/sensor_drivers/gnss/gnss_raw_msgs/
  msg/RawFrame.msg
  msg/Observation.msg
  msg/ObservationEpoch.msg
  msg/Ephemeris.msg

src/sensor_drivers/gnss/um982_raw_driver/
  include/.../binary_framer.hpp
  include/.../crc32.hpp
  include/.../observation_decoder.hpp
  include/.../time_converter.hpp
  src/...
  test/fixtures/

src/perception/fgo_gil_localizer/
  include/fgo_gil_localizer/gnss/
  include/fgo_gil_localizer/imu/
  include/fgo_gil_localizer/lidar/
  include/fgo_gil_localizer/graph/
  include/fgo_gil_localizer/ambiguity/
  include/fgo_gil_localizer/calibration/
  src/...
  test/...

src/bringup/config/fgo_gil.yaml
src/bringup/launch/system_fgo_gil_shadow.launch.py
scripts/evaluate_fgo_gil_bag.py
```

`gnss_raw_msgs` expresses protocol-independent observations and contains no FGO policy. `um982_raw_driver` owns serial protocol and canonicalization without depending on GTSAM. `fgo_gil_localizer` never reads a serial device directly.

## 6. Data Contracts

### 6.1 Raw frame

Every `RawFrame` contains at least monotonic/ROS reception time, port, raw bytes, message ID, sequence, GNSS week/TOW when valid, time status, and CRC status. A checksum-valid unknown message ID may still be recorded before a decoder exists.

### 6.2 Observation epoch

Every epoch uniquely identifies:

- receiver: master, secondary, or base;
- GNSS week and TOW;
- clock/time status;
- satellite system, PRN, and signal/frequency;
- pseudorange, carrier-phase cycles, Doppler, and C/N0;
- pseudorange/carrier standard deviation;
- lock time, tracking status, validity, and half-cycle flags.

Downstream code deduplicates on `(receiver, week, tow)`. A ROS timer must never count one GNSS epoch multiple times for recovery or graph insertion.

### 6.3 Time rules

- GNSS factor time comes from UM982 week/TOW, never callback `now()`.
- LiDAR/IMU device time is preferred; ROS reception time is diagnostic only.
- Synchronization state is one of `UNSYNCED`, `COARSE`, or `PPS_LOCKED`, with estimated offset and jitter.
- `UNSYNCED` may record and parse but cannot produce high-weight joint GNSS/LiDAR factors.
- Odometry uses the estimated state time, not publication time.

## 7. Estimator Mathematical Design

### 7.1 State

At keyframe `k`:

```text
X_k = {delta_p_e_b, v_e_b, q_e_b, b_a, b_g}
A_k = {DD ambiguity per constellation, reference satellite, signal}
```

The estimator also maintains a fixed ECEF origin, `T_ecef_map_fgo`, gravity/Earth-rotation model, master GNSS lever arm, LiDAR-IMU extrinsic, and time offsets. Ambiguities are not permanent globals: signal loss, cycle slip, reference change, or signal change creates a new arc ID.

### 7.2 IMU factor

- Use bias-corrected preintegration.
- Model ECEF mechanization, gravity, and Earth rotation. A first implementation that temporarily uses local-ENU GTSAM preintegration must be labeled `engineering_baseline`, not paper-equivalent.
- Non-finite IMU, negative time delta, or excessive gap fails closed and breaks the affected interval.
- Load IMU noise and bias random walk from parameters. Current YAML numbers are placeholders and cannot support final experimental claims.

### 7.3 LiDAR factor

- De-skew using per-point offset time and IMU prediction.
- Optimize non-keyframes scan-to-scan/inter-frame to seed the next keyframe, without inserting every frame into the global window.
- Keyframe policy includes translation, rotation, elapsed time, and GNSS-availability triggers.
- Edge features create point-to-line residuals; plane features create point-to-plane residuals.
- Correspondences come from a local keyframe submap with robust loss, minimum feature counts, and line/normal degeneracy checks.
- FAST-LIO odometry is a comparator or initialization fallback. The paper configuration cannot substitute a FAST-LIO BetweenFactor for a raw LiDAR factor.

### 7.4 GNSS double-difference factor

Processing order:

1. Compute satellite position, clock, and wavelength from ephemerides.
2. Apply quality, elevation/CN0, and gross-error screening.
3. Align rover and base epochs.
4. Select a reference per constellation and signal, normally highest elevation with hysteresis.
5. Form rover-base single differences and reference-nonreference double differences.
6. Apply the master lever arm with `p_ant^e = p_imu^e + R_b^e l_ant^b`.
7. Add separate DD pseudorange and DD carrier-phase factors.

If the CORS baseline exceeds 20 km, estimate differential ionosphere/troposphere terms or use a closer base. The paper's short-baseline omission cannot be applied silently.

### 7.5 Cycle slips and outliers

- Combine tracking flags, lock time, half-cycle status, Doppler/phase consistency, and geometry-free or Melbourne-Wubbena checks when signals permit.
- A slip resets only the affected `(receiver, constellation, satellite, signal, arc)` ambiguity.
- GNSS residuals use innovation gating and robust loss, but robust loss must not hide CRC failure, epoch mismatch, or wrong ephemeris.
- Every rejection produces a structured reason and counter.

### 7.6 Integer ambiguity resolution

- Do not implement an ad hoc integer search. Use a validated LAMBDA implementation, with RTKLIB's LAMBDA module and license compatibility evaluated first.
- Feed float ambiguity and covariance and support partial ambiguity resolution.
- Parameterize ratio, success-rate/residual checks, and fixed-solution back-substitution validation.
- A failed fixed candidate leaves the float solution intact and cannot contaminate the window.
- A reference-satellite change transforms the ambiguity basis correctly rather than silently reusing all variables.

### 7.7 Window and marginalization

- Use GTSAM fixed-lag smoothing or equivalent Schur-complement marginalization.
- Limit the window by both elapsed time and maximum states; `duration_s` must have behavioral effect.
- Preserve marginalized information in a prior. Never clear the graph and add an artificially strong prior on the current estimate.
- Record variables, factors, linearization/optimization time, condition/degeneracy metrics, and marginalization count.

## 8. Calibration Parameters

Initial configuration:

```yaml
calibration:
  vehicle_axes: FLU
  gnss:
    master_is_right: true
    master_in_base_m: [0.000, -0.184, 0.154]
    secondary_in_base_m: [0.000, 0.186, 0.154]
    baseline_length_m: 0.370
    heading_offset_deg: 88.5
    longitudinal_uncertainty_m: 0.010
    calibrated: false
  imu_in_base:
    translation_m: [0.0, 0.0, 0.02]
    rotation_rpy_rad: [0.0, 0.0, 0.0]
    calibrated: false
  lidar_in_imu:
    translation_m: [-0.011, -0.02329, 0.04412]
    rotation_matrix: [1,0,0, 0,1,0, 0,0,1]
    source: fastlio2_current
    calibrated: false
time_sync:
  pps_enabled: false
  status: UNSYNCED
```

Code transforms `master_in_base` through the TF chain into the paper's required `master_in_imu` lever arm and checks conventions among URDF, FAST-LIO, and FGO parameters. Factors must not hard-code these values.

## 9. Development Phases and Definitions of Done

### Phase 0: Baseline and interface freeze

- [ ] Create a dedicated branch from a common baseline containing corridor Task 2 and serial-integrity fixes.
- [ ] Freeze message schema, frames, time scale, units, and calibration schema.
- [ ] Reserve a dedicated UM982 raw COM while preserving the existing 115200 NMEA/NTRIP path.
- [ ] Record firmware with `VERSIONA` and confirm compressed-observation support.

Done: interface review passes and production GNSS behavior is unchanged.

### Phase 1: UM982 raw frame acquisition

- [x] Implement bounded streaming `AA 44 B5` framing, length checks, CRC, and resynchronization.
- [x] Handle arbitrary fragmentation, coalescing, multiple frames, noise prefixes, truncation, and unknown IDs.
- [x] Publish/record raw frames and framing/CRC/drop diagnostics.
- [ ] Default to 921600 on a dedicated device/udev name; never store CORS credentials.

Done: synthetic/official fixtures pass, fuzz input cannot crash or overrun, and existing NMEA tests do not regress.

### Phase 2: Observation and ephemeris canonicalization

- [x] Decode uncompressed OBSVM/OBSVH and tracking-status semantics.
- [ ] Decode OBSVMCMP/OBSVHCMP when firmware supports them.
- [x] Decode uncompressed OBSVBASE.
- [ ] If CORS does not expose OBSVBASE, adapt RTCM MSM into canonical epochs.
- [ ] Decode required ephemerides and compute satellite states.
- [x] Preserve week/TOW/time status and deduplicate on receiver + week/TOW with bounded memory.
- [ ] Add explicit week-rollover and receiver-time-reset policy.

Done: zero-noise synthetic observations recover known ranges; bad time, ephemeris, and non-finite fields are rejected.

### Phase 3: Time synchronization and IMU preintegration

- [ ] Map GNSS, LiDAR, and IMU clock domains with status diagnostics.
- [ ] Accept PPS status and propagate timing uncertainty before lock.
- [ ] Implement ECEF IMU propagation/preintegration and bias Jacobian tests.
- [ ] Use the 2026-07-10 bag to test the approximately 193.5 Hz IMU buffer, gaps, and duplicate timestamps.

Done: synthetic propagation meets frozen tolerances; time reversal, gaps, and NaN/Inf fail closed.

### Phase 4: Raw LiDAR factor frontend

- [ ] Reuse or extract MID360 point preprocessing without copying unmaintainable FAST-LIO private state.
- [ ] Implement de-skew, edge/plane features, keyframes, and KF-map management.
- [ ] Implement point-to-line/point-to-plane factors and numerical Jacobian checks.
- [ ] Publish feature counts, match residual, degeneracy, and latency.

Done: synthetic line/plane zero residuals and finite-difference Jacobians pass; degenerate scenes cannot emit false high-confidence constraints.

### Phase 5: GNSS DD and float FGO

- [ ] Implement reference selection, SD/DD builder, lever-arm correction, and DD factors.
- [ ] Implement cycle-slip/outlier/arc management.
- [ ] Jointly optimize IMU, LiDAR, GNSS, and float ambiguities in a fixed-lag graph.
- [ ] During GNSS outage, add no duplicate factor and continue as LIO.

Done: float state and ambiguities converge on synthetic rover/base data; reference switch and single-satellite slip tests pass.

### Phase 6: Integer fixing

- [ ] Integrate LAMBDA with covariance ordering and unit tests.
- [ ] Implement ratio test, partial fixing, back-substitution, and fixed rejection.
- [ ] Publish FLOAT/FIXED state, ratio, fixed-satellite count, and rejection reason.

Done: known-integer fixtures fix correctly, bad candidates cannot contaminate the next window, and a slip resets only related ambiguities.

### Phase 7: ROS shadow integration and replay

- [ ] Add `system_fgo_gil_shadow.launch.py` and a dedicated bag profile.
- [ ] Publish `/fgo_gil/odom`, path, factor diagnostics, ambiguity status, timing status, and performance.
- [ ] Add new nodes to `make kill-runtime`, while forcing `publish_tf=false` and `nav2_use_fgo=false`.
- [ ] Extend evaluation with APE/RPE, availability, fixing rate, outage drift, CPU/RAM, and real-time factor.

Done: desktop tests and Jetson clean build pass. Existing bags verify non-raw-GNSS paths and comparators; missing raw topics explicitly report `RAW_GNSS_UNAVAILABLE`.

### Phase 8: Weather-dependent collection and acceptance

- [ ] Record VERSION, raw master/secondary/base, ephemerides, PPS, LiDAR, IMU, TF, and temperature.
- [ ] Measure antenna phase-center X/Z and the true LiDAR-IMU extrinsic.
- [ ] Confirm CORS station coordinates, station ID, baseline distance, and RTCM content.
- [ ] Collect static, straight, turning, open-sky, tree-cover, short-outage, and reacquisition datasets.
- [ ] Compare UM982 RTK, FAST-LIO2, existing `rtk_fgo_localizer`, and FGO-GIL on identical bags.

Done: only after frozen accuracy, continuity, fixing-rate, and resource budgets pass may a controlled TF A/B experiment be proposed. Shadow remains the default.

## 10. Test Matrix

| Level | Required coverage |
|---|---|
| Parser unit | CRC, length, endianness, fragmentation/coalescing, unknown ID, week/TOW, NaN/Inf |
| GNSS unit | Satellite state, SD/DD sign, frequency/wavelength, lever arm, reference switch, cycle slip |
| IMU unit | Static, constant velocity/rate, bias Jacobian, gap/out-of-order |
| LiDAR unit | De-skew, line/plane residuals, Jacobian, degeneracy |
| Graph unit | Marginalization, outage/re-entry, float ambiguity, fixed rollback |
| Synthetic integration | Known trajectory with controlled GNSS outage/multipath/slips and LiDAR degeneration |
| Existing bag | Time association, IMU/LIO comparator, state machine, missing-raw-GNSS fail closed |
| New raw bag | Full observation-level replay, fixing rate, APE/RPE, resource use |
| Vehicle | Low-speed shadow after motor-disabled bench; controlled TF A/B only last |

All Jetson test commands use `--parallel-workers 1`, and the workspace must be re-sourced after build. Parser tests remain ROS-free for fast workstation execution.

## 11. Diagnostics and Safety

Expose at least:

- raw frame rate, CRC failures, framing resync, and unknown IDs;
- epoch rate per receiver, week/TOW, time status, age, and duplicate count;
- usable satellites, reference satellite, and DD count per constellation;
- cycle slips, outliers, arc resets, and ambiguity dimension;
- IMU/LiDAR/GNSS factor counts, residuals, and rejection reasons;
- float/fixed state, ratio, fixed-satellite count, and rollback count;
- window span, states, factors, marginalization, and optimization latency;
- calibration validity, PPS lock, and time offset/jitter;
- output stamp age, non-finite detection, and shadow/control ownership.

Any of the following blocks ownership: uncalibrated extrinsics, unlocked time, non-finite state, graph divergence, stale output, bad ECEF origin, duplicate GNSS epochs, or conflicting TF owner. Motor safety continues to use PS2 `X` and the physical e-stop. Never use `B` as an e-stop.

## 12. Decision Record

1. Implement raw acquisition before GNSS factors that require real observations; do not add more `/fix` GPSFactors.
2. Physically and logically isolate production NMEA and research binary streams so binary traffic cannot corrupt the 115200 line parser.
3. Use measured antenna Y/Z and 88.5 deg as uncertain initial values; do not guess the sign of the millimeter-scale X offset.
4. Use FAST-LIO extrinsics as current algorithm initial values, but retain `calibrated=false` because they disagree with the URDF.
5. Treat July bags as software-regression assets, not carrier-phase validation data.
6. Report paper baseline and vehicle extensions separately: secondary heading, wheel, and solution-level RTK factors are ablations/extensions.

## 13. First Implementation Batch

The first batch covers only Phase 0-1:

1. Add the minimal `gnss_raw_msgs` set.
2. Add a ROS-free UM982 binary framer, CRC, and fixture tests.
3. Add a raw-driver shadow node connected only to a dedicated raw port.
4. Add raw-frame bag topics and diagnostics.
5. Add disabled-by-default bringup parameters without changing current UM982 NMEA/NTRIP startup.
6. Update bilingual commands, architecture, knowledge, and devlog documentation.

This batch adds no GTSAM factor, changes no Nav2 or production TF behavior, and does not require outdoor collection before parser code can merge.
