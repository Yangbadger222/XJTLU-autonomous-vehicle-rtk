# Conditions before on-site acceptance

No new-stack physical motion or firmware flashing has occurred. MID360 internal IMU, existing mounting/factory transform, original STM equations and 41 recorded-response bags are available and used. No new external IMU calibration is required to implement the recorded nominal chassis-reference transformation.

Current software evidence: exact Super2/EGO11 patches; actual fresh SDK/14 foundation plus current runtime rebuild; 157 source/parsed/Humble effective checks; 16 owned default-entry nodes and seven override denials; 222 portable tests; 10 EGO interface / 2 query tests; 44 linear / 50 yaw / 5 RTK / 11 HMI final-wire cases; 10 immediate deny/restore edge cases; 35-second heading and arc goal-stop loops. All mock actuator sinks were allocated PTYs.

Real MID360 support remains a live-motion prerequisite. July15 full raw replay and July21 AM/PM prefixes run the actual ground pipeline but produce no confirmed vehicle-width surface strips. A separate at-most-0.4-second point-union diagnostic retains the same thresholds and also produces no strips. That diagnostic is not enabled for permission/control. Positive fixture support and simulator depth do not qualify the actual surface.

Target preparation can begin with read-only, actuator-disabled inspection: establish the target deployment identity, current CLI/environment/launch overlays and sensor timing, compare source/parsed/effective originals, and run the submitted algorithm in an isolated installation with serial and mission disabled. Any target compilation must use one worker and preserve production checkout/install. Do not edit source on Jetson. The recorded Jetson100.79.128.21 SSH timed out; authorized100.88.131.52 is the non-Jetson Humble/bag executor.

Human hardware checks, after applicable software/target gates:

- Confirm flashed STM identity against the preserved source. Keep documented command/feedback geometry discrepancies visible; do not retune firmware to fit bag estimates.
- Observe KEY, PS2 loss/manual takeover, X zero-current/coasting, B active brake and physical estop at the actual actuator. A final PTY zero command proves only software propagation.
- Measure stopping distance, actuation latency and slip under the original caps; check chassis control pivot, swept footprint and terrain assumptions.
- Confirm real raw acquisition/arrival timing and odometry reference against the deployed mount/configuration.
- Verify RTK authority loss stops despite healthy LIO and recovery does not silently resume a latched operator task.
- Verify IMU/LiDAR loss, TF ownership/freshness, trajectory expiry, map-version faults and controller loss under original deadlines.
- Qualify current local ground and any actual registered prior/camera input before using it for motion.

For optional real RGB-D observation, identify the actual depth stream, K, depth convention and acquisition-time camera-to-body transform. The virtual 1.2 m simulation sensor supplies none of these real calibration values. Repository records should be checked before requesting a missing item.

No automatic driving, firmware flash, production merge, force push or old asset removal. Software completion, strategy benefit and physical acceptance are separate.
