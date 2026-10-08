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

## 2026-10-08 sensor ingress and ARM64 cohort

The optional supervised sensor-only ingress has 17 actual Humble isolation gates and 12 recorded-data gates passing. Six July21 PM inputs cross lab domains134/135 at 100% observed delivery over an explicit20-second prefix; old TF/LIO outputs each have199 source/zero target samples. Complete native received CDR streams/counts match target; each original target payload matches direct source observation. The absent/rtk/health topic is recorded. Actual queue-order and depth-one burst failures are retained; the source-writer GID check, bounded depth10 and fresh-attempt stdlib supervisor resolve these failures. Both independent review axes report no remaining newP1/P2 for the qualified tool. No source-domain publishers, TF/control/authority/consent/clock forwarding or default-entry changes occur.

The main SDK/14 packages compile as actual ARM64ELF in a separate qemu-emulated Humble container, from read-only a66cc7e source. The initially empty workspace is resumed after a missing-nmea dependency repair and a verified Ubuntu mirror-index mismatch; it is not a single uninterrupted fresh pass. Exact failed attempts, final exit0, SDK and executable hashes are preserved. This auxiliary ingress is separate from that14-package allowlist.

ARM64 default-entry startup is FAIL: all16 owned processes stay alive and native Super/EGO initialize, but the persistent discovery graph is empty and parameter services cannot be found. Owned launch cleanup exits0. Runtime and target performance therefore remain unqualified. The x86 runtime/core26 Python files, protected src tree, original parameter locks and earlier157/222/final-wire/simulation results remain unchanged.

Actual positive road support still FAILS: full-input density and at-most0.4-second endpoint-pose union diagnostics yield no vehicle-width strips and are not promoted as a perception/permission producer. Current policy benefit remains PENDING, with0/2 tasks for each strategy. Jetson live shadow, actual deployment/effective values, physical stop/brake/slip and motion acceptance remain PENDING. MID360 mounting/internal IMU, nominal chassis transform and STM/bag evidence are already used and are not missing-specification blockers.

Evidence: audit/vehicle_ready/shadow-ingress/; audit/vehicle_ready/arm64-emulated/. Public tool/build/qualification instructions: tools/shadow_sensor_ingress/README.md. The source-bound core qualification remains63fc00c and ARM compile sourcea66cc7e; this cohort adds an optional tool and reports without relabeling old receipts.

## ARM64 emulator diagnosis — finite native-socket counterfactual

The original unmodified QEMU6.2 startup failure is retained. Minimal std_msgs tests separated the failure from the vehicle algorithms: same-context communication passed, distinct contexts/processes failed, and native x86 control passed. Actual syscall tracing and 4/8/12-byte probes show errno92 for IP_MULTICAST_IF; QEMU6.2 upstream linux-user source has no handler for this option. UDP-only/unicast-profile and ABI-size trials also failed and remain recorded.

A finite, task-container-bound host broker configures and verifies the actual duplicated UDP socket to loopback. With this explicit emulator-only intervention, all three minimal DDS cases pass (433 cross-process messages unchanged), the unchanged a66 core default startup exposes all16 owned nodes and12 parameter services with one research_safety_bridge cmd_vel writer, and the byte-matched b55 auxiliary ingress compiles as aarch64 and passes all17 fixtures. No vehicle source/default launch/parameter, host binfmt or production process was changed. These are emulator startup/transport gates, not native Jetson performance, live road or physical acceptance.

Clean shutdown remains FAIL: after the controlled SIGINT, research_operator_console and super_lio_cloud_frame_adapter report rclpy take_message RuntimeError; all owned processes exit and launch/container exit0. The original guard's KeyboardInterrupt is recorded separately. No exception is suppressed or promoted to a clean-shutdown PASS. See audit/vehicle_ready/arm64-emulated/native-socket-diagnostic-summary.json and the source-bound receipts/recipes/manifest.

Existing repository MID360 mounting/internal-IMU and STM/bag records are available and already used, not missing-data blockers. Actual road support still produces zero qualified vehicle-width strips; current strategy benefit remains unproven. Read-only target TCP22 timed out from the non-Jetson executor. Actuator-disabled target inspection/shadow and physical actuator/stop/deployment checks remain distinct outstanding gates. Goal remains active.

The emulator helper review also closed optimization-removable assert boundaries, empty/partial container identity and non-finite duration. Explicit checks retain their behavior under PYTHONOPTIMIZE=1:6CLI/old-receipt,5native-request and1foreign-peer rejection gates pass;3minimal ARM DDS cases pass with the revised helper. The prior final-graph-only false-negative is retained; current probe records observed discovery/writer presence during live reception. The current probe receives431of432publications and makes no100percent-delivery claim. See strict-broker-qualification.json and both historical/current helper sources. No core runtime source or parameter changed.
