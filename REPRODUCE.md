# Source-bound reproduction

Runtime source63fc00c3c92c23b873b00ece8586f676921a87e1 is on codex/superlio-ego-active-road, descended from e54c6af. Later report/checker edits are not installed runtime changes. Exact pins are in dependencies.research.repos. Apply Super patches0001–0002 and EGO patches0001–0011 with the repository scripts; never re-port ROS1/3D EGO.

The executed workspace is /dev/shm/codex-stop-fresh-ws. Its SDK/14-package build was fresh at8c937ed5c242fc3e8a08cedcd35c736b957ac43b; research_runtime was then rebuilt at63fc00c and all26 installed Python hashes were checked. fresh-build.json and current-build.json record the distinction. To reproduce from scratch, use a new absolute workspace:

```bash
bash scripts/build_active_road_research.sh /absolute/new-research-ws
python3 scripts/verify_current_upstream_patches.py --repo "$PWD" \
  --verification-root /absolute/new-source-proof --output /absolute/source-proof.json
```

The build script clears inherited overlays, builds the original SDK and14 allowlisted packages with one worker, and uses an isolated install. Production install/checkouts and the preserved domain100/port8765 preview are not test resources.

Run each finite ROS validator in its specified domain with ROS_LOCALHOST_ONLY=1. Source /opt/ros/humble/setup.bash and the isolated install; activate the task research-venv for rasterio/pyproj while retaining system ROS. Set ROS_LOG_DIR and FYP_RUNTIME_ROOT to dedicated task directories and TMPDIR=/dev/shm. PYTHONDONTWRITEBYTECODE=1 avoids cache pollution. Test sinks are allocated PTYs, never physical ports.

| Executed validator | Domain and flags | Evidence under audit/vehicle_ready/control-stop-qualified/ |
|---|---|---|
| validate_ego_vehicle_ros.py |91; --repo, --install, --output. Explicit analytical interface curvature1/jerk3 fixture | current-ego-interface.json |
| validate_mock_serial_ros.py |91; --default-profile; add --rotation-fixture or --rotation-fixture --rtk-classifier | current-linear-wire.json, current-rotation-wire.json, current-rtk-wire.json |
| validate_mock_serial_ros.py |91; --default-profile --heading-recovery-loop --loop-budget-s 35 | current-heading-loop.json |
| validate_mock_serial_ros.py |91; --default-profile --arc-loop --arc-radius 4 --loop-budget-s 35 | current-arc-loop.json |
| validate_rotation_denial_wire_ros.py |109; --repo, --install, --output; actual installed callbacks deny/restore before tick | receipt-denial-final-wire.json |
| validate_console_mock_ros.py |99; --repo, --install, --output; separate test HTTP port | current-hmi-wire.json |
| validate_observer_session_ros.py |98; --install, --output | current-observer-session.json |
| validate_research_launch.py |93; --repo, --output, --serial-evidence current-linear-wire.json, --console-port8876 --override-probes | current-default-entry.json |
| audit_research_runtime_parameters.py |--root, --default-entry current-default-entry.json, --serial current-linear-wire.json, --output | current-three-layer-parameters.json |
| validate_lio_reference_replay.py |104; --repo, --install, --bag, --output; --duration-s 30 --ego-stop-probe | raw-control-ego-stop.json |
| validate_lio_reference_replay.py |104; --duration-s 30 --ground-pipeline --ground-diagnostics | raw-ground-0721-am-prefix.json, raw-ground-0721-pm-prefix.json |
| validate_restricted_policy_ros.py |94; --repo, --install, --output, --budget-s 60; default tasks2 / all3 strategies on source profile | policy-comparison.json |

All ROS validators accept --output /absolute/result.json; use their --help for exact required inputs. Test HMI/default-entry ports are separate from the preserved8765 preview.

Raw bag identities: /home/badger/fgo-gil-analysis/rosbags/2026-07-15-18-31-12/fgo_gil; /home/badger/fgo-gil-analysis/rosbags/2026-07-21-07-06-18/fgo_gil; /home/badger/fgo-gil-analysis/rosbags/2026-07-21-14-34-13/bag/fgo_gil. Verify the sealed catalog/metadata hashes before using them. duration-s0 means actual full EOF; positive durations are PREFIX and never full-replay proof. Full July15 control-reference replay is recorded separately in control-reference-qualified/. A single bag cannot evaluate alternative-view active policies.

Portable validation uses PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 and pytest -p no:cacheprovider with PYTHONPATH including src/research_runtime, src/active_road_mapping and src/super_lio_vehicle_adapter. Run research_runtime/test, super_lio_vehicle_adapter/test and the repository bag/recorded-response/replay/native contract test files. The current actual receipt is222 PASS; the earlier unrelated anyio/pytest plugin error and intended RED callback counterexamples remain preserved. Generate-check command: python3 scripts/generate_research_parameter_lock.py --check.

Static provenance/report verification: python3 scripts/verify_research_delivery.py --output /absolute/static.json. On the remote executor, --external-report /absolute/separate/report.md checks a distinct remote report copy; it cannot stand in for Mac local sync. Current package sources must match the source-bound build receipt even when the tip includes report edits.

The read-only short-window density diagnostic does not publish ground or grant permission and is not part of production runtime. Its script/hash and results are retained with the evidence. Its negative result does not justify changing original limits or marking unknown ground free.

Real ground-to-road support remains FAIL, current strategy benefit is unproven, and Jetson shadow / physical motion acceptance remain PENDING. Mac local sync/cache repair waits for disk space. Normal research pushes and draft PR22 do not merge production.

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
