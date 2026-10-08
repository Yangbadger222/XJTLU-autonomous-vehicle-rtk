# Control-reference and stop qualification — 2026-10-08

The goal remains active. The only vehicle baseline is corridor-authority-stability@e54c6afbcb5a58db22d7c468085a87d658b0b932; work stays on codex/superlio-ego-active-road. Current installed runtime source is 63fc00c3c92c23b873b00ece8586f676921a87e1. This report separates executed software gates, research outcomes and physical acceptance. Earlier reports and negative receipts remain historical.

| Executed check | Result and scope | Evidence |
|---|---|---|
| Original source protection | 1011 originals unchanged; firmware, calibration, protocol, limits and authority unchanged | audit/vehicle_baseline/PROTECTED_FILES.sha256, PROTECTED_ADDITIONAL_FILES.sha256 |
| Source / parsed / effective Humble values | 157 checks, seven changed-original override probes and 16 owned default-entry children pass; target overlays remain unverified | audit/vehicle_ready/control-stop-qualified/current-three-layer-parameters.json, current-default-entry.json |
| Exact native source | Fixed Super-LIO plus 2 patches and fixed already-2D EGO plus 11 patches apply sequentially and match every compiled dependency source file | audit/vehicle_ready/control-stop-qualified/current-source-proof.json |
| Actual compilation | Fresh original SDK and 14 packages at 8c937ed; subsequent research_runtime rebuild at 63fc00c, all 26 installed Python source hashes match. This is a staged build, not an all-14 fresh build at 63fc00c | fresh-build.json, current-build.json, build-summary.log in the same evidence directory |
| Typed EGO interface | 10 cases and 2 planning queries pass; this interface fixture explicitly uses analytical curvature 1 / jerk 3 settings | current-ego-interface.json |
| Current source-default motion loops | Actual EGO / tracker / original guard / original serial PTY reaches goals in 35-second heading-recovery and R=4 m arc loops | current-heading-loop.json, current-arc-loop.json |
| Final serial stop and authority | 44 linear, 50 pure-yaw, 5 original RTK-classifier and 11 HMI cases pass. RTK loss stops even with healthy LIO | current-linear-wire.json, current-rotation-wire.json, current-rtk-wire.json, current-hmi-wire.json |
| Deny/restore before next tick | 10 actual installed callback cases revoke admission immediately; restored inputs cannot bypass original one-second confirmation. Final PTY tails are all zero | receipt-denial-final-wire.json |
| Portable regression | 222 pass. Five counterexamples fail against the earlier 8c callback implementation, then pass after the fix | portable-current.log, receipt-denial-red.log |
| Real acquisition state to EGO | July15 raw 30-second PREFIX yields 271 exact pairs and three recovery trajectories preserving actual measured w/alpha; independent continuous derivative checks pass. Grid/reference here are explicitly analytical | raw-control-ego-stop.json |
| Full raw control-reference stream | July15 full 1x EOF replay: 57008 IMU, 2867 LiDAR, 2850 native/legacy/control/cloud outputs; 27 source/interface checks pass at its recorded implementation | audit/vehicle_ready/control-reference-qualified/raw-control-full-0715.json |
| MID360/STM records and bags | Existing nominal mounting/factory transform produces a separate chassis control reference. 41 recorded-response bags and original STM equations were audited without changing limits | audit/vehicle_ready/control-reference-qualified/recorded-control-response.json |
| Real local ground | July21 AM/PM 30-second PREFIX trials have 288/263 exact odometry/cloud pairs and valid source health; both FAIL the positive geometry/persistence loop, with zero supported cells | raw-ground-0721-am-prefix.json, raw-ground-0721-pm-prefix.json |
| Bounded accumulation diagnostic | Read-only 0 / 0.2 / 0.4-second unions retain all original ground thresholds; maximum support cells 0 / 0 / 1, zero vehicle-width strips throughout. No union was enabled in runtime | ground-short-window-diagnostic.json |
| Observer persistence/settling | Seven installed checks pass, including historical session separation, rollback and full three-dimensional speed/nonfinite rejection | current-observer-session.json |
| Same-foundation strategy experiment | Six protocol-valid tasks, two per strategy, all fail to reach the goal. Prior bytes and common foundation are preserved | policy-comparison.json |
| Independent Standards / Spec reviews | Both close the receipt-time revocation P2; no new hard P1/P2 in this increment. Neither review certifies terrain or physical motion | audit/vehicle_ready/FINAL_REVIEW.md |

Evidence names without a directory above are under audit/vehicle_ready/control-stop-qualified/.

The original navigation topic/TF retains its IMU-origin convention for RTK compatibility. The new /research/odom_control explicitly uses the recorded chassis XY origin at nominal ground level, rather than silently relabelling an IMU point. The transformation applies full pose/twist covariance and the angular lever-arm velocity term. Original URDF attitude is a configured model, not newly surveyed metrology.

The real-state probe exposed an overly strict research stop test: 0.001 m/s / 0.001 rad/s and a blanket negative-vx rejection admitted no continuous one-second window in a stationary raw segment. The fix reuses the original 0.05 m/s, 2 degrees/s and one-second definition, keeps actual yaw rate/acceleration as boundary conditions, and requires independent admission in the final bridge. This is rate-tolerance confirmation; it is not proof of a physical brake. Every denial revokes the admission token immediately.

Simulation depth uses a virtual simulation_depth_sensor at 1.2 m; its height, registration, source-health fixture and uncertainty assumptions are labelled simulation-only. The MID360 raw simulator emits the existing mount/factory lever rather than assuming IMU equals chassis. No synthetic depth calibration is attributed to the real camera.

Real positive ground-to-road support remains FAIL. Three strategies in the current finite cohort are PASSIVE 0/2, PERIODIC_LOOK 0/2 and TASK_AWARE_LOOK 0/2. Earlier cohorts, including TASK_AWARE_LOOK 1/2 and PASSIVE 1/2, retain their own source identities. No statistical or physical policy benefit is claimed.

Jetson SSH at the recorded address timed out again. Target effective overlays, actuator-disabled shadow, flashed STM identity and physical KEY/PS2/brake/estop/slip acceptance remain PENDING. Original baseline regressions remain 191 PASS / 3 FAIL and 26 PASS / 1 FAIL; protected code was not changed to force green. No physical driving, firmware flashing, production merge or force push occurred.

The Mac filesystem refuses small writes. Runtime work and this committed checkpoint are on the authorized non-Jetson Humble host. Local sync and one temporary verification cache pack-path repair remain pending; intact original pack and uploaded transport bundles are retained. This does not block remote source qualification or ordinary research publication.
