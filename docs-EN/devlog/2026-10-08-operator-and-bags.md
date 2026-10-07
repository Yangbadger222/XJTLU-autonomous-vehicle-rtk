## 2026.10.08

### Operator consent, cockpit and independent raw-bag validation

#### File
- `src/research_runtime/`, `src/research_interfaces/`, `src/active_road_mapping/`
- `src/bringup/launch/system_active_road_research.launch.py`
- `scripts/catalog_research_bags.py`, `scripts/validate_console_*`, `scripts/validate_raw_transport_probe_ros.py`
- `audit/optimization_v2/`, research reports and CN/EN cockpit/command documentation

#### Change
Added a loopback cockpit, typed ordered consent and registered-task service. Browser cannot change environment, actuator configuration, physical parameters or directly write velocity. Transport loss and competing consent latch; RTK loss still stops even healthy LIO. Actual paused-clock red at f8 retained a .216m/s final wire; SystemClock arbitration in the original headerless guard makes the ce fault-phase final five wire frames zero without editing original source/limits.

#### Reason
Provide explicit inspectable human task consent and independent data identity. Prevent continued renewal during state-read loss, automatic transport-recovery rearm, frozen stop arbitration under paused bag clocks, or interpreting derived bags/monitor backpressure as independent estimator trials. Actual software and policy runs retain provenance and failures.

#### Effect
Read-only122 metadata entries yield3 independent raw CDR streams. Two more full1x sequential FAST/Super trials retain UNKNOWN source health and frozen08d binary provenance. The actual simultaneous receiver probe demonstrates monitor backpressure: deep3975/3975, shallow3271; no estimator packet-loss inference. Actual HTTP raw start/pause/resume/owned-stop passed; SIGSTOP resume pacing remains a stated limitation.

ce clean SDK/all14 Humble packages built in2m51s. Current128 portable tests,35 final-PTY faults, paused-clock stop,7HTTP cases, actual CUA GET outage, default entry and122 source/parsed/runtime checks passed. Recovery never automatically enables motion. Desktop/tablet show no horizontal overflow. Delivered default replay preview has actuator/mission disabled and no player started.

Six new60s policy tasks passed protocol but reached0/6 goals; benefit remainsFAIL. The earlier45s cohort is retained, not mixed into60s comparisons. Policy shares foundation/perception/safety, without truth input. Synthetic assumptions do not constitute physical acceptance. Jetson deployment, localization/ground/control qualification and new-stack physical stopping remainPENDING; existing installation/chassis records are corrected and used in the following audit. No firmware flash, physical driving, asset deletion, production merge or force push.


### Repository vehicle evidence and response-audit correction

#### File
- `scripts/audit_recorded_chassis_response.py`, `scripts/test_recorded_chassis_response.py`
- `audit/vehicle_contract_review/`, `LIVE_ACCEPTANCE_CHECKLIST.md`, `RESULTS.json`, delivery report

#### Change
Re-read mounting notes, the legacy FAST IMU-origin navigation convention and original STM32 sources. Derived command RPM bound1853.7696, theoretical wheel-feedback ratios1.0105263/.9296842 (the latter does not correct the serialized gyro) and speed-dependent curvature from unchanged caps. Actually analyzed all41 command/LIO bags,68732 qualified intervals and twoJuly7 original TX/RX logs read-only, Y forward feedback is nonzero607/571 times and has exploratory coupled-response fits. Among1186 zero-command transitions including rotation,15 have at least1s continuous source-header/planar stillness coverage; outliers/unconfirmed items retained.

#### Reason
Earlier broad claims that installation/chassis data was missing were inaccurate. Distinguish supplied evidence, reference conventions, unfinished research integration and new-stack physical acceptance. Do not invent parameters to enable motion or request all existing measurements again.

#### Effect
Source equations and permission limits are known; feedback qualification limitations have concrete counts. Nine independent fit/excitation/finite-value/feedback-axis/sign/two-clock/lateral/gap tests pass. No firmware/calibration/limits/installed runtime change. New Super health/reference/ground/control integration and target acceptance remain incomplete and explicit. The research curvature-zero gate is unfinished configuration, not proof the repo lacks chassis information.

### Physical acceptance preparation: observation certificate and original reference

#### File
- Super patch0002/application recipe, vehicle adapter/reference YAML/launch
- `scripts/validate_super_lio_information.cpp`, `audit/vehicle_ready/`

#### Change
Derived a sufficient conservative legacy-information lower bound from actual Super matched observations under fixed extrinsics. Retain75/50 features/3 IMU checks, worst native iteration and full covariance cross blocks. Invalid observations do not advance the map. Preserve the documented IMU-origin navigation point and own explicit local-world/odom gauge; pair source health by measurement stamp, latch source clock/pose jumps.

#### Reason
Permanent UNKNOWN and an unconnected reference prevent useful shadow operation. Source mathematics supports a stricter sufficient criterion without fixed healthy, relabeling a physical origin or requesting existing calibrations again.

#### Effect
Original firmware/calibration/caps/RTK-loss stop unchanged. Actual native C++/Humble/replay/final-wire evidence is still required; this source progress does not finish the goal. Ground and EGO source-constraint integration and target shadow qualification remain incomplete, with no automatic physical driving.

### Native failure paths, body covariance and world ownership

#### File
Super patch0002; EGO patch0005/application recipe; vehicle/cloud adapters; native probes/runner; mock-wire TF cases; reference contract and patch READMEs.

#### Change
Check IMU count/time/point coverage before propagation; interpolate scan end from real bracketing samples, retain the lookahead, and use valid interpolation pairs. Restore dynamic/forward caches on failed iterations and use posterior gyro bias. Publish both twist blocks in IMU coordinates with the complete18-state attitude/velocity/bias Jacobian. Reject incompatible covariance convention and deeply nested malformed health JSON. Guard unique static identity world ownership and independently validate cloud lookups. Add four world-fault/latching cases at the original PTY sink.

#### Reason
Independent reviews found an end-iterator dereference before the late IMU guard, incomplete rollback/posterior rate, omitted attitude covariance, and competing-world TF behavior. These are software correctness defects despite valid original mounting records.

#### Effect
156 portable checks pass. The earlier b40 SDK/14-package build is superseded evidence; this revision requires a separate source-bound Humble build, native probes, raw replay and final serial cases. Original protected settings and production assets remain preserved; no physical result is inferred.

### Raw replay coverage and unordered MID scan-end regression

#### File
Super patch0002; native packet probe/runner; strict source replay/captured-callback/read-only packet audits; three-layer/launch probes; `audit/vehicle_ready/diagnosis.md`.

#### Change
Actual9f build/native/43 final-wire/136 parameter checks pass. Tighten replay to sustained coverage and terminal output; record a two-frame actual CDR reference-fault fixture. Native MID parser now uses the maximum accepted offset, preserving point times; malformed array/count is rejected before indexing.

#### Reason
The first replay check incorrectly passed1866 source/363 vehicle frames. A tight callback reproduces23.48deg over.399s; original15deg protection correctly stops.248 actual packets have unordered maximum/last offsets,1147 overlap previous packet intervals, and the fault-window old-bag gyro exceeds current .7 caps. A valid-constructor actual parser probe fails the maximum-end assertion before the fix.

#### Effect
No stop threshold/clock/calibration/cap relaxation. Three native probes,35 original faults,8 world fault/latch cases and136 triple-layer checks are concrete evidence; sustained source integration remains FAIL until the tightened replay is rerun. Jetson connectivity is currently unavailable. This is ongoing authorized goal work, not completion or physical acceptance.

### Actual overlapping scans and replay terminal audit

#### File
Super patch0002; native deskew/packet probes; `scripts/replay_acceptance.py` and counterexamples; strict raw replay; `audit/vehicle_ready/cc5ddc6/`; reference contract/commands.

#### Change
Actualcc5 clean SDK/14-package build and four native probes pass; the parser red regression is green. An85s raw prefix has627/627 paired outputs and no reference latch but200 overlapping scans denied and terminal health FAIL, so overall FAIL remains. Retain9f captured actual CDR/packet audit and old red fixtures. Test both raw input tails, sustained span/gaps, terminal health, quaternion and pre-cleanup PREFIX success; six false-PASS counterexamples pass. Packet Python/native entries enforce105/localhost before ROS initialization; actual unisolated native execution exits2.

#### Reason
A subset of good frames does not prove a continuous usable interface. Original MID arrival-clock packets overlap, while the current native deskew rejects every point before the current state. The minimal actual-native fixture fails on a1.035 point after a1.04 state. Keep real integrated history and posterior endpoints instead of relaxing the original15deg/.50m protection or rewriting time.

#### Effect
The next source revision adds finite historical deskew while still requiring3 current IMUs, actual endpoint brackets and rejection of older points; actual build/green regression/full replay remain required. Original firmware/protocol/calibration/physical limits/RTK-loss stopping are unchanged. Ground/EGO/target qualification and benefit remain unfinished; the goal continues with no automatic physical motion.

### Posterior history endpoint continuity regression

#### File
Super patch0002/native deskew probe/reference contract/diagnosis; `audit/vehicle_ready/cd54c70/posterior-boundary-red.log`.

#### Change
After the actualcd54 fourteen-package build, a real ESKF UpdateObserve with synthetic PSD position/bias cross covariance changes posterior acceleration and fails the shared-boundary continuity assertion. Historical segments now use endpoint-exact position/velocity Hermite interpolation; current IMU segments retain native propagation.

#### Reason
Re-integrating a past segment with posterior acceleration can miss its saved position endpoint. The earlier translation/yaw-only zero-acceleration fixture missed this defect. The independent review finding is confirmed by an actual native red test.

#### Effect
Finite history/real brackets/rollback/original safety limits remain. Green native regression and full raw replay still require execution; a successful compile is not continuous interface or physical acceptance.
