# Current qualification checkpoint — 2026-10-08

The goal remains active. Vehicle baseline is corridor-authority-stability@e54c6afbcb5a58db22d7c468085a87d658b0b932; research branch is codex/superlio-ego-active-road. Earlier reports are preserved under audit/vehicle_ready/history_pre_qualification/ and their runtime identities remain historical.

| Executed check | Evidence | Result and scope |
|---|---|---|
| Original source protection | audit/vehicle_baseline/PROTECTED_FILES.sha256 and PROTECTED_ADDITIONAL_FILES.sha256 | 1011 originals unchanged; firmware/calibration/protocol/limits/permissions preserved |
| Source/parsed/runtime parameters | audit/vehicle_ready/da361f2/entry.json | 143 checks and seven deliberate override probes; final repeated entry receipt reported separately; target overlays remain pending |
| Native source/patch provenance | audit/vehicle_ready/da361f2/patch-source-proof.json | Exact pinned Super2/EGO6 patches; fresh sequential application and all-file equality against actual runtime checkout |
| Clean Humble build | audit/vehicle_ready/da361f2/clean-build.json and build.log | Fresh independent original SDK/14 packages, one worker, no production overlay; non-Jetson x86_64 |
| Native filter/time/covariance | audit/vehicle_ready/69b9b75/native-results.json | Four actual native probes; malformed packet, overlapping IMU history, failed-update rollback, posterior-boundary continuity, sufficient observation bound/full body covariance |
| Source reference/health | audit/vehicle_ready/69b9b75/adapter.json | Four installed ROS cases, lever arm/rotation/covariance/cloud height and explicit expiry; original navigation IMU origin retained |
| Full raw data qualification | audit/vehicle_ready/69b9b75/raw-*.json | Three full1x EOF0 trials, 2850/3372/4151 paired native/vehicle/cloud outputs; max gaps .1055/.3792/.3820 s;5/2/2 failed source certificates retained |
| Positive ground geometry | audit/vehicle_ready/ground-source/adjacent-step-red.log; current portable suite | Actual red adjacent-step counterexample then green; dense subcell/local-neighbor plane checks, holes/low obstacles/sparse/missing support |
| Ground ROS transport | audit/vehicle_ready/da361f2/ground.json |16 installed ROS cases including combined grid, atomic map/session, same acquisition, reverse arrival, paused-clock expiry, low obstacle and step/drop |
| EGO ROS contract | audit/vehicle_ready/da361f2/ego-interface.json |10 actual cases through adapted pinned core; explicit analytical control settings |
| Final actuator sink | audit/vehicle_ready/da361f2/serial-faults.json |39 faults at original serial binary -> allocated PTY, including four mismatched grid identity/payload cases; no physical serial device |

The first old permissive full replay and tight replay failures are retained, not relabeled. Packet end now uses the maximum accepted original offset; one actual preceding integration interval is retained, posterior historical interpolation is endpoint-exact, and missing/out-of-window IMUs still deny eligibility. The five July15 history denials are visible. These receipts establish functional source/interface behavior for these inputs, not localization ground truth or all-bag accuracy.

The MID360 internal IMU, factory LiDAR/IMU translation, mounting height and STM command model are already recorded. Original FAST uses raw g times10 without amplitude norm correction; Super uses initial g/mean_norm scaling. Different known normalization is not an absent unit. The information gate is a stricter sufficient bound under the original fixed extrinsics; it is not the identical old degeneracy metric. Missing/expired/malformed/failed certificates deny health; RTK loss independently stops motion.

New positive-support thresholds are NEW_ALGORITHM_SETTING and assume locally level, densely observed terrain. Ground messages atomically bind acquisition, map and localization session; local fusion never refreshes older support using a newer obstacle timestamp. nav_msgs/OccupancyGrid remains a visualization boundary; planner and final bridge consume typed LocalEvidenceGrid2D. Synthetic fixtures are explicit and disabled in the ordinary entry.

Software still unfinished: source-derived nonzero default execution settings/command-model checks and in-place handling; real raw support/road-evidence integration and persistence; local perception qualification when global TF is unavailable. Target shadow and physical stop/brake/terrain/slip acceptance are PENDING. The old finite strategy comparison remains a negative result,0/6 goals; no new benefit or novelty is asserted. RESULTS.json distinguishes these states.
