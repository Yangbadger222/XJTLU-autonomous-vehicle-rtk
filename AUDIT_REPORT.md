# Current software and research audit — 2026-10-08

The goal remains ACTIVE on codex/superlio-ego-active-road. Native runtime source 77d5c5a5ce26b52b22d5631d67ba88a71ad410b3 descends from the sole approved e54c6afbcb5a58db22d7c468085a87d658b0b932 baseline. All1011 protected originals remain byte-identical. Exact external pins and current2Super/11EGO patches are checked against every dependency source byte. No protected firmware, calibration, serial protocol, tested limit or RTK permission changed.

| Executed gate | Result | Evidence under audit/vehicle_ready/rigid-geometry-qualified/ |
|---|---|---|
| Native Humble default | PASS:16 children,12 parameter dumps,one command bridge,all15 new/infrastructure children exit0 | ground-signal-native-default.json |
| Current source/parsed/effective values | PASS:157; seven invalid-original override probes PASS at85eb28c | ground-signal-three-layer-parameters.json; mapping-signal-native-default.json |
| Final original mock serial | PASS:13 actual HTTP/typed-permit/observer/guard/serialPTY cases at77d5c5a; RTK loss despite healthy LIO stops | ground-signal-hmi-qualified.json |
| Portable behavior regression | PASS:244; seven actual entry ASTs,21 signal/error cases | ground-signal-portable-qualified.log |
| Native C++ geometry | PASS:four previous unchanged probes plus corrected orientation and noncommuting integration probes;60000 main/forward predictions | rigid-group-native-checks/; rigid-group-orientation-qualified/; rigid-group-noncommuting-integration/ |
| C++ source/binary binding | PASS:61 frozen basic/Super files match current source; actual loader traces and both ELF architectures | current-cpp-source-loader-proof.json |
| ARM C++ rebuild and two probes | PASS:actual basic+Super build; current orientation and60000/noncommuting integration | rigid-group-arm-build.log; rigid-group-arm-orientation-qualified/; rigid-group-arm-integration/ |
| Full real July15 EOF at49b44ee | PASS:57008IMU,2867LiDAR,2850 native/vehicle/control/cloud outputs; no rotation rejection or gap latch | rigid-group-raw-full-0715.json |
| Actual July21AM30s ground prefix at49b44ee | FAIL:281 exact ground acquisitions,zero positive support/strips/persisted geometry | rigid-group-raw-ground-0721-am-prefix.json |
| Six same-foundation policy tasks at49b44ee | FAIL strict protocol:5PASS,1no-actuationFAIL; all0/6goals; benefit unproved | rigid-group-default-comparison/result.json |
| Target and physical acceptance | PENDING | LIVE_ACCEPTANCE_CHECKLIST.md at repository root |

Architecture-specific current default/loader results are in COHORT.json and current-build.json. The protected original guard's SIGINT KeyboardInterrupt/exit-2 remains an explicit baseline exception. Every other nonzero/missing/live child fails. Builds are staged: native SDK14 at8c937ed, ARM SDK14 at a66cc7e, then rebuilt basic/Super at49b44ee and the changed Python packages. No all14 fresh build at77d5 is claimed. The ARM finite native-fd loopback helper explicitly diagnoses QEMU6.2 networking; it cannot certify native Jetson performance or hardware.

Actual behavioral REDs are preserved. The old quaternion output exceeded the unit-norm guard. An invalid state still leaked a world cloud. Output normalization alone at72e150b failed the real EOF with702 source rotation denials. An ideal constant-IMU ESKF failed after4000 float compositions. The final rigid composition passes60000 without relaxing the numeric bound. Old default signal handling reproducibly closed context mid-message; the seven owned-entry fixes preserve cleanup order and ordinary exception visibility.

The orientation QA's Eigen auto expression held dead temporaries. ASAN reproduced stack-use-after-scope; an owning Vector3d fixes the test without widening1e-6. Corrected native/ARM receipts supersede that assertion. One initial native ldd receipt used the wrong loader environment; the actual loader reexecution closes the provenance defect. The ARM source-loader receipt's hardcoded native foundation metadata is corrected separately to actual a66cc7e; raw measurements are retained. COHORT.json labels setup failures, real behavioral failures, superseded QA and finite timeouts separately.

Existing MID360 mounting/internal-IMU/factory/URDF records and STM formulas with41bag datasets have been used. D455f is recorded. The sealed122-entry catalog has121 valid metadata and no image/CameraInfo/CompressedImage bags; this inventory does not prove that calibration notes or other assets do not exist. Current flashed identity and labelled physical KEY/estop/braking/slip are not inferable from source equations alone.

Prior successful interface, trajectory, sensor-ingress and terminal-console mutation tests retain their original cohorts. Current policy failure is kept visible; no cross-cohort winner is claimed. The user's older ce46418 actuator-disabled preview remains separate. No Jetson source edit, physical drive, firmware flash, production merge or force push occurred.
