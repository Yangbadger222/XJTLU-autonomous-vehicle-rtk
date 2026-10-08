# Current software qualification — 2026-10-08

The goal remains ACTIVE. The only vehicle baseline is corridor-authority-stability@e54c6afbcb5a58db22d7c468085a87d658b0b932; development stays on codex/superlio-ego-active-road. Current qualified runtime source is a7dc39774c4421b40a6a0abb7d1709a81bdc72b0. All 1011 protected originals remain byte-identical. Firmware, calibration, serial protocol, motion limits and RTK authority are unchanged.

| Executed gate | Result | Source-bound evidence |
|---|---|---|
| Native Humble default entry | PASS:16 owned children,12 parameter queries,one cmd_vel bridge writer; all15 new/infrastructure nodes exit0 | shutdown-qualified/terminal-native-default.json |
| ARM64 emulated entry | PASS with explicit QEMU6.2 native-fd loopback diagnostic helper; same16 node cleanup and30 source hashes | shutdown-qualified/arm-terminal-default-qualified.json |
| Current compilation/loading | PASS: prior fresh SDK / 14/C++ foundation plus isolated current Python overlays;30 loaded Python source hashes match on each architecture | shutdown-qualified/current-build.json, arm-terminal-current-build.json |
| Source/parsed/effective values | PASS: 157; previously executed7invalid-original override probes retained at their original source | shutdown-qualified/terminal-three-layer-parameters.json; control-stop-qualified/current-default-entry.json |
| Console/authority/final serial | PASS: 13actualHMI/originalguard/originalserialPTYcases; signals receive newer generated STOP before console exit; RTK loss stops despite healthy LIO | shutdown-qualified/terminal-hmi-qualified/result.json |
| Exit race and negative control | PASS: 19 consoleunitchecks; actual old close interleaving RED; omitted-final-STOP mutation correctly fails2 signal cases even though timeout eventually yields zeros | shutdown-qualified/close-http-race-green.log; hmi-no-final-stop-counterexample/result.json |
| Portable regression | PASS: 223 | shutdown-qualified/terminal-portable.log |
| Upstream/trajectory/bag interfaces | Prior source-bound PASS receipts preserved:2 Super patches/11 EGO patches; fullJuly15EOF and default heading/arc loops | control-stop-qualified/current-source-proof.json; control-reference-qualified/raw-control-full-0715.json |
| Actual road support | FAIL:0 qualified vehicle-width strips; density-window diagnostic has at most1 cell | control-stop-qualified/raw-ground-0721-am-prefix.json, raw-ground-0721-pm-prefix.json |
| Research benefit | Not established: three strategies using the same foundation each reached 0/2 goals | control-stop-qualified/policy-comparison.json |
| Target shadow and physical acceptance | PENDING | LIVE_ACCEPTANCE_CHECKLIST.md |

Evidence paths in this table are under audit/vehicle_ready/. The current compile proof is staged; it is not a fresh all 14 build at a7. Native Socket diagnostic startup does not certify unmodified QEMU networking, Jetson performance or hardware. The protected original guard's SIGINT KeyboardInterrupt/exit-2 is an explicit baseline exception; other nonzero, missing or live children fail the validator.

Two runtime changes close actual defects: console/cloud main loops finish owned cleanup before ROS context shutdown without catching normal RuntimeError; terminal console close atomically revokes its owner and rejects all later commands before request-history lookup. No control ceiling or authority permission was relaxed. Both independent Standards and Spec reviews closed their actionableP2findings.

Existing MID360 internal-IMU/mount/factory records, URDF and STM formulas plus 41 bags have already been used. These are available vehicle inputs. Recorded estimates cannot establish the current flashed image, labelled physical KEY/estop/braking or individual wheel slip.

Current ground failure and strategy failure are unchanged by this cohort and keep their own source identities. Earlier default-emulator DDS failure, exit errors and all setup/test failures remain retained; COHORT.json classifies behavioralRED separately from environment failures. Pre-update reports are preserved in shutdown-qualified/prior-reports/.

Research jobs and the finite native-fd broker are terminal. The user's older actuator-disabled preview remains untouched. No Jetson source edit, firmware flash, vehicle movement, production merge or force push occurred. Mac sync/cache recovery remains pending disk space.
