# Control-reference and immediate-denial review

Review source: 63fc00c3c92c23b873b00ece8586f676921a87e1 on the authorized non-Jetson Humble host. Both reviews were read-only.

## Standards

The independent Standards review closes the timer-gap P2: denial callbacks immediately revoke the rotation token and clear stop confirmation; restoring inputs before the timer cannot bypass the original one-second window. It independently checked 1011 original hashes, source-generator --check, 26 installed current runtime hashes and exact Super2/EGO11 patch proof. No new hard Standards P1/P2 was found in this increment.

## Spec

The independent Spec review closes the same receipt-ordering P2 after inspecting authority/mode/health/TF/map revocation. It checked five old-implementation RED counterexamples, current 222 portable passes and ten actual installed callbacks-to-original-serial PTY cases. No new Spec P1/P2 was found in this increment.

The original measured stop rates are tolerances, not physical brake confirmation. Measured yaw velocity/acceleration remain recovery boundary conditions; admission is committed only after the final gates and invalidated on denial or stamp discontinuity.

Executed evidence is audit/vehicle_ready/control-stop-qualified/: immediate-denial10, linear44, yaw50, RTK5, HMI11, heading/arc goals, EGO10 plus2 queries, observer7, entry16 and parameter157 checks. Current build is explicitly an 8c937ed fresh SDK/14 foundation plus a63fc00c runtime rebuild, with fresh=False in the staged receipt. Historical reviews and receipts remain in history_pre_control_stop_qualification/.

Neither review certifies real terrain, strategy benefit, target deployment or physical braking. Actual ground support remains FAIL; the current strategy cohort reaches0/2 goals for each strategy; target shadow and physical acceptance remain pending.
