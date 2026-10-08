Ranked hypotheses before the adapter fix:
1. Default rclpy SIGINT shuts context while subscription take converts a message. Actual ARM RuntimeError appears after the interrupt; actual production-main AST has the same default init/spin pattern. Deterministic actual-main SIGINT/SIGTERM: 2 failures, ordinary RuntimeError test passes.
2. Message/ABI conversion mismatch independent of signals. Native/ARM loader hashes and successful parameter startup are evidence against this being the observed exit-only problem. Ordinary RuntimeError must continue propagating.
3. An unrelated application callback error. No application callback frame appears in the failure; conversion fails inside rclpy take after SIGINT.

The separate 240s ARM outer deadline terminated the test before a final JSON receipt. Preserve timeout124 and the exit1 adapter failure; neither is PASS. Do not use retry-only success to claim the race fixed.
