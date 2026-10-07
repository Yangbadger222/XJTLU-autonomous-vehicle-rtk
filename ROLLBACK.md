# Rollback without changing production history

The protected source baseline is corridor-authority-stability at e54c6afbcb5a58db22d7c468085a87d658b0b932. The research branch is separate; no production merge, force push, firmware flash or Jetson source edit is part of this task.

Stop only recorded research process groups. Leave old maps, logs, production install and current_scene intact. Select the preserved production checkout/install and its original corridor launch under the operator's existing procedure; do not delete the research branch or overwrite production runtime-data. If a separate clean checkout is needed, create it at the verified baseline ref and verify audit/vehicle_baseline/PROTECTED_FILES.sha256 plus PROTECTED_ADDITIONAL_FILES.sha256. No reset --hard, git clean or broad pkill is required.

A research graph increment is rolled back with a persisted tombstone; its original observation UUIDs and prior bytes remain. Historical submaps are not reassigned to a new LIO origin. A target-side rollback or live restart remains a human deployment action.
