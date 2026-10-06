# Rollback

The production baseline remains `corridor-authority-stability` at
`e54c6afbcb5a58db22d7c468085a87d658b0b932`; it is not merged or force-pushed.
To roll back a research checkout, stop its processes, switch to that branch,
restore the original install/runtime-data path and use the original corridor
launch. The protected file hashes in `audit/vehicle_baseline` provide a check
before any deployment. Do not delete the research branch or old maps.
