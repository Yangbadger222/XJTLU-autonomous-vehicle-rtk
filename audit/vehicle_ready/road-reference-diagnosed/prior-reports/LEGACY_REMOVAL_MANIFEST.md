# Active-road entry legacy manifest

Current qualified native runtime is 77d5c5a5ce26b52b22d5631d67ba88a71ad410b3. Native16-node cleanup, current13HMI/final serialPTY,157three-layer checks and244portable cases PASS. Architecture-specific staged compilation/loading and ARM diagnostic results are in AUDIT_REPORT.md and audit/vehicle_ready/rigid-geometry-qualified/COHORT.json. Actual ground support FAIL, policy protocol5PASS/1FAIL with0/6goals; target/physical and Mac sync PENDING. Prior reports and measurements retain their original source identities in this cohort's prior-reports/.

The legacy source remains in the baseline checkout and is not deleted. The
new entry point `system_active_road_research.launch.py` has an allowlist of
research components and does not include `system_explore.launch.py`, Nav2,
SLAM Toolbox, PGO/FGO shadow, CUDA MPPI, route task dispatchers or the fake
vehicle simulator.

| Component | New entry status | Reason/evidence |
|---|---|---|
| FAST-LIO2 process | excluded | `super_lio` is the only LIO candidate in the research launch; old package remains for rollback |
| Super-LIO node | default in replay/shadow/live when enabled | `enable_super_lio=true`; default replay starts actual estimator while drivers, physical serial and mission execution remain disabled |
| SLAM Toolbox | excluded | no include in active-road launch |
| Nav2 planner/controller/MPPI/BT | excluded | no Nav2 include; `nav_msgs/Path` is not a control contract |
| PGO/FGO shadow | excluded | `rtk_map_odom_corrector` remains the sole map→odom authority |
| FRC/learning sidecars | excluded | no include in active-road launch |
| old route/menu/explore/travel dispatchers | excluded | active-road evidence manager owns research events; old packages remain rollback assets |
| Livox driver | retained in shadow/live | existing `/livox/lidar` and `/livox/imu` inputs are reused |
| RTK driver and authority | retained in shadow/live | authority loss continues to stop motion |
| corridor command guard | retained | final stop and speed authority remains unchanged |
| serial reader/twistctl | retained | final mock/STM32 command contract remains unchanged |
| local ground/obstacle inputs | adapter boundary | `research_runtime.grid_map` provides the conservative odom-frame projection and unknown-as-occupied contract; actual acquisition-matched unfiltered Super-LIO cloud feeds the conservative ground model; target-side sensor/TF timing requires shadow audit |
| active-road evidence ingest/policy | new research boundary | `active_road_evidence` accepts typed measured `RoadEvidence2D` and externally generated observation candidates; it does not replace the legacy route/task dispatchers or infer unobserved roads |
| `rviz_car_sim/fake_sim_node` | excluded from new entry | analytical harnesses are separate finite tests and do not own target TF/command |
