# Active-road entry legacy manifest

The legacy source remains in the baseline checkout and is not deleted. The
new entry point `system_active_road_research.launch.py` has an allowlist of
research components and does not include `system_explore.launch.py`, Nav2,
SLAM Toolbox, PGO/FGO shadow, CUDA MPPI, route task dispatchers or the fake
vehicle simulator.

| Component | New entry status | Reason/evidence |
|---|---|---|
| FAST-LIO2 process | excluded | `super_lio` is the only LIO candidate in the research launch; old package remains for rollback |
| Super-LIO node | shadow/live default, replay excluded | `enable_super_lio=true` in shadow/live; replay remains actuator/sensor-free and the vehicle adapter still blocks unverified output |
| SLAM Toolbox | excluded | no include in active-road launch |
| Nav2 planner/controller/MPPI/BT | excluded | no Nav2 include; `nav_msgs/Path` is not a control contract |
| PGO/FGO shadow | excluded | `rtk_map_odom_corrector` remains the sole map→odom authority |
| FRC/learning sidecars | excluded | no include in active-road launch |
| old route/menu/explore/travel dispatchers | excluded | active-road evidence manager owns research events; old packages remain rollback assets |
| Livox driver | retained in shadow/live | existing `/livox/lidar` and `/livox/imu` inputs are reused |
| RTK driver and authority | retained in shadow/live | authority loss continues to stop motion |
| corridor command guard | retained | final stop and speed authority remains unchanged |
| serial reader/twistctl | retained | final mock/STM32 command contract remains unchanged |
| local ground/obstacle inputs | adapter boundary | `research_runtime.grid_map` provides the conservative odom-frame projection and unknown-as-occupied contract; Super-LIO output, existing height filters and target-side sensor/TF wiring require runtime audit |
| active-road evidence ingest/policy | new research boundary | `active_road_evidence` accepts typed measured `RoadEvidence2D` and externally generated observation candidates; it does not replace the legacy route/task dispatchers or infer unobserved roads |
| `rviz_car_sim/fake_sim_node` | replay only | it cannot own live TF or final command |
