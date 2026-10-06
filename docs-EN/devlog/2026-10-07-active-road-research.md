## 2026.10.07

### Super-LIO + Ego-Planner-2D-ROS2 research entry

#### Changed files
- `src/research_runtime/`
- `src/research_interfaces/`
- `src/super_lio_vehicle_adapter/`
- `src/bringup/launch/system_active_road_research.launch.py`
- `audit/` and research reports

#### Changes
Pinned both external commits, locked vehicle parameters and launch overrides,
and added TimedTrajectory2D, dynamic/curvature/footprint checks, evidence map,
active-observation scoring, and mock-serial replay.

#### Why
The EGO demo's unit initial state, visualization-only Path, disabled feasibility
repair, and MPPI chain cannot be treated as an executable vehicle interface.

#### Impact
The research entry defaults to replay/shadow. Until Super-LIO's IMU-to-vehicle
mapping is verified, the adapter reports UNKNOWN and stops. Jetson, bag, camera,
and physical acceptance evidence remain pending.
