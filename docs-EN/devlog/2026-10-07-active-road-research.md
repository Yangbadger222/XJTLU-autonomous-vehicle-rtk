## 2026.10.07

### Isolated Super-LIO / EGO2D active-road research

#### File
- `dependencies.research.repos`, `patches/super_lio/`, `patches/ego_planner_2d/`
- `src/research_interfaces/`, `src/research_runtime/`, `src/active_road_mapping/`, `src/super_lio_vehicle_adapter/`, `src/ego_vehicle_adapter/`
- `src/bringup/launch/system_active_road_research.launch.py`, `src/bringup/config/*research*`, `src/bringup/config/ego_vehicle_adapter.yaml`
- `scripts/*research*`, `scripts/validate_*`, `audit/`, root research reports, bilingual architecture and commands

#### Change
Pinned the specified Super-LIO ros2 and already-2D EGO develop commits on the exact e54c6af corridor vehicle baseline. Four replayable EGO patches add measured-state/GridMap/road-reference/TimedTrajectory interfaces, dynamic repair, main turning costs and continuous nonholonomic/footprint certification. Separate research modules implement tracking through the original guard/serial chain, finite observation queries, reversible measured evidence and session-bound verified history. The new replay/shadow/live entry excludes the legacy task stack and preserves necessary sensor, localization, safety and logging foundations.

#### Reason
The migration must use the actual upstream estimator/planner, preserve tested vehicle parameters and authority protection, and examine finite task-driven observation without granting unknown ground or LIO health permission to bypass RTK loss. A new localization epoch must not reanchor old odom evidence, and later unqualified evidence must remain local while verified historical topology can be reused.

#### Effect
The authorized non-Jetson Humble host actually executes untouched upstream simulation, separate complete original raw-bag estimator replays, fresh SDK/14-package builds, source/parsed/runtime override audits and final original-serial PTY tracking/fault tests. Three restricted-sensor strategies run two tasks each on the same hashed foundation. Red/green and negative outcomes remain in audit; current outcomes are in RESULTS.json/AUDIT_REPORT.md. Parameter source bytes, firmware and safety permissions are preserved. No Jetson editing, automatic driving, flashing, asset deletion or production merge occurred. Missing physical calibration, source health, ground/camera integration and live acceptance remain explicit PENDING; software passes do not establish policy benefit or physical readiness.

Persisted histories retain per-item measurement/anchor/graph/rollback validation and check the full hash once after transactional loading. This removes quadratic reload work that starved the original state/permission deadlines in second tasks; deadlines and final stop conditions remain unchanged.
