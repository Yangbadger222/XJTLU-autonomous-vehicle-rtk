# Migration report

The research branch starts at vehicle baseline `e54c6afbcb5a58db22d7c468085a87d658b0b932` and pins Super-LIO `f89f48dc7aea6cfa262f18e4d03b319e04e0dbd2` and Ego-Planner-2D-ROS2 `7f5be6d4cee34871e85aa1f15285cfaf17b23877` in `dependencies.research.repos`.

Super-LIO is configured with the audited Livox topics and the locked FAST-LIO2
range/stride/extrinsic values. Its upstream `world→imu` result is not relabeled
as `odom→base_footprint`; `super_lio_vehicle_adapter` rejects source-frame
mismatch and refuses a `world`→`odom` string rewrite. A timestamped source-frame
TF, measured IMU→base transform, covariance handling and health equivalence are
required before vehicle odometry can publish. This is a real algorithm
replacement boundary, but target-side build and runtime proof is pending.

The reproducible Super-LIO patch is
`patches/super_lio/0001-publish-source-aware-odom-health.patch`. It preserves
the estimator, exposes source covariance/angular state, and emits explicit
UNKNOWN health rather than pretending to match FAST-LIO2 degeneracy.

The pinned EGO2D source is treated as upstream code. The local boundary adds
real state fields, a vehicle ROS edge for odom/road-reference/OccupancyGrid,
typed timed trajectories, dynamic feasibility, nonholonomic curvature/yaw-rate
checks and footprint rejection. It does not use Nav2 MPPI or the upstream fake
simulator in the active-road entry. The original vehicle parameters and safety
guard are preserved and hashed. The physical serial sink now requires both
`execution_mode:=live` and explicit `enable_serial:=true`; replay and shadow do
not attach production serial.

The reproducible source patches are
`patches/ego_planner_2d/0001-vehicle-state-and-feasibility.patch` followed by
`0002-vehicle-ros-timed-trajectory.patch` and
`0003-clear-stale-plan-on-failure.patch`. They apply only at the pinned EGO
commit, and their sequential check is enforced by
`scripts/apply_ego_vehicle_patch.sh`; a fresh checkout was verified with
`git apply --check`. The raw EGO node and a prior patched EGO revision, plus
Super-LIO core/ROS interfaces, were compiled in the isolated ARM64 ROS 2 Humble
evidence environment. The current EGO patch set 0001+0002 has the original
`ros2-go2:humble` ARM64 compile pass; patch 0003 has exact sequential
apply-check, and the complete 0001+0002+0003 stack now compiles and starts a
bounded `motion_plan` smoke under OrbStack `linux/aarch64` with public
`ros:humble`. The alternate base is recorded separately and does not replace
the target-image or Jetson gates. The compile-only Livox message contract is deliberately
recorded separately from the missing physical SDK/driver, ordinary container
startup and Jetson runtime gates.

The local obstacle boundary is implemented in
`src/research_runtime/research_runtime/grid_map.py` and transported by
`research_local_obstacle_grid`: it accepts only odom-frame points, requires a
non-UNKNOWN map version, applies the explicit height-window inputs, marks
measured cells occupied, and preserves every unobserved cell as
safety-blocking unknown. Super-LIO's `world` cloud is passed through the strict
`super_lio_cloud_frame_adapter`, which uses the point-cloud timestamp for a
`world`→`odom` TF lookup and withholds on missing TF; target-side TF and cloud
runtime evidence remain pending.

`active_road_map` now publishes a persisted map version only when an EvidenceStore loads successfully and relays only non-empty odom-frame road references; missing or corrupt evidence remains UNKNOWN and blocks EGO output.

`active_road_evidence` is the typed ROS edge for the remaining research loop.
It accepts only measured `RoadEvidence2D` geometry already expressed in `odom`,
persists it atomically through the CRS-bound store, republishes observed
geometry events, and selects only externally supplied candidates marked
reachable, safe, pose-trustworthy and sensor-valid. It never supplies camera
calibration, global transforms, simulator truth or an unobserved road segment.

The timed trajectory no longer jumps directly from the first `TimedTrajectory2D`
sample to `/cmd_vel`. `research_runtime.trajectory_tracker` interpolates the
time-indexed plan against measured `/lio/odom_vehicle` pose, applies bounded
nonholonomic feedback, and hands the request to the original authority and
command guard. The bridge gates on `/lio/vehicle_health` and a matching
odom-frame obstacle grid/map version plus explicit measured footprint; this
remains a compile-and-contract result until the target ROS graph is run.
