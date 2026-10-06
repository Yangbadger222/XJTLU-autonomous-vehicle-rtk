# Migration report

The research branch starts at vehicle baseline `e54c6afbcb5a58db22d7c468085a87d658b0b932` and pins Super-LIO `f89f48dc7aea6cfa262f18e4d03b319e04e0dbd2` and Ego-Planner-2D-ROS2 `7f5be6d4cee34871e85aa1f15285cfaf17b23877` in `dependencies.research.repos`.

Super-LIO is configured with the audited Livox topics and the locked FAST-LIO2
range/stride/extrinsic values. Its upstream `world→imu` result is not relabeled
as `base_footprint`; `super_lio_vehicle_adapter` reports UNKNOWN and blocks
vehicle odometry until a measured IMU→base transform and health equivalence are
provided. This is a real algorithm replacement boundary, but target-side build
and runtime proof is pending.

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
`0002-vehicle-ros-timed-trajectory.patch`. They apply only at the pinned EGO
commit, and their sequential check is enforced by
`scripts/apply_ego_vehicle_patch.sh`; a fresh checkout was verified with
`git apply --check`. A full ROS/Humble compilation remains a target-side gate.
