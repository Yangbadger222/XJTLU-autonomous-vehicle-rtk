# Audit report

## Completed locally

- Baseline branch and commit are recorded in `audit/vehicle_baseline/SOURCE_IDENTITY.json`.
- Source-file hashes cover master parameters, corridor launch overrides, FAST-LIO2 output, serial bridge and firmware README.
- The lock records source YAML, launch rewrite and the still-pending target runtime parameter layer.
- `scripts/audit_vehicle_baseline.py` was run successfully and emitted
  `audit/vehicle_baseline/PARAMETER_LAYERS.json`; the third layer remains
  `PENDING_JETSON` rather than being inferred.
- Both pinned upstream adaptation patches pass `git apply --check`; their
  target ROS builds remain pending because this host has no ROS 2 Humble.
- The serial contract is tested through the final mock sink: RTK authority false produces exactly `vcx=0,wc=0\\n`.
- Dynamic feasibility, real start state, curvature/yaw-rate consistency, map version, expiry and footprint checks have replay tests.
- Evidence persistence is CRS-bound and idempotent by observation UUID.

## Pending or blocked by environment

- ROS 2 Humble and Jetson runtime are unavailable on this macOS host; `colcon` and target launch/parameter dumps cannot run here.
- Docker is installed but its daemon is not running, so no ROS container replay was possible.
- No verified vehicle IMU→base extrinsic, expanded URDF footprint/wheel geometry, camera intrinsics/extrinsics/depth stream or vehicle bag identity was supplied in this workspace.
- Super-LIO parser output has no covariance/health equivalent at the pinned commit; adapter remains UNKNOWN and fails closed.
- Existing bags were inventoried locally but cannot be replayed without ROS 2 bag tooling.
- A concrete 15.5 s bag was inspected at SQLite topic level: it contains old
  `/fastlio2/lio_odom`, RTK and costmap topics, but no raw `/livox/lidar` or
  `/livox/imu`; it cannot support a Super-LIO replay claim. See
  `audit/VEHICLE_BAG_REPLAY_INPUT.json`.
- Physical e-stop, STM32 behavior, Jetson shadow and live motion remain pending and were not simulated as passes.

## Regression evidence

The new transport-independent suite passes (`10 passed`). The original
`gps_waypoint_dispatcher` guard/authority tests run with the package path and
recorded `191 passed, 3 failed`; the failures are baseline/environment evidence
(two diagnostic-string expectations already absent at the pinned commit and one
test importing unavailable `rclpy`). The existing corridor launch suite records
`26 passed, 1 failed` because the pinned baseline lacks the expected
`enable_local_odom_bridge: false` text. No protected file was modified to make
these tests pass.

## Follow-up implementation evidence

The pinned EGO source now has a second sequential project patch that replaces its
interactive demo edge with a vehicle ROS edge. A fresh checkout at the exact
commit accepted both patches with `git apply --check`. The edge refuses output
when map resolution, inflation/footprint radius, curvature, lateral-speed
limits, map version, measured odom state, or local obstacle grid are absent.
It emits `TimedTrajectory2D`; the visual `nav_msgs/Path` is not consumed by the
safety bridge. The active launch condition test also verifies that replay has no
planner/authority/serial processes and that the physical serial sink requires
both `live` and `enable_serial:=true`.

The new EGO ROS edge is intentionally fail-closed when the target local
obstacle-grid and road-reference producers are absent. Their ROS contracts are
locked in `audit/vehicle_baseline/RUNTIME_CONTRACT.json`, but their target-side
frame/ground-separation wiring is still pending and is not represented as a
software PASS.
