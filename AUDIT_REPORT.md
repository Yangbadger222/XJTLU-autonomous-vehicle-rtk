# Audit report

## Completed locally

- Baseline branch and commit are recorded in `audit/vehicle_baseline/SOURCE_IDENTITY.json`.
- Source-file hashes cover master parameters, corridor launch overrides, FAST-LIO2 output, serial bridge and firmware README.
- The lock records source YAML, launch rewrite and the still-pending target runtime parameter layer.
- `scripts/audit_vehicle_baseline.py` was run successfully and emitted
  `audit/vehicle_baseline/PARAMETER_LAYERS.json`; the third layer remains
  `PENDING_JETSON` rather than being inferred.
- Both pinned upstream adaptation patches pass `git apply --check`. The raw
  pinned EGO node was built and started under a bounded timeout, the patched
  EGO vehicle edge compiled, and the Super-LIO core compiled in an isolated
  ARM64 ROS 2 Humble container. The logs are retained under
  `audit/container/`; these are build/provenance results, not Jetson runtime
  acceptance.
- The serial contract is tested through the final mock sink: RTK authority false produces exactly `vcx=0,wc=0\\n`.
- Dynamic feasibility, real start state, curvature/yaw-rate consistency, map version, expiry and footprint checks have replay tests.
- Evidence persistence is CRS-bound and idempotent by observation UUID.

## Pending or blocked by environment

- Jetson runtime and target launch/parameter dumps are unavailable in this
  session. The isolated container builds do not provide the target device's
  sensors, serial device, TF tree or runtime parameter layer.
- No verified vehicle IMU→base extrinsic, expanded URDF footprint/wheel geometry, camera intrinsics/extrinsics/depth stream or vehicle bag identity was supplied in this workspace.
- Super-LIO parser output has no covariance/health equivalent at the pinned commit; adapter remains UNKNOWN and fails closed.
- Existing bags were inventoried locally but cannot be replayed without ROS 2 bag tooling.
- A concrete 15.5 s bag was inspected at SQLite topic level: it contains old
  `/fastlio2/lio_odom`, RTK and costmap topics, but no raw `/livox/lidar` or
  `/livox/imu`; it cannot support a Super-LIO replay claim. See
  `audit/VEHICLE_BAG_REPLAY_INPUT.json`.
- Physical e-stop, STM32 behavior, Jetson shadow and live motion remain pending and were not simulated as passes.

## Regression evidence

The new transport-independent and contract suite passes (`20 passed`). The original
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

The LIO field mapping is now machine-readable at
`audit/vehicle_baseline/LIO_FIELD_MAPPING.json`. It records the exact source
agreement for ROS stamps and gyro units, and marks acceleration scaling,
Livox offset-time semantics, synchronization boundaries, height/obstacle cloud
semantics, base extrinsics, covariance and health as unresolved where the
sources disagree. The mapping test refuses an unqualified health equivalence.

The isolated build evidence has two deliberate boundaries. The raw
`Ego-Planner-2D-ROS2@7f5be6d4cee34871e85aa1f15285cfaf17b23877` package builds and
`motion_plan` starts until a five-second timeout. The patched package then builds
with the project interface and feasibility patches. Super-LIO
`f89f48dc7aea6cfa262f18e4d03b319e04e0dbd2` also builds its C++ core and ROS
interfaces; the container supplies only a compile-only `livox_ros_driver2`
message contract because the physical Livox SDK is absent. No sensor runtime,
bag replay, or Jetson shadow result is inferred from these builds.
