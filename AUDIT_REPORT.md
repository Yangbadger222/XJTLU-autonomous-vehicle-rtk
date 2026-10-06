# Audit report

## Completed locally

- Baseline branch and commit are recorded in `audit/vehicle_baseline/SOURCE_IDENTITY.json`.
- Source-file hashes cover master parameters, corridor launch overrides, FAST-LIO2 output, serial bridge and firmware README.
- The lock records source YAML, launch rewrite and the still-pending target runtime parameter layer.
- `scripts/audit_vehicle_baseline.py` was run successfully and emitted
  `audit/vehicle_baseline/PARAMETER_LAYERS.json`; the third layer remains
  `PENDING_JETSON` rather than being inferred.
- Both pinned upstream adaptation patches pass `git apply --check`. The raw
  pinned EGO node was built and started under a bounded timeout, and a prior
  vehicle-patch revision plus the Super-LIO core compiled in an isolated ARM64
  ROS 2 Humble container. The current corrected EGO patch has a fresh exact
  apply-check, but its rebuild is pending because the local Docker runtime
  remains stuck in `Created`; the old patched log is not reused as current
  compile evidence. These are build/provenance results, not Jetson runtime
  acceptance. Accordingly, `RESULTS.json` keeps the relevant runtime and
  current patched-build gates pending.
- The serial contract is tested through the final mock sink: RTK authority false produces exactly `vcx=0,wc=0\\n`.
- Dynamic feasibility, real start state, curvature/yaw-rate consistency, map version, expiry and footprint checks have replay tests.
- Evidence persistence is CRS-bound and idempotent by observation UUID; schema,
  duplicate-ID, geometry, uncertainty, length and depth-interval validation
  rejects malformed persisted or newly ingested observations.

## Pending or blocked by environment

- Jetson runtime and target launch/parameter dumps are unavailable in this
  session. The isolated container builds do not provide the target device's
  sensors, serial device, TF tree or runtime parameter layer.
- No verified vehicle IMU→base extrinsic, expanded URDF footprint/wheel geometry, camera intrinsics/extrinsics/depth stream or vehicle bag identity was supplied in this workspace.
- Super-LIO parser output has no covariance/health equivalent at the pinned commit; adapter remains UNKNOWN and fails closed.
- A local 135.1 s bag was reindexed, inspected and replayed in the isolated
  ARM64 ROS 2 Humble container. The replay delivered 1347 legacy
  `/fastlio2/lio_odom`, 481 local costmap, 130 global costmap and 2118
  `/cmd_vel` messages. It contains no raw `/livox/lidar` or `/livox/imu`, so
  it proves only legacy topic replay and cannot support a Super-LIO replay or
  interface-equivalence claim. A second inspected candidate remains a
  macOS `compressed,dataless` placeholder with zero allocated blocks. See
  `audit/VEHICLE_BAG_REPLAY_INPUT.json` and
  `audit/container/vehicle-bag-replay-20260326.log`.
- Physical e-stop, STM32 behavior, Jetson shadow and live motion remain pending and were not simulated as passes.

## Regression evidence

The new transport-independent and contract suite passes (`47 passed`). The replay
entry point was also executed through `research_safety_bridge --mode replay`; it
accepted the mode flag, wrote the deterministic replay JSON, and retained the
final mock stop bytes. The original
`gps_waypoint_dispatcher` guard/authority tests run with the package path and
recorded `191 passed, 3 failed`; the failures are baseline/environment evidence
(two diagnostic-string expectations already absent at the pinned commit and one
test importing unavailable `rclpy`). The existing corridor launch suite records
`26 passed, 1 failed` because the pinned baseline lacks the expected
`enable_local_odom_bridge: false` text. No protected file was modified to make
these tests pass.

The final mock sink fault matrix now includes RTK authority false/stale,
unknown LIO health, invalid TF, invalid map, invalid/expired trajectory,
stop-override, manual stop and non-finite command inputs; each asserts the
exact `vcx=0,wc=0\\n` output.

The shared trajectory validator also has explicit rejection cases for map and
frame mismatch, expiry, time rollback, pose jumps, and curvature/yaw-rate
inconsistency; these are separate from the upstream planner compile evidence.
Footprint and centerline collision checks now sample every segment at the
caller-supplied map resolution; a footprint oracle without a positive sweep
resolution is rejected instead of being treated as a point-only proof.

The launch-contract suite also asserts that the vehicle EGO configuration
contains the locked corridor limits, keeps curvature/inflation at explicit
fail-closed zero values until measured, treats unknown occupancy as blocked,
and does not activate the upstream demo defaults for speed, acceleration,
jerk, resolution or inflation.

Active-observation candidates now hard-filter negative/non-finite cost,
impact and observability inputs before scoring; the test suite covers those
invalid heuristic inputs separately from reachability and safety filters. The
evidence store also rejects unsupported schemas and malformed measurements
before they can be persisted or replayed. MaGRoad GeoJSON now requires an
explicit CRS and finite geometry; GeoTransform rejects missing or non-finite
metadata instead of inferring a coordinate convention.

The active-road bringup manifest now declares every research runtime and safety
edge it launches, including the pinned estimator/planner package names,
research interfaces, Livox/RTK inputs, authority guard and serial bridge; a
static contract test prevents future dependency omissions.

The timed-trajectory safety edge now consumes `/lio/odom_vehicle` and the
adapter's `/lio/vehicle_health`, interpolates the trajectory in time, applies
measured-pose longitudinal/lateral/heading feedback, and only then passes the
request to the original authority/command guard. It no longer sends the first
trajectory sample directly to `/cmd_vel`; expired, invalid or stale state still
reaches the mock sink as a stop, and a health message older than the explicit
0.50 s timeout is treated as UNKNOWN. The bridge also requires a matching
odom-frame `/research/local_obstacle_grid`, `/research/map_version`, and
explicit measured footprint before enabling its collision oracle; absent or
stale map/footprint data remains a stop.

The deterministic replay harness now drives its allowed command through the
same tracker using a synthetic measured pose before injecting RTK authority
loss; the resulting evidence contains both the bounded tracked command and
the final zero serial bytes.

The EGO ROS trajectory wrapper now derives yaw-rate as
`cross(v,a)/|v|^2` and curvature as `yaw-rate/|v|`; the source-level contract
test rejects the previous dimensionally incorrect formula and the patch hash in
`audit/UPSTREAM_PATCH_VERIFICATION.json` was refreshed.

After that formula change, a fresh temporary checkout at the exact upstream
commit reapplied patch 0001 and passed `git apply --check` for patch 0002;
the command and current hashes are recorded in `audit/ego_patch_check_latest.log`.

The same post-check now rejects a scalar speed that disagrees with the
trajectory tangent magnitude, so the ROS timed samples cannot carry a
world-vector/scalar-speed mismatch into the tracker.

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

The transport-independent local-grid adapter in
`src/research_runtime/research_runtime/grid_map.py` now projects only measured
odom-frame points inside the explicit obstacle height window. It preserves
unknown cells as blocked, rejects invalid frame/map/resolution contracts, and
does not infer free space from absent returns; its boundary and unknown-space
tests are included in the suite.

The Super-LIO vehicle adapter now also rejects non-unit source/extrinsic
quaternions, non-finite covariance and non-identity IMU-to-base rotations while
covariance rotation is unimplemented. This prevents a future verified flag
from turning copied IMU-frame covariance into a false base-frame contract.

The `research_runtime` Python package was also built as a wheel and inspected
to contain the GridMap projector, safety bridge and trajectory validator. This
checks the installed-package path separately from the source-tree test run.

The LIO field mapping is now machine-readable at
`audit/vehicle_baseline/LIO_FIELD_MAPPING.json`. It records parser/source
provenance for ROS stamps, IMU copies and Livox point offsets. The vehicle
driver source explicitly assigns ROS 2 LiDAR and IMU headers from
`cur_node_->now()` while the pinned Super-LIO parser consumes those headers
directly; this makes the ROS encoding a carry-over but leaves the measurement
clock and deskew relationship target-runtime pending. It also records that
both parser paths copy IMU acceleration without scaling while the locked
FAST-LIO2 callback applies an unexplained factor of `10.0`. Acceleration units,
Livox hardware timebase, synchronization boundaries, height/obstacle cloud
semantics, base extrinsics, covariance and health remain unresolved where the
sources do not prove equivalence. The mapping test refuses an unqualified
health equivalence and checks the new source provenance fields.

The isolated build evidence has two deliberate boundaries. The raw
`Ego-Planner-2D-ROS2@7f5be6d4cee34871e85aa1f15285cfaf17b23877` package builds and
`motion_plan` starts until a five-second timeout. The patched package then builds
with the project interface and feasibility patches at the prior recorded
revision; the current corrected patch only has exact sequential apply-check
evidence because the Docker runtime cannot start a new ARM64 build. Super-LIO
`f89f48dc7aea6cfa262f18e4d03b319e04e0dbd2` also builds its C++ core and ROS
interfaces; the container supplies only a compile-only `livox_ros_driver2`
message contract because the physical Livox SDK is absent. No sensor runtime,
bag replay, or Jetson shadow result is inferred from these builds.
