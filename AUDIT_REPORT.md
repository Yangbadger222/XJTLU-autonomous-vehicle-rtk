# Audit report

## Completed locally

- Baseline branch and commit are recorded in `audit/vehicle_baseline/SOURCE_IDENTITY.json`.
- Source-file hashes cover master parameters, corridor launch overrides, FAST-LIO2 output, serial bridge and firmware README.
- The lock records source YAML, launch rewrite and the still-pending target runtime parameter layer.
- `scripts/audit_vehicle_baseline.py` was run successfully and emitted
  `audit/vehicle_baseline/PARAMETER_LAYERS.json`; the third layer remains
  `PENDING_JETSON` rather than being inferred.
- All three pinned upstream adaptation patches pass `git apply --check`. The raw
  pinned EGO node was built and started under a bounded timeout, and a prior
  vehicle-patch revision plus the Super-LIO core compiled in an isolated ARM64
  ROS 2 Humble container. Patches 0001+0002 have an isolated ARM64 BuildKit compile pass recorded in
  `audit/container/ego-current-build.log`; patch 0003 has an exact sequential
  apply-check in `audit/ego_patch_check_current.log` and needs a fresh rebuild.
  The old patched log remains clearly labeled prior revision. Docker ordinary container startup and Jetson runtime
  acceptance remain pending. Accordingly, `RESULTS.json` keeps the relevant
  runtime gates pending.
- The serial contract is tested through the final mock sink: RTK authority false produces exactly `vcx=0,wc=0\\n`.
- Dynamic feasibility, real start state, curvature/yaw-rate consistency, map version, expiry and footprint checks have replay tests.
- Evidence persistence is CRS-bound and idempotent by observation UUID; schema,
  duplicate-ID, geometry, uncertainty, length and depth-interval validation
  rejects malformed persisted or newly ingested observations.

## Pending or blocked by environment

- Jetson runtime and target launch/parameter dumps are unavailable in this
  session. The isolated container builds do not provide the target device's
  sensors, serial device, TF tree or runtime parameter layer.
- No verified vehicle IMU→base extrinsic, runtime confirmation of the locked corridor footprint/wheel geometry, camera intrinsics/extrinsics/depth stream or vehicle bag identity was supplied in this workspace. The corridor source polygon is now recorded and transported as an exact carry-over, but it is not treated as a target runtime dump.
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

The new transport-independent and contract suite passes (`59 passed`). The replay
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

The Super-LIO output boundary is frame-audited: `/lio/odom` and
`/lio/cloud_world` remain in the pinned source `world` frame, while the local
grid consumes only `/lio/cloud_odom` produced by a timestamped TF lookup. The
cloud adapter never assigns a target `frame_id`; the odometry adapter rejects a
`world`→`odom` mismatch until a real source-frame TF path is implemented and
verified. Therefore no source message is presented as vehicle odometry or a
body cloud by string relabeling.

The ROS safety bridge now parses `enable_serial` by value, rejects `UNKNOWN`
map versions, and expires both the persisted map-version heartbeat and local
obstacle grid after 0.50 s. It uses the locked corridor speed/acceleration/yaw
limits and consumes the exact corridor source footprint polygon. Curvature/lateral
limits remain disabled until the target runtime confirms the vehicle geometry. The map publisher runs at 0.20 s to remain inside that
freshness window. New contract tests cover these boundaries.

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
fail-closed zero values until their semantics are verified, treats unknown occupancy as blocked,
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

The active-road research loop now has a typed `RoadEvidence2D` ROS ingest
message and `active_road_evidence` node. The node requires a valid persisted
map identity, accepts only finite odom-frame geometry, saves accepted UUIDs by
atomic replace, emits observed-geometry events, and filters externally supplied
observation candidates by reachability/safety/pose/sensor flags. The node does
not infer camera parameters, TF, truth labels or traversability.

The timed-trajectory safety edge now consumes `/lio/odom_vehicle` and the
adapter's `/lio/vehicle_health`, interpolates the trajectory in time, applies
measured-pose longitudinal/lateral/heading feedback, and only then passes the
request to the original authority/command guard. It no longer sends the first
trajectory sample directly to `/cmd_vel`; expired, invalid or stale state still
reaches the mock sink as a stop, and a health message older than the explicit
0.50 s timeout is treated as UNKNOWN. The bridge also requires a matching
odom-frame `/research/local_obstacle_grid`, `/research/map_version`, and
the locked source footprint polygon before enabling its collision oracle; absent or
stale map/footprint data remains a stop, and target runtime confirmation is still required.

The deterministic replay harness now drives its allowed command through the
same tracker using a synthetic measured pose before injecting RTK authority
loss; the resulting evidence contains both the bounded tracked command and
the final zero serial bytes.

The EGO ROS trajectory wrapper now derives yaw-rate as
`cross(v,a)/|v|^2` and curvature as `yaw-rate/|v|`; the source-level contract
test rejects the previous dimensionally incorrect formula and the patch hash in
`audit/UPSTREAM_PATCH_VERIFICATION.json` was refreshed.

The third sequential EGO patch clears the planner's previous timed-result and
A* result before every replan. A failed optimizer or missing measured state
therefore cannot be followed by republishing an older feasible-looking plan;
the ROS edge receives an empty result and publishes a failure/stop contract.
The exact three-patch apply check passes in `audit/ego_patch_check_current.log`.
The existing ARM64 BuildKit compile predates this third patch, so the current
three-patch ROS rebuild remains pending.

After that formula change, a fresh sparse checkout at the exact upstream
commit applied patches 0001, 0002 and 0003; the command and current hashes are
recorded in `audit/ego_patch_check_current.log`.

The same post-check now rejects a scalar speed that disagrees with the
trajectory tangent magnitude, so the ROS timed samples cannot carry a
world-vector/scalar-speed mismatch into the tracker.

## Follow-up implementation evidence

The pinned EGO source now has sequential project patches that replaces its
interactive demo edge with a vehicle ROS edge. A fresh sparse checkout at the exact commit accepted all three patches with
`git apply --check`. The edge refuses output
when map resolution, inflation/footprint radius, curvature, lateral-speed
limits, map version, measured odom state, or local obstacle grid are absent.
It emits `TimedTrajectory2D`; the visual `nav_msgs/Path` is not consumed by the
safety bridge. The active launch condition test also verifies that replay has no
planner/authority/serial processes and that the physical serial sink requires
both `live` and `enable_serial:=true`.

The new EGO ROS edge is intentionally fail-closed when the target local
obstacle-grid and road-reference producers are absent. The active-road launch now
includes `research_local_obstacle_grid` and `active_road_map`: the former only
marks measured odom-frame cells, while the latter publishes a persisted map
version and relays only an external odom-frame reference. Their ROS contracts
are locked in `audit/vehicle_baseline/RUNTIME_CONTRACT.json`; target-side
frame/ground-separation, evidence ingestion and runtime wiring remain pending
and are not represented as a software PASS.

The local-grid projector in `src/research_runtime/research_runtime/grid_map.py`
and ROS transport node `local_obstacle_grid_node.py` project only measured
odom-frame points inside the explicit obstacle height window. The node refuses
non-odom clouds and UNKNOWN map versions, preserves unknown cells as blocked,
and does not infer free space from absent returns; the boundary, unknown-space
and launch-wiring tests are included in the suite.

The Super-LIO vehicle adapter now also rejects non-unit source/extrinsic
quaternions, non-finite covariance and non-identity IMU-to-base rotations while
covariance rotation is unimplemented. This prevents a future verified flag
from turning copied IMU-frame covariance into a false base-frame contract.

The `research_runtime` and `active_road_mapping` Python packages were built as
wheels and inspected to contain the GridMap projector, safety bridge,
trajectory validator, `research_local_obstacle_grid` and `active_road_map`
and `active_road_evidence` entrypoints. This checks the installed-package path
separately from the source-tree test run. The added ROS interface message still
needs a current ROS/ARM64 message-package build; the last Docker retry stalled
before container startup.

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
`motion_plan` starts until a five-second timeout. The patched package with patches 0001+0002 builds with the project interface
and feasibility changes at the recorded ARM64 revision; patch 0003 has exact
sequential apply-check, while the current three-patch ARM64 rebuild remains
pending. Super-LIO
`f89f48dc7aea6cfa262f18e4d03b319e04e0dbd2` also builds its C++ core and ROS
interfaces; the container supplies only a compile-only `livox_ros_driver2`
message contract because the physical Livox SDK is absent. No sensor runtime,
bag replay, or Jetson shadow result is inferred from these builds.
