# Active-road research method (implemented scope)

The research chain keeps three evidence layers: read-only satellite/MaGRoad
prior, timestamped local geometry observations, and reversible road-graph
updates. `EvidenceStore` binds every observation to a CRS/datum transform,
local-submap ID, pose uncertainty, depth range and one UUID. Replaying a UUID
is idempotent. Evidence states are `UNOBSERVED`, `OBSERVED_GEOMETRY`,
`TRAVERSED`, `BLOCKED_EVIDENCE`, and `UNCERTAIN`; observing an entrance never
promotes its unseen continuation to traversed.

`MaGRoadPrior.load_geojson` is a read-only, CRS-checked road-graph input that
preserves a model version. `GeoTiffPrior.load` uses rasterio only when present,
requires a declared CRS, and exposes metadata without turning pixels into free
space. Missing rasterio or CRS is an explicit unavailable/error state.

Candidates are hard-filtered for reachability, safety, pose trust and sensor
validity, then scored as `impact * observable_fraction / (cost + epsilon)`.
This is an interpretable heuristic, not a proven information gain. The
repository contains the policy and evaluator-only synthetic harness; the policy
does not receive simulator truth. Three policy labels are reserved for the
future same-base comparison: `PASSIVE`, `PERIODIC_LOOK`, and `TASK_AWARE_LOOK`.

The vehicle adapter consumes real `/lio/odom_vehicle` state, an odom-frame
road reference and an odom-frame `OccupancyGrid` whose unknown cells are
occupied. The second reproducible EGO patch calls the pinned planner and emits
`TimedTrajectory2D`; `nav_msgs/Path` remains visualization/reference only. It
rejects missing measured state, unknown map versions, unconfigured footprint
inflation/curvature bounds, footprint collisions, speed/acceleration,
discrete body-lateral-speed and curvature/yaw-rate inconsistencies. It never
clips an infeasible trajectory.

The command edge uses `TimedTrajectoryTracker`: it interpolates the trajectory
at the current ROS time, computes longitudinal/lateral/heading errors from
measured `/lio/odom_vehicle`, bounds the resulting forward-only `v,w` request,
and then hands it to the unchanged authority/guard chain. Vehicle health is
read from `/lio/vehicle_health`, which is the source-aware adapter output;
unknown or stale state produces a stop. The ROS edge additionally requires a
matching odom-frame `OccupancyGrid`, map-version message and explicit measured
footprint before enabling the continuous collision oracle; unknown cells remain
occupied.

The local grid producer is conservative: measured odom-frame points inside an
explicit obstacle height window become occupied cells, while points outside
the window, invalid points and absent returns do not create free cells. Unknown
cells remain blocked, and invalid frame, map-version, resolution or
height-window inputs are rejected before a grid can reach the planner.

Active-road geometry now has an explicit pixel/depth contract: declared depth
unit and optical-Z/ray-range convention, measured camera→base transform, and
acquisition-time base→odom transform. Invalid/zero depth is rejected. The
repository has synthetic round-trip tests; camera intrinsics and extrinsics
remain deployment inputs rather than guessed values.
The current repository does not contain a measured wheel/track/footprint dump
or camera calibration, so no live vehicle claim is made.
