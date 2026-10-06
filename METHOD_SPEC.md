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

The vehicle adapter consumes real odometry and a future typed local GridMap, and
emits `TimedTrajectory2D`. It rejects stale map versions, expired trajectories,
unknown cells, footprint collisions, speed/acceleration violations and
curvature/yaw-rate inconsistencies. It never clips an infeasible trajectory.
The current repository does not contain a measured wheel/track/footprint dump
or camera calibration, so no live vehicle claim is made.
