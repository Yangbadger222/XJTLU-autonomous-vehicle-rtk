# Ego-Planner-2D vehicle adaptation patch

`0001-vehicle-state-and-feasibility.patch` applies to the exact
`JackJu-HIT/Ego-Planner-2D-ROS2` `develop` commit pinned in
`dependencies.research.repos`. It keeps the upstream planner and GridMap
implementation, while making the vehicle boundary explicit:

- measured start velocity/acceleration are required instead of unit yaw vectors;
- optimizer feasibility limits receive the configured vehicle values;
- the disabled B-spline time repair path runs and a final check rejects failures;
- discrete UGV yaw-rate/curvature checks run after optimization;
- sampled trajectory results retain velocity and acceleration components.

This is a project patch named “Ego-Planner-2D-ROS2 vehicle adaptation”; it does
not claim the upstream authors provide this vehicle integration.

`0002-vehicle-ros-timed-trajectory.patch` is applied second. It replaces the
interactive demo ROS edge with a vehicle adapter that consumes odom-frame
measured state, an odom-frame road reference and an OccupancyGrid whose unknown
cells are occupied. It calls the patched upstream `PlannerInterface`, publishes
`research_interfaces/TimedTrajectory2D` for control, and retains
`nav_msgs/Path` only for visualization. It refuses output until a measured
footprint-derived inflation radius, curvature bound, lateral-speed bound and
map version are configured.

`0003-clear-stale-plan-on-failure.patch` is applied third. It clears the
planner's previous timed-result and A* buffers before each replan, so an
optimizer failure or missing measured state cannot republish an older plan.
The current ARM64 compile log predates this safety patch; its exact sequential
apply-check is recorded separately.
