# research_interfaces

`TimedTrajectory2D` is the control contract. `nav_msgs/Path` remains a
visualization-only output and is deliberately absent from these messages.
Every trajectory carries a map version, validity interval, per-sample time,
pose, yaw, speed, yaw-rate, acceleration and curvature. Consumers must reject
expired, unknown-map or infeasible trajectories.
