# research_interfaces

`TimedTrajectory2D` is the control contract. `nav_msgs/Path` remains a
visualization-only output and is deliberately absent from these messages.
Every trajectory carries a map version, validity interval, per-sample time,
pose, yaw, speed, yaw-rate, acceleration and curvature. Consumers must reject
expired, unknown-map or infeasible trajectories.

`RoadEvidence2D` is the measured-evidence transport. Its geometry must already
be in `odom`; source, local-submap identity, uncertainty and depth interval are
carried with the observation. `ObservationGoal` candidate messages include
hard reachability, safety, pose-trust and sensor-validity flags so the active
observation node can filter before ranking.

`OperatorPermit` adds human task consent. It never replaces RTK/KEY/stop
authority. Consumers require fresh ordered receipt, one publisher, matching
startup mode/current map and an explicit READY→AUTONOMOUS sequence after loss.
`ManageResearchTask` inspects/selects existing registered-prior endpoints while
consent is paused. Successful selection does not grant motion.
