# research_interfaces

TimedTrajectory2D is the control contract; nav_msgs/Path is visualization only. Every trajectory binds the odom frame, control_reference_contract, map version, trajectory ID, generation time, validity and per-sample t/x/y/yaw/v/w/a/alpha/curvature. Consumers reject stale, wrong-reference or infeasible payloads. ROTATE_STATIONARY is an explicit mode with zero commanded translation and undefined curvature; it needs independent original-rate stop confirmation before admission. Measured yaw derivatives remain boundary conditions.

The recorded nominal chassis reference is /research/odom_control, child chassis_control_origin, contract corridor_e54c6af_mid360_ground_control_origin_v1. The original /lio/odom_vehicle retains its legacy IMU-origin convention for RTK compatibility. See docs-EN/research_control_reference.md.

LocalEvidenceGrid2D binds acquisition, map, session and support model atomically. RoadEvidence2D geometry is already in odom and carries source/submap/uncertainty/depth-range/observed-length; it never means an unseen edge was traversed. ObservationGoal flags keep reachability, safety, pose trust and sensor validity as hard filters.

OperatorPermit adds human task consent while retaining RTK/KEY/stop gates. Consumers require fresh ordered receipt, one publisher, matching startup mode/map and an explicit READY-to-AUTONOMOUS sequence. Reset and task start both require renewed original continuous stop confirmation. ManageResearchTask selects registered endpoints while paused; selection never grants motion.
