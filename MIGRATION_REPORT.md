# Current migration checkpoint

Current qualified native runtime is 77d5c5a5ce26b52b22d5631d67ba88a71ad410b3, descended only from corridor-authority-stability@e54c6afbcb5a58db22d7c468085a87d658b0b932. Super-LIO stays at ros2@f89f48dc7aea6cfa262f18e4d03b319e04e0dbd2 with 2 project patches; EGO stays at the already-2D develop@7f5be6d4cee34871e85aa1f15285cfaf17b23877 with 11 patches. Untouched EGO reproduction and historical checks retain their source identities.

The default entry starts real Super-LIO in replay, shadow and live modes when enable_super_lio=true. Replay disables sensor drivers, physical serial and mission execution. Original RTK authority, control guards, measured control odometry, local ground/obstacle evidence, typed TimedTrajectory2D tracking, persistence, logs and console remain. FAST-LIO2, Nav2/MPPI, SLAM Toolbox, FGO/FRC and old task bypasses are excluded; old assets are retained.

Recorded MID360 internal-IMU/mount/factory geometry and STM equations plus 41 bags are used. Legacy IMU-origin odometry and RTK conventions remain; the separate control reference transports the full lever-arm mean/covariance. Firmware, calibration, serial protocol, tested motion limits and authority permissions are unchanged.

This cohort closes actual geometry and exit defects: normalized shared output rotation; rejected rotations withdraw odometry/TF/world cloud; valid SO(3) composition preserves noncommuting left/right order and prevents float drift; invalid inputs remain invalid. Seven research entry points keep ROS alive until pending callbacks and node cleanup finish. Ordinary RuntimeError still propagates.

Native compilation, six source-bound native geometry checks, default16-node exit, 157 three-layer checks, 244 portable cases and source-matching installed Python files qualify software. Architecture-specific staged builds and ARM diagnostic outcomes are recorded in AUDIT_REPORT.md; this is not a fresh all14 build at current tip or native Jetson acceptance.

The corrected geometry at 49b44ee passes the full July15 EOF replay with 2850 native/vehicle/control/cloud outputs. Its actual July21AM30s ground prefix still has zero positive support. Six same-foundation60s policy tasks ran at49b44ee: five protocol PASS, first PASSIVE protocol FAIL with no actuation, all0/6 goals. No policy benefit is established. Source binding prevents relabelling these measurements as a77d5 rerun. Target/physical acceptance and Mac synchronization remain PENDING.
