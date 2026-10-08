# Current migration checkpoint

Current qualified runtime is a7dc39774c4421b40a6a0abb7d1709a81bdc72b0, descended from the sole corridor-authority-stability@e54c6afbcb5a58db22d7c468085a87d658b0b932 baseline. Super-LIO is the fixed ros2 revision with2 project patches. EGO is the fixed already-2D JackJu-HIT develop revision with11 vehicle-adaptation patches; upstream unmodified reproduction and all-source byte proof remain source-bound to their recorded cohorts.

The default research entry retains sensors, source-aware LIO adapters, original RTK authority, local ground/obstacle evidence, typed EGO tracking, original command guard, persistence, active observation, logs and operator console. Physical serial and mission execution are disabled by default. FAST-LIO2, Nav2/MPPI, SLAM Toolbox, FGO/FRC and the old mission bypasses are excluded from this entry; their original assets are retained.

The original IMU-origin /lio/odom_vehicle and RTK TF convention remain intact. The separate acquisition-matched /research/odom_control uses chassis_control_origin, the recorded MID360 mount/factory transform and full lever-arm twist/covariance. STM and41bag records are used. No firmware, tested calibration, serial protocol, motion ceiling or RTK stop permission changed.

Console/cloud exit now keeps ROS alive through owned cleanup. Console close atomically makes its permission terminal so authenticated late reset/start and duplicate commands cannot reactivate it. Native and diagnostic ARM16 node entry/cleanup,13 HMI final-PTY cases,157 three-layer checks,30 loaded source hashes per architecture and223portable checks pass. Existing14 package foundations and the current Python overlays are recorded separately.

Actual replay still gives0 qualified vehicle-width ground strips. Each policy currently reaches0/2 goals; research benefit is unproved. Target shadow, current deployment/firmware identity and physical stopping remain PENDING. Prior reports and failed receipts remain archived, and the old user preview stays at its own ce46418 source.
