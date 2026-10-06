# Live acceptance checklist (human and hardware required)

All entries below remain `PENDING` until performed on the Jetson with actuator
disabled first and a human at the physical e-stop.

- [ ] Jetson clean worktree builds the exact research commit with
  `colcon build --parallel-workers 1`.
- [ ] Super-LIO parser, time units, IMU units, frame and cloud filters are
  verified against the vehicle bag and runtime parameter dump.
- [ ] Measured IMU→base transform, wheel/track/footprint and camera calibration
  are supplied; adapter health becomes equivalent and covariance is addressed.
- [ ] Replay of a named vehicle bag compares FAST-LIO2 and Super-LIO without
  segment-wise drift alignment.
- [ ] Shadow launch has exactly one map→odom owner, one odom→base owner and no
  Nav2/MPPI/SLAM/FGO/FRC task processes.
- [ ] Mock serial sink receives zero command on authority false, stale authority,
  RTK quality loss, LIO UNKNOWN, TF loss, trajectory expiry and stop override.
- [ ] Physical e-stop, PS2 X/KEY loss and STM32 braking are tested by the human
  operator; mock output is not accepted as physical evidence.
- [ ] Straight, turn, obstacle, narrow passage and unknown-ground tests pass
  footprint sweep, curvature, yaw-rate and stop-distance checks.
- [ ] Only after all above: manual low-speed live enable, with RTK authority
  loss confirming the vehicle stops. No automatic driving is performed here.
