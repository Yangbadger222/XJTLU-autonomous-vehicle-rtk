# Physical acceptance, all PENDING

Software/PTY checks and original-bag estimator replay have been executed on the authorized non-Jetson Humble host. They do not satisfy the physical items below.

- [ ] Build the exact research source in a separate Jetson worktree with one worker; preserve the production branch/install/runtime-data.
- [ ] Read current_scene, CLI/environment/generated launch overlays and actual deployed parameter dumps. Reconcile differences with the locked corridor profile before deployment.
- [ ] Verify flashed firmware identity and resolve command-track .46 m versus feedback/URDF .50 m, gear 19.2 versus 19.0, and source radius .10 m versus documented .085 m diameter. Preserve original values until a measured/approved calibration exists.
- [ ] Supply measured IMU/base and GNSS/dual-antenna lever arms, source/driver clocks, validated pose/twist covariance and a mathematically justified Super-LIO health gate. UNKNOWN continues to block motion.
- [ ] Supply a calibrated positive ground/depth provider, camera model/K/distortion/aligned-depth units/extrinsics and acquisition TF if used; verify holes, steps, slopes, 3D obstacle heights and sensor blind spots.
- [ ] Supply real GeoTIFF/MaGRoad files, hashes, CRS/datum and surveyed registration control points. Verify separate-session map reuse and independent historical-submap registration.
- [ ] Run actuator-disabled shadow; verify unique map→odom, odom→base and final cmd owners and absence of old navigation/FAST/SLAM/FGO/FRC/fake simulator.
- [ ] Human-test KEY, PS2 X, controller loss and the physical e-stop at the final MCU/motor output. PS2 B active braking does not replace X or physical e-stop.
- [ ] Measure low-speed straight/curved tracking, skid/slip, wheel speed/current, actual stopping distance and angular deceleration. Software command caps/model braking are not minimum physical braking guarantees.
- [ ] With an operator at the physical e-stop, verify RTK authority loss stops the vehicle despite healthy LIO. RTK holdover motion remains unapproved and disabled.
- [ ] Only after these gates and explicit on-site permission: manually enable a bounded low-speed live test. This delivery does not automatically drive or flash firmware.
