# Source-health and navigation-reference contract

The pinned Super-LIO core remains f89f48dc7aea6cfa262f18e4d03b319e04e0dbd2. Patch0002 follows the unchanged output patch0001; no protected FAST/firmware/serial/vehicle configuration is edited.

## Conservatively sufficient legacy information bound

In original `lidar_processor.cpp`, with locked `esti_il=false`, the twelve-column measurement Jacobian is `[Jpose,0]`, where `Jpose=[-n_W^T R_WI [p_I]x,n_W^T]` and inverse residual covariance is1000. The Super-LIO matched/deskewed IMU point observation has exactly its transpose, `[p_I cross R_WI^T n_W;n_W]`, and the same1000 weight. Correspondences remain those actually selected by Super-LIO; no old FAST output enters this estimator.

Original `imu_processor.cpp:40–42` initializes the fixed six-dimensional extrinsic covariance to1e-5 I. The original prediction has identity dynamics/no noise in those fixed states and no measurement cross terms when `esti_il=false`; these states remain independent. Hence the legacy twelve-block system is `diag(Hobs + Apose_prior,100000 I)`, with a positive semidefinite pose-prior information block. Its minimum eigenvalue is at least `min(lambda_min(Hobs),100000)`. Testing that lower bound against the unchanged75 threshold is sufficient, and stricter than using a prior to make an unobservable observation pass. It is not the raw legacy eigenvalue or an empirical threshold mapping.

The certificate records the worst bound over all native filter iterations, the actual effective-point count and exact output measurement stamp. Insufficient matched points (<original50), fewer than the original3 synchronized IMU samples, malformed/nonfinite matrices or nonpositive filter prior deny the certificate. A single plane is a required negative case. Invalid observations neither update the map nor publish predicted odometry as a valid measurement. The residual/filter method, calibration, noise configuration and physical caps remain unchanged.

Pose covariance uses world fixed-axis orientation errors, with all position/orientation cross blocks. Twist covariance retains the native world-linear/IMU-angular convention and uses `omega=measured gyro-bias` under the native conditional white-noise model, including bias covariance, configured sample variance and signed velocity/bias cross blocks. This is an estimator covariance contract, not independently validated real-world accuracy.

## Original navigation reference and local gauge

Original FAST `publishOdometry` publishes the IMU state directly as `base_footprint`; July notes explicitly record this convention. The new `locked_fast_imu_origin` profile preserves that navigation reference with an audited identity, separately from the measured physical MID360 mount (-.07,+.12,.447m). It never claims IMU/chassis-center coincidence. Physical LiDAR/IMU calibration remains the locked factory transform.

For a fresh localization session, odom is defined as the new estimator's continuous local-world gauge. The adapter owns the explicit static `odom <- world` identity and publishes only `odom -> base_footprint`; RTK still owns `map -> odom`. No global identity or physical mounting is inferred from this local gauge. Cloud transforms still use acquisition-stamped TF. Invalid/replaced reference IDs or nonidentity overrides are rejected, clock regression and original .50m/15deg planar-step guards latch until a fresh session.

A native source certificate is paired to odometry by exact acquisition stamp even when the two topics arrive in reverse order. Headerless 'OK' cannot qualify this profile. Coordinate-valid odom remains observable for shadow when health fails, but vehicle health stays UNKNOWN and the existing final bridge denies motion. RTK loss remains an independent mandatory stop.

This source contract is subject to actual C++ equation/covariance checks, actual Humble build, native bag replay and final mock serial fault checks. Those execution results must be inspected before assigning PASS; current work does not automatically enable actuators or prove physical acceptance.
