# Recorded MID360 chassis control reference

This adaptation uses e54c6af hardware_spec, factory LiDAR/IMU translation and original URDF attitude. MID360 already contains the IMU. No extra IMU or newly guessed transform is introduced.

The documented LiDAR position in the chassis XY origin at nominal ground height is (-.07,.12,.447)m. Original zero mounting RPY and t_IL=(-.011,-.02329,.04412)m give the chassis origin in IMU coordinates r_IB=(.059,-.14329,-.40288)m. That point is not URDF base_link, whose nominal z is .229m.

With R_WI unchanged: p_WB=p_WI+R_WI r_IB; v_B=v_I+omega_I cross r_IB. The adapter propagates both full six-by-six pose and twist covariances including cross terms. This prevents a fixed chassis yaw pivot from being treated as translating merely because the off-centre internal IMU moves.

The legacy /lio/odom_vehicle and odom->base_footprint retain the original IMU-origin convention for original RTK consumers. /research/odom_control has child chassis_control_origin and explicit contract corridor_e54c6af_mid360_ground_control_origin_v1. EGO, tracker, observer and console consume the explicit reference; timed messages carry the contract. There is no second map->odom owner.

Stationary heading recovery is admitted after the original .05m/s,2deg/s,1s continuous rate-tolerance test. The full3D speed norm and acquisition continuity matter. This is not active-brake certification. The recovery preserves measured yaw rate and acquisition-derived yaw acceleration, proves continuous derivative bounds and commands zero translation. Every authority/health/TF/stop/speed/map/trajectory/consent denial revokes admission immediately. RTK loss stops regardless of LIO health.

Tests in AUDIT_REPORT.md use the original serial binary and allocated PTYs. The source-derived transform is an existing nominal configured model, not newly surveyed hardware accuracy. Deployment identity, real terrain, KEY/PS2/physical brake and estop remain field acceptance.
