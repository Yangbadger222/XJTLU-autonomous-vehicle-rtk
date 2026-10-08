# Existing vehicle data and current blockers

The previous blanket claim that MID360/IMU-to-body or STM records were missing was incorrect. The approved baseline's notes, URDF, factory transform, firmware and bags are used.

| Item | Existing source and implementation | Remaining qualification |
|---|---|---|
| MID360 | Internal IMU; measured mount (-.07,.12,.447)m; factory t_IL[-.011,-.02329,.04412], identity R_IL; original configured zero mounting RPY | No additional external IMU or guessed mounting calibration |
| Control origin | B is the recorded chassis XY origin at nominal ground level, not URDF base_link. r_IB=(.059,-.14329,-.40288). p_WB=p_WI+R_WI r_IB; v_B=v_I+omega_I cross r_IB; full covariance Jacobians | Existing nominal configuration is not new surveyed metrology; confirm deployed identity read-only |
| Legacy navigation | Original /lio/odom_vehicle and odom->base_footprint retain the IMU-origin convention; original RTK keeps consuming it | No RTK owner or authority rewrite |
| STM command | r=.10,C=.46,G=19.2,factor9.55; targets[L,L,-R,-R], source RPM cap20000 | Source checks implemented; flashed identity is on-site |
| STM feedback | r=.10,C=.50,G=19; actual serial forward axis is Y, angular field is gyro Z. Computed wheel real_w is not serialized | No wheel-ratio correction of gyro; no per-wheel slip inference without per-wheel feedback |
| Recorded response | All41 eligible cmd/LIO bags; source acquisition time for derivatives, recording clock for command association; 68732 intervals,1186 zero-command candidates,15 >=1s quiet windows retained | Exploratory response fit is not current braking latency or labelled physical estop certification |
| Stop semantics | Original .05m/s,2deg/s,1s continuous confirmation, full3D velocity norm and original .2s state continuity. Actual raw noise can pass the original definition | Does not mean physical zero velocity or active brake |
| Real ground | Exact-acquisition pose/certificate/cloud and local-only persistence now execute; July21 prefixes qualify localization but not dense surface strips | Actual measured density/model mismatch remains FAIL; .4s diagnostic union gives no vehicle-width strips |
| Target | Non-Jetson Humble/bag host used, recorded Jetson SSH timed out | Actual overlays, actuator-disabled shadow and physical acceptance pending |

Evidence is source-bound under control-reference-qualified/ and control-stop-qualified/. Earlier cohorts and failures retain their identities. Do not ask again for the already recorded nominal mount or STM equations. For an optional real camera path, locate the actual K, depth convention, registered stream and acquisition-time body transform before declaring a specific missing record.

The goal remains active. No production source edits, firmware flash, physical drive or force push.
