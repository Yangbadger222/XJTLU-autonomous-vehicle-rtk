# Existing vehicle data and remaining qualification

The previous blanket claim that MID360 IMU/chassis data were missing was incorrect. This checkpoint uses the approved repository's recorded data, without requesting replacement values or inventing a calibration.

| Item | Source and use | Remaining scope |
|---|---|---|
| MID360 internal IMU | `docs-CN/hardware_spec.md`: approximately200Hz, recorded installation(-.07,+.12,.447)m; original factory LiDAR/IMU translation[-.011,-.02329,.04412] and identity rotation | No additional external IMU is required for this LIO migration |
| Navigation reference | Original FAST publishes the IMU state under the navigation body label; adapter explicitly retains that audited convention. Three full source replays check mean/quaternion/full covariance and stamps | No newly measured chassis-center equivalence is claimed or used as a missing input gate |
| STM command model | Original firmware uses r=.10m,C=.46m,G=19.2,conversion9.55 and four signed motor targets; source RPM ceiling20000 | Source conversion is implemented and tested; actual flashed firmware identity is an on-site check |
| Feedback and bags |41 bags and original feedback Y-forward/gyro-Z semantics audited; command/feedback geometry differences and exploratory response estimates preserved | Physical slip, KEY/emergency stop and braking require actual hardware observation; final PTY zeros are software evidence |
| Execution limits | Original v<=.85,abs(w)<=.7 andabs(v*w)<=.25 unchanged. Moving curvature cap is conservatively derived; pure rotation has an explicit separate mode and all original yaw/permission gates | New jerk/lateral tolerances are labelled algorithm settings, not measurements |
| Real local ground | Native padded PCL input and acquisition-preserving static TF now work; local TF stays available when global motion TF is denied | Actual raw support remains FAIL: finite observed returns do not satisfy the configured dense surface model. No obstacle-free/unknown ray is turned into support |
| Target deployment | Authorized100.88.131.52 Humble/bag machine was used. Recorded Jetson100.79.128.21 SSH timed out again | Target build/effective deployment overlay/shadow and physical acceptance remain separate |

Current executed receipts are under `heading-qualified/`, `road-qualified/` and `road-pcl-fixed/`. Each names its actual source; earlier failures remain visible. Goal remains active, with no production merge, physical drive, firmware flash or source editing on Jetson.
