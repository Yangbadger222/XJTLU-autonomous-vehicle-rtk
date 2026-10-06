# Super-LIO vehicle output patch

`0001-publish-source-aware-odom-health.patch` applies to the pinned
Super-LIO `ros2` commit. It preserves the upstream estimator and adds the
minimum traceability needed by the vehicle adapter: `child_frame_id=imu`,
angular velocity, the estimator covariance blocks, and an explicit
`/lio/health` value of UNKNOWN because no equivalent FAST-LIO2 degeneracy
metric exists. UNKNOWN is consumed as a motion stop by the adapter.
