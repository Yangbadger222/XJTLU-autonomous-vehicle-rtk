# Super-LIO vehicle patches

`0001-publish-source-aware-odom-health.patch` applies to the pinned
Super-LIO `ros2` commit. It preserves the upstream estimator and adds the
minimum traceability needed by the vehicle adapter: `child_frame_id=imu`,
angular velocity, the estimator covariance blocks, and an explicit
`/lio/health` value of UNKNOWN because no equivalent FAST-LIO2 degeneracy
metric exists. UNKNOWN is consumed as a motion stop by the adapter.

`0002-certify-source-observations-and-covariance.patch` applies second. It
computes an actual observation-information lower bound sufficient for the
unchanged legacy75 test under locked fixed extrinsics. Every native iteration
must pass; a plane, insufficient matches/IMU or invalid matrices deny health.
Invalid observations do not insert a predicted scan into the map or publish
valid measured odometry. Full pose/twist covariance includes signed cross
blocks and coordinate conversion. The native stamp and certificate identity
are published as structured source health, matched exactly by the adapter.

See `audit/vehicle_ready/SOURCE_HEALTH_CONTRACT.md` for the equation and its
limits. Patch0001's UNKNOWN is historical intermediate behavior; patch0002 is
required by the current build/application recipe. No physical limit, firmware,
serial protocol or factory extrinsic is changed. A conservative certificate is
not physical accuracy or vehicle acceptance.
