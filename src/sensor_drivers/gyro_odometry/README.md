# gyro_odometry

`gyro_odometry` is an experimental serial velocity-feedback package. Its intended output is `geometry_msgs/msg/TwistWithCovarianceStamped`, but the current implementation is not an active production feedback source.

## Current Behavior

- Executable: `gyro_odometry`
- Node output topic: `twist_with_covariance`
- Default serial port in code: `/dev/serial_twistctl`
- Nominal send/read period: `20 ms`

The source currently has `serialopen` set to `0`, so the default path publishes zero-valued `twist_with_covariance` messages instead of reading live serial feedback.

## Build And Run

```bash
cd ~/XJTLU-autonomous-vehicle
colcon build --packages-select gyro_odometry --symlink-install --parallel-workers 1
source install/setup.bash
ros2 run gyro_odometry gyro_odometry
```

## Current Limitations

- Not used as the main odometry feedback source in the active vehicle stack.
- Depends on the repository-local `serial` package.
- Before enabling it for closed-loop use, verify the serial protocol, remove or refactor the hard-coded `serialopen` behavior, and validate covariance values.
