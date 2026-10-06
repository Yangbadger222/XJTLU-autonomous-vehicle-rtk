# Pinned upstream source audit

Date of audit: 2026-10-07. This is a read-only audit of the exact source trees used for this research branch. No upstream or vehicle source file was changed while preparing this report. Claims below are tied to source paths and line ranges at the pinned commits; README text is used only for declared demo contracts, not as a substitute for implementation behavior.

## Commit identities and primary sources

| tree | pinned ref | source link |
|---|---|---|
| vehicle baseline | `Yangbadger222/XJTLU-autonomous-vehicle-rtk`, `corridor-authority-stability`, `e54c6afbcb5a58db22d7c468085a87d658b0b932` | [vehicle commit](https://github.com/Yangbadger222/XJTLU-autonomous-vehicle-rtk/tree/e54c6afbcb5a58db22d7c468085a87d658b0b932) |
| Super-LIO | `Liansheng-Wang/Super-LIO`, `ros2`, `f89f48dc7aea6cfa262f18e4d03b319e04e0dbd2` | [Super-LIO commit](https://github.com/Liansheng-Wang/Super-LIO/tree/f89f48dc7aea6cfa262f18e4d03b319e04e0dbd2) |
| EGO 2D | `JackJu-HIT/Ego-Planner-2D-ROS2`, `develop`, `7f5be6d4cee34871e85aa1f15285cfaf17b23877` | [EGO commit](https://github.com/JackJu-HIT/Ego-Planner-2D-ROS2/tree/7f5be6d4cee34871e85aa1f15285cfaf17b23877) |

The local checkouts resolve to these exact object IDs (`git rev-parse HEAD`). The vehicle comparison below is against `src/perception/fastlio2` and the same branch's Livox driver; a similarly named external FAST-LIO implementation was not substituted.

## Super-LIO sensor parser and time/units audit

### ROS inputs and QoS

`ROSWrapper::setupIO()` creates best-effort, volatile subscriptions: IMU depth 500 and LiDAR depth 20. The topics are parameters, with the Livox path selected when `lio.sensor.lidar_type == LIVOX` ([ROSWrapper.cpp:287-323](https://github.com/Liansheng-Wang/Super-LIO/blob/f89f48dc7aea6cfa262f18e4d03b319e04e0dbd2/src/super_lio/src/ros/ROSWrapper.cpp#L287-L323)). The pinned example resolves them to `/livox/lidar` and `/livox/imu`, type `1`, blind `2.0`, max range `60.0`, filter rate `3`, and LiDAR-to-IMU identity rotation with translation `[-0.011,-0.02329,0.04412]` ([livox_360.yaml:10-29](https://github.com/Liansheng-Wang/Super-LIO/blob/f89f48dc7aea6cfa262f18e4d03b319e04e0dbd2/src/super_lio/config/livox_360.yaml#L10-L29)). Those are upstream example values, not vehicle values.

### IMU fields and units

`imuHandler()` converts the ROS stamp as

```text
t_imu = header.stamp.sec + 1e-9 * header.stamp.nanosec  [s]
acc_S = (linear_acceleration.x, y, z)
gyro_S = (angular_velocity.x, y, z)
```

It copies both vectors without a scale, axis swap, gravity removal, or frame transform, and clears the buffer when timestamps go backward ([ROSWrapper.cpp:347-367](https://github.com/Liansheng-Wang/Super-LIO/blob/f89f48dc7aea6cfa262f18e4d03b319e04e0dbd2/src/super_lio/src/ros/ROSWrapper.cpp#L347-L367)). `ESKF::Predict()` then subtracts estimated biases, integrates the acceleration with `R * (acc-ba) + g`, and treats gyro as radians per second ([ESKF.cpp:148-174](https://github.com/Liansheng-Wang/Super-LIO/blob/f89f48dc7aea6cfa262f18e4d03b319e04e0dbd2/src/super_lio/src/lio/ESKF.cpp#L148-L174)). The parser has no IMU unit assertion.

This differs from the vehicle FAST-LIO callback, which multiplies the received acceleration by `10.0` before buffering while copying gyro unchanged ([vehicle lio_node.cpp:417-449](https://github.com/Yangbadger222/XJTLU-autonomous-vehicle-rtk/blob/e54c6afbcb5a58db22d7c468085a87d658b0b932/src/perception/fastlio2/src/lio_node.cpp#L417-L449)). The old callback's factor is not documented as a physical conversion in the source. It must be explained against the actual IMU driver/bag units before any Super adapter can carry the value. Copying the Super parser verbatim would remove that factor; retaining it without unit evidence would also be unsafe.

### Livox point fields and timing

For `CustomMsg`, `livoxHandler()` performs this mapping ([ROSWrapper.cpp:419-441](https://github.com/Liansheng-Wang/Super-LIO/blob/f89f48dc7aea6cfa262f18e4d03b319e04e0dbd2/src/super_lio/src/ros/ROSWrapper.cpp#L419-L441)):

```text
point.x/y/z          -> PointXTZIT.x/y/z                         [m]
point.reflectivity   -> intensity                               [0..255 integer copied to float]
point.offset_time    -> PointXTZIT.offset_time = 1e-9*offset_time [s]
header.stamp         -> start_time                                [s]
end_time             = start_time + offset_time(last accepted point)
```

Points are sampled at indices `0, filter_rate, ...`; only Livox tags whose `tag & 0x30` is `0x10` or `0x00` are accepted; acceptance also requires `blind^2 < x^2+y^2+z^2 < maxrange^2`. There is no `line < 4` test and `timebase` is not read. The standalone `livox2pcl()` helper is different (starts its index at 1 and also excludes the first point), but the live `livoxHandler()` is the path used by `super_lio_node`.

The vehicle path uses the same tag and radial range test but additionally requires `line < 4`, stores `offset_time/1e6` as `curvature` in milliseconds, and later divides by `1000` when computing the cloud end time ([vehicle utils.cpp:2-27](https://github.com/Yangbadger222/XJTLU-autonomous-vehicle-rtk/blob/e54c6afbcb5a58db22d7c468085a87d658b0b932/src/perception/fastlio2/src/utils.cpp#L2-L27), [vehicle lio_node.cpp:482-512](https://github.com/Yangbadger222/XJTLU-autonomous-vehicle-rtk/blob/e54c6afbcb5a58db22d7c468085a87d658b0b932/src/perception/fastlio2/src/lio_node.cpp#L482-L512)).

The vehicle Livox driver makes the timing risk concrete. It comments out the hardware `pkg.base_time` stamp and assigns `header.stamp = cur_node_->now()` and `timebase = cur_node_->now().nanoseconds()` ([vehicle lddc.cpp:570-603](https://github.com/Yangbadger222/XJTLU-autonomous-vehicle-rtk/blob/e54c6afbcb5a58db22d7c468085a87d658b0b932/src/sensor_drivers/livox_ros_driver2/src/lddc.cpp#L570-L603)); point `offset_time` remains `points[i].offset_time - pkg.base_time` ([vehicle lddc.cpp:614-629](https://github.com/Yangbadger222/XJTLU-autonomous-vehicle-rtk/blob/e54c6afbcb5a58db22d7c468085a87d658b0b932/src/sensor_drivers/livox_ros_driver2/src/lddc.cpp#L614-L629)). IMU messages likewise use `cur_node_->now()` while copying the device values ([vehicle lddc.cpp:758-788](https://github.com/Yangbadger222/XJTLU-autonomous-vehicle-rtk/blob/e54c6afbcb5a58db22d7c468085a87d658b0b932/src/sensor_drivers/livox_ros_driver2/src/lddc.cpp#L758-L788)). Therefore, Super's `header.stamp + offset_time` inherits arrival/system-time semantics from this driver; the source does not prove that the header is the first-point hardware measurement time. A replay/live acceptance test must establish the relationship and clock domain before claiming deskew correctness.

### Synchronization equations and edge cases

Super retains a LiDAR frame until an IMU sample at or after its `end_time` is available. It pops IMU samples with `imu_time <= lidar.end_time`, records `last_timestamp_lidar_ = end_time`, and drops an already queued frame when `last_timestamp_lidar_ > frame.end_time` ([ROSWrapper.cpp:532-565](https://github.com/Liansheng-Wang/Super-LIO/blob/f89f48dc7aea6cfa262f18e4d03b319e04e0dbd2/src/super_lio/src/ros/ROSWrapper.cpp#L532-L565)). The vehicle synchronizer instead sorts accepted points by millisecond `curvature`, uses `cloud_start + last.curvature/1000`, waits for `last_imu_time >= cloud_end`, and consumes IMU samples with `time < cloud_end` ([vehicle lio_node.cpp:482-513](https://github.com/Yangbadger222/XJTLU-autonomous-vehicle-rtk/blob/e54c6afbcb5a58db22d7c468085a87d658b0b932/src/perception/fastlio2/src/lio_node.cpp#L482-L513)). This means the boundary inclusion rule differs (`<=` in Super's loop versus `<` in vehicle), and Super does not sort the filtered Livox points before choosing the last accepted offset. The adapter must test non-monotonic offsets, skipped points, timestamp rollback, and bag pause/seek behavior rather than assuming equivalent synchronization.

### Point-to-state transform and deskew

Super's `g_lidar_imu` is stored as an SE(3) with the source comment “lidar in imu frame” ([params.h:58-60](https://github.com/Liansheng-Wang/Super-LIO/blob/f89f48dc7aea6cfa262f18e4d03b319e04e0dbd2/src/super_lio/include/lio/params.h#L58-L60)). The map initializer applies `T_WI^0 T_IL` to each LiDAR point ([super_lio.cpp:163-190](https://github.com/Liansheng-Wang/Super-LIO/blob/f89f48dc7aea6cfa262f18e4d03b319e04e0dbd2/src/super_lio/src/lio/super_lio.cpp#L163-L190)). During deskew, for raw point `p_L` at query time `t`, it evaluates interpolated IMU pose `(R_WI(t),p_WI(t))`, endpoint `(R_WI(e),p_WI(e))`, then computes

```text
p_I(t) = R_IL p_L + t_IL
p_W(t) = R_WI(t) p_I(t) + p_WI(t)
p_L^deskew = R_WI(e)^T [p_W(t) - p_WI(e)]
             = R_WI(e)^T [R_WI(t)(R_IL p_L+t_IL) + p_WI(t)-p_WI(e)]
```

([super_lio.cpp:361-426](https://github.com/Liansheng-Wang/Super-LIO/blob/f89f48dc7aea6cfa262f18e4d03b319e04e0dbd2/src/super_lio/src/lio/super_lio.cpp#L361-L426)). This is an endpoint-IMU-frame cloud; it is not automatically a `base_footprint` cloud. The vehicle FAST deskew has the analogous formula but uses `r_il`, `t_il` and the state endpoint ([vehicle imu_processor.cpp:101-132](https://github.com/Yangbadger222/XJTLU-autonomous-vehicle-rtk/blob/e54c6afbcb5a58db22d7c468085a87d658b0b932/src/perception/fastlio2/src/map_builder/imu_processor.cpp#L101-L132)).

## Super-LIO outputs versus vehicle FAST-LIO2

### State and velocity semantics

`ESKF::GetNavState()` returns `(timestamp,R,p,v)` directly from the nominal state; `GetDynamicState()` additionally returns body gyro and global acceleration ([ESKF.h:60-66](https://github.com/Liansheng-Wang/Super-LIO/blob/f89f48dc7aea6cfa262f18e4d03b319e04e0dbd2/src/super_lio/include/lio/ESKF.h#L60-L66)). The propagation equations update `p` and `v` in the world frame and update `R` as the IMU orientation ([ESKF.cpp:154-183](https://github.com/Liansheng-Wang/Super-LIO/blob/f89f48dc7aea6cfa262f18e4d03b319e04e0dbd2/src/super_lio/src/lio/ESKF.cpp#L154-L183)).

The Super ROS odometry publisher sets `header.frame_id = "world"`, stamps it with the measurement state timestamp, copies `state.p`, `state.R`, and **world-frame** `state.v` into `twist.twist.linear`, and leaves `child_frame_id` and both covariance arrays at their message defaults ([ROSWrapper.cpp:568-587](https://github.com/Liansheng-Wang/Super-LIO/blob/f89f48dc7aea6cfa262f18e4d03b319e04e0dbd2/src/super_lio/src/ros/ROSWrapper.cpp#L568-L587)). In contrast, vehicle FAST publishes `header.frame_id = world_frame`, `child_frame_id = body_frame`, IMU pose `t_WI,R_WI`, and body-frame twist

```text
v_body = R_WI^T v_world
```

([vehicle lio_node.cpp:676-697](https://github.com/Yangbadger222/XJTLU-autonomous-vehicle-rtk/blob/e54c6afbcb5a58db22d7c468085a87d658b0b932/src/perception/fastlio2/src/lio_node.cpp#L676-L697)). In the corridor baseline, the launch contract explicitly gives the RTK corrector `base_frame: base_footprint` for `/fastlio2/lio_odom` ([system_gps_corridor.launch.py:249-266](https://github.com/Yangbadger222/XJTLU-autonomous-vehicle-rtk/blob/e54c6afbcb5a58db22d7c468085a87d658b0b932/src/bringup/launch/system_gps_corridor.launch.py#L249-L266)); the old node's `tf` publisher sends `world_frame -> body_frame` ([vehicle lio_node.cpp:736-751](https://github.com/Yangbadger222/XJTLU-autonomous-vehicle-rtk/blob/e54c6afbcb5a58db22d7c468085a87d658b0b932/src/perception/fastlio2/src/lio_node.cpp#L736-L751)). Super sends only `world -> imu` ([ROSWrapper.cpp:617-632](https://github.com/Liansheng-Wang/Super-LIO/blob/f89f48dc7aea6cfa262f18e4d03b319e04e0dbd2/src/super_lio/src/ros/ROSWrapper.cpp#L617-L632)). Merely changing Super's string `frame_id` would therefore be a frame lie.

### Required rigid-body mapping

Let Super's nominal output be `T_WI = (R_WI,p_WI)`, world-frame IMU velocity `v_WI`, angular velocity `omega_I`, and let the locked vehicle body reference be `B`. If the measured extrinsic is `T_IB=(R_IB,t_IB)` (IMU-to-body transform), the physically correct adapter output is

```text
R_WB = R_WI R_IB
p_WB = p_WI + R_WI t_IB
v_B  = R_IB^T (R_WI^T v_WI + omega_I x t_IB)  (body coordinates)
```

If the locked extrinsic is stored in the opposite direction, invert it first (`R_BI=R_IB^T`, `t_BI=-R_IB^T t_IB`). A covariance must be transformed with the corresponding Jacobian/SE(3) adjoint, e.g. `Sigma_B = J Sigma_I J^T`, including the lever-arm position/velocity cross term; zero-filled covariance is not valid evidence. Super's ESKF exposes `GetCov()` internally ([ESKF.h:70-74](https://github.com/Liansheng-Wang/Super-LIO/blob/f89f48dc7aea6cfa262f18e4d03b319e04e0dbd2/src/super_lio/include/lio/ESKF.h#L70-L74)) but `ROSWrapper::pub_odom()` never reads or publishes it. The exact locked `IMU↔base_footprint` transform is not represented by Super's `g_odom_robo` name unambiguously: its parser builds XYZ Euler angles in degrees, transposes the rotation, then stores the supplied translation unchanged ([ROSWrapper.cpp:98-123](https://github.com/Liansheng-Wang/Super-LIO/blob/f89f48dc7aea6cfa262f18e4d03b319e04e0dbd2/src/super_lio/src/ros/ROSWrapper.cpp#L98-L123)). The adapter must resolve direction and reference point against vehicle URDF/static TF and a lever-arm test; this audit does not declare that mapping complete.
If the locked extrinsic is stored in the opposite direction, invert it first (`R_BI=R_IB^T`, `t_BI=-R_IB^T t_IB`). A covariance must be transformed with the corresponding Jacobian/SE(3) adjoint, e.g. `Sigma_B = J Sigma_I J^T`, including the lever-arm position/velocity cross term; zero-filled covariance is not valid evidence. Super's ESKF exposes `GetCov()` internally ([ESKF.h:70-74](https://github.com/Liansheng-Wang/Super-LIO/blob/f89f48dc7aea6cfa262f18e4d03b319e04e0dbd2/src/super_lio/include/lio/ESKF.h#L70-L74)) but `ROSWrapper::pub_odom()` never reads or publishes it. The `pub_odom()` path also does not copy angular velocity; the only internal dynamic state carrying `body_omega_` is `GetDynamicState()` ([ESKF.h:64-66](https://github.com/Liansheng-Wang/Super-LIO/blob/f89f48dc7aea6cfa262f18e4d03b319e04e0dbd2/src/super_lio/include/lio/ESKF.h#L64-L66)), so a lever-arm velocity mapping cannot be implemented from the published message alone. The exact locked `IMU↔base_footprint` transform is not represented by Super's `g_odom_robo` name unambiguously: its parser builds XYZ Euler angles in degrees, transposes the rotation, then stores the supplied translation unchanged ([ROSWrapper.cpp:98-123](https://github.com/Liansheng-Wang/Super-LIO/blob/f89f48dc7aea6cfa262f18e4d03b319e04e0dbd2/src/super_lio/src/ros/ROSWrapper.cpp#L98-L123)). The adapter must resolve direction and reference point against vehicle URDF/static TF and a lever-arm test; this audit does not declare that mapping complete.

Super's alternate `Predict(..., DynamicState&, DynamicState&)` computes a robot pose using `new_R = R_WI g_odom_robo.R_` and `new_p = p_WI - R_WI g_odom_robo.R_ g_odom_robo.t_`, but explicitly leaves robot `v,w,a` as TODO ([ESKF.cpp:176-183](https://github.com/Liansheng-Wang/Super-LIO/blob/f89f48dc7aea6cfa262f18e4d03b319e04e0dbd2/src/super_lio/src/lio/ESKF.cpp#L176-L183)). This cannot be used as a complete vehicle odometry contract.

### Cloud topics, frames, and height filtering

Super advertises `/lio/cloud_world` as `sensor_msgs/PointCloud2` ([ROSWrapper.cpp:325-344](https://github.com/Liansheng-Wang/Super-LIO/blob/f89f48dc7aea6cfa262f18e4d03b319e04e0dbd2/src/super_lio/src/ros/ROSWrapper.cpp#L325-L344)). `Output()` transforms the deskewed endpoint-IMU-frame cloud using `state.R,state.p` and labels it `world` ([super_lio.cpp:560-585](https://github.com/Liansheng-Wang/Super-LIO/blob/f89f48dc7aea6cfa262f18e4d03b319e04e0dbd2/src/super_lio/src/lio/super_lio.cpp#L560-L585)). It does not publish the vehicle's filtered body cloud or its obstacle-specific cloud in this path. The optional `CloudPose` methods publish an unlabelled `CloudPose` pose plus cloud, and the cloud's `frame_id` is not set by those methods ([ROSWrapper.cpp:665-705](https://github.com/Liansheng-Wang/Super-LIO/blob/f89f48dc7aea6cfa262f18e4d03b319e04e0dbd2/src/super_lio/src/ros/ROSWrapper.cpp#L665-L705); [CloudPose.msg](https://github.com/Liansheng-Wang/Super-LIO/blob/f89f48dc7aea6cfa262f18e4d03b319e04e0dbd2/src/super_lio/msg/CloudPose.msg)).

The vehicle node publishes body/world clouds and a separate `body_cloud_nav2_obstacles`; its filters use world cloud `z - t_wi.z` (relative to the **IMU state origin**, not the vehicle footprint) and retain configured intervals ([vehicle lio_node.cpp:526-607](https://github.com/Yangbadger222/XJTLU-autonomous-vehicle-rtk/blob/e54c6afbcb5a58db22d7c468085a87d658b0b932/src/perception/fastlio2/src/lio_node.cpp#L526-L607), [vehicle lio_node.cpp:609-673](https://github.com/Yangbadger222/XJTLU-autonomous-vehicle-rtk/blob/e54c6afbcb5a58db22d7c468085a87d658b0b932/src/perception/fastlio2/src/lio_node.cpp#L609-L673)). The FAST-LIO2 package YAML carries `publish_cloud_height_filter_enabled=true`, `[-0.33,0.30]`, and obstacle `[-0.20,1.20]` ([vehicle lio.yaml:38-43](https://github.com/Yangbadger222/XJTLU-autonomous-vehicle-rtk/blob/e54c6afbcb5a58db22d7c468085a87d658b0b932/src/perception/fastlio2/config/lio.yaml#L38-L43)); the corridor `master_params.yaml` separately overrides the Nav2 obstacle window to `[0.08,1.20]` ([master_params.yaml:62-76](https://github.com/Yangbadger222/XJTLU-autonomous-vehicle-rtk/blob/e54c6afbcb5a58db22d7c468085a87d658b0b932/src/bringup/config/master_params.yaml#L62-L76)). This is evidence that source YAML and launch-effective values must be tracked separately. Super has no equivalent output filter. A direct `/lio/cloud_world` rename would lose both filter semantics and the separate obstacle contract.

### Covariance and health

FAST-LIO2's vehicle IESKF publishes a three-element `Float32MultiArray` `/fastlio2/degeneracy`: minimum eigenvalue, condition number, and whether measurement-block eigenvalues were regularized. The threshold is `75.0`, and the metrics are reset each update ([vehicle ieskf.cpp:6-55](https://github.com/Yangbadger222/XJTLU-autonomous-vehicle-rtk/blob/e54c6afbcb5a58db22d7c468085a87d658b0b932/src/perception/fastlio2/src/map_builder/ieskf.cpp#L6-L55), [vehicle lio_node.cpp:719-733](https://github.com/Yangbadger222/XJTLU-autonomous-vehicle-rtk/blob/e54c6afbcb5a58db22d7c468085a87d658b0b932/src/perception/fastlio2/src/lio_node.cpp#L719-L733)). The health aggregator interprets this stream along with PGO, RTK status, and odometry velocity ([vehicle health_aggregator_node.cpp:17-55](https://github.com/Yangbadger222/XJTLU-autonomous-vehicle-rtk/blob/e54c6afbcb5a58db22d7c468085a87d658b0b932/src/frc/frc_nodes_cpp/src/health_aggregator_node.cpp#L17-L55), [Health.msg:1-20](https://github.com/Yangbadger222/XJTLU-autonomous-vehicle-rtk/blob/e54c6afbcb5a58db22d7c468085a87d658b0b932/src/frc/frc_msgs/msg/Health.msg#L1-L20)). Super has no degeneracy publisher, health message, measurement eigenvalue metric, or ROS covariance publication. Its ESKF's `need_converge_` flag is an internal optimizer iteration flag, not equivalent health ([ESKF.cpp:269-276](https://github.com/Liansheng-Wang/Super-LIO/blob/f89f48dc7aea6cfa262f18e4d03b319e04e0dbd2/src/super_lio/src/lio/ESKF.cpp#L269-L276)). A compatibility adapter must therefore report Super-specific health as `UNKNOWN` until a mathematically validated metric and conservative stop policy exist; mapping “no field” to healthy would weaken the existing safety gate.

### Map saving is not localization acceptance

Super accumulates transformed scans and saves PCD fragments/final maps when enabled ([super_lio.cpp:212-350](https://github.com/Liansheng-Wang/Super-LIO/blob/f89f48dc7aea6cfa262f18e4d03b319e04e0dbd2/src/super_lio/src/lio/super_lio.cpp#L212-L350)). Its README advertises a relocation launch after a map exists ([README.md:72-83](https://github.com/Liansheng-Wang/Super-LIO/blob/f89f48dc7aea6cfa262f18e4d03b319e04e0dbd2/README.md#L72-L83)), but neither source proves vehicle map-version identity, RTK authority handoff, or cross-run acceptance. Treat map persistence and relocation as separate research interfaces until replay and authority tests pass.

## EGO Planner 2D exact integration audit

### Public library seam and GridMap2D behavior

The upstream library's intended C++ seam is `PlannerInterface::initParam`, `initEsdfMap`, `setPathPoint`, `setObstacles`, `setCurrentVehiclePos`, `makePlan`, `getLocalPlanTrajResults`, `setGridMap`, `getAStarPath`, `resetMap`, and `getInflateOccupancy` ([planner_interface.h:37-103](https://github.com/JackJu-HIT/Ego-Planner-2D-ROS2/blob/7f5be6d4cee34871e85aa1f15285cfaf17b23877/src/EgoPlanner-ROS2/planner/plan_manage/include/plan_manage/planner_interface.h#L37-L103)). `PathPoint` has only `x,y,z,v`; `ObstacleInfo` has `x,y,z`. There is no timestamp, yaw, angular velocity, covariance, source frame, map version, or validity status in these types.

`initEsdfMap()` constructs `GridMap2D(resolution,map_size)` (the `origin` and `z_size` arguments are only printed), sets an inflation radius, and initializes A* to a fixed `100x100` grid ([planner_interface.cpp:26-43](https://github.com/JackJu-HIT/Ego-Planner-2D-ROS2/blob/7f5be6d4cee34871e85aa1f15285cfaf17b23877/src/EgoPlanner-ROS2/planner/plan_manage/src/planner_interface.cpp#L26-L43)). The map is actually centered around the current pose when `setCurPose(x,y)` is called; it allocates dimensions from `world_size/resolution` and sets `origin = current_pose - world_size/2` ([grid_map.cpp:8-49](https://github.com/JackJu-HIT/Ego-Planner-2D-ROS2/blob/7f5be6d4cee34871e85aa1f15285cfaf17b23877/src/EgoPlanner-ROS2/planner/GridMap2D/src/grid_map.cpp#L8-L49)). World-to-grid is `round((world-origin)/resolution)` ([grid_map.cpp:239-248](https://github.com/JackJu-HIT/Ego-Planner-2D-ROS2/blob/7f5be6d4cee34871e85aa1f15285cfaf17b23877/src/EgoPlanner-ROS2/planner/GridMap2D/src/grid_map.cpp#L239-L248)); out-of-range points are occupied ([grid_map.cpp:132-147](https://github.com/JackJu-HIT/Ego-Planner-2D-ROS2/blob/7f5be6d4cee34871e85aa1f15285cfaf17b23877/src/EgoPlanner-ROS2/planner/GridMap2D/src/grid_map.cpp#L132-L147)). Inflation is a Euclidean-radius test over a square index neighborhood and rounds `radius/resolution` up ([grid_map.cpp:150-220](https://github.com/JackJu-HIT/Ego-Planner-2D-ROS2/blob/7f5be6d4cee34871e85aa1f15285cfaf17b23877/src/EgoPlanner-ROS2/planner/GridMap2D/src/grid_map.cpp#L150-L220)). This is a local rolling map, not a persistent world GridMap contract; a vehicle adapter must provide frame/origin/version and explicitly decide how to ingest obstacle evidence and prior roads.

`setObstacles()` converts each input `x,y` in the assumed current world frame, sets the corresponding raw cells, and calls `inflate()` ([planner_interface.cpp:45-74](https://github.com/JackJu-HIT/Ego-Planner-2D-ROS2/blob/7f5be6d4cee34871e85aa1f15285cfaf17b23877/src/EgoPlanner-ROS2/planner/plan_manage/src/planner_interface.cpp#L45-L74)). It does not clear the map; the demo calls `resetMap()` before each cycle. It also cannot distinguish raw obstacle evidence from unknown space, free-space evidence, or road prior. Those distinctions must be added outside this upstream API for the research loop.

### Demo parameters and state injection

The node header hardcodes `max_vel=2.0`, `max_acc=3.0`, `max_jerk=4.0`, `map_resolution=0.1`, map size `50x50x10`, origin zero, and inflate radius `0.5` ([trajectory_obstacles_publisher.h:130-139](https://github.com/JackJu-HIT/Ego-Planner-2D-ROS2/blob/7f5be6d4cee34871e85aa1f15285cfaf17b23877/src/EgoPlanner-ROS2/include/trajectory_obstacles_publisher.h#L130-L139)). `PlannerInterface::initParam()` accepts the first three but itself hardcodes feasibility tolerance `0.05`, control-point spacing `0.4`, and planning horizon `5.0` ([planner_interface.cpp:16-24](https://github.com/JackJu-HIT/Ego-Planner-2D-ROS2/blob/7f5be6d4cee34871e85aa1f15285cfaf17b23877/src/EgoPlanner-ROS2/planner/plan_manage/src/planner_interface.cpp#L16-L24)). Separately, `BsplineOptimizer::setParam()` stores its own `max_vel_=1.0` and `max_acc_=0.5` ([bspline_optimizer.cpp:7-23](https://github.com/JackJu-HIT/Ego-Planner-2D-ROS2/blob/7f5be6d4cee34871e85aa1f15285cfaf17b23877/src/EgoPlanner-ROS2/planner/bspline_opt/src/bspline_optimizer.cpp#L7-L23)); no call in this path copies `PlannerInterface::pp_` into those optimizer fields. Consequently the optimizer's active axis-wise feasibility cost can use `1.0/0.5` even when the caller supplies different limits. None of these demo values can be promoted to the locked vehicle limits without source/parsed/runtime parameter checks.

`makePlan()` converts global path points to 3D with `z=0.2`, chooses the final point as target, and constructs the initial derivatives as

```text
start_vel = [cos(yaw), sin(yaw), 0]
start_acc = [cos(yaw), sin(yaw), 0]
target_vel = [0,0,0]
```

([planner_interface.cpp:125-177](https://github.com/JackJu-HIT/Ego-Planner-2D-ROS2/blob/7f5be6d4cee34871e85aa1f15285cfaf17b23877/src/EgoPlanner-ROS2/planner/plan_manage/src/planner_interface.cpp#L125-L177)). Thus the start velocity and acceleration are unit vectors independent of the measured vehicle speed and acceleration. The source comments explicitly say to change this for actual integration. The correct vehicle injection must use measured state, rotate world velocity/acceleration to the body/world convention expected by the optimizer, and enforce forward/side-slip policy before planning.

### B-spline timing and feasibility

The initial knot interval is `ctrl_pt_dist/max_vel*1.2` when the target is farther than `0.1 m`, otherwise `ctrl_pt_dist/max_vel*5` ([planner_interface.cpp:199-217](https://github.com/JackJu-HIT/Ego-Planner-2D-ROS2/blob/7f5be6d4cee34871e85aa1f15285cfaf17b23877/src/EgoPlanner-ROS2/planner/plan_manage/src/planner_interface.cpp#L199-L217)). After optimization, the code sets physical limits but wraps the only `checkFeasibility()`/`refineTrajAlgo()` call in `if (false)`, so no time reallocation occurs ([planner_interface.cpp:235-263](https://github.com/JackJu-HIT/Ego-Planner-2D-ROS2/blob/7f5be6d4cee34871e85aa1f15285cfaf17b23877/src/EgoPlanner-ROS2/planner/plan_manage/src/planner_interface.cpp#L235-L263)).

The dormant library routine checks each component of B-spline velocity and acceleration against `limit*(1+tolerance)+1e-4`, computes `ratio=max(max_vel/limit_vel, sqrt(max_acc/limit_acc))`, and `lengthenTime(ratio)` stretches knots ([uniform_bspline.cpp:128-207](https://github.com/JackJu-HIT/Ego-Planner-2D-ROS2/blob/7f5be6d4cee34871e85aa1f15285cfaf17b23877/src/EgoPlanner-ROS2/planner/bspline_opt/src/uniform_bspline.cpp#L128-L207)). This is an axis-wise world-coordinate test, not a body-frame speed/acceleration or a complete vehicle feasibility certificate.

The exported plan result is also untimed. `updateTrajInfo()` internally retains position, first derivative, second derivative, duration, and trajectory ID ([planner_interface.cpp:298-306](https://github.com/JackJu-HIT/Ego-Planner-2D-ROS2/blob/7f5be6d4cee34871e85aa1f15285cfaf17b23877/src/EgoPlanner-ROS2/planner/plan_manage/src/planner_interface.cpp#L298-L306)), but `getTraj()` samples using a hardcoded `0.1` seconds, stores only `x,y` into `PathPoint`, and drops the sampled `v` and `acc` ([planner_interface.cpp:328-380](https://github.com/JackJu-HIT/Ego-Planner-2D-ROS2/blob/7f5be6d4cee34871e85aa1f15285cfaf17b23877/src/EgoPlanner-ROS2/planner/plan_manage/src/planner_interface.cpp#L328-L380)). A production contract must expose either the B-spline control points/knots (with trajectory ID, start stamp, frame and map version) or a timed sample message containing pose, tangent/yaw, `v`, `a`, curvature, yaw rate, and validity.

### Curvature/yaw-rate code exists but is not in the main cost

The optimizer computes per-segment velocity `v_i=(q_{i+1}-q_i)/ts`, acceleration `a_i=(q_{i+2}-2q_{i+1}+q_i)/ts^2`, and has `calKappaCost()` and `calTurnCost()` implementations ([bspline_optimizer.cpp:1068-1228](https://github.com/JackJu-HIT/Ego-Planner-2D-ROS2/blob/7f5be6d4cee34871e85aa1f15285cfaf17b23877/src/EgoPlanner-ROS2/planner/bspline_opt/src/bspline_optimizer.cpp#L1068-L1228)). Their current constants are `k_max=1` and `w_max=1`, with no vehicle wheelbase/slip model. More critically, `combineCostRebound()` calls smoothness, distance and axis-wise feasibility, while `calTurnCost()` and `calKappaCost()` are commented and excluded from the combined objective ([bspline_optimizer.cpp:1038-1065](https://github.com/JackJu-HIT/Ego-Planner-2D-ROS2/blob/7f5be6d4cee34871e85aa1f15285cfaf17b23877/src/EgoPlanner-ROS2/planner/bspline_opt/src/bspline_optimizer.cpp#L1038-L1065)). The refine objective likewise combines only smoothness, fitness, and feasibility ([bspline_optimizer.cpp:1230-1248](https://github.com/JackJu-HIT/Ego-Planner-2D-ROS2/blob/7f5be6d4cee34871e85aa1f15285cfaf17b23877/src/EgoPlanner-ROS2/planner/bspline_opt/src/bspline_optimizer.cpp#L1230-L1248)).

For a forward-moving planar vehicle, the adapter must compute from the timed curve

```text
v = ||p_dot||
kappa = (p_dot_x p_ddot_y - p_dot_y p_ddot_x) / ||p_dot||^3    (when ||p_dot|| > eps)
omega = v * kappa
```

then apply the locked differential/slip steering constraints, including sign/forward policy, `|omega|`, curvature, yaw-rate/angle-acceleration and body-frame lateral-velocity (`v_y≈0`) checks. The upstream `W=(a_y v_x-v_y a_x)/(v_x²+v_y²)` helper is not a substitute for these checks, especially near zero speed and when the vehicle's admissible turning model is not unit-radius.

### Demo ROS interface and fake vehicle limitations

The demo subscribes to `current_pose`, `/goal_pose`, `/clicked_point`, `/initialpose`, and `/trigger_plan`; `/goal_pose` is used to add an obstacle rather than a navigation goal ([trajectory_publisher.cpp:18-69](https://github.com/JackJu-HIT/Ego-Planner-2D-ROS2/blob/7f5be6d4cee34871e85aa1f15285cfaf17b23877/src/EgoPlanner-ROS2/src/trajectory_publisher.cpp#L18-L69), [trajectory_publisher.cpp:109-165](https://github.com/JackJu-HIT/Ego-Planner-2D-ROS2/blob/7f5be6d4cee34871e85aa1f15285cfaf17b23877/src/EgoPlanner-ROS2/src/trajectory_publisher.cpp#L109-L165)). It has no subscription for the vehicle `/lio/odom`, actual `PointCloud2`, GridMap evidence, GeoTIFF/MaGRoad prior, RTK authority, or map version. Every 50 ms it filters manually accumulated obstacles to 10 m, resamples a manually clicked path at `0.3 m`, truncates to 50 points, and repeatedly calls `makePlan()` ([trajectory_publisher.cpp:167-300](https://github.com/JackJu-HIT/Ego-Planner-2D-ROS2/blob/7f5be6d4cee34871e85aa1f15285cfaf17b23877/src/EgoPlanner-ROS2/src/trajectory_publisher.cpp#L167-L300)).

The output is a visualization-only `nav_msgs/Path`: every pose has a zero-yaw quaternion and carries no B-spline time, speed, acceleration, curvature, validity, or source map identity ([trajectory_publisher.cpp:405-448](https://github.com/JackJu-HIT/Ego-Planner-2D-ROS2/blob/7f5be6d4cee34871e85aa1f15285cfaf17b23877/src/EgoPlanner-ROS2/src/trajectory_publisher.cpp#L405-L448)). It cannot be passed directly to a vehicle controller.

`rviz_car_sim/fake_sim_node.cpp` is a separate demo. It integrates a unicycle with `dt=0.02`, publishes `map -> base_link`, follows the visualization path, uses a fixed lookahead `1.0`, sets `current_v=1.0`, then clips `v<=0.5` and `|w|<=1.0` ([fake_sim_node.cpp:114-155](https://github.com/JackJu-HIT/Ego-Planner-2D-ROS2/blob/7f5be6d4cee34871e85aa1f15285cfaf17b23877/src/rviz_car_sim/src/fake_sim_node.cpp#L114-L155), [fake_sim_node.cpp:157-177](https://github.com/JackJu-HIT/Ego-Planner-2D-ROS2/blob/7f5be6d4cee34871e85aa1f15285cfaf17b23877/src/rviz_car_sim/src/fake_sim_node.cpp#L157-L177), [fake_sim_node.cpp:237-303](https://github.com/JackJu-HIT/Ego-Planner-2D-ROS2/blob/7f5be6d4cee34871e85aa1f15285cfaf17b23877/src/rviz_car_sim/src/fake_sim_node.cpp#L237-L303)). Starting it alongside the vehicle chain would compete for `map -> base_link`; it must remain isolated and must not be used as proof of real tracking or authority behavior.

## Adapter conclusions and safety blockers

1. **Super parser cannot be declared equivalent yet.** The point coordinate units are metres and offset fields are nanoseconds in the ROS message, but the vehicle driver deliberately stamps both LiDAR and IMU with system `now`, and the old node applies an unexplained acceleration factor of `10.0`. A bag with the raw driver timing plus an IMU unit check is required.
2. **The odometry contract needs a real adapter.** Super's output is world/IMU pose and world velocity with no child frame, covariance, or health. The vehicle contract is world/base-footprint pose, body velocity, a TF edge, and degeneracy health. The rigid transform and lever-arm velocity/covariance equations above must be implemented and tested; frame IDs alone are insufficient.
3. **Cloud safety inputs must be recreated deliberately.** Super's world cloud does not preserve the old body/world height filters or the separate obstacle-cloud interval. A research cloud for ground evidence may be added, but the old obstacle semantics cannot be silently replaced.
4. **RTK authority is outside both upstream libraries.** Neither Super nor EGO has authority/stop semantics. The Super adapter must never set motion permission, publish a competing `map -> odom`, or treat LIO health as permission to move when the vehicle RTK gate has stopped the car.
5. **EGO requires a timed vehicle interface.** The pinned library has useful `UniformBspline`, GridMap, A*, and optimizer calls, but its demo state derivatives are fabricated, time repair is disabled, axis-wise feasibility is incomplete, curvature terms are disabled, and its only ROS output is a zero-yaw `Path`. The narrow integration should preserve the upstream planner core while adding measured derivatives, timed trajectory metadata, explicit dynamic/kinematic checks, and a final safety/authority bridge.
6. **Unknowns that block real-motion acceptance:** the exact IMU acceleration unit and sensor timestamp relationship in the current Livox driver; the direction and reference point of the locked IMU/LiDAR/body extrinsics; the correct covariance transformation and a Super-specific degeneracy metric; a validated vehicle curvature/yaw-rate/slip model; the real obstacle/ground frame and camera/GeoTIFF/MaGRoad calibration; and a physical emergency-stop test showing the final mock serial bytes. These are pending evidence, not values to infer from examples.

## Source-to-adapter mapping checklist

| contract item | Super-LIO source behavior | old vehicle behavior | adapter action / acceptance |
|---|---|---|---|
| IMU stamp | header sec/nsec, raw vectors | header sec/nsec, acceleration multiplied by 10 | identify units and preserve measured stamp domain; replay monotonic/rollback tests |
| Livox offset | `uint32 ns` → seconds; no line filter | `ns` → ms curvature; `line<4` | test field-level equivalence and ordering; retain locked range/line semantics |
| pose | `world -> imu`, state timestamp | `world -> base_footprint`, measurement odom and future TF | apply verified rigid transform; one TF/odom publisher only |
| velocity | world-frame `state.v` | body-frame `R^T v` | rotate and add lever-arm term when reference differs |
| covariance | ESKF internal only; ROS arrays default | ROS arrays also default, but health uses degeneracy stream | expose transformed covariance only when valid; otherwise explicit unknown gate |
| cloud | `/lio/cloud_world`, endpoint-IMU transform | body/world + filtered Nav2 obstacle cloud | preserve filters and provide explicit body/world frames |
| health | no equivalent | eigen min/condition/regularized | source-aware health; no unconditional healthy alias |
| EGO path | visualization `nav_msgs/Path` only | no timed trajectory contract | add trajectory ID/start stamp/knots or timed samples with kinematic validity |
| authority | no authority or stop | RTK gate owns stop/motion permission | keep RTK authority and serial safety bridge unchanged |

## Project ROS edge follow-up (2026-10-07)

The second project patch (`0002-vehicle-ros-timed-trajectory.patch`) replaces
only the upstream interactive ROS edge. It subscribes to `/lio/odom_vehicle`,
`/research/road_reference`, `/research/local_obstacle_grid`, and
`/research/map_version`; it calls the patched `PlannerInterface` with measured
world-frame velocity/acceleration and publishes `research_interfaces/TimedTrajectory2D`.
The old `nav_msgs/Path` publisher remains visualization-only. Unknown occupancy
cells, missing map version, zero/unmeasured curvature/footprint limits and stale
state produce a typed failure and no executable trajectory. This patch was
applied sequentially after `0001` on a fresh checkout; target ROS compilation
remains pending because ROS 2 Humble is unavailable here.
