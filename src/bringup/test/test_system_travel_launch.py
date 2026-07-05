from pathlib import Path


TRAVEL_LAUNCH = Path("src/bringup/launch/system_travel.launch.py")
NAV2_TRAVEL = Path("src/bringup/config/nav2_travel.yaml")
LAUNCH_WRAPPER = Path("scripts/launch_with_logs.sh")
BRINGUP_CMAKE = Path("src/bringup/CMakeLists.txt")
BRINGUP_PACKAGE = Path("src/bringup/package.xml")
INITIALPOSE_BRIDGE = Path("src/bringup/scripts/initialpose_relocalize_bridge.py")
NAV2_CLOUD_RETIME = Path("src/bringup/scripts/nav2_cloud_retime.py")
TRAVEL_FAIL_STOP_BT = Path("src/bringup/behavior_trees/travel_nav_to_pose_fail_stop.xml")
TRAVEL_THROUGH_POSES_FAIL_STOP_BT = Path(
    "src/bringup/behavior_trees/travel_nav_through_poses_fail_stop.xml"
)


def _travel_launch_text():
    return TRAVEL_LAUNCH.read_text(encoding="utf-8")


def test_travel_launch_exposes_prior_map_arguments():
    text = _travel_launch_text()

    for argument_name in ("map_yaml", "pcd_map", "use_rviz", "use_pgo"):
        assert f'"{argument_name}"' in text


def test_travel_launch_wires_localizer_and_nav2():
    text = _travel_launch_text()

    assert 'package="localizer"' in text
    assert 'executable="localizer_node"' in text
    assert 'executable="initialpose_relocalize_bridge.py"' in text
    assert '"pcd_map": LaunchConfiguration("pcd_map")' in text
    assert 'executable="nav2_cloud_retime.py"' in text
    assert '("cloud_in", "/fastlio2/body_cloud_nav2_obstacles")' in text
    assert '("cloud_out", "/fastlio2/body_cloud_nav2")' in text
    assert "navigation_launch.py" in text
    assert "localization_launch.py" in text
    assert "RewrittenYaml" in text
    assert "LaunchConfiguration(\"map_yaml\")" in text
    assert "robot_description.launch.py" in text
    assert "travel_rviz" in text


def test_travel_nav2_config_lets_localizer_own_map_to_odom():
    text = NAV2_TRAVEL.read_text(encoding="utf-8")
    local_costmap_text = text.split("# 全局代价地图参数块", maxsplit=1)[0]
    global_costmap_text = text.split("# 全局代价地图参数块", maxsplit=1)[1]

    assert "tf_broadcast: false" in text
    assert "topic: /fastlio2/body_cloud_nav2" in text
    assert "topic: /fastlio2/body_cloud\n" not in text
    assert 'plugins: ["static_layer", "inflation_layer"]' in global_costmap_text
    assert 'plugins: ["obstacle_layer", "inflation_layer"]' in local_costmap_text
    assert "topic: /fastlio2/body_cloud_nav2" in local_costmap_text
    assert "topic: /fastlio2/body_cloud_nav2" not in global_costmap_text
    assert "rolling_window: false" in global_costmap_text
    assert "rolling_window: true" in local_costmap_text


def test_travel_uses_fail_stop_behavior_tree_without_motion_recovery():
    launch_text = _travel_launch_text()
    nav2_text = NAV2_TRAVEL.read_text(encoding="utf-8")
    bt_navigator_text = nav2_text.split("# Navigate Through Poses", maxsplit=1)[0]

    assert "travel_nav_to_pose_fail_stop.xml" in launch_text
    assert "travel_nav_through_poses_fail_stop.xml" in launch_text
    assert '"default_nav_to_pose_bt_xml": travel_bt_xml' in launch_text
    assert '"default_nav_through_poses_bt_xml": travel_through_poses_bt_xml' in launch_text
    assert "default_nav_to_pose_bt_xml:" in bt_navigator_text
    assert "default_nav_through_poses_bt_xml:" in bt_navigator_text

    bt_expectations = {
        TRAVEL_FAIL_STOP_BT: "ComputePathToPose",
        TRAVEL_THROUGH_POSES_FAIL_STOP_BT: "ComputePathThroughPoses",
    }
    for bt_file, planner_node in bt_expectations.items():
        bt_text = bt_file.read_text(encoding="utf-8")
        assert planner_node in bt_text
        assert "SmoothPath" in bt_text
        assert 'smoother_id="savitzky_golay_smoother"' in bt_text
        assert "FollowPath" in bt_text

        for unsafe_motion_recovery in ("<Spin", "<BackUp", "RecoveryNode", "ClearEntireCostmap"):
            assert unsafe_motion_recovery not in bt_text


def test_travel_uses_smooth_low_load_mppi_controller_profile():
    text = NAV2_TRAVEL.read_text(encoding="utf-8")
    controller_text = text.split("# 局部代价地图参数块", maxsplit=1)[0]
    behavior_text = text.split("# Behavior Server 节点参数块", maxsplit=1)[1]

    assert "controller_frequency: 20.0" in controller_text
    assert 'plugin: "nav2_mppi_controller::MPPIController"' in controller_text
    assert "time_steps: 48" in controller_text
    assert "model_dt: 0.05" in controller_text
    assert "batch_size: 700" in controller_text
    assert "vx_std: 0.22" in controller_text
    assert "wz_std: 0.18" in controller_text
    assert "vx_max: 0.55" in controller_text
    assert "vx_min: 0.0" in controller_text
    assert "vy_max: 0.0" in controller_text
    assert "wz_max: 0.9" in controller_text
    assert "ax_max: 0.8" in controller_text
    assert "ax_min: -1.2" in controller_text
    assert "az_max: 3.0" in controller_text
    assert "PathAlignCritic:" in controller_text
    assert "offset_from_furthest: 6" in controller_text
    assert "PathFollowCritic:" in controller_text
    assert "cost_weight: 16.0" in controller_text
    assert 'behavior_plugins: ["wait"]' in behavior_text

    assert 'plugin: "dwb_core::DWBLocalPlanner"' not in controller_text
    assert "vx_samples:" not in controller_text
    assert "BaseObstacle.scale:" not in controller_text
    assert "batch_size: 1000" not in controller_text
    assert "vx_max: 1.0" not in controller_text


def test_travel_local_costmap_uses_stable_field_runtime_rates():
    text = NAV2_TRAVEL.read_text(encoding="utf-8")
    local_costmap_text = text.split("# 全局代价地图参数块", maxsplit=1)[0]

    assert "update_frequency: 15.0" in local_costmap_text
    assert "publish_frequency: 5.0" in local_costmap_text
    assert "width: 6" in local_costmap_text
    assert "height: 6" in local_costmap_text
    assert "resolution: 0.05" in local_costmap_text
    assert "obstacle_max_range: 6.0" in local_costmap_text
    assert "raytrace_max_range: 6.0" in local_costmap_text
    assert "update_frequency: 40.0" not in local_costmap_text
    assert "publish_frequency: 40.0" not in local_costmap_text
    assert "width: 15" not in local_costmap_text
    assert "height: 15" not in local_costmap_text
    assert "resolution: 0.02" not in local_costmap_text


def test_travel_global_costmap_uses_lower_static_map_inflation_than_local():
    text = NAV2_TRAVEL.read_text(encoding="utf-8")
    local_costmap_text = text.split("# 全局代价地图参数块", maxsplit=1)[0]
    global_costmap_text = text.split("# 全局代价地图参数块", maxsplit=1)[1]

    assert "robot_radius: 0.38625" in local_costmap_text
    assert "robot_radius: 0.22" in global_costmap_text
    assert "inflation_radius: 0.4" in local_costmap_text
    assert "inflation_radius: 0.25" in global_costmap_text
    assert "robot_radius: 0.38625" not in global_costmap_text
    assert "inflation_radius: 0.4" not in global_costmap_text


def test_travel_uses_path_and_velocity_smoothing_for_indoor_navigation():
    text = NAV2_TRAVEL.read_text(encoding="utf-8")
    smoother_text = text.split("# Behavior Server 节点参数块", maxsplit=1)[0].split(
        "# Smoother Server 节点参数块", maxsplit=1
    )[1]
    velocity_text = text.split("# Velocity Smoother 节点参数块", maxsplit=1)[1]

    assert 'smoother_plugins: ["savitzky_golay_smoother"]' in smoother_text
    assert 'plugin: "nav2_smoother::SavitzkyGolaySmoother"' in smoother_text
    assert "window_size: 7" in smoother_text
    assert "poly_order: 3" in smoother_text
    assert "do_refinement: true" in smoother_text
    assert "refinement_num: 2" in smoother_text

    assert "max_velocity: [0.55, 0.0, 0.9]" in velocity_text
    assert "min_velocity: [0.0, 0.0, -0.9]" in velocity_text
    assert "max_accel: [0.8, 0.0, 2.0]" in velocity_text
    assert "max_decel: [-1.2, 0.0, -2.5]" in velocity_text


def test_bringup_installs_travel_runtime_helper_nodes():
    cmake_text = BRINGUP_CMAKE.read_text(encoding="utf-8")
    package_text = BRINGUP_PACKAGE.read_text(encoding="utf-8")

    assert "scripts/initialpose_relocalize_bridge.py" in cmake_text
    assert "scripts/nav2_cloud_retime.py" in cmake_text

    for dependency in ("rclpy", "geometry_msgs", "sensor_msgs", "interface"):
        assert f"<exec_depend>{dependency}</exec_depend>" in package_text


def test_initialpose_bridge_calls_localizer_relocalize_from_rviz_pose():
    text = INITIALPOSE_BRIDGE.read_text(encoding="utf-8")

    assert "PoseWithCovarianceStamped" in text
    assert "/initialpose" in text
    assert "/localizer/relocalize" in text
    assert "Relocalize.Request()" in text
    assert "req.pcd_path" in text
    assert "req.yaw = yaw_from_quaternion" in text


def test_nav2_cloud_retime_republishes_pointcloud_with_current_stamp():
    text = NAV2_CLOUD_RETIME.read_text(encoding="utf-8")

    assert "PointCloud2" in text
    assert "cloud_in" in text
    assert "cloud_out" in text
    assert "out.header.stamp = self.get_clock().now().to_msg()" in text


def test_launch_wrapper_forwards_extra_launch_arguments():
    text = LAUNCH_WRAPPER.read_text(encoding="utf-8")

    assert 'EXTRA_LAUNCH_ARGS=("${@:2}")' in text
    assert '"${EXTRA_LAUNCH_ARGS[@]}"' in text


def test_runtime_cleanup_includes_travel_localizer_and_direct_ros2_launch():
    launch_wrapper_text = LAUNCH_WRAPPER.read_text(encoding="utf-8")
    makefile_text = Path("Makefile").read_text(encoding="utf-8")

    for text in (launch_wrapper_text, makefile_text):
        assert "[l]ocalizer_node" in text
        assert "[r]os2 launch" in text
        assert "[i]nitialpose_relocalize_bridge" in text
        assert "[n]av2_cloud_retime" in text
        assert "[j]oint_state_publisher" in text
