import stat
from pathlib import Path

import yaml


TRAVEL_LAUNCH = Path("src/bringup/launch/system_travel.launch.py")
NAV2_TRAVEL = Path("src/bringup/config/nav2_travel.yaml")
LAUNCH_WRAPPER = Path("scripts/launch_with_logs.sh")
BRINGUP_CMAKE = Path("src/bringup/CMakeLists.txt")
BRINGUP_PACKAGE = Path("src/bringup/package.xml")
INITIALPOSE_BRIDGE = Path("src/bringup/scripts/initialpose_relocalize_bridge.py")
NAV2_CLOUD_RETIME = Path("src/bringup/scripts/nav2_cloud_retime.py")
LOCALIZATION_CMD_GATE = Path("src/bringup/scripts/localization_cmd_gate.py")
TRAVEL_FAIL_STOP_BT = Path("src/bringup/behavior_trees/travel_nav_to_pose_fail_stop.xml")
TRAVEL_THROUGH_POSES_FAIL_STOP_BT = Path(
    "src/bringup/behavior_trees/travel_nav_through_poses_fail_stop.xml"
)


def _travel_launch_text():
    return TRAVEL_LAUNCH.read_text(encoding="utf-8")


def _nav2_travel_yaml():
    return yaml.safe_load(NAV2_TRAVEL.read_text(encoding="utf-8"))


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
        assert "SmoothPath" not in bt_text
        assert "FollowPath" in bt_text

        for unsafe_motion_recovery in ("<Spin", "<BackUp", "RecoveryNode", "ClearEntireCostmap"):
            assert unsafe_motion_recovery not in bt_text


def test_travel_uses_smooth_low_load_mppi_controller_profile():
    text = NAV2_TRAVEL.read_text(encoding="utf-8")
    controller_text = text.split("# 局部代价地图参数块", maxsplit=1)[0]
    behavior_text = text.split("# Behavior Server 节点参数块", maxsplit=1)[1]

    assert "controller_frequency: 20.0" in controller_text
    assert "controller_frequency: 15.0" not in controller_text
    assert "required_movement_radius: 0.10" in controller_text
    assert "movement_time_allowance: 15.0" in controller_text
    assert "yaw_goal_tolerance: 0.20" in controller_text
    assert 'plugin: "nav2_rotation_shim_controller::RotationShimController"' in controller_text
    assert 'primary_controller: "nav2_mppi_controller::MPPIController"' in controller_text
    assert "angular_dist_threshold: 0.35" in controller_text
    assert "rotate_to_heading_angular_vel: 0.30" in controller_text
    assert "max_angular_accel: 0.80" in controller_text
    assert "rotate_to_goal_heading: true" in controller_text
    assert "time_steps: 48" in controller_text
    assert "model_dt: 0.05" in controller_text
    assert "batch_size: 300" in controller_text
    assert "vx_std: 0.16" in controller_text
    assert "wz_std: 0.14" in controller_text
    assert "vx_max: 0.30" in controller_text
    assert "vx_min: 0.0" in controller_text
    assert "vy_max: 0.0" in controller_text
    assert "wz_max: 0.65" in controller_text
    assert "ax_max: 0.40" in controller_text
    assert "ax_min: -0.8" in controller_text
    assert "az_max: 2.0" in controller_text
    assert "regenerate_noises: true" in controller_text
    assert "PathAlignCritic:" in controller_text
    assert "offset_from_furthest: 6" in controller_text
    assert "PathFollowCritic:" in controller_text
    assert "cost_weight: 16.0" in controller_text
    assert '        - "VelocityDeadbandCritic"' in controller_text
    assert "deadband_velocities: [0.14, 0.0, 0.18]" in controller_text
    assert 'behavior_plugins: ["wait"]' in behavior_text

    assert 'plugin: "dwb_core::DWBLocalPlanner"' not in controller_text
    assert "vx_samples:" not in controller_text
    assert "BaseObstacle.scale:" not in controller_text
    assert "batch_size: 1000" not in controller_text
    assert "vx_max: 1.0" not in controller_text


def test_travel_declares_runtime_controller_plugins():
    package_text = BRINGUP_PACKAGE.read_text(encoding="utf-8")

    assert "<exec_depend>nav2_mppi_controller</exec_depend>" in package_text
    assert "<exec_depend>nav2_rotation_shim_controller</exec_depend>" in package_text


def test_travel_local_costmap_uses_stable_field_runtime_rates():
    text = NAV2_TRAVEL.read_text(encoding="utf-8")
    local_costmap_text = text.split("# 全局代价地图参数块", maxsplit=1)[0]

    assert "update_frequency: 15.0" in local_costmap_text
    assert "publish_frequency: 5.0" in local_costmap_text
    assert "width: 6" in local_costmap_text
    assert "height: 6" in local_costmap_text
    assert "resolution: 0.05" in local_costmap_text
    assert "min_obstacle_height: 0.12" in local_costmap_text
    assert "max_obstacle_height: 1.0" in local_costmap_text
    assert "obstacle_max_range: 3.5" in local_costmap_text
    assert "obstacle_min_range: 0.45" in local_costmap_text
    assert "raytrace_max_range: 4.0" in local_costmap_text
    assert "raytrace_min_range: 0.1" in local_costmap_text
    assert "observation_persistence: 0.0" in local_costmap_text
    assert "expected_update_rate: 0.2" in local_costmap_text
    assert "update_frequency: 40.0" not in local_costmap_text
    assert "publish_frequency: 40.0" not in local_costmap_text
    assert "width: 15" not in local_costmap_text
    assert "height: 15" not in local_costmap_text
    assert "resolution: 0.02" not in local_costmap_text


def test_travel_global_costmap_uses_lower_static_map_inflation_than_local():
    text = NAV2_TRAVEL.read_text(encoding="utf-8")
    local_costmap_text = text.split("# 全局代价地图参数块", maxsplit=1)[0]
    global_costmap_text = text.split("# 全局代价地图参数块", maxsplit=1)[1]

    footprint = 'footprint: "[[0.35, 0.275], [0.35, -0.275], [-0.35, -0.275], [-0.35, 0.275]]"'
    assert footprint in local_costmap_text
    assert footprint in global_costmap_text
    assert "inflation_radius: 0.40" in local_costmap_text
    assert "inflation_radius: 0.25" in global_costmap_text
    assert "inflation_radius: 0.4" not in global_costmap_text
    assert "consider_footprint: true" in local_costmap_text


def test_travel_uses_measured_polygon_with_local_safety_inflation():
    config = _nav2_travel_yaml()
    local = config["local_costmap"]["local_costmap"]["ros__parameters"]

    assert local["footprint"] == "[[0.35, 0.275], [0.35, -0.275], [-0.35, -0.275], [-0.35, 0.275]]"
    assert local["inflation_layer"]["inflation_radius"] >= 0.4


def test_travel_loads_map_bundle_and_safety_output_chain():
    launch_text = _travel_launch_text()
    collision_text = Path("src/bringup/config/collision_monitor_travel.yaml").read_text(
        encoding="utf-8"
    )

    assert '"map_bundle"' in launch_text
    assert "_resolve_map_bundle" in launch_text
    assert '"alignment_file": LaunchConfiguration("map_alignment_file")' in launch_text
    assert '"descriptor_index": LaunchConfiguration("descriptor_index")' in launch_text
    assert 'executable="localization_cmd_gate.py"' in launch_text
    assert 'package="nav2_collision_monitor"' in launch_text
    assert 'remappings=[("/cmd_vel", "/cmd_vel_safe")]' in launch_text
    assert '"enable_acceleration_limit": True' in launch_text
    assert '"max_linear_acceleration": 0.30' in launch_text
    assert '"max_angular_acceleration": 0.80' in launch_text
    assert 'cmd_vel_in_topic: /cmd_vel_localized' in collision_text
    assert 'cmd_vel_out_topic: /cmd_vel_safe' in collision_text
    assert 'topic: /fastlio2/body_cloud_nav2' in collision_text

    collision_config = yaml.safe_load(collision_text)["collision_monitor"]["ros__parameters"]
    stop = collision_config["PolygonStop"]
    slow = collision_config["PolygonSlow"]
    assert stop["points"] == [0.40, 0.32, 0.40, -0.32, -0.40, -0.32, -0.40, 0.32]
    assert stop["max_points"] == 10
    assert slow["points"] == [0.85, 0.30, 0.85, -0.30, 0.45, -0.30, 0.45, 0.30]
    assert slow["max_points"] == 10
    assert slow["slowdown_ratio"] == 0.60
    assert min(slow["points"][::2]) > 0.35

    gate_text = LOCALIZATION_CMD_GATE.read_text(encoding="utf-8")
    assert '"/fastlio2/body_cloud_nav2"' in gate_text
    assert 'declare_parameter("obstacle_timeout_s", 0.5)' in gate_text
    assert 'declare_parameter("cmd_timeout_s", 0.25)' in gate_text
    assert "self.last_obstacle_time" in gate_text
    assert "self.last_cmd_time" in gate_text
    assert "self.publisher.publish(Twist())" in gate_text


def test_travel_exposes_foxglove_control_surface():
    launch_text = _travel_launch_text()
    wrapper_text = LAUNCH_WRAPPER.read_text(encoding="utf-8")
    package_text = BRINGUP_PACKAGE.read_text(encoding="utf-8")
    cmake_text = BRINGUP_CMAKE.read_text(encoding="utf-8")

    assert 'default_value=os.environ.get("FYP_USE_FOXGLOVE", "true")' in launch_text
    assert 'package="foxglove_bridge"' in launch_text
    assert 'executable="foxglove_navigation_adapter_node"' in launch_text
    assert "FYP_USE_FOXGLOVE" in wrapper_text
    assert "<exec_depend>foxglove_bridge</exec_depend>" in package_text
    assert "install(DIRECTORY foxglove/" in cmake_text


def test_travel_follows_collision_checked_navfn_path_without_unsafe_recovery():
    for bt_file in (TRAVEL_FAIL_STOP_BT, TRAVEL_THROUGH_POSES_FAIL_STOP_BT):
        text = bt_file.read_text(encoding="utf-8")
        assert "ComputePath" in text
        assert "SmoothPath" not in text
        assert "FollowPath" in text


def test_travel_keeps_velocity_smoothing_for_indoor_navigation():
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

    assert "max_velocity: [0.30, 0.0, 0.65]" in velocity_text
    assert "min_velocity: [0.0, 0.0, -0.65]" in velocity_text
    assert "max_accel: [0.40, 0.0, 1.4]" in velocity_text
    assert "max_decel: [-0.8, 0.0, -1.8]" in velocity_text


def test_bringup_installs_travel_runtime_helper_nodes():
    cmake_text = BRINGUP_CMAKE.read_text(encoding="utf-8")
    package_text = BRINGUP_PACKAGE.read_text(encoding="utf-8")

    assert "scripts/initialpose_relocalize_bridge.py" in cmake_text
    assert "scripts/nav2_cloud_retime.py" in cmake_text
    assert "scripts/localization_cmd_gate.py" in cmake_text
    assert LOCALIZATION_CMD_GATE.stat().st_mode & stat.S_IXUSR

    for dependency in ("rclpy", "geometry_msgs", "sensor_msgs", "interface"):
        assert f"<exec_depend>{dependency}</exec_depend>" in package_text
    for dependency in ("nav2_collision_monitor", "indoor_navigation_manager"):
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


def test_localization_cmd_gate_fails_closed_on_missing_or_stale_status():
    text = LOCALIZATION_CMD_GATE.read_text(encoding="utf-8")

    assert "LocalizationStatus.LOCALIZED" in text
    assert "LocalizationStatus.DEGRADED" in text
    assert "msg.sensors_ready" in text
    assert "status_timeout_s" in text
    assert "if not self.is_allowed()" in text
    assert "self.publisher.publish(Twist())" in text


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
