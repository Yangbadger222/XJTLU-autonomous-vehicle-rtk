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
POST_COLLISION_CONDITIONER = Path(
    "src/bringup/scripts/post_collision_cmd_conditioner.py"
)
PRIOR_MAP_TF_AUTHORITY = Path("src/bringup/scripts/prior_map_tf_authority.py")
PRIOR_MAP_TF_MATH = Path("src/bringup/scripts/prior_map_tf_math.py")
TRAVEL_RECOVERY_BT = Path("src/bringup/behavior_trees/travel_nav_to_pose_recovery.xml")
TRAVEL_THROUGH_POSES_RECOVERY_BT = Path(
    "src/bringup/behavior_trees/travel_nav_through_poses_recovery.xml"
)


def _travel_launch_text():
    return TRAVEL_LAUNCH.read_text(encoding="utf-8")


def _nav2_travel_yaml():
    return yaml.safe_load(NAV2_TRAVEL.read_text(encoding="utf-8"))


def test_travel_launch_exposes_prior_map_arguments():
    text = _travel_launch_text()

    for argument_name in ("map_yaml", "pcd_map", "use_rviz", "use_pgo"):
        assert f'"{argument_name}"' in text


def test_travel_records_low_load_navigation_evidence_by_default():
    text = _travel_launch_text()

    assert 'os.environ.get("FYP_TRAVEL_RECORD_BAG", "true")' in text
    assert 'os.environ.get("FYP_TRAVEL_BAG_PROFILE", "lean")' in text
    assert 'os.path.join(bag_session_dir, "travel_bag")' in text
    assert '"/fastlio2/lio_odom"' in text
    assert '"/odom_CBoar"' in text
    assert '"/cmd_vel_nav"' in text
    assert '"/cmd_vel_localized"' in text
    assert '"/cmd_vel_safe"' in text
    assert '"/plan"' in text
    assert '"/local_plan"' in text
    assert '"/local_costmap/costmap"' in text
    assert '"/localizer/status"' in text
    assert '"/localizer/map_to_odom"' in text
    assert '"/amcl_pose"' in text
    assert '"/travel/prior_map_tf/status"' in text
    assert '"/travel/control_gate/status"' in text
    assert '"/foxglove/navigation/status"' in text
    assert '"/chassis/status"' in text
    assert '"/fastlio2/body_cloud"' in text
    assert "_TRAVEL_BAG_DEBUG_TOPICS" in text
    assert "*_travel_bag_topics(bag_profile)" in text


def test_travel_launch_wires_localizer_and_nav2():
    text = _travel_launch_text()

    assert 'package="localizer"' in text
    assert 'executable="localizer_node"' in text
    assert 'executable="initialpose_relocalize_bridge.py"' in text
    assert 'executable="prior_map_tf_authority.py"' in text
    assert '"publish_tf": False' in text
    assert 'SetRemap(src="/initialpose", dst="/amcl/initialpose")' in text
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


def test_travel_nav2_config_reserves_map_to_odom_for_guarded_authority():
    text = NAV2_TRAVEL.read_text(encoding="utf-8")
    local_costmap_text = text.split("# 全局代价地图参数块", maxsplit=1)[0]
    global_costmap_text = text.split("# 全局代价地图参数块", maxsplit=1)[1]

    assert "tf_broadcast: false" in text
    assert "prior_map_tf_authority" in text
    assert "topic: /fastlio2/body_cloud_nav2" in text
    assert "topic: /fastlio2/body_cloud\n" not in text
    assert 'plugins: ["static_layer", "inflation_layer"]' in global_costmap_text
    assert 'plugins: ["obstacle_layer", "inflation_layer"]' in local_costmap_text
    assert "topic: /fastlio2/body_cloud_nav2" in local_costmap_text
    assert "topic: /fastlio2/body_cloud_nav2" not in global_costmap_text
    assert "rolling_window: false" in global_costmap_text
    assert "rolling_window: true" in local_costmap_text

    config = _nav2_travel_yaml()
    amcl = config["amcl"]["ros__parameters"]
    authority = config["prior_map_tf_authority"]["ros__parameters"]
    assert amcl["tf_broadcast"] is False
    assert amcl["max_particles"] == 1000
    assert amcl["min_particles"] == 300
    assert amcl["max_beams"] == 50
    assert amcl["update_min_d"] == 0.05
    assert amcl["update_min_a"] == 0.05
    assert authority["max_target_base_jump_m"] == 0.75
    assert authority["max_translation_step_m"] == 0.03
    assert authority["max_yaw_step_rad"] == 0.01
    assert authority["max_base_step_m"] == 0.04


def test_travel_uses_smoothed_paths_and_bounded_recovery_without_backup():
    launch_text = _travel_launch_text()
    nav2_text = NAV2_TRAVEL.read_text(encoding="utf-8")
    bt_navigator_text = nav2_text.split("# Navigate Through Poses", maxsplit=1)[0]

    assert "travel_nav_to_pose_recovery.xml" in launch_text
    assert "travel_nav_through_poses_recovery.xml" in launch_text
    assert '"default_nav_to_pose_bt_xml": travel_bt_xml' in launch_text
    assert '"default_nav_through_poses_bt_xml": travel_through_poses_bt_xml' in launch_text
    assert "default_nav_to_pose_bt_xml:" in bt_navigator_text
    assert "default_nav_through_poses_bt_xml:" in bt_navigator_text

    bt_expectations = {
        TRAVEL_RECOVERY_BT: "ComputePathToPose",
        TRAVEL_THROUGH_POSES_RECOVERY_BT: "ComputePathThroughPoses",
    }
    for bt_file, planner_node in bt_expectations.items():
        bt_text = bt_file.read_text(encoding="utf-8")
        assert planner_node in bt_text
        assert "SmoothPath" in bt_text
        assert 'smoother_id="savitzky_golay_smoother"' in bt_text
        assert 'check_for_collisions="false"' in bt_text
        assert '<RateController hz="1.0">' in bt_text
        assert "FollowPath" in bt_text
        assert 'RecoveryNode number_of_retries="6"' in bt_text
        assert '<IsStuck/>' in bt_text
        assert '<Spin spin_dist="0.52"/>' in bt_text
        assert "ClearEntireCostmap" in bt_text
        assert '<Wait wait_duration="0.5"/>' in bt_text
        assert "ClearLocalAndReplan" in bt_text
        assert "<BackUp" not in bt_text


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
    assert "angular_dist_threshold: 0.65" in controller_text
    assert "angular_disengage_threshold: 0.35" in controller_text
    assert "rotate_to_heading_angular_vel: 0.24" in controller_text
    assert "max_angular_accel: 0.65" in controller_text
    assert "rotate_to_goal_heading: false" in controller_text
    assert "closed_loop: true" in controller_text
    assert "time_steps: 32" in controller_text
    assert "model_dt: 0.05" in controller_text
    assert "batch_size: 160" in controller_text
    assert 'plugin: "nav2_controller::PoseProgressChecker"' in controller_text
    assert "required_movement_angle: 0.15" in controller_text
    assert "batch_size: 200" not in controller_text
    assert "failure_tolerance: 2.0" in controller_text
    assert "vx_std: 0.16" in controller_text
    assert "wz_std: 0.10" in controller_text
    assert "vx_max: 0.30" in controller_text
    assert "vx_min: 0.0" in controller_text
    assert "vy_max: 0.0" in controller_text
    assert "wz_max: 0.65" in controller_text
    assert "ax_max: 0.40" in controller_text
    assert "ax_min: -0.8" in controller_text
    assert "az_max: 2.0" in controller_text
    assert "open_loop:" not in controller_text
    assert "regenerate_noises: false" in controller_text
    assert "PathAlignCritic:" in controller_text
    assert "cost_weight: 6.0" in controller_text
    assert "trajectory_point_step: 5" in controller_text
    assert "PathFollowCritic:" in controller_text
    assert "cost_weight: 12.0" in controller_text
    assert '        - "VelocityDeadbandCritic"' in controller_text
    assert "deadband_velocities: [0.12, 0.0, 0.08]" in controller_text
    assert '        - "PreferForwardCritic"' not in controller_text
    assert "trajectory_point_step: 2" not in controller_text
    assert 'behavior_plugins: ["spin", "wait"]' in behavior_text
    assert 'plugin: "nav2_behaviors/Spin"' in behavior_text
    assert "max_rotational_vel: 0.50" in behavior_text
    assert "min_rotational_vel: 0.20" in behavior_text
    assert "rotational_acc_lim: 0.80" in behavior_text

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

    assert "update_frequency: 10.0" in local_costmap_text
    assert "publish_frequency: 2.0" in local_costmap_text
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
    assert "inflation_radius: 0.30" in global_costmap_text
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
    assert 'cmd_vel_out_topic: /cmd_vel_safe_raw' in collision_text
    assert 'executable="post_collision_cmd_conditioner.py"' in launch_text
    assert 'executable="prior_map_tf_authority.py"' in launch_text
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
    assert 'declare_parameter("cmd_timeout_s", 0.40)' in gate_text
    assert '"/travel/control_gate/status"' in gate_text
    assert '"/travel/prior_map_tf/status"' in gate_text
    for reason in (
        "LOCALIZATION_BLOCKED",
        "LOCALIZATION_AUTHORITY_TIMEOUT",
        "POINTCLOUD_TIMEOUT",
        "COLLISION_STOP",
        "COMMAND_TIMEOUT",
    ):
        assert reason in gate_text
    assert "self.last_obstacle_time" in gate_text
    assert "self.last_cmd_time" in gate_text
    assert "self.publish_gated(Twist())" in gate_text


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


def test_travel_smooths_navfn_path_without_automatic_backup():
    for bt_file in (TRAVEL_RECOVERY_BT, TRAVEL_THROUGH_POSES_RECOVERY_BT):
        text = bt_file.read_text(encoding="utf-8")
        assert "ComputePath" in text
        assert "SmoothPath" in text
        assert "FollowPath" in text
        assert "<BackUp" not in text


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
    assert "scripts/post_collision_cmd_conditioner.py" in cmake_text
    assert "scripts/prior_map_tf_authority.py" in cmake_text
    assert "scripts/prior_map_tf_math.py" in cmake_text
    assert LOCALIZATION_CMD_GATE.stat().st_mode & stat.S_IXUSR
    assert POST_COLLISION_CONDITIONER.stat().st_mode & stat.S_IXUSR
    assert PRIOR_MAP_TF_AUTHORITY.stat().st_mode & stat.S_IXUSR

    for dependency in (
        "rclpy",
        "geometry_msgs",
        "sensor_msgs",
        "interface",
        "diagnostic_msgs",
        "std_msgs",
        "nav_msgs",
        "tf2_ros",
    ):
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
    assert 'amcl_initialpose_topic", "/amcl/initialpose"' in text
    assert 'msg.header.frame_id.lstrip("/") != "map"' in text
    assert "self.amcl_initialpose_publisher.publish(msg)" in text


def test_nav2_cloud_retime_republishes_pointcloud_with_current_stamp():
    text = NAV2_CLOUD_RETIME.read_text(encoding="utf-8")

    assert "PointCloud2" in text
    assert "cloud_in" in text
    assert "cloud_out" in text
    assert "out.header.stamp = self.get_clock().now().to_msg()" in text


def test_travel_python_nodes_do_not_double_shutdown_ros_context():
    paths = (
        INITIALPOSE_BRIDGE,
        NAV2_CLOUD_RETIME,
        LOCALIZATION_CMD_GATE,
        POST_COLLISION_CONDITIONER,
        PRIOR_MAP_TF_AUTHORITY,
        Path(
            "src/navigation/indoor_navigation_manager/"
            "indoor_navigation_manager/manager_node.py"
        ),
        Path(
            "src/navigation/indoor_navigation_manager/"
            "indoor_navigation_manager/foxglove_adapter_node.py"
        ),
    )
    for path in paths:
        text = path.read_text(encoding="utf-8")
        assert "except KeyboardInterrupt:\n        pass" in text
        assert "except Exception:\n        if rclpy.ok():\n            raise" in text
        assert "if rclpy.ok():\n            rclpy.shutdown()" in text


def test_post_collision_conditioner_turns_stalled_commands_in_place():
    text = POST_COLLISION_CONDITIONER.read_text(encoding="utf-8")

    assert 'declare_parameter("cmd_vel_in", "/cmd_vel_safe_raw")' in text
    assert 'declare_parameter("cmd_vel_out", "/cmd_vel_safe")' in text
    assert 'declare_parameter("reference_cmd_vel", "/cmd_vel_localized")' in text
    assert 'declare_parameter("slowdown_ratio_threshold", 0.80)' in text
    assert 'declare_parameter("linear_deadband", 0.14)' in text
    assert 'declare_parameter("in_place_linear_threshold", 0.02)' in text
    assert 'declare_parameter("turning_angular_threshold", 0.16)' in text
    assert 'declare_parameter("angular_zero_threshold", 0.01)' in text
    assert "def condition_command" in text
    assert "def is_slowdown_output" in text
    assert "slowdown_stalled_turn = stalled_turn and self.is_slowdown_output(msg)" in text
    assert "math.hypot(msg.linear.x, msg.linear.y)" in text
    assert "out.linear.x = 0.0" in text
    assert "out.angular.z =" not in text
    assert "min_in_place_angular_speed" not in text


def test_localization_cmd_gate_fails_closed_on_missing_or_stale_status():
    text = LOCALIZATION_CMD_GATE.read_text(encoding="utf-8")

    assert "LocalizationStatus.LOCALIZED" in text
    assert "LocalizationStatus.DEGRADED" in text
    assert "msg.sensors_ready" in text
    assert "status_timeout_s" in text
    assert "authority_timeout_s" in text
    assert "self.authority_active" in text
    assert "if not self.is_allowed()" in text
    assert "self.publish_gated(Twist())" in text


def test_prior_map_tf_authority_gates_amcl_and_is_the_only_travel_tf_owner():
    launch_text = _travel_launch_text()
    authority_text = PRIOR_MAP_TF_AUTHORITY.read_text(encoding="utf-8")

    assert '"publish_tf": False' in launch_text
    assert "tf_broadcast: false" in NAV2_TRAVEL.read_text(encoding="utf-8")
    assert "TransformBroadcaster" in authority_text
    assert "bounded_map_to_odom_update" in authority_text
    assert "AMCL_REJECTED_COVARIANCE" in authority_text
    assert "AMCL_REJECTED_ODOM_SKEW" in authority_text
    assert "AMCL_REJECTED_TARGET_JUMP" in authority_text
    assert '"/travel/prior_map_tf/status"' in authority_text


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
        assert "[p]ost_collision_cmd_conditioner" in text
        assert "[p]rior_map_tf_authority" in text
        assert "[j]oint_state_publisher" in text
