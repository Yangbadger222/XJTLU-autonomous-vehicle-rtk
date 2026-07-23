from pathlib import Path
import re
import xml.etree.ElementTree as ET

import yaml


EXPLORE_LAUNCH = Path("src/bringup/launch/system_explore.launch.py")
CORRIDOR_LAUNCH = Path("src/bringup/launch/system_gps_corridor.launch.py")
CORRIDOR_NO_RECOVERY_BT = Path(
    "src/bringup/behavior_trees/navigate_to_pose_w_replanning_5hz_no_motion_recovery.xml"
)
CORRIDOR_NO_RECOVERY_THROUGH_BT = Path(
    "src/bringup/behavior_trees/navigate_through_poses_w_replanning_5hz_no_motion_recovery.xml"
)
LIVOX_LDDC = Path("src/sensor_drivers/livox_ros_driver2/src/lddc.cpp")
FASTLIO_NODE = Path("src/perception/fastlio2/src/lio_node.cpp")
PGO_NODE = Path("src/perception/pgo_gps_fusion/src/pgo_node.cpp")
PGO_LAUNCH = Path("src/perception/pgo_gps_fusion/launch/pgo_launch.py")
MASTER_PARAMS = Path("src/bringup/config/master_params.yaml")
UM982_RTK_PARAMS = Path(
    "src/sensor_drivers/gnss/um982_rtk_driver/config/um982_rtk.yaml"
)
NMEA_SERIAL_PARAMS = Path(
    "src/sensor_drivers/gnss/nmea_navsat_driver/config/nmea_serial_driver.yaml"
)
FASTLIO_LEGACY_PARAMS = Path("src/perception/fastlio2/config/lio.yaml")
NAV2_EXPLORE_PARAMS = Path("src/bringup/config/nav2_explore.yaml")
CORRIDOR_NAV2_PARAMS = Path("src/bringup/config/nav2_corridor_rtk.yaml")
PGO_CORRIDOR_PARAMS = Path("src/bringup/config/pgo_corridor_no_gps.yaml")
PGO_CORRIDOR_LEGACY_PARAMS = Path("src/bringup/config/pgo_corridor_no_tf.yaml")
ROUTE_RUNNER = Path(
    "src/navigation/gps_waypoint_dispatcher/gps_waypoint_dispatcher/gps_route_runner_node.py"
)
MAKEFILE = Path("Makefile")
LAUNCH_WITH_LOGS = Path("scripts/launch_with_logs.sh")


def test_explore_launch_exposes_nav2_params_file_for_mode_specific_profiles():
    text = EXPLORE_LAUNCH.read_text(encoding="utf-8")

    assert '"nav2_params_file"' in text
    assert 'source_file=LaunchConfiguration("nav2_params_file")' in text


def test_corridor_launch_uses_corner_turn_nav2_profile_for_rtk_corridor():
    text = CORRIDOR_LAUNCH.read_text(encoding="utf-8")

    assert "nav2_corridor_rtk.yaml" in text
    assert "corridor_nav2_params = _make_corridor_nav2_params" in text
    assert "bt_params = data['bt_navigator']['ros__parameters']" in text
    assert "bt_params['bt_loop_duration'] = 50" in text
    assert "bt_params['default_server_timeout'] = 1000" in text
    assert "controller_params['controller_frequency'] = 20.0" in text
    assert "controller_params['controller_frequency'] = 15.0" not in text
    assert "controller_params['failure_tolerance'] = 1.5" in text
    assert "controller_params['progress_checker']['required_movement_radius'] = 0.10" in text
    assert "controller_params['progress_checker']['movement_time_allowance'] = 15.0" in text
    assert "follow_path['vx_std'] = 0.20" in text
    assert "follow_path['wz_std'] = 0.15" in text
    assert "follow_path['vx_max'] = 0.85" in text
    assert "follow_path['wz_max'] = 0.70" in text
    assert "follow_path['ax_max'] = 0.85" in text
    assert "follow_path['ax_min'] = -1.2" in text
    assert "follow_path['az_max'] = 1.4" in text
    assert "follow_path['temperature'] = 0.45" in text
    assert "follow_path['regenerate_noises'] = True" in text
    assert "controller_params['general_goal_checker']['stateful'] = False" in text
    assert "smoother_params['max_velocity'] = [0.85, 0.0, 0.70]" in text
    assert "smoother_params['min_velocity'] = [0.0, 0.0, -0.70]" in text
    assert "smoother_params['max_accel'] = [0.85, 0.0, 1.4]" in text
    assert "smoother_params['max_decel'] = [-1.2, 0.0, -1.8]" in text
    assert "behavior_params['behavior_plugins'] = ['wait']" in text
    assert "'nav2_params_file': corridor_nav2_params" in text
    assert "'terminal_stop_hold_s': 1.2" in text


def test_corridor_nav2_global_costmap_is_route_planning_only():
    config = yaml.safe_load(CORRIDOR_NAV2_PARAMS.read_text(encoding="utf-8"))
    global_costmap = config["global_costmap"]["global_costmap"]["ros__parameters"]
    planner = config["planner_server"]["ros__parameters"]["GridBased"]

    assert global_costmap["track_unknown_space"] is False
    assert global_costmap["plugins"] == ["inflation_layer"]
    assert global_costmap["robot_radius"] <= 0.25
    assert global_costmap["inflation_layer"]["inflation_radius"] <= 0.35
    assert planner["allow_unknown"] is True
    assert planner["tolerance"] >= 1.0


def test_corridor_nav2_keeps_live_obstacles_in_local_costmap():
    config = yaml.safe_load(CORRIDOR_NAV2_PARAMS.read_text(encoding="utf-8"))
    local_costmap = config["local_costmap"]["local_costmap"]["ros__parameters"]
    stvl = local_costmap["stvl_layer"]

    assert "stvl_layer" in local_costmap["plugins"]
    assert "frc_layer" in local_costmap["plugins"]
    assert local_costmap["robot_radius"] >= 0.38
    assert stvl["enabled"] is True
    assert local_costmap["inflation_layer"]["inflation_radius"] >= 0.4
    assert stvl["pointcloud_mark"]["topic"] == "/fastlio2/body_cloud_nav2_obstacles"
    assert stvl["pointcloud_clear"]["topic"] == "/fastlio2/body_cloud_nav2_obstacles"
    assert stvl["pointcloud_mark"]["min_obstacle_height"] == 0.08
    assert stvl["pointcloud_mark"]["max_obstacle_height"] >= 1.20
    assert stvl["pointcloud_clear"]["min_z"] <= -0.20
    assert stvl["pointcloud_clear"]["max_z"] >= 1.20


def test_corridor_mppi_uses_measured_odometry_feedback():
    config = yaml.safe_load(CORRIDOR_NAV2_PARAMS.read_text(encoding="utf-8"))
    follow_path = config["controller_server"]["ros__parameters"]["FollowPath"]
    launch_text = CORRIDOR_LAUNCH.read_text(encoding="utf-8")

    assert follow_path["open_loop"] is False
    assert "follow_path['open_loop'] = False" in launch_text


def test_corridor_local_costmap_is_near_field_for_mppi_stability():
    config = yaml.safe_load(CORRIDOR_NAV2_PARAMS.read_text(encoding="utf-8"))
    controller = config["controller_server"]["ros__parameters"]
    local_costmap = config["local_costmap"]["local_costmap"]["ros__parameters"]
    stvl = local_costmap["stvl_layer"]
    cost_critic = controller["FollowPath"]["CostCritic"]

    assert controller["failure_tolerance"] >= 1.5
    assert local_costmap["width"] <= 12
    assert local_costmap["height"] <= 12
    assert stvl["pointcloud_mark"]["obstacle_range"] <= 5.0
    assert cost_critic["cost_weight"] >= 7.0


def test_corridor_route_runner_defaults_to_short_rtk_subgoals():
    text = ROUTE_RUNNER.read_text(encoding="utf-8")

    assert 'self._route.get("segment_length_m", 5.0)' in text
    assert 'self._route.get("segment_length_m", 30.0)' not in text


def test_corridor_uses_no_motion_recovery_bt_for_rtk_acceptance():
    explore_text = EXPLORE_LAUNCH.read_text(encoding="utf-8")
    corridor_text = CORRIDOR_LAUNCH.read_text(encoding="utf-8")
    to_pose_tree = ET.parse(CORRIDOR_NO_RECOVERY_BT)
    to_pose_root = to_pose_tree.getroot()
    through_poses_tree = ET.parse(CORRIDOR_NO_RECOVERY_THROUGH_BT)
    through_poses_root = through_poses_tree.getroot()

    assert "nav_to_pose_bt_xml" in explore_text
    assert "nav_through_poses_bt_xml" in explore_text
    assert "corridor_no_recovery_bt_xml" in corridor_text
    assert "corridor_no_recovery_through_poses_bt_xml" in corridor_text
    assert "'nav_to_pose_bt_xml': corridor_no_recovery_bt_xml" in corridor_text
    assert "'nav_through_poses_bt_xml': corridor_no_recovery_through_poses_bt_xml" in corridor_text
    assert to_pose_root.findall(".//Spin") == []
    assert to_pose_root.findall(".//BackUp") == []
    assert to_pose_root.find(".//FollowPath") is not None
    assert through_poses_root.findall(".//Spin") == []
    assert through_poses_root.findall(".//BackUp") == []
    assert through_poses_root.find(".//ComputePathThroughPoses") is not None
    rate_controller = to_pose_root.find(".//RateController")
    assert rate_controller is not None
    assert rate_controller.attrib["hz"] == "5.0"


def test_corridor_relies_on_explore_for_robot_description_once():
    explore_text = EXPLORE_LAUNCH.read_text(encoding="utf-8")
    corridor_text = CORRIDOR_LAUNCH.read_text(encoding="utf-8")

    assert "robot_description.launch.py" in explore_text
    assert "system_explore.launch.py" in corridor_text
    assert "robot_description.launch.py" not in corridor_text


def test_nav2_profiles_expose_both_bt_xml_rewrite_slots():
    for params_file in [NAV2_EXPLORE_PARAMS, CORRIDOR_NAV2_PARAMS]:
        params = yaml.safe_load(params_file.read_text(encoding="utf-8"))
        bt_params = params["bt_navigator"]["ros__parameters"]

        assert bt_params["default_nav_to_pose_bt_xml"] == ""
        assert bt_params["default_nav_through_poses_bt_xml"] == ""


def test_corridor_bag_keeps_fgo_shadow_topics_without_raw_lidar():
    text = CORRIDOR_LAUNCH.read_text(encoding="utf-8")

    base_topics = re.search(
        r"_CORRIDOR_BAG_BASE_TOPICS = \[(?P<topics>.*?)\]",
        text,
        re.DOTALL,
    ).group("topics")
    debug_topics = re.search(
        r"_CORRIDOR_BAG_DEBUG_TOPICS = \[(?P<topics>.*?)\]",
        text,
        re.DOTALL,
    ).group("topics")

    assert "FYP_CORRIDOR_BAG_PROFILE" in text
    assert "def _corridor_bag_topics" in text
    assert "'/fastlio2/lio_odom'," in base_topics
    assert "'/fastlio2/degeneracy'," in base_topics
    assert "'/odom_CBoar'," in base_topics
    assert "'/livox/imu'," in base_topics
    assert "'/rtk_fgo/odom'," in base_topics
    assert "'/rtk_fgo/status'," in base_topics
    assert "'/rtk_fgo/factor_diagnostics'," in base_topics
    assert "'/livox/lidar'," not in base_topics
    assert "'/fastlio2/body_cloud'," not in base_topics
    assert "'/livox/lidar'," in debug_topics
    assert "'/fastlio2/body_cloud'," in debug_topics
    assert "'/fastlio2/body_cloud_nav2_obstacles'," in debug_topics


def test_corridor_launch_keeps_fgo_shadow_opt_in_without_owning_tf_or_nav2():
    text = CORRIDOR_LAUNCH.read_text(encoding="utf-8")

    assert "rtk_fgo_params_file" in text
    assert "enable_fgo_shadow_arg" in text
    assert "FYP_CORRIDOR_ENABLE_FGO_SHADOW" in text
    assert "os.environ.get('FYP_CORRIDOR_ENABLE_FGO_SHADOW', 'false')" in text
    assert "rtk_fgo_localizer" in text
    assert "rtk_fgo_node" in text
    assert "'publish_tf': False" in text
    assert "'nav2_use_fgo': False" in text
    assert "delayed_fgo_shadow" in text
    assert "IfCondition(LaunchConfiguration('enable_fgo_shadow'))" in text


def test_corridor_uses_rtk_authoritative_map_odom_owner():
    explore_text = EXPLORE_LAUNCH.read_text(encoding="utf-8")
    corridor_text = CORRIDOR_LAUNCH.read_text(encoding="utf-8")
    pgo_launch_text = PGO_LAUNCH.read_text(encoding="utf-8")
    pgo_override_text = PGO_CORRIDOR_PARAMS.read_text(encoding="utf-8")
    pgo_legacy_override_text = PGO_CORRIDOR_LEGACY_PARAMS.read_text(encoding="utf-8")
    master_params_text = MASTER_PARAMS.read_text(encoding="utf-8")

    assert "rtk_map_odom_corrector_node" in corridor_text
    assert "rtk_map_odom_corrector" in corridor_text
    assert "'/localization_authority/mode'," in corridor_text
    assert "'/localization_authority/status'," in corridor_text
    assert "'/localization_authority/diagnostics'," in corridor_text
    assert '"pgo_config": LaunchConfiguration("pgo_config_file")' in explore_text
    assert "'pgo_config_file': pgo_corridor_config_file" in corridor_text
    assert 'pgo_config = LaunchConfiguration("pgo_config").perform(context).strip()' in pgo_launch_text
    assert 'pgo_params.append({"config_path": legacy_pgo_config})' in pgo_launch_text
    assert "extra_params_file = LaunchConfiguration(\"extra_params_file\")" in pgo_launch_text
    assert "pgo_params.append(extra_params_file)" in pgo_launch_text
    assert "publish_tf: false" in pgo_override_text
    assert "publish_tf: false" in pgo_legacy_override_text
    assert "enable: false" in pgo_legacy_override_text
    assert '"gps.enable": false' in pgo_override_text
    assert "base_frame: base_footprint" in master_params_text
    assert "observation_fifo_capacity: 10" in master_params_text
    assert "max_pending_observation_s: 0.30" in master_params_text
    assert "fix_quality_wait_s: 0.25" in master_params_text
    assert "heading_quality_wait_s: 0.30" in master_params_text
    assert "max_translation_rate_mps: 0.20" in master_params_text
    assert "max_yaw_rate_degps: 2.0" in master_params_text
    assert "heading_locked_innovation_deg: 15.0" in master_params_text
    assert "position_locked_innovation_m: 1.0" in master_params_text
    assert "backlog_translation_m: 0.50" in master_params_text
    assert "fault_translation_m: 2.0" in master_params_text


def test_corridor_command_topology_has_one_guarded_cmd_vel_publisher():
    explore_text = EXPLORE_LAUNCH.read_text(encoding="utf-8")
    corridor_text = CORRIDOR_LAUNCH.read_text(encoding="utf-8")
    runner_text = ROUTE_RUNNER.read_text(encoding="utf-8")

    assert '"guarded_cmd_vel"' in explore_text
    assert 'src="/cmd_vel"' in explore_text
    assert 'dst="/cmd_vel_guarded"' in explore_text
    assert "actions=[nav2_launch]" in explore_text
    assert "'guarded_cmd_vel': 'true'" in corridor_text
    assert "corridor_cmd_vel_guard_node" in corridor_text
    assert "on_exit=Shutdown(reason='corridor_cmd_vel_guard exited')" in corridor_text
    assert "create_publisher(Twist" not in runner_text
    assert "'/cmd_vel_nav'," in corridor_text
    assert "'/cmd_vel_guarded'," in corridor_text
    assert "'/localization_authority/motion_allowed'," in corridor_text
    assert "'/gps_corridor/stop_override'," in corridor_text


def test_corridor_guard_and_hold_defaults_match_safety_spec():
    params = yaml.safe_load(MASTER_PARAMS.read_text(encoding="utf-8"))
    guard = params["/corridor_cmd_vel_guard"]["ros__parameters"]
    runner = params["/gps_route_runner"]["ros__parameters"]

    assert guard["input_topic"] == "/cmd_vel"
    assert guard["output_topic"] == "/cmd_vel_guarded"
    assert guard["straight_max_mps"] == 0.85
    assert guard["turn_product_limit"] == 0.25
    assert guard["command_timeout_s"] == 0.25
    assert guard["heartbeat_timeout_s"] == 0.50
    assert runner["cancel_ack_timeout_s"] == 2.0
    assert runner["authority_ready_confirmation_s"] == 1.0
    assert runner["global_hold_timeout_s"] == 15.0


def test_runtime_cleanup_kills_rtk_authoritative_map_odom_owner():
    makefile_text = MAKEFILE.read_text(encoding="utf-8")
    launch_script_text = LAUNCH_WITH_LOGS.read_text(encoding="utf-8")

    required_cleanup_patterns = [
        "[r]tk_map_odom_corrector",
        "[c]orridor_cmd_vel_guard",
        "[s]erial_reader_node",
        "[j]oint_state_publisher",
    ]
    for pattern in required_cleanup_patterns:
        assert pattern in makefile_text
        assert pattern in launch_script_text


def test_runtime_cleanup_kills_stateful_navigation_mode_nodes():
    makefile_text = MAKEFILE.read_text(encoding="utf-8")
    launch_script_text = LAUNCH_WITH_LOGS.read_text(encoding="utf-8")

    required_cleanup_patterns = [
        "[g]ps_anchor_localizer",
        "[r]oute_server",
        "[g]oal_manager_node",
        "[a]sync_slam_toolbox_node",
        "[m]ap_saver_server",
    ]
    for pattern in required_cleanup_patterns:
        assert pattern in makefile_text
        assert pattern in launch_script_text


def test_livox_packet_logging_is_explicitly_opt_in():
    text = LIVOX_LDDC.read_text(encoding="utf-8")

    assert "LIVOX_VERBOSE_PACKET_LOGS" in text
    assert "livoxVerbosePacketLogsEnabled()" in text
    assert "enabling by default" not in text
    assert "Default to quiet" in text
    assert text.count("if (livoxVerbosePacketLogsEnabled())") >= 3


def test_corridor_runtime_rejects_low_imu_fastlio_packages():
    lio_text = FASTLIO_NODE.read_text(encoding="utf-8")
    params_text = MASTER_PARAMS.read_text(encoding="utf-8")

    assert "min_imu_samples_per_lidar" in lio_text
    assert "Dropping LIDAR package with only" in lio_text
    assert "m_builder->process(m_package)" in lio_text
    assert lio_text.index("Dropping LIDAR package with only") < lio_text.index(
        "Processing sync package"
    )
    assert lio_text.index("Dropping LIDAR package with only") < lio_text.index(
        "m_builder->process(m_package)"
    )
    assert "min_imu_samples_per_lidar: 3" in params_text


def test_fastlio_outdoor_profile_keeps_enough_lidar_structure():
    master_params = yaml.safe_load(MASTER_PARAMS.read_text(encoding="utf-8"))
    profiles = [
        master_params["/fastlio2"]["lio_node"]["ros__parameters"],
        yaml.safe_load(FASTLIO_LEGACY_PARAMS.read_text(encoding="utf-8")),
    ]

    for lio_params in profiles:
        assert lio_params["lidar_filter_num"] <= 4
        assert lio_params["lidar_max_range"] >= 25.0


def test_um982_and_chassis_serial_links_keep_their_verified_baud_rates():
    master_params = yaml.safe_load(MASTER_PARAMS.read_text(encoding="utf-8"))
    standalone_rtk = yaml.safe_load(UM982_RTK_PARAMS.read_text(encoding="utf-8"))
    legacy_nmea = yaml.safe_load(NMEA_SERIAL_PARAMS.read_text(encoding="utf-8"))

    assert master_params["/um982_rtk_driver"]["ros__parameters"]["baud"] == 115200
    assert master_params["/nmea_navsat_driver"]["ros__parameters"]["baud"] == 115200
    assert standalone_rtk["um982_rtk_driver"]["ros__parameters"]["baud"] == 115200
    assert legacy_nmea["nmea_navsat_driver"]["ros__parameters"]["baud"] == 115200
    assert master_params["/serial_twistctl_node"]["ros__parameters"]["baudrate"] == 115200
    assert master_params["/serial_reader_node"]["ros__parameters"]["baud"] == 115200


def test_fastlio_publishes_a_separate_wide_nav2_obstacle_cloud():
    master_params = yaml.safe_load(MASTER_PARAMS.read_text(encoding="utf-8"))
    lio_params = master_params["/fastlio2"]["lio_node"]["ros__parameters"]
    legacy_text = FASTLIO_LEGACY_PARAMS.read_text(encoding="utf-8")
    lio_text = FASTLIO_NODE.read_text(encoding="utf-8")

    assert lio_params["publish_cloud_height_filter_enabled"] is True
    assert lio_params["publish_cloud_max_z"] <= 0.30
    assert lio_params["nav2_obstacle_cloud_enabled"] is True
    assert lio_params["nav2_obstacle_cloud_min_z"] == 0.08
    assert lio_params["nav2_obstacle_cloud_max_z"] >= 1.20
    assert "nav2_obstacle_cloud_max_z: 1.20" in legacy_text
    assert 'create_publisher<sensor_msgs::msg::PointCloud2>("body_cloud_nav2_obstacles"' in lio_text
    assert "publishNav2ObstacleCloud(body_cloud, world_cloud" in lio_text


def test_fastlio_rejects_imu_only_prediction_when_lidar_update_is_invalid():
    map_builder_text = Path("src/perception/fastlio2/src/map_builder/map_builder.cpp").read_text(
        encoding="utf-8"
    )
    lidar_text = Path("src/perception/fastlio2/src/map_builder/lidar_processor.cpp").read_text(
        encoding="utf-8"
    )
    ieskf_text = Path("src/perception/fastlio2/src/map_builder/ieskf.cpp").read_text(
        encoding="utf-8"
    )
    lio_text = FASTLIO_NODE.read_text(encoding="utf-8")

    assert "State state_before_prediction = m_kf->x();" in map_builder_text
    assert "m_kf->x() = state_before_prediction;" in map_builder_text
    assert "bool LidarProcessor::process" in lidar_text
    assert "if (!update_valid)" in lidar_text
    assert "return have_valid_measurement;" in ieskf_text
    assert "if (!process_accepted)" in lio_text
    assert "FAST-LIO2 rejected package without valid LiDAR correction" in lio_text


def test_corridor_watchdogs_use_stamped_local_and_global_rate_thresholds():
    params = yaml.safe_load(MASTER_PARAMS.read_text(encoding="utf-8"))
    runner = params["/gps_route_runner"]["ros__parameters"]

    assert runner["local_rate_abort_mps"] == 3.0
    assert runner["local_yaw_rate_abort_radps"] == 3.0
    assert runner["local_rate_abort_count"] == 3
    assert runner["local_catastrophic_rate_mps"] == 10.0
    assert runner["local_catastrophic_yaw_rate_radps"] == 10.0
    assert runner["global_correction_rate_mps"] == 0.50
    assert runner["global_correction_yaw_rate_degps"] == 5.0


def test_pgo_and_fastlio_logging_default_to_quiet_when_switch_is_missing():
    pgo_text = PGO_NODE.read_text(encoding="utf-8")
    lio_text = FASTLIO_NODE.read_text(encoding="utf-8")

    assert "enabling by default" not in pgo_text
    assert "enabling by default" not in lio_text
    assert "disabling by default" in pgo_text
    assert "disabling by default" in lio_text
    assert "PGO_VERBOSE_DIAG" in pgo_text
    assert pgo_text.count("if (pgoVerboseDiagEnabled())") >= 6
    assert "RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, \"Received synced cloud and odom" in pgo_text


def test_pgo_rebroadcasts_last_map_odom_tf_when_lio_has_gaps():
    pgo_text = PGO_NODE.read_text(encoding="utf-8")

    assert "last_tf_valid" in pgo_text
    assert "publishLastTfWithCurrentTime()" in pgo_text
    assert "m_state.last_tf_rotation = q;" in pgo_text
    assert "m_state.last_tf_translation = t;" in pgo_text
    assert "m_state.cloud_buffer.empty()" in pgo_text
    assert "publishLastTfWithCurrentTime();" in pgo_text
