from pathlib import Path
import re
import xml.etree.ElementTree as ET


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
MASTER_PARAMS = Path("src/bringup/config/master_params.yaml")


def test_explore_launch_exposes_nav2_params_file_for_mode_specific_profiles():
    text = EXPLORE_LAUNCH.read_text(encoding="utf-8")

    assert '"nav2_params_file"' in text
    assert 'source_file=LaunchConfiguration("nav2_params_file")' in text


def test_corridor_launch_uses_slow_nav2_rewrites_for_rtk_acceptance():
    text = CORRIDOR_LAUNCH.read_text(encoding="utf-8")

    assert "corridor_nav2_params = _make_corridor_nav2_params" in text
    assert "bt_params = data['bt_navigator']['ros__parameters']" in text
    assert "bt_params['bt_loop_duration'] = 50" in text
    assert "bt_params['default_server_timeout'] = 1000" in text
    assert "controller_params['controller_frequency'] = 20.0" in text
    assert "controller_params['controller_frequency'] = 15.0" not in text
    assert "follow_path['vx_max'] = 0.45" in text
    assert "follow_path['wz_max'] = 0.65" in text
    assert "follow_path['ax_max'] = 0.45" in text
    assert "controller_params['general_goal_checker']['stateful'] = False" in text
    assert "smoother_params['max_velocity'] = [0.45, 0.0, 0.65]" in text
    assert "smoother_params['max_decel'] = [-0.8, 0.0, -1.8]" in text
    assert "behavior_params['behavior_plugins'] = ['wait']" in text
    assert "'nav2_params_file': corridor_nav2_params" in text


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


def test_corridor_bag_defaults_to_lean_profile_with_debug_raw_topics_opt_in():
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
    assert "'/livox/lidar'," not in base_topics
    assert "'/livox/imu'," not in base_topics
    assert "'/fastlio2/body_cloud'," not in base_topics
    assert "'/livox/lidar'," in debug_topics
    assert "'/livox/imu'," in debug_topics
    assert "'/fastlio2/body_cloud'," in debug_topics


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


def test_corridor_tf_watchdog_allows_short_fastlio_gaps():
    params_text = MASTER_PARAMS.read_text(encoding="utf-8")

    assert "odom_watchdog_tf_stale_abort_count: 3" in params_text
    assert "odom_watchdog_tf_stale_abort_s: 3.0" in params_text
    assert "tf_pose_max_age_s: 3.0" in params_text


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
