from pathlib import Path


LOCALIZER_LAUNCH = Path("src/perception/localizer/launch/localizer_launch.py")
LOCALIZER_NODE = Path("src/perception/localizer/src/localizer_node.cpp")
LOCALIZER_CONFIG = Path("src/perception/localizer/config/localizer.yaml")


def test_localizer_launch_can_be_reused_without_lio_or_rviz():
    text = LOCALIZER_LAUNCH.read_text(encoding="utf-8")

    for argument_name in ("config_path", "use_lio", "use_rviz"):
        assert f'"{argument_name}"' in text
    assert "IfCondition(LaunchConfiguration(\"use_lio\"))" in text
    assert "IfCondition(LaunchConfiguration(\"use_rviz\"))" in text


def test_localizer_node_can_load_startup_pcd_map():
    text = LOCALIZER_NODE.read_text(encoding="utf-8")

    assert 'declare_parameter("pcd_map", "")' in text
    assert "loadStartupMap" in text
    assert "m_localizer->loadMap(m_config.pcd_map)" in text


def test_localizer_config_documents_tf_staleness_guard():
    text = LOCALIZER_CONFIG.read_text(encoding="utf-8")

    assert "max_tf_input_age_s" in text


def test_localizer_config_exposes_current_time_tf_republish_rate():
    text = LOCALIZER_CONFIG.read_text(encoding="utf-8")

    assert "tf_republish_hz" in text


def test_localizer_does_not_publish_stale_or_unvalidated_map_to_odom():
    text = LOCALIZER_NODE.read_text(encoding="utf-8")
    non_update_branch = text.split("if (!update_tf)", maxsplit=1)[1].split("m_state.last_send_tf_time", maxsplit=1)[0]

    assert "sendBroadCastTF(m_state.last_message_time)" not in non_update_branch
    assert "if (!localize_success && !service_received)" in text
    assert 'RCLCPP_WARN_THROTTLE' in text
    assert "isTransformStampFresh(current_time)" in text
    assert "isNewerStamp(current_time, m_state.last_tf_time)" in text
    assert "m_state.last_tf_time = current_time" in text


def test_localizer_republishes_last_valid_map_to_odom_with_current_time():
    text = LOCALIZER_NODE.read_text(encoding="utf-8")

    assert "tf_republish_hz" in text
    assert "m_state.last_republish_tf_time" in text
    assert "republishLatestTF" in text
    republish_block = text.split("void republishLatestTF", maxsplit=1)[1].split(
        "void publishMapCloud", maxsplit=1
    )[0]
    assert "sendBroadCastTF" in republish_block
    assert "this->now()" in republish_block


def test_localizer_tf_throttle_uses_node_clock_only():
    text = LOCALIZER_NODE.read_text(encoding="utf-8")

    assert "rclcpp::Clock().now()" not in text
    assert "this->now() - m_state.last_send_tf_time" in text
