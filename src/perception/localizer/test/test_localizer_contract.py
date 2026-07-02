from pathlib import Path


LOCALIZER_LAUNCH = Path("src/perception/localizer/launch/localizer_launch.py")
LOCALIZER_NODE = Path("src/perception/localizer/src/localizer_node.cpp")


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
