from pathlib import Path


TRAVEL_LAUNCH = Path("src/bringup/launch/system_travel.launch.py")
NAV2_TRAVEL = Path("src/bringup/config/nav2_travel.yaml")
LAUNCH_WRAPPER = Path("scripts/launch_with_logs.sh")


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
    assert 'plugins: ["static_layer", "obstacle_layer", "inflation_layer"]' in global_costmap_text
    assert "rolling_window: false" in global_costmap_text
    assert "rolling_window: true" in local_costmap_text


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
