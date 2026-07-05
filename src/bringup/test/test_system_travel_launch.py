from pathlib import Path


TRAVEL_LAUNCH = Path("src/bringup/launch/system_travel.launch.py")
NAV2_TRAVEL = Path("src/bringup/config/nav2_travel.yaml")
LAUNCH_WRAPPER = Path("scripts/launch_with_logs.sh")
BRINGUP_CMAKE = Path("src/bringup/CMakeLists.txt")
BRINGUP_PACKAGE = Path("src/bringup/package.xml")
INITIALPOSE_BRIDGE = Path("src/bringup/scripts/initialpose_relocalize_bridge.py")
NAV2_CLOUD_RETIME = Path("src/bringup/scripts/nav2_cloud_retime.py")


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
    assert '("cloud_in", "/fastlio2/body_cloud")' in text
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
    assert 'plugins: ["static_layer", "obstacle_layer", "inflation_layer"]' in global_costmap_text
    assert "rolling_window: false" in global_costmap_text
    assert "rolling_window: true" in local_costmap_text


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
