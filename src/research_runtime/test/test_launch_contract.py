from pathlib import Path


LAUNCH = Path(__file__).parents[3] / "src" / "bringup" / "launch" / "system_active_road_research.launch.py"


def test_active_road_launch_allowlist_and_explicit_serial_gate():
    source = LAUNCH.read_text()
    assert "package=\"ego_planner\", executable=\"motion_plan\"" in source
    assert "default_value=\"true\"" in source  # Super-LIO is the shadow/live estimator
    assert "DeclareLaunchArgument(\"enable_serial\", default_value=\"false\"" in source
    assert "' == 'live' and '" in source
    assert "LaunchConfiguration(\"enable_serial\")" in source
    assert "package=\"serial_twistctl\"" in source
    assert '"/lio/vehicle_health"' in source
    assert '"/lio/odom_vehicle"' in source
    assert '"/research/local_obstacle_grid"' in source
    assert '"/research/map_version"' in source
    assert 'executable="research_local_obstacle_grid"' in source
    assert 'research_local_grid.yaml' in source
    bridge = (Path(__file__).parents[3] / "src" / "research_runtime" /
              "research_runtime" / "safety_bridge.py").read_text()
    assert "health_timeout_s" in bridge
    assert "health_fresh" in bridge


def test_research_entry_has_no_legacy_navigation_or_fake_sim_include():
    source = LAUNCH.read_text().lower().split('"""', 2)[-1]
    for forbidden in ("system_explore", "package=\"nav2", "package=\"slam_toolbox",
                      "package=\"mppi", "package=\"fastlio2", "fake_sim_node"):
        assert forbidden not in source


def test_bringup_declares_active_road_runtime_dependencies():
    package = (Path(__file__).parents[3] / "src" / "bringup" / "package.xml").read_text()
    for dependency in ("super_lio", "ego_planner", "research_interfaces",
                       "research_runtime", "super_lio_vehicle_adapter",
                       "livox_ros_driver2", "um982_rtk_driver",
                       "gps_waypoint_dispatcher", "serial_twistctl"):
        assert f"<exec_depend>{dependency}</exec_depend>" in package


def test_safety_bridge_declares_its_odom_message_dependency():
    package = (Path(__file__).parents[3] / "src" / "research_runtime" / "package.xml").read_text()
    assert "<exec_depend>nav_msgs</exec_depend>" in package
    assert "<exec_depend>sensor_msgs_py</exec_depend>" in package


def test_vehicle_config_overrides_upstream_demo_limits_and_keeps_unknown_fail_closed():
    config = (Path(__file__).parents[3] / "config" / "ego_vehicle_adapter.yaml").read_text()
    assert "max_speed_mps: 0.85" in config
    assert "max_accel_mps2: 0.85" in config
    assert "max_yaw_rate_rps: 0.70" in config
    assert "max_curvature_1pm: 0.0" in config
    assert "max_lateral_speed_mps: 0.0" in config
    assert "inflate_radius_m: 0.0" in config
    assert "map_resolution_m: 0.30" in config
    assert "grid_unknown_is_occupied: true" in config
    assert 'initial_map_version: "UNKNOWN"' in config

    patch = (Path(__file__).parents[3] / "patches" / "ego_planner_2d" /
             "0002-vehicle-ros-timed-trajectory.patch").read_text()
    active_additions = "\n".join(line[1:] for line in patch.splitlines()
                                    if line.startswith("+") and not line.startswith("+++"))
    for demo_default in ("double max_vel_ = 2.0", "double max_acc_ = 3.0",
                         "double max_jerk_ = 4.0", "double map_resolution_ = 0.1",
                         "double map_inflate_value_ = 0.5"):
        assert demo_default not in active_additions
    assert "(point.vx * point.ay - point.vy * point.ax) / (speed * speed)" in active_additions
    assert "sample.curvature = speed > kEpsilon ? yaw_rate / speed" in active_additions
    assert "(vx * point.ay - vy * point.ax) / (speed * speed)" in active_additions
    assert "const double curvature = speed > kEpsilon ? yaw_rate / speed" in active_additions
    assert "const double speed_tolerance = std::max(0.05, 0.25 * std::max(std::abs(static_cast<double>(point.v)), 0.1));" in active_additions
    assert "std::abs(point.v - speed) > speed_tolerance" in active_additions
    assert "sample.curvature = speed > kEpsilon ? yaw_rate / (speed * speed)" not in active_additions
    assert "const double curvature = speed > kEpsilon ? yaw_rate / (speed * speed)" not in active_additions
