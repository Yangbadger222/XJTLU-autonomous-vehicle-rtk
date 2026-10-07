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
    assert '"source_frame": "world", "world_frame": "odom"' in source
    assert '"source_child_frame": "imu", "tf_timeout_s": 0.05' in source
    assert "ego_vehicle_adapter.yaml" in source
    assert '"/research/local_obstacle_grid"' in source
    assert '"/research/map_version"' in source
    assert 'executable="research_local_obstacle_grid"' in source
    assert 'research_local_grid.yaml' in source
    assert 'executable="active_road_map"' in source
    assert 'active_road_mapping.yaml' in source
    assert 'executable="active_road_evidence"' in source
    assert 'active_road_evidence.yaml' in source
    assert 'research_safety_bridge.yaml' in source
    assert 'executable="super_lio_cloud_frame_adapter"' in source
    assert 'super_lio_cloud_frame.yaml' in source
    active_road_config = (Path(__file__).parents[3] / "src" / "bringup" / "config" /
                          "active_road_mapping.yaml").read_text()
    assert "reload_period_s: 0.20" in active_road_config
    bridge = (Path(__file__).parents[3] / "src" / "research_runtime" /
              "research_runtime" / "safety_bridge.py").read_text()
    assert "health_timeout_s" in bridge
    assert "health_fresh" in bridge
    assert "_parameter_bool" in bridge
    assert "_valid_map_version" in bridge
    assert "_limits_configured" in bridge
    local_grid = (Path(__file__).parents[3] / "src" / "research_runtime" /
                  "research_runtime" / "local_obstacle_grid_node.py").read_text()
    assert "def _parameter_bool" in local_grid
    assert "unknown_is_occupied must remain true" in local_grid


def test_research_entry_has_no_legacy_navigation_or_fake_sim_include():
    source = LAUNCH.read_text().lower().split('"""', 2)[-1]
    for forbidden in ("system_explore", "package=\"nav2", "package=\"slam_toolbox",
                      "package=\"mppi", "package=\"fastlio2", "fake_sim_node"):
        assert forbidden not in source


def test_bringup_declares_active_road_runtime_dependencies():
    package = (Path(__file__).parents[3] / "src" / "bringup" / "package.xml").read_text()
    for dependency in ("ament_index_python", "launch", "launch_ros",
                       "super_lio", "ego_planner", "research_interfaces",
                       "research_runtime", "super_lio_vehicle_adapter",
                       "active_road_mapping",
                       "livox_ros_driver2", "um982_rtk_driver",
                       "gps_waypoint_dispatcher", "serial_twistctl"):
        assert f"<exec_depend>{dependency}</exec_depend>" in package


def test_active_road_entry_assets_exist_in_source_tree():
    root = Path(__file__).parents[3]
    for relative in (
            "src/bringup/config/master_params.yaml",
            "src/bringup/config/super_lio_vehicle.yaml",
            "src/bringup/config/super_lio_cloud_frame.yaml",
            "src/bringup/config/research_safety_bridge.yaml",
            "src/bringup/config/research_local_grid.yaml",
            "src/bringup/config/active_road_mapping.yaml",
            "src/bringup/config/active_road_evidence.yaml",
            "src/bringup/config/ego_vehicle_adapter.yaml",
            "src/sensor_drivers/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py",
            "src/sensor_drivers/gnss/um982_rtk_driver/launch/um982_rtk.launch.py"):
        assert (root / relative).is_file(), relative


def test_safety_bridge_declares_its_odom_message_dependency():
    package = (Path(__file__).parents[3] / "src" / "research_runtime" / "package.xml").read_text()
    assert "<exec_depend>nav_msgs</exec_depend>" in package
    assert "<exec_depend>sensor_msgs_py</exec_depend>" in package


def test_active_road_evidence_package_exposes_typed_ingest_boundary():
    package = (Path(__file__).parents[3] / "src" / "active_road_mapping" /
               "package.xml").read_text()
    setup = (Path(__file__).parents[3] / "src" / "active_road_mapping" /
             "setup.py").read_text()
    interface = (Path(__file__).parents[3] / "src" / "research_interfaces" /
                 "msg" / "RoadEvidence2D.msg").read_text()
    assert "<exec_depend>research_interfaces</exec_depend>" in package
    assert "active_road_evidence = active_road_mapping.evidence_node:main" in setup
    assert "geometry_msgs/Point[] geometry" in interface
    evidence_node = (Path(__file__).parents[3] / "src" / "active_road_mapping" /
                     "active_road_mapping" / "evidence_node.py").read_text()
    assert 'external boolean reachability cannot authorize' in evidence_node
    cloud_adapter = (Path(__file__).parents[3] / "src" / "super_lio_vehicle_adapter" /
                     "super_lio_vehicle_adapter" / "cloud_frame_node.py").read_text()
    assert "lookup_transform" in cloud_adapter
    assert "refusing frame relabel" in cloud_adapter
    vehicle_adapter = (Path(__file__).parents[3] / "src" / "super_lio_vehicle_adapter" /
                       "super_lio_vehicle_adapter" / "adapter_node.py").read_text()
    assert 'declare_parameter("source_frame", "world")' in vehicle_adapter
    assert "needs timestamped TF" in vehicle_adapter
    assert "_parameter_bool" in vehicle_adapter
    assert "source odometry has no acquisition timestamp" in vehicle_adapter
    assert "refusing latest-TF lookup" in cloud_adapter


def test_vehicle_config_overrides_upstream_demo_limits_and_keeps_unknown_fail_closed():
    config = (Path(__file__).parents[3] / "src" / "bringup" / "config" / "ego_vehicle_adapter.yaml").read_text()
    assert "max_speed_mps: 0.85" in config
    assert "max_accel_mps2: 0.85" in config
    assert "max_yaw_rate_rps: 0.70" in config
    assert "max_curvature_1pm: 0.0" in config
    assert "max_lateral_speed_mps: 0.0" in config
    assert "inflate_radius_m: 0.0" in config
    assert "map_resolution_m: 0.30" in config
    assert "grid_unknown_is_occupied: true" in config
    safety = (Path(__file__).parents[3] / "src" / "bringup" / "config" /
              "research_safety_bridge.yaml").read_text()
    assert "footprint_xy: [0.33, 0.305, 0.33, -0.305, -0.33, -0.305, -0.33, 0.305]" in safety
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
    assert "planner_initialized_ = false" in active_additions
    assert "next_version != map_version_" in active_additions
    assert "const double speed_tolerance = std::max(0.05, 0.25 * std::max(std::abs(static_cast<double>(point.v)), 0.1));" in active_additions
    assert "std::abs(point.v - speed) > speed_tolerance" in active_additions
    assert "sample.curvature = speed > kEpsilon ? yaw_rate / (speed * speed)" not in active_additions
    assert "const double curvature = speed > kEpsilon ? yaw_rate / (speed * speed)" not in active_additions

    stale_plan_patch = (Path(__file__).parents[3] / "patches" / "ego_planner_2d" /
                        "0003-clear-stale-plan-on-failure.patch").read_text()
    assert "_plan_traj_results_.clear();" in stale_plan_patch
    assert "a_star_pathes_.clear();" in stale_plan_patch
