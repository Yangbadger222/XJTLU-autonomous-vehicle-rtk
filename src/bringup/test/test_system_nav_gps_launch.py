import ast
from pathlib import Path
import re


NAV_GPS_LAUNCH = Path("src/bringup/launch/system_nav_gps.launch.py")
BUILD_SCENE_RUNTIME = Path("scripts/build_scene_runtime.py")
LAUNCH_WITH_LOGS = Path("scripts/launch_with_logs.sh")
RTK_CORRECTOR = Path(
    "src/navigation/gps_waypoint_dispatcher/gps_waypoint_dispatcher/"
    "rtk_map_odom_corrector_node.py"
)
MASTER_PARAMS = Path("src/bringup/config/master_params.yaml")


def _launch_topic_list(text: str, variable_name: str) -> list[str]:
    match = re.search(
        rf"{variable_name} = \[(?P<topics>.*?)\]",
        text,
        re.DOTALL,
    )
    assert match is not None
    return ast.literal_eval(f"[{match.group('topics')}]")


def test_nav_gps_reuses_corridor_rtk_authoritative_stack():
    text = NAV_GPS_LAUNCH.read_text(encoding="utf-8")

    assert "system_explore.launch.py" in text
    assert "pgo_corridor_no_tf.yaml" in text
    assert "pgo_corridor_no_gps.yaml" in text
    assert "nav2_corridor_rtk.yaml" in text
    assert "_make_nav_gps_rtk_nav2_params" in text
    assert '"pgo_config_file": pgo_nav_gps_config_file' in text
    assert '"pgo_extra_params_file": pgo_nav_gps_override_file' in text
    assert '"nav2_params_file": nav_gps_rtk_nav2_params' in text


def test_nav_gps_starts_rtk_map_odom_corrector_from_scene_identity_alignment():
    text = NAV_GPS_LAUNCH.read_text(encoding="utf-8")

    assert "rtk_map_odom_corrector_node" in text
    assert "rtk_map_odom_corrector" in text
    assert '"scene_points_file": scene_points_file' in text
    assert '"use_scene_identity_alignment": True' in text
    assert '"alignment_topic": "/gps_scene/enu_to_map"' in text
    assert "GroupAction(actions=[rtk_launch], scoped=True)" in text
    assert '"/localization_authority/mode",' in text
    assert '"/localization_authority/status",' in text
    assert '"/localization_authority/diagnostics",' in text


def test_nav_gps_uses_local_astar_goal_manager_without_route_server():
    text = NAV_GPS_LAUNCH.read_text(encoding="utf-8")

    assert '"require_nav_ready": False' in text
    assert 'executable="route_server"' not in text
    assert '"scene_points_file": scene_points_file' in text


def test_nav_gps_applies_qgis_road_keepout_to_both_costmaps():
    text = NAV_GPS_LAUNCH.read_text(encoding="utf-8")

    assert 'for costmap_name in ("local_costmap", "global_costmap")' in text
    assert '"plugin": "nav2_costmap_2d::KeepoutFilter"' in text
    assert 'name="road_keepout_mask_server"' in text
    assert '"mask_topic": "/road_keepout_mask"' in text
    assert '"road_keepout_yaml": road_keepout_yaml' in text
    assert '"local_costmap_node": "/local_costmap/local_costmap"' in text


def test_nav_gps_routes_commands_through_authority_guard():
    text = NAV_GPS_LAUNCH.read_text(encoding="utf-8")
    explore_text = Path("src/bringup/launch/system_explore.launch.py").read_text(
        encoding="utf-8"
    )

    assert '"guarded_cmd_vel": "true"' in text
    assert 'executable="corridor_cmd_vel_guard_node"' in text
    assert '"stop_override_topic": "/gps_nav/stop_override"' in text
    assert '"road_rejoin_active_topic": "/gps_nav/road_rejoin_active"' in text
    assert '"/localization_authority/motion_allowed"' in text
    assert 'src="/cmd_vel"' in explore_text
    assert 'dst="/cmd_vel_guarded"' in explore_text
    assert '"/cmd_vel_guarded",' in text
    assert '"/cmd_vel_controller",' not in text


def test_nav_gps_keeps_fgo_shadow_side_effect_free():
    text = NAV_GPS_LAUNCH.read_text(encoding="utf-8")

    assert "enable_fgo_shadow_arg" in text
    assert "FYP_NAV_GPS_ENABLE_FGO_SHADOW" in text
    assert 'os.environ.get("FYP_NAV_GPS_ENABLE_FGO_SHADOW", "false")' in text
    assert "rtk_fgo_localizer" in text
    assert "rtk_fgo_node" in text
    assert '"publish_tf": False' in text
    assert '"nav2_use_fgo": False' in text


def test_nav_gps_reduces_mppi_work_without_breaking_model_timing():
    text = NAV_GPS_LAUNCH.read_text(encoding="utf-8")

    assert 'controller_params["controller_frequency"] = 20.0' in text
    assert 'follow_path["time_steps"] = 32' in text
    assert 'follow_path["batch_size"] = 200' in text
    assert 'follow_path["publish_critics_stats"] = False' in text
    assert 'follow_path["retry_attempt_limit"] = 3' in text
    assert 'follow_path["open_loop"] = False' in text
    assert 'follow_path["vx_max"] = 1.5' in text
    assert 'smoother_params["max_velocity"] = [1.5, 0.0, 0.70]' in text
    assert '"straight_max_mps": 1.5' in text
    assert '"road_rejoin_max_mps": 0.35' in text
    assert '"path_density_m": 0.35' in text

    master_params = MASTER_PARAMS.read_text(encoding="utf-8")
    assert "rtk_authoritative_max_linear_speed_mps: 1.5" in master_params


def test_nav_gps_cuda_mppi_shadow_is_explicit_and_non_authoritative_by_default():
    text = NAV_GPS_LAUNCH.read_text(encoding="utf-8")

    assert '"FYP_NAV_GPS_ENABLE_CUDA_MPPI_SHADOW", "false"' in text
    assert '"nav2_cuda_mppi_controller::CudaMppiShadowController"' in text
    assert 'follow_path["cuda_shadow_batch_size"] = 4096' in text
    assert 'follow_path["cuda_shadow_time_steps"] = 48' in text
    assert '"/controller_server/FollowPath/cuda_shadow_diagnostics"' in text
    assert 'follow_path["primary_controller"] = "nav2_mppi_controller::MPPIController"' in text


def test_nav_gps_waits_and_replans_when_dynamic_obstacles_block_mppi():
    text = NAV_GPS_LAUNCH.read_text(encoding="utf-8")

    assert '"blocked_retry_delay_s": 2.0' in text
    assert '"blocked_wait_timeout_s": 60.0' in text
    assert '"blocked_recovery_confirmation_s": 3.0' in text


def test_nav_gps_rotates_to_large_path_heading_changes_before_mppi():
    text = NAV_GPS_LAUNCH.read_text(encoding="utf-8")

    assert '"nav2_controller::PoseProgressChecker"' in text
    assert (
        'controller_params["progress_checker"]["required_movement_angle"] = 0.15'
        in text
    )
    assert '"nav2_rotation_shim_controller::RotationShimController"' in text
    assert (
        'follow_path["primary_controller"] = '
        '"nav2_mppi_controller::MPPIController"' in text
    )
    assert 'follow_path["rotate_to_heading_angular_vel"] = 0.35' in text
    assert 'follow_path["closed_loop"] = False' in text


def test_nav_gps_lean_bag_and_default_nodes_respect_vehicle_cpu_budget():
    text = NAV_GPS_LAUNCH.read_text(encoding="utf-8")
    base_topics = _launch_topic_list(text, "_NAV_GPS_BAG_BASE_TOPICS")
    debug_topics = _launch_topic_list(text, "_NAV_GPS_BAG_DEBUG_TOPICS")

    assert "/local_costmap/costmap" not in base_topics
    assert "/local_costmap/costmap" in debug_topics
    assert "/global_costmap/costmap" not in base_topics
    assert "/global_costmap/costmap" in debug_topics
    assert "/livox/imu" not in base_topics
    assert "/livox/imu" in debug_topics
    assert "/odom_CBoar" not in base_topics
    assert "/odom_CBoar" in debug_topics
    assert "/fastlio2/lio_odom" in base_topics
    assert "/fastlio2/degeneracy" in base_topics
    assert "/cmd_vel" in base_topics
    assert "/plan" in base_topics
    assert "FYP_NAV_GPS_ENABLE_LEGACY_ANCHOR_LOCALIZER" in text
    assert '"FYP_NAV_GPS_ENABLE_LEGACY_ANCHOR_LOCALIZER", "false"' in text
    assert 'condition=IfCondition(LaunchConfiguration("enable_legacy_anchor_localizer"))' in text


def test_nav_gps_costmap_rates_preserve_obstacle_updates_with_lower_cpu_load():
    text = NAV_GPS_LAUNCH.read_text(encoding="utf-8")

    assert 'local_costmap_params["update_frequency"] = 8.0' in text
    assert 'local_costmap_params["publish_frequency"] = 2.0' in text
    assert 'global_costmap_params["update_frequency"] = 2.0' in text
    assert 'global_costmap_params["publish_frequency"] = 1.0' in text


def test_scene_runtime_writes_rtk_authority_scene_origin():
    text = BUILD_SCENE_RUNTIME.read_text(encoding="utf-8")

    assert 'params["/rtk_map_odom_corrector"]' in text
    assert '"scene_points_file": str(SCENE_POINTS_FILE)' in text
    assert '"use_scene_identity_alignment": True' in text
    assert '"alignment_topic": "/gps_scene/enu_to_map"' in text
    assert '"enu_origin_lat": origin["lat"]' in text
    assert '"enu_origin_lon": origin["lon"]' in text
    assert '"enu_origin_alt": origin["alt"]' in text


def test_scene_identity_seeds_first_absolute_map_to_odom_without_fault_release():
    text = RTK_CORRECTOR.read_text(encoding="utf-8")

    assert "if self._scene_identity_alignment is not None" in text
    assert "else Pose2D(0.0, 0.0, 0.0)" in text


def test_scene_runtime_can_rebuild_from_installed_bundle():
    text = BUILD_SCENE_RUNTIME.read_text(encoding="utf-8")

    assert "source_image.resolve() != ROAD_KEEPOUT_IMAGE.resolve()" in text
    assert "bundle_path.resolve() != SCENE_BUNDLE_COPY.resolve()" in text


def test_launch_with_logs_passes_rviz_override_to_nav_gps():
    text = LAUNCH_WITH_LOGS.read_text(encoding="utf-8")

    assert '"$MODE" == "nav-gps"' in text
