from pathlib import Path


NAV_GPS_LAUNCH = Path("src/bringup/launch/system_nav_gps.launch.py")
BUILD_SCENE_RUNTIME = Path("scripts/build_scene_runtime.py")
LAUNCH_WITH_LOGS = Path("scripts/launch_with_logs.sh")
RTK_CORRECTOR = Path(
    "src/navigation/gps_waypoint_dispatcher/gps_waypoint_dispatcher/"
    "rtk_map_odom_corrector_node.py"
)


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


def test_nav_gps_routes_commands_through_authority_guard():
    text = NAV_GPS_LAUNCH.read_text(encoding="utf-8")

    assert '"guarded_cmd_vel": "true"' in text
    assert 'executable="corridor_cmd_vel_guard_node"' in text
    assert '"stop_override_topic": "/gps_nav/stop_override"' in text
    assert '"/localization_authority/motion_allowed"' in text


def test_nav_gps_keeps_fgo_shadow_side_effect_free():
    text = NAV_GPS_LAUNCH.read_text(encoding="utf-8")

    assert "enable_fgo_shadow_arg" in text
    assert "FYP_NAV_GPS_ENABLE_FGO_SHADOW" in text
    assert 'os.environ.get("FYP_NAV_GPS_ENABLE_FGO_SHADOW", "false")' in text
    assert "rtk_fgo_localizer" in text
    assert "rtk_fgo_node" in text
    assert '"publish_tf": False' in text
    assert '"nav2_use_fgo": False' in text


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
