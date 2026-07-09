from pathlib import Path


NAV_GPS_LAUNCH = Path("src/bringup/launch/system_nav_gps.launch.py")
BUILD_SCENE_RUNTIME = Path("scripts/build_scene_runtime.py")
LAUNCH_WITH_LOGS = Path("scripts/launch_with_logs.sh")


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
    assert '"/localization_authority/mode",' in text
    assert '"/localization_authority/status",' in text
    assert '"/localization_authority/diagnostics",' in text


def test_nav_gps_route_server_uses_nearest_graph_search_for_pose_requests():
    text = NAV_GPS_LAUNCH.read_text(encoding="utf-8")

    assert '"enable_nn_search": True' in text
    assert '"path_density": 0.2' in text
    assert '"smooth_corners": False' in text


def test_nav_gps_goal_manager_uses_pose_start_without_anchor_gate():
    text = NAV_GPS_LAUNCH.read_text(encoding="utf-8")

    assert '"use_route_pose_start": True' in text
    assert '"require_nav_ready": False' in text


def test_nav_gps_keeps_fgo_shadow_side_effect_free():
    text = NAV_GPS_LAUNCH.read_text(encoding="utf-8")

    assert "enable_fgo_shadow_arg" in text
    assert "FYP_NAV_GPS_ENABLE_FGO_SHADOW" in text
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


def test_launch_with_logs_passes_rviz_override_to_nav_gps():
    text = LAUNCH_WITH_LOGS.read_text(encoding="utf-8")

    assert '"$MODE" == "nav-gps"' in text
