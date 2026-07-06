from pathlib import Path


EXPLORE_LAUNCH = Path("src/bringup/launch/system_explore.launch.py")
CORRIDOR_LAUNCH = Path("src/bringup/launch/system_gps_corridor.launch.py")


def test_explore_launch_exposes_nav2_params_file_for_mode_specific_profiles():
    text = EXPLORE_LAUNCH.read_text(encoding="utf-8")

    assert '"nav2_params_file"' in text
    assert 'source_file=LaunchConfiguration("nav2_params_file")' in text


def test_corridor_launch_uses_slow_nav2_rewrites_for_rtk_acceptance():
    text = CORRIDOR_LAUNCH.read_text(encoding="utf-8")

    assert "corridor_nav2_params = _make_corridor_nav2_params" in text
    assert "follow_path['vx_max'] = 0.45" in text
    assert "follow_path['wz_max'] = 0.65" in text
    assert "follow_path['ax_max'] = 0.45" in text
    assert "smoother_params['max_velocity'] = [0.45, 0.0, 0.65]" in text
    assert "smoother_params['max_decel'] = [-0.8, 0.0, -1.8]" in text
    assert "'nav2_params_file': corridor_nav2_params" in text


def test_corridor_bag_records_raw_livox_and_fastlio_diagnostics():
    text = CORRIDOR_LAUNCH.read_text(encoding="utf-8")

    assert "'/livox/lidar'," in text
    assert "'/livox/imu'," in text
    assert "'/fastlio2/body_cloud'," in text
