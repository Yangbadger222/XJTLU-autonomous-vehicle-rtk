from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[3]
LAUNCH_FILE = REPO_ROOT / "src/bringup/launch/system_fgo_gil_time_sync.launch.py"
LIDAR_LAUNCH_FILE = (
    REPO_ROOT / "src/bringup/launch/system_fgo_gil_lidar_frontend.launch.py"
)
CONFIG_FILE = REPO_ROOT / "src/bringup/config/fgo_gil.yaml"
MAKEFILE = REPO_ROOT / "Makefile"
LAUNCH_WRAPPER = REPO_ROOT / "scripts/launch_with_logs.sh"


def test_phase3_launch_is_shadow_only():
    text = LAUNCH_FILE.read_text(encoding="utf-8")

    assert 'package="fgo_gil_localizer"' in text
    assert 'executable="fgo_gil_time_sync_node"' in text
    assert "tf2_ros" not in text
    assert "cmd_vel" not in text
    assert "nav2" not in text.lower()


def test_phase3_config_defaults_to_current_ros_stamp_reality():
    config = yaml.safe_load(CONFIG_FILE.read_text(encoding="utf-8"))
    parameters = config["fgo_gil_time_sync"]["ros__parameters"]

    assert parameters["imu_stamp_domain"] == "ros"
    assert parameters["lidar_stamp_domain"] == "ros"
    assert parameters["topics"]["gnss_pps"] == "/gnss/pps/time_reference"
    assert parameters["time_sync"]["coarse_uncertainty_floor_s"] == 0.02
    assert parameters["time_sync"]["coarse_timeout_s"] == 5.0
    assert parameters["time_sync"]["pps_uncertainty_floor_s"] == 0.0001
    assert parameters["imu_buffer"]["capacity"] == 4096
    assert parameters["imu_buffer"]["max_gap_s"] == 0.05


def test_phase3_build_launch_and_cleanup_entry_points_exist():
    makefile = MAKEFILE.read_text(encoding="utf-8")
    wrapper = LAUNCH_WRAPPER.read_text(encoding="utf-8")

    assert "build-fgo-gil:" in makefile
    assert "gnss_raw_msgs fgo_gil_localizer bringup" in makefile
    assert "launch-fgo-gil-time-sync" in makefile
    assert "fgo-gil-time-sync)" in wrapper
    assert "[f]go_gil_time_sync_node" in makefile
    assert "[f]go_gil_time_sync_node" in wrapper


def test_phase4_launch_remains_shadow_only():
    text = LIDAR_LAUNCH_FILE.read_text(encoding="utf-8")

    assert 'executable="fgo_gil_time_sync_node"' in text
    assert 'executable="fgo_gil_lidar_frontend_node"' in text
    assert "tf2_ros" not in text
    assert "cmd_vel" not in text
    assert "nav2" not in text.lower()


def test_phase4_config_fails_closed_and_bounds_the_map():
    config = yaml.safe_load(CONFIG_FILE.read_text(encoding="utf-8"))
    parameters = config["fgo_gil_lidar_frontend"]["ros__parameters"]

    assert parameters["initialization"]["require_time_sync"] is True
    assert parameters["initialization"]["acceleration_scale"] == 9.80665
    assert parameters["deskew"]["maximum_imu_gap_s"] == 0.05
    assert parameters["keyframes"]["maximum_keyframes"] == 20
    assert parameters["matcher"]["minimum_line_matches"] == 8
    assert parameters["matcher"]["minimum_plane_matches"] == 20
    assert parameters["matcher"]["minimum_plane_second_eigenvalue"] == 0.0001


def test_phase4_launch_and_cleanup_entry_points_exist():
    makefile = MAKEFILE.read_text(encoding="utf-8")
    wrapper = LAUNCH_WRAPPER.read_text(encoding="utf-8")

    assert "launch-fgo-gil-lidar" in makefile
    assert "fgo-gil-lidar)" in wrapper
    assert "[f]go_gil_lidar_frontend_node" in makefile
    assert "[f]go_gil_lidar_frontend_node" in wrapper
