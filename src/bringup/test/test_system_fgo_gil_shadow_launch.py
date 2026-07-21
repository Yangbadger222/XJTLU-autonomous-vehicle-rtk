from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[3]
SHADOW_LAUNCH = REPO_ROOT / "src/bringup/launch/system_fgo_gil_shadow.launch.py"
FLOAT_LAUNCH = REPO_ROOT / "src/bringup/launch/system_fgo_gil_float.launch.py"
CONFIG_FILE = REPO_ROOT / "src/bringup/config/fgo_gil.yaml"
LOCALIZER_CMAKE = (
    REPO_ROOT / "src/perception/fgo_gil_localizer/CMakeLists.txt"
)
MAKEFILE = REPO_ROOT / "Makefile"
LAUNCH_WRAPPER = REPO_ROOT / "scripts/launch_with_logs.sh"
EVALUATOR = REPO_ROOT / "scripts/evaluate_fgo_gil_bag.py"
REPLAY_SCRIPT = REPO_ROOT / "scripts/replay_fgo_gil_bag.sh"


def test_phase7_shadow_launch_starts_only_the_observation_stack():
    text = SHADOW_LAUNCH.read_text(encoding="utf-8")

    assert 'package="livox_ros_driver2"' not in text
    assert '"msg_MID360_launch.py"' in text
    assert 'package="fastlio2"' in text
    assert 'package="um982_rtk_driver"' in text
    assert 'executable="um982_rtk_node"' in text
    assert "um982_raw_node" not in text
    assert '"system_fgo_gil_float.launch.py"' in text
    assert "serial_twistctl" not in text
    assert "controller_server" not in text
    assert "cmd_vel" not in text


def test_phase7_shadow_launch_forces_safety_and_supports_replay():
    shadow = SHADOW_LAUNCH.read_text(encoding="utf-8")
    estimator = FLOAT_LAUNCH.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("publish_tf", default_value="false")' in shadow
    assert 'DeclareLaunchArgument("nav2_use_fgo", default_value="false")' in shadow
    assert "_validate_shadow_safety" in shadow
    assert "publish_tf and nav2_use_fgo must be false" in shadow
    assert 'DeclareLaunchArgument("use_sim_time", default_value="false")' in shadow
    assert 'DeclareLaunchArgument("start_livox", default_value="true")' in shadow
    assert 'DeclareLaunchArgument("start_fastlio", default_value="true")' in shadow
    assert 'DeclareLaunchArgument("start_um982_driver", default_value="true")' in shadow
    assert '"safety.publish_tf": LaunchConfiguration("publish_tf")' in estimator
    assert '"safety.nav2_use_fgo": LaunchConfiguration("nav2_use_fgo")' in estimator


def test_phase7_bag_profiles_capture_inputs_comparators_and_outputs():
    text = SHADOW_LAUNCH.read_text(encoding="utf-8")

    for topic in (
        "/fix",
        "/heading",
        "/rtk/nmea_sentence",
        "/rtk/status",
        "/gnss/raw/observation_epoch",
        "/gnss/raw/ephemeris",
        "/gnss/rtcm/reference_station",
        "/livox/imu",
        "/fastlio2/lio_odom",
        "/fgo_gil/lidar_constraints",
        "/fgo_gil/odom",
        "/fgo_gil/path",
        "/fgo_gil/factor_diagnostics",
        "/fgo_gil/ambiguity_status",
        "/fgo_gil/timing_status",
        "/fgo_gil/performance",
    ):
        assert f'"{topic}"' in text
    assert 'if profile == "minimal"' in text
    assert 'if profile != "full"' in text
    assert '"ros2",' in text and '"bag",' in text and '"record",' in text
    assert 'DeclareLaunchArgument("record_bag", default_value="true")' in text


def test_phase7_config_exposes_outputs_and_refuses_control_ownership():
    config = yaml.safe_load(CONFIG_FILE.read_text(encoding="utf-8"))
    time_parameters = config["fgo_gil_time_sync"]["ros__parameters"]
    parameters = config["fgo_gil_float_fgo"]["ros__parameters"]
    buffers = parameters["buffers"]
    optimizer = parameters["optimizer"]

    assert time_parameters["topics"]["status"] == "/fgo_gil/timing_status"
    assert parameters["topics"]["output_odometry"] == "/fgo_gil/odom"
    assert parameters["topics"]["path"] == "/fgo_gil/path"
    assert parameters["topics"]["factor_diagnostics"] == "/fgo_gil/factor_diagnostics"
    assert parameters["topics"]["ambiguity_status"] == "/fgo_gil/ambiguity_status"
    assert parameters["topics"]["performance"] == "/fgo_gil/performance"
    assert parameters["raw_input"]["observation_stale_timeout_s"] == 2.0
    assert parameters["raw_input"]["ephemeris_stale_timeout_s"] == 300.0
    assert parameters["topics"]["reference_station"] == "/gnss/rtcm/reference_station"
    assert parameters["calibration"]["gnss"]["dynamic_base"] == {
        "enabled": True,
        "change_threshold_m": 0.01,
    }
    assert buffers["imu_qos_depth"] == 512
    assert buffers["raw_input_qos_depth"] == 512
    assert buffers["pending_lidar_batches"] == 16
    assert buffers["pending_lidar_timeout_s"] == 0.5
    assert buffers["pending_raw_epochs"] == 1024
    assert buffers["pending_ephemerides"] == 64
    assert buffers["pending_reference_stations"] == 16
    assert optimizer["maximum_condition_estimate"] == 1.0e12
    assert optimizer["maximum_line_factors_per_keyframe"] == 48
    assert optimizer["maximum_plane_factors_per_keyframe"] == 96
    assert parameters["output"]["maximum_path_poses"] == 2000
    assert parameters["safety"] == {"publish_tf": False, "nav2_use_fgo": False}


def test_phase7_localizer_defaults_to_an_optimized_build():
    text = LOCALIZER_CMAKE.read_text(encoding="utf-8")

    assert "if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)" in text
    assert 'set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)' in text


def test_phase7_make_launch_and_cleanup_cover_the_full_stack():
    makefile = MAKEFILE.read_text(encoding="utf-8")
    wrapper = LAUNCH_WRAPPER.read_text(encoding="utf-8")

    assert "launch-fgo-gil-shadow" in makefile
    assert "livox_ros_driver2 serial nmea_msgs gnss_raw_msgs um982_raw_driver" in makefile
    assert 'fgo-gil-shadow) LAUNCH_FILE="system_fgo_gil_shadow.launch.py"' in wrapper
    for process_pattern in (
        "[r]os2 bag",
        "[l]ivox_ros_driver2_node",
        "[l]io_node",
        "[u]m982_rtk_node",
        "[f]go_gil_time_sync_node",
        "[f]go_gil_lidar_frontend_node",
        "[f]go_gil_float_fgo_node",
    ):
        assert process_pattern in makefile
        assert process_pattern in wrapper


def test_phase7_evaluator_reports_all_frozen_metrics_and_missing_raw_status():
    text = EVALUATOR.read_text(encoding="utf-8")

    for metric in (
        '"ape_m"',
        '"rpe_m"',
        '"availability"',
        '"fixing_rate"',
        '"outage_drift"',
        '"cpu_mean_percent"',
        '"ram_used_mb"',
        '"real_time_factor"',
        '"RAW_GNSS_UNAVAILABLE"',
    ):
        assert metric in text


def test_phase7_raw_health_is_based_on_valid_messages():
    text = (
        REPO_ROOT
        / "src/perception/fgo_gil_localizer/src/float_fgo_node.cpp"
    ).read_text(encoding="utf-8")

    assert "last_valid_raw_observation_reception_steady_" in text
    assert "last_valid_ephemeris_reception_steady_" in text
    assert '"RAW_GNSS_INVALID"' in text
    assert '"RAW_EPHEMERIS_INVALID"' in text


def test_phase7_replay_uses_an_input_only_topic_allowlist():
    text = REPLAY_SCRIPT.read_text(encoding="utf-8")

    for topic in (
        "/livox/lidar",
        "/livox/imu",
        "/fastlio2/lio_odom",
        "/gnss/raw/observation_epoch",
        "/gnss/raw/ephemeris",
        "/gnss/rtcm/reference_station",
        "/gnss/pps/time_reference",
    ):
        assert topic in text
    assert "--clock --topics" in text
    assert "/fgo_gil/" not in text
