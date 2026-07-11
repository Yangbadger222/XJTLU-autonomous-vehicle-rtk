from pathlib import Path

import yaml


SLAM_LAUNCH = Path("src/bringup/launch/system_slam.launch.py")
SLAM_PARAMS = Path("src/bringup/config/slam_toolbox_mapping.yaml")
PGO_SLAM_PARAMS = Path("src/perception/pgo/config/pgo_slam.yaml")


def _slam_config():
    data = yaml.safe_load(SLAM_PARAMS.read_text(encoding="utf-8"))
    return data["slam_toolbox"]["ros__parameters"]


def test_slam_launch_uses_repository_owned_mapping_profile():
    text = SLAM_LAUNCH.read_text(encoding="utf-8")

    assert 'bringup_share, "config", "slam_toolbox_mapping.yaml"' in text
    assert "mapper_params_online_async.yaml" not in text
    assert "parameters=[slam_params_file]" in text


def test_slam_mapping_profile_matches_fastlio_tf_contract():
    config = _slam_config()

    assert config["mode"] == "mapping"
    assert config["map_frame"] == "map"
    assert config["odom_frame"] == "odom"
    assert config["base_frame"] == "base_footprint"
    assert config["scan_topic"] == "/scan"
    assert config["use_sim_time"] is False
    assert config["transform_timeout"] == 1.0
    assert config["tf_buffer_duration"] == 60.0
    assert config["resolution"] == 0.05
    assert config["do_loop_closing"] is True


def test_slam_keeps_pgo_as_non_tf_map_artifact_builder():
    launch_text = SLAM_LAUNCH.read_text(encoding="utf-8")
    pgo_config = yaml.safe_load(PGO_SLAM_PARAMS.read_text(encoding="utf-8"))

    assert "pgo_slam.yaml" in launch_text
    assert pgo_config["publish_tf"] is False
    assert pgo_config["map_frame"] == "map"
    assert pgo_config["local_frame"] == "odom"
    assert pgo_config["gps"]["enable"] is False
    assert pgo_config["cloud_topic"] == "/fastlio2/body_cloud_localization"


def test_slam_wires_fastlio_cloud_to_laserscan_and_delays_mapper_start():
    text = SLAM_LAUNCH.read_text(encoding="utf-8")

    assert '("cloud_in", "/fastlio2/body_cloud")' in text
    assert '("scan", "/scan")' in text
    assert "period=5.0" in text
    assert "slam_toolbox_node" in text
    assert "map_saver_server" in text
    assert "lifecycle_manager_mapping" in text
    assert "robot_description.launch.py" in text


def test_slam_records_replayable_mapping_evidence_by_default():
    text = SLAM_LAUNCH.read_text(encoding="utf-8")

    assert '"record_bag"' in text
    assert 'FYP_SLAM_BAG_PROFILE' in text
    assert '"/fastlio2/degeneracy"' in text
    assert '"/pgo/loop_markers"' in text
    assert '"/fastlio2/body_cloud_localization"' in text
    assert 'ExecuteProcess' in text
