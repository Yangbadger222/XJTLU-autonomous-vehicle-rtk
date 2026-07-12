from pathlib import Path


FASTLIO_NODE = Path("src/perception/fastlio2/src/lio_node.cpp")
FASTLIO_COMMONS = Path("src/perception/fastlio2/src/map_builder/commons.h")
MASTER_PARAMS = Path("src/bringup/config/master_params.yaml")
TRAVEL_LAUNCH = Path("src/bringup/launch/system_travel.launch.py")
NAV2_TRAVEL = Path("src/bringup/config/nav2_travel.yaml")


def test_fastlio2_publishes_separate_nav2_obstacle_cloud():
    node_text = FASTLIO_NODE.read_text(encoding="utf-8")
    commons_text = FASTLIO_COMMONS.read_text(encoding="utf-8")
    params_text = MASTER_PARAMS.read_text(encoding="utf-8")
    launch_text = TRAVEL_LAUNCH.read_text(encoding="utf-8")
    nav2_text = NAV2_TRAVEL.read_text(encoding="utf-8")

    assert "body_cloud_nav2_obstacles" in node_text
    assert "m_nav2_obstacle_cloud_pub" in node_text
    assert "publishNav2ObstacleCloud" in node_text
    assert "publishNav2ObstacleCloud(body_cloud, world_cloud, cloud_publish_time)" in node_text
    assert "nav2_obstacle_cloud_enabled" in commons_text
    assert "nav2_obstacle_cloud_enabled: true" in params_text
    assert "nav2_obstacle_cloud_max_z: 1.20" in params_text
    assert "filterNav2VehicleReturns" in node_text
    assert "nav2_obstacle_self_filter_enabled" in commons_text
    assert "nav2_obstacle_self_filter_enabled: true" in params_text
    assert "nav2_obstacle_self_filter_min_x: -0.40" in params_text
    assert "nav2_obstacle_self_filter_max_x: 0.40" in params_text
    assert "nav2_obstacle_self_filter_min_y: -0.30" in params_text
    assert "nav2_obstacle_self_filter_max_y: 0.30" in params_text
    assert "nav2_obstacle_self_patch_enabled" in commons_text
    assert "nav2_obstacle_self_patch_enabled: true" in params_text
    assert "nav2_obstacle_self_patch_max_x: 0.82" in params_text
    assert "nav2_obstacle_self_patch_min_z: 0.25" in params_text
    assert "inside_measured_patch" in node_text
    assert '("cloud_in", "/fastlio2/body_cloud_nav2_obstacles")' in launch_text
    assert "topic: /fastlio2/body_cloud_nav2" in nav2_text


def test_fastlio2_publishes_dedicated_mapping_and_localization_cloud():
    node_text = FASTLIO_NODE.read_text(encoding="utf-8")
    commons_text = FASTLIO_COMMONS.read_text(encoding="utf-8")
    params_text = MASTER_PARAMS.read_text(encoding="utf-8")

    assert "body_cloud_localization" in node_text
    assert "m_localization_cloud_pub" in node_text
    assert "publishLocalizationCloud" in node_text
    assert "publishLocalizationCloud(body_cloud, world_cloud, cloud_publish_time)" in node_text
    assert "localization_cloud_enabled" in commons_text
    assert "localization_cloud_enabled: true" in params_text
    assert "localization_cloud_min_z: -0.20" in params_text
    assert "localization_cloud_max_z: 1.80" in params_text
    assert "publish_cloud_min_z: -0.33" in params_text
    assert "publish_cloud_max_z: 0.30" in params_text
