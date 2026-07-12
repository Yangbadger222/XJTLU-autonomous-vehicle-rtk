import json
from pathlib import Path


PACKAGE_ROOT = Path("src/navigation/indoor_navigation_manager")
ADAPTER = PACKAGE_ROOT / "indoor_navigation_manager" / "foxglove_adapter_node.py"
SETUP = PACKAGE_ROOT / "setup.py"
PACKAGE_XML = PACKAGE_ROOT / "package.xml"
TRAVEL_LAUNCH = Path("src/bringup/launch/system_travel.launch.py")
LAYOUT = Path("src/bringup/foxglove/indoor_navigation.json")
STATUS_MSG = Path("src/perception/interface/msg/NavigationStatus.msg")


def test_adapter_uses_actions_and_never_writes_velocity():
    text = ADAPTER.read_text(encoding="utf-8")

    assert "ActionClient(self, NavigateToPose" in text
    assert "NavigateNamedDestination" in text
    assert '"/foxglove/goal_pose"' in text
    assert '"/move_base_simple/goal"' in text
    assert '"/foxglove/named_destination"' in text
    assert '"/foxglove/cancel_navigation"' in text
    assert '"/cmd_vel"' not in text
    assert '"/cmd_vel_safe"' not in text


def test_adapter_gates_and_cancels_on_localization_health():
    text = ADAPTER.read_text(encoding="utf-8")

    assert "msg.localized" in text
    assert "msg.sensors_ready" in text
    assert "LocalizationStatus.LOCALIZED" in text
    assert "if not self.localized:" in text
    assert "self.active_goal_handle.cancel_goal_async()" in text
    assert '"localization degraded; canceling active goal"' in text
    assert "cancel_when_accepted" in text
    assert "if self.pending_goal:" in text


def test_adapter_publishes_foxglove_status_catalog_and_markers():
    text = ADAPTER.read_text(encoding="utf-8")

    assert '"/foxglove/navigation/status"' in text
    assert "NavigationStatus" in text
    assert '"/foxglove/navigation/destinations"' in text
    assert '"/foxglove/navigation/destination_markers"' in text
    assert "DurabilityPolicy.TRANSIENT_LOCAL" in text
    assert "Marker.TEXT_VIEW_FACING" in text
    status_text = STATUS_MSG.read_text(encoding="utf-8")
    for field in (
        "state_label",
        "target",
        "localization_state",
        "localization_ready",
        "remaining_distance_m",
    ):
        assert field in status_text


def test_adapter_is_packaged_with_required_ros_dependencies():
    setup_text = SETUP.read_text(encoding="utf-8")
    package_text = PACKAGE_XML.read_text(encoding="utf-8")

    assert "foxglove_navigation_adapter_node" in setup_text
    for dependency in ("std_msgs", "std_srvs", "visualization_msgs"):
        assert f"<depend>{dependency}</depend>" in package_text


def test_travel_starts_official_bridge_and_adapter():
    text = TRAVEL_LAUNCH.read_text(encoding="utf-8")

    assert '"use_foxglove"' in text
    assert '"foxglove_port"' in text
    assert 'package="foxglove_bridge"' in text
    assert 'executable="foxglove_navigation_adapter_node"' in text
    assert 'LaunchConfiguration("foxglove_port")' in text
    assert '"client_topic_whitelist"' in text
    assert '"^/initialpose$"' in text
    assert '"^/foxglove/goal_pose$"' in text
    assert '"^/move_base_simple/goal$"' in text
    assert '"^/foxglove/named_destination$"' in text
    assert '"service_whitelist"' in text
    assert '"parameters"' not in text.split('"capabilities": [', maxsplit=1)[1].split(
        "]", maxsplit=1
    )[0]


def test_importable_layout_contains_navigation_controls_and_views():
    layout = json.loads(LAYOUT.read_text(encoding="utf-8"))
    configs = layout["configById"]
    scene = configs["3D!indoor"]

    assert scene["followTf"] == "map"
    assert scene["publish"]["poseTopic"] == "/foxglove/goal_pose"
    assert scene["publish"]["poseEstimateTopic"] == "/initialpose"
    assert "/foxglove/navigation/destination_markers" in scene["topics"]
    assert configs["Publish!named"]["topicName"] == "/foxglove/named_destination"
    assert configs["CallService!cancel"]["serviceName"] == "/foxglove/cancel_navigation"
    assert configs["CallService!region"]["serviceName"] == "/localizer/global_relocalize"
    assert "TopicGraph!indoor" in configs
