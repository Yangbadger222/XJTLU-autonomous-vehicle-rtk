from pathlib import Path


MANAGER = Path(
    "src/navigation/indoor_navigation_manager/indoor_navigation_manager/manager_node.py"
)
ACTION = Path("src/perception/interface/action/NavigateNamedDestination.action")
TRAVEL_LAUNCH = Path("src/bringup/launch/system_travel.launch.py")


def test_named_navigation_action_exposes_goal_feedback_and_result():
    text = ACTION.read_text(encoding="utf-8")

    assert "string destination_name" in text
    assert "string backend" in text
    assert "bool success" in text
    assert "float32 remaining_distance" in text
    assert "string localization_state" in text


def test_manager_wraps_nav2_and_cancels_on_localization_loss():
    text = MANAGER.read_text(encoding="utf-8")

    assert "ActionServer" in text
    assert "NavigateNamedDestination" in text
    assert "NavigateToPose" in text
    assert "resolve_destination" in text
    assert 'request.backend != "indoor"' in text
    assert "self.goal_reserved" in text
    assert "LocalizationStatus.LOCALIZED" in text
    assert "LocalizationStatus.DEGRADED" in text
    assert "msg.sensors_ready" in text
    assert "self.nav_goal_handle.cancel_goal_async()" in text


def test_travel_launches_named_navigation_from_bundle_destinations():
    text = TRAVEL_LAUNCH.read_text(encoding="utf-8")

    assert 'package="indoor_navigation_manager"' in text
    assert '"destinations_file": LaunchConfiguration("destinations_file")' in text
    assert '"map_id": LaunchConfiguration("map_id")' in text


def test_manager_carries_cancel_across_nav_goal_acceptance_race():
    text = MANAGER.read_text(encoding="utf-8")

    assert "self.cancel_requested = True" in text
    assert "await self.nav_goal_handle.cancel_goal_async()" in text
    assert "navigation canceled before dispatch" in text
    assert "finally:" in text
    assert "self.cancel_requested = False" in text
    assert "wait_for_server" not in text
