from pathlib import Path


GOAL_MANAGER = Path(
    "src/navigation/gps_waypoint_dispatcher/gps_waypoint_dispatcher/goal_manager_node.py"
)
NAV_GPS_MENU = Path("scripts/nav_gps_menu.py")


def test_goal_manager_can_route_from_current_pose_without_nearest_anchor():
    text = GOAL_MANAGER.read_text(encoding="utf-8")

    assert 'self.declare_parameter("use_route_pose_start", False)' in text
    assert 'self.declare_parameter("require_nav_ready", True)' in text
    assert "if self.require_nav_ready and self.system_status != \"NAV_READY\":" in text
    assert "if not self.use_route_pose_start and self.nearest_anchor_id is None:" in text
    assert "route_goal.use_poses = True" in text
    assert "route_goal.start = start_pose" in text
    assert "route_goal.goal = dest_pose" in text
    assert "pose_start" in text


def test_goal_manager_pose_start_skips_anchor_stage():
    text = GOAL_MANAGER.read_text(encoding="utf-8")

    assert "if self.use_route_pose_start:" in text
    assert "self._send_follow_path()" in text
    assert "NAVIGATING_TO_ANCHOR" in text


def test_nav_gps_menu_waits_for_rtk_authoritative_mode():
    text = NAV_GPS_MENU.read_text(encoding="utf-8")

    assert "/localization_authority/mode" in text
    assert "self.localization_authority_mode == \"RTK_AUTHORITATIVE\"" in text
    assert "RTK_AUTHORITATIVE reached" in text
    assert "navigate_to_pose_client" not in text
