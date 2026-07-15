from pathlib import Path


GOAL_MANAGER = Path(
    "src/navigation/gps_waypoint_dispatcher/gps_waypoint_dispatcher/goal_manager_node.py"
)
NAV_GPS_MENU = Path("scripts/nav_gps_menu.py")


def test_goal_manager_uses_local_astar_from_current_pose_without_anchor_gate():
    text = GOAL_MANAGER.read_text(encoding="utf-8")

    assert "RouteGraphPlanner" in text
    assert "self.route_planner.plan(" in text
    assert 'self.declare_parameter("require_nav_ready", False)' in text
    assert "if self.require_nav_ready and self.system_status != \"NAV_READY\":" in text
    assert "ComputeRoute" not in text
    assert "nearest_anchor_id" not in text
    assert "densify_polyline" in text


def test_goal_manager_accepts_named_map_and_geographic_destinations():
    text = GOAL_MANAGER.read_text(encoding="utf-8")

    assert '"/gps_waypoint_dispatcher/goto_name"' in text
    assert 'self.declare_parameter("goal_pose_topic", "/goal_pose")' in text
    assert 'self.declare_parameter("geo_goal_topic", "/gps_goal")' in text
    assert "self.projector.forward" in text
    assert "self._send_follow_path()" in text


def test_goal_manager_holds_and_replans_on_authority_loss():
    text = GOAL_MANAGER.read_text(encoding="utf-8")

    assert 'self.declare_parameter("stop_override_topic", "/gps_nav/stop_override")' in text
    assert '"AUTHORITY_HOLD"' in text
    assert '"GLOBAL_CORRECTION_HOLD"' in text
    assert "self._request_cancel()" in text
    assert "self._plan_from_current_pose()" in text
    assert '"nav2_false_success_remaining=%.2f"' in text
    assert "active_goal_handle.cancel_goal_async()" in text
    assert "if generation != self.generation or not self.busy:" in text
    assert "goal_handle.cancel_goal_async()" in text


def test_goal_manager_ignores_reliable_republish_of_the_active_goal():
    text = GOAL_MANAGER.read_text(encoding="utf-8")

    assert "same_request = (" in text
    assert 'self.get_logger().debug(f"Ignoring duplicate goal request: {label}")' in text


def test_nav_gps_menu_waits_for_authority_agnostic_motion_permission():
    text = NAV_GPS_MENU.read_text(encoding="utf-8")

    assert "/localization_authority/mode" in text
    assert "/localization_authority/motion_allowed" in text
    assert "self.localization_motion_allowed and self.action_servers_ready()" in text
    assert "ComputeRoute" not in text
    assert "navigate_to_pose_client" not in text
