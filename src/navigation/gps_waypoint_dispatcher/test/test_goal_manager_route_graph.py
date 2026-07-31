from pathlib import Path


GOAL_MANAGER = Path(
    "src/navigation/gps_waypoint_dispatcher/gps_waypoint_dispatcher/goal_manager_node.py"
)
NAV_GPS_MENU = Path("scripts/nav_gps_menu.py")


def test_goal_manager_uses_local_astar_from_current_pose_without_anchor_gate():
    text = GOAL_MANAGER.read_text(encoding="utf-8")

    assert "RouteGraphPlanner" in text
    assert "self.route_planner.plan(" in text
    assert "route_snap_candidate_count" in text
    assert "route_snap_candidate_distance_slack_m" in text
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


def test_goal_manager_debounces_authority_loss_before_replan():
    text = GOAL_MANAGER.read_text(encoding="utf-8")

    assert 'self.declare_parameter("authority_loss_replan_delay_s", 2.0)' in text
    assert '"AUTHORITY_GRACE"' in text
    assert "authority_loss_s >= self.authority_loss_replan_delay_s" in text
    grace_index = text.index('"AUTHORITY_GRACE"')
    cancel_index = text.index('self.cancel_reason = "AUTHORITY_HOLD"', grace_index)
    assert grace_index < cancel_index


def test_goal_manager_reports_heading_hold_timeout_with_authority_evidence():
    text = GOAL_MANAGER.read_text(encoding="utf-8")

    assert 'self.declare_parameter("authority_diagnostics_topic", "/localization_authority/diagnostics")' in text
    assert "def _authority_hold_timeout_detail(self)" in text
    assert '"HEADING_NOT_RECOVERED_TIMEOUT"' in text
    assert '("heading_jump_deg", diagnostics.get("release_yaw_gap_deg", "nan"))' in text
    assert '("heading_reject_delta", diagnostics.get("heading_rejects_delta", "0"))' in text
    assert "self._finish_failure(self._authority_hold_timeout_detail())" in text


def test_goal_manager_waits_and_replans_after_blocked_follow_path():
    text = GOAL_MANAGER.read_text(encoding="utf-8")

    assert 'self.declare_parameter("blocked_retry_delay_s", 2.0)' in text
    assert 'self.declare_parameter("blocked_wait_timeout_s", 60.0)' in text
    assert "wrapped.status == GoalStatus.STATUS_ABORTED" in text
    assert '"BLOCKED_WAIT"' in text
    assert '"BLOCKED_RETRY"' in text
    assert '"BLOCKED_RECOVERED"' in text
    assert "self.blocked_retry.poll(" in text
    assert "retry_allowed=ready" in text
    assert 'self.cancel_reason = "BLOCKED_TIMEOUT"' in text
    assert "and ready\n            and self.blocked_retry.confirm_action_running" in text
    assert "moving=local_motion_observed" in text
    assert "and self.cancel_reason is None" in text
    assert "if self._plan_from_current_pose():" in text


def test_goal_manager_recovers_only_a_short_verified_offroad_drift():
    text = GOAL_MANAGER.read_text(encoding="utf-8")

    assert "RoadKeepoutMap" in text
    assert "make_road_rejoin_target" in text
    assert '"ROAD_REJOIN_PREPARE"' in text
    assert '"DISABLING_LOCAL_KEEPOUT"' in text
    assert '"RESTORING_LOCAL_KEEPOUT"' in text
    assert '"road_keepout_filter.enabled"' in text
    assert "SetParameters" in text
    assert "create_client(" in text
    assert "ROAD_REJOIN_NO_ROAD_ENTRY" in text
    assert "self._publish_road_rejoin_active(False)" in text


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


def test_nav_gps_menu_shutdown_is_idempotent_after_interrupt():
    text = NAV_GPS_MENU.read_text(encoding="utf-8")

    assert "if rclpy.ok():\n            rclpy.shutdown()" in text
