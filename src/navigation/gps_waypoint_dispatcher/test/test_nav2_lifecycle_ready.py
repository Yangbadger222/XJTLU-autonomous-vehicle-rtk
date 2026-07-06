from gps_waypoint_dispatcher.nav2_lifecycle_ready import (
    ACTIVE_LIFECYCLE_STATE_ID,
    DEFAULT_REQUIRED_NAV2_LIFECYCLE_NODES,
    summarize_lifecycle_states,
)


def test_nav2_ready_requires_bt_navigator_active():
    states = {
        "controller_server": ACTIVE_LIFECYCLE_STATE_ID,
        "planner_server": ACTIVE_LIFECYCLE_STATE_ID,
        "behavior_server": ACTIVE_LIFECYCLE_STATE_ID,
        "bt_navigator": 2,
    }

    summary = summarize_lifecycle_states(DEFAULT_REQUIRED_NAV2_LIFECYCLE_NODES, states)

    assert summary.ready is False
    assert summary.inactive_nodes == ("bt_navigator",)


def test_nav2_ready_when_all_required_lifecycle_nodes_are_active():
    states = {
        "controller_server": ACTIVE_LIFECYCLE_STATE_ID,
        "planner_server": ACTIVE_LIFECYCLE_STATE_ID,
        "behavior_server": ACTIVE_LIFECYCLE_STATE_ID,
        "bt_navigator": ACTIVE_LIFECYCLE_STATE_ID,
    }

    summary = summarize_lifecycle_states(DEFAULT_REQUIRED_NAV2_LIFECYCLE_NODES, states)

    assert summary.ready is True
    assert summary.missing_nodes == ()
    assert summary.inactive_nodes == ()
