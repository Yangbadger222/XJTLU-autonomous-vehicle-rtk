from pathlib import Path


LAUNCH = Path(__file__).parents[3] / "src" / "bringup" / "launch" / "system_active_road_research.launch.py"


def test_active_road_launch_allowlist_and_explicit_serial_gate():
    source = LAUNCH.read_text()
    assert "package=\"ego_planner\", executable=\"motion_plan\"" in source
    assert "default_value=\"true\"" in source  # Super-LIO is the shadow/live estimator
    assert "DeclareLaunchArgument(\"enable_serial\", default_value=\"false\"" in source
    assert "' == 'live' and '" in source
    assert "LaunchConfiguration(\"enable_serial\")" in source
    assert "package=\"serial_twistctl\"" in source


def test_research_entry_has_no_legacy_navigation_or_fake_sim_include():
    source = LAUNCH.read_text().lower().split('"""', 2)[-1]
    for forbidden in ("system_explore", "package=\"nav2", "package=\"slam_toolbox",
                      "package=\"mppi", "package=\"fastlio2", "fake_sim_node"):
        assert forbidden not in source
