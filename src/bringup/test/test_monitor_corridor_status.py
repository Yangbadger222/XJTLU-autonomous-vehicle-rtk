import importlib.util
from pathlib import Path


MONITOR = Path("scripts/monitor_corridor_status.py")


def _load_monitor_module():
    spec = importlib.util.spec_from_file_location("monitor_corridor_status", MONITOR)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_monitor_reports_nav2_false_success_as_navigation_failure():
    monitor = _load_monitor_module()

    line, running, code = monitor._format_status(
        "gps_route_runner",
        "NAV2_FALSE_SUCCESS_ABORT|1|1|target=13.71|progress=0.00|shortfall=13.71",
    )

    assert "Nav2 误报到达" in line
    assert "progress=0.00" in line
    assert running is True
    assert code == 1


def test_monitor_ignores_clean_aligner_shutdown_status():
    monitor = _load_monitor_module()

    line, running, code = monitor._format_status(
        "gps_global_aligner",
        "ALIGNER_SHUTDOWN",
    )

    assert line == ""
    assert running is False
    assert code is None
