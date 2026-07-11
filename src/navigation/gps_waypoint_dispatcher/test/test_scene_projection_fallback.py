from pathlib import Path


SCENE_RUNTIME = Path("src/navigation/gps_waypoint_dispatcher/gps_waypoint_dispatcher/scene_runtime.py")
BUILD_SCENE_RUNTIME = Path("scripts/build_scene_runtime.py")
GPS_ANCHOR_LOCALIZER = Path(
    "src/sensor_drivers/gnss/gnss_calibration/gnss_calibration/gps_anchor_localizer_node.py"
)


def test_scene_runtime_imports_pyproj_optionally():
    text = SCENE_RUNTIME.read_text(encoding="utf-8")

    assert "try:" in text
    assert "from pyproj import CRS, Transformer" in text
    assert "PYPROJ_AVAILABLE = False" in text
    assert "LocalENUProjector" in text


def test_build_scene_runtime_has_no_pyproj_hard_failure():
    text = BUILD_SCENE_RUNTIME.read_text(encoding="utf-8")

    assert "try:" in text
    assert "from pyproj import Transformer" in text
    assert "PYPROJ_AVAILABLE = False" in text
    assert "LocalENUProjector" in text


def test_gps_anchor_localizer_has_projection_fallback():
    text = GPS_ANCHOR_LOCALIZER.read_text(encoding="utf-8")

    assert "try:" in text
    assert "from pyproj import Transformer" in text
    assert "PYPROJ_AVAILABLE = False" in text
    assert "LocalENUProjector" in text
