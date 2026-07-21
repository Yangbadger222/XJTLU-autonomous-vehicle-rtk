import importlib.util
import math
from pathlib import Path


SCRIPT = Path(__file__).resolve().parent / "qgis_keepout.py"
spec = importlib.util.spec_from_file_location("qgis_keepout", SCRIPT)
qgis_keepout = importlib.util.module_from_spec(spec)
spec.loader.exec_module(qgis_keepout)


def test_rasterize_keepout_keeps_polygon_free_and_outside_lethal():
    polygons = [[[[0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [0.0, 2.0], [0.0, 0.0]]]]

    pixels, width, height, origin_x, origin_y = qgis_keepout.rasterize_keepout(
        polygons,
        resolution_m=0.5,
        padding_m=0.5,
    )

    assert (width, height) == (6, 6)
    assert (origin_x, origin_y) == (-0.5, -0.5)
    assert pixels[0] == 0
    assert pixels[2 * width + 2] == 254


def test_rasterize_keepout_expands_drivable_polygon_on_every_side():
    polygons = [[[[0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [0.0, 2.0], [0.0, 0.0]]]]

    pixels, width, height, origin_x, origin_y = qgis_keepout.rasterize_keepout(
        polygons,
        resolution_m=0.5,
        padding_m=0.5,
        drivable_expansion_m=0.5,
    )

    assert (width, height) == (8, 8)
    assert (origin_x, origin_y) == (-1.0, -1.0)
    assert pixels[3 * width + 1] == 254
    assert pixels[0] == 0


def test_rasterize_keepout_rejects_negative_expansion():
    polygons = [[[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]]]

    try:
        qgis_keepout.rasterize_keepout(
            polygons,
            resolution_m=0.5,
            padding_m=0.5,
            drivable_expansion_m=-0.1,
        )
    except ValueError as exc:
        assert "drivable_expansion_m" in str(exc)
    else:
        raise AssertionError("negative drivable expansion must be rejected")


def test_utm_fallback_matches_qgis_map_location():
    lon, lat = qgis_keepout._utm_to_latlon(284500.0, 3462350.0, 51, True)

    assert math.isclose(lon, 120.7365, abs_tol=0.002)
    assert math.isclose(lat, 31.2750, abs_tol=0.002)
