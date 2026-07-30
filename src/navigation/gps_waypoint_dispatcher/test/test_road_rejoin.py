from pathlib import Path

import pytest

from gps_waypoint_dispatcher.road_rejoin import (
    RoadKeepoutMap,
    RoadRejoinReason,
    make_road_rejoin_target,
)


def _write_road_keepout(tmp_path: Path, pixels: bytes) -> Path:
    image = tmp_path / "road_keepout.pgm"
    image.write_bytes(b"P5\n5 5\n255\n" + pixels)
    metadata = tmp_path / "road_keepout.yaml"
    metadata.write_text(
        "\n".join(
            [
                "image: road_keepout.pgm",
                "mode: trinary",
                "resolution: 1.0",
                "origin: [0.0, 0.0, 0.0]",
                "negate: 0",
                "occupied_thresh: 0.65",
                "free_thresh: 0.25",
                "",
            ]
        ),
        encoding="utf-8",
    )
    return metadata


def test_road_keepout_uses_map_origin_and_pgm_row_orientation(tmp_path):
    pixels = bytearray(25)
    # PGM row 2 is world y in [2, 3) for a five-cell map with origin y=0.
    pixels[2 * 5 + 2] = 254
    keepout = RoadKeepoutMap.load(_write_road_keepout(tmp_path, bytes(pixels)))

    assert keepout.is_drivable(2.5, 2.5)
    assert not keepout.is_drivable(2.5, 1.5)
    assert not keepout.is_drivable(-0.1, 2.5)
    assert keepout.nearest_drivable_distance(1.5, 2.5, 1.1) == pytest.approx(1.0)


def test_binary_pgm_does_not_discard_a_whitespace_valued_first_pixel(tmp_path):
    # The first pixel is LF (10). A binary PGM parser must consume only the
    # header delimiter, not treat this valid pixel as additional whitespace.
    pixels = bytes([10]) + bytes(24)
    keepout = RoadKeepoutMap.load(_write_road_keepout(tmp_path, pixels))

    assert keepout.pixels == pixels


def test_rejoin_requires_short_verified_road_projection(tmp_path):
    pixels = bytearray(25)
    pixels[2 * 5 + 2] = 254
    keepout = RoadKeepoutMap.load(_write_road_keepout(tmp_path, bytes(pixels)))

    decision = make_road_rejoin_target(
        keepout,
        current_xy=(1.5, 2.5),
        graph_xy=(2.5, 2.5),
        max_outside_distance_m=1.1,
        max_graph_distance_m=1.25,
    )

    assert decision.approved
    assert decision.reason is RoadRejoinReason.REJOIN_APPROVED
    target = decision.target
    assert target is not None
    assert (target.x, target.y) == (2.5, 2.5)
    assert target.outside_distance_m == pytest.approx(1.0)
    assert target.graph_distance_m == pytest.approx(1.0)
    rejected = make_road_rejoin_target(
        keepout,
        current_xy=(1.5, 2.5),
        graph_xy=(4.5, 2.5),
        max_outside_distance_m=1.1,
        max_graph_distance_m=4.0,
    )
    assert not rejected.approved
    assert rejected.reason is RoadRejoinReason.GRAPH_PROJECTION_OUTSIDE_MASK
    assert rejected.nearest_mask_xy == (2.5, 2.5)
    assert rejected.graph_projection_distance_m == pytest.approx(3.0)
    assert "nearest_road_distance_m=1.00" in rejected.status_fields()
