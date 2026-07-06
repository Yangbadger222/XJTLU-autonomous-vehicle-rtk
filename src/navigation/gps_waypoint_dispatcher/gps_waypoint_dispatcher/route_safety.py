from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class MapGpsConsistencySummary:
    ok: bool
    warn: bool
    distance_m: float


def summarize_map_gps_consistency(
    map_xy: tuple[float, float],
    gps_map_xy: tuple[float, float],
    warn_m: float,
    abort_m: float,
) -> MapGpsConsistencySummary:
    distance_m = math.hypot(map_xy[0] - gps_map_xy[0], map_xy[1] - gps_map_xy[1])
    return MapGpsConsistencySummary(
        ok=distance_m <= abort_m,
        warn=distance_m > warn_m,
        distance_m=distance_m,
    )
