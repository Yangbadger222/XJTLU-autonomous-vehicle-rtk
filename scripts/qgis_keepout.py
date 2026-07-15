#!/usr/bin/env python3
from __future__ import annotations

import math
import sqlite3
import struct
from pathlib import Path

import yaml

try:
    from pyproj import Transformer

    PYPROJ_AVAILABLE = True
except ImportError:
    Transformer = None
    PYPROJ_AVAILABLE = False


EARTH_RADIUS_M = 6378137.0


def _quoted_identifier(identifier: str) -> str:
    if not identifier or not all(char.isalnum() or char == "_" for char in identifier):
        raise ValueError(f"unsafe GeoPackage identifier: {identifier!r}")
    return f'"{identifier}"'


def _gpkg_wkb(blob: bytes) -> memoryview:
    if len(blob) < 8 or blob[:2] != b"GP":
        raise ValueError("invalid GeoPackage geometry header")
    envelope_code = (blob[3] >> 1) & 0x07
    envelope_doubles = {0: 0, 1: 4, 2: 6, 3: 6, 4: 8}.get(envelope_code)
    if envelope_doubles is None:
        raise ValueError(f"unsupported GeoPackage envelope code: {envelope_code}")
    return memoryview(blob)[8 + envelope_doubles * 8 :]


def _wkb_dimensions(type_code: int) -> tuple[int, int]:
    has_z = bool(type_code & 0x80000000)
    has_m = bool(type_code & 0x40000000)
    normalized = type_code & 0x0FFFFFFF
    if normalized >= 3000:
        return normalized - 3000, 4
    if normalized >= 2000:
        return normalized - 2000, 3
    if normalized >= 1000:
        return normalized - 1000, 3
    return normalized, 2 + int(has_z) + int(has_m)


def _read_wkb_geometry(buffer: memoryview, offset: int = 0):
    if offset + 5 > len(buffer):
        raise ValueError("truncated WKB geometry")
    endian = "<" if buffer[offset] == 1 else ">"
    type_code = struct.unpack_from(endian + "I", buffer, offset + 1)[0]
    geometry_type, dimensions = _wkb_dimensions(type_code)
    offset += 5

    if geometry_type == 3:
        ring_count = struct.unpack_from(endian + "I", buffer, offset)[0]
        offset += 4
        rings = []
        for _ in range(ring_count):
            point_count = struct.unpack_from(endian + "I", buffer, offset)[0]
            offset += 4
            ring = []
            for _ in range(point_count):
                values = struct.unpack_from(endian + "d" * dimensions, buffer, offset)
                offset += dimensions * 8
                ring.append((float(values[0]), float(values[1])))
            rings.append(ring)
        return [rings], offset

    if geometry_type == 6:
        polygon_count = struct.unpack_from(endian + "I", buffer, offset)[0]
        offset += 4
        polygons = []
        for _ in range(polygon_count):
            parsed, offset = _read_wkb_geometry(buffer, offset)
            polygons.extend(parsed)
        return polygons, offset

    raise ValueError(f"GeoPackage drivable area must be Polygon/MultiPolygon, got {geometry_type}")


def read_gpkg_polygons(
    gpkg_path: str | Path,
    layer_name: str | None = None,
) -> tuple[list[list[list[tuple[float, float]]]], int, str]:
    path = Path(gpkg_path).expanduser()
    with sqlite3.connect(path) as connection:
        if layer_name:
            row = connection.execute(
                "SELECT table_name, column_name, srs_id, geometry_type_name "
                "FROM gpkg_geometry_columns WHERE table_name = ?",
                (layer_name,),
            ).fetchone()
        else:
            rows = connection.execute(
                "SELECT table_name, column_name, srs_id, geometry_type_name "
                "FROM gpkg_geometry_columns"
            ).fetchall()
            if len(rows) != 1:
                raise ValueError(
                    "GeoPackage must contain one geometry layer or specify "
                    "--road-area-layer"
                )
            row = rows[0]
        if row is None:
            raise ValueError(f"GeoPackage layer not found: {layer_name}")

        table_name, geometry_column, srs_id, geometry_type = row
        if str(geometry_type).upper() not in {"POLYGON", "MULTIPOLYGON"}:
            raise ValueError(
                f"drivable area layer is {geometry_type}, "
                "expected Polygon/MultiPolygon"
            )
        query = "SELECT %s FROM %s WHERE %s IS NOT NULL" % (
            _quoted_identifier(str(geometry_column)),
            _quoted_identifier(str(table_name)),
            _quoted_identifier(str(geometry_column)),
        )
        polygons = []
        for (blob,) in connection.execute(query):
            parsed, _ = _read_wkb_geometry(_gpkg_wkb(blob))
            polygons.extend(parsed)
    if not polygons:
        raise ValueError(f"GeoPackage layer contains no polygons: {path}")
    return polygons, int(srs_id), str(table_name)


def _utm_to_latlon(
    easting: float,
    northing: float,
    zone: int,
    northern: bool,
) -> tuple[float, float]:
    a = 6378137.0
    eccentricity = 0.08181919084262149
    e1sq = eccentricity * eccentricity / (1.0 - eccentricity * eccentricity)
    k0 = 0.9996
    x = float(easting) - 500000.0
    y = float(northing) if northern else float(northing) - 10000000.0
    meridional_arc = y / k0
    mu = meridional_arc / (
        a
        * (
            1.0
            - eccentricity**2 / 4.0
            - 3.0 * eccentricity**4 / 64.0
            - 5.0 * eccentricity**6 / 256.0
        )
    )
    e1 = (1.0 - math.sqrt(1.0 - eccentricity**2)) / (
        1.0 + math.sqrt(1.0 - eccentricity**2)
    )
    j1 = 3.0 * e1 / 2.0 - 27.0 * e1**3 / 32.0
    j2 = 21.0 * e1**2 / 16.0 - 55.0 * e1**4 / 32.0
    j3 = 151.0 * e1**3 / 96.0
    j4 = 1097.0 * e1**4 / 512.0
    fp = mu + j1 * math.sin(2.0 * mu) + j2 * math.sin(4.0 * mu)
    fp += j3 * math.sin(6.0 * mu) + j4 * math.sin(8.0 * mu)
    sin_fp = math.sin(fp)
    cos_fp = math.cos(fp)
    tan_fp = math.tan(fp)
    c1 = e1sq * cos_fp**2
    t1 = tan_fp**2
    n1 = a / math.sqrt(1.0 - eccentricity**2 * sin_fp**2)
    r1 = a * (1.0 - eccentricity**2) / (
        1.0 - eccentricity**2 * sin_fp**2
    ) ** 1.5
    d = x / (n1 * k0)
    latitude = fp - (n1 * tan_fp / r1) * (
        d**2 / 2.0
        - (5.0 + 3.0 * t1 + 10.0 * c1 - 4.0 * c1**2 - 9.0 * e1sq)
        * d**4
        / 24.0
        + (
            61.0
            + 90.0 * t1
            + 298.0 * c1
            + 45.0 * t1**2
            - 252.0 * e1sq
            - 3.0 * c1**2
        )
        * d**6
        / 720.0
    )
    longitude = (
        d
        - (1.0 + 2.0 * t1 + c1) * d**3 / 6.0
        + (
            5.0
            - 2.0 * c1
            + 28.0 * t1
            - 3.0 * c1**2
            + 8.0 * e1sq
            + 24.0 * t1**2
        )
        * d**5
        / 120.0
    ) / cos_fp
    longitude += math.radians((zone - 1) * 6 - 180 + 3)
    return math.degrees(longitude), math.degrees(latitude)


class GpkgToLocalENU:
    def __init__(
        self,
        srs_id: int,
        origin_lat: float,
        origin_lon: float,
        origin_alt: float,
    ) -> None:
        self.srs_id = int(srs_id)
        self.origin_lat = float(origin_lat)
        self.origin_lon = float(origin_lon)
        self.origin_alt = float(origin_alt)
        self._origin_lat_rad = math.radians(self.origin_lat)
        self._to_wgs84 = None
        self._to_enu = None
        if PYPROJ_AVAILABLE:
            self._to_wgs84 = Transformer.from_crs(
                f"EPSG:{self.srs_id}", "EPSG:4326", always_xy=True
            )
            pipeline = (
                "+proj=pipeline +step +proj=cart +ellps=WGS84 "
                f"+step +proj=topocentric +ellps=WGS84 +lat_0={self.origin_lat} "
                f"+lon_0={self.origin_lon} +h_0={self.origin_alt}"
            )
            self._to_enu = Transformer.from_pipeline(pipeline)

    def _fallback_lonlat(self, x: float, y: float) -> tuple[float, float]:
        if 32601 <= self.srs_id <= 32660:
            return _utm_to_latlon(x, y, self.srs_id - 32600, True)
        if 32701 <= self.srs_id <= 32760:
            return _utm_to_latlon(x, y, self.srs_id - 32700, False)
        if self.srs_id == 4326:
            return float(x), float(y)
        raise RuntimeError(
            f"pyproj is required to transform EPSG:{self.srs_id}; install python3-pyproj"
        )

    def transform(self, x: float, y: float) -> tuple[float, float]:
        if self._to_wgs84 is not None and self._to_enu is not None:
            lon, lat = self._to_wgs84.transform(float(x), float(y))
            east, north, _ = self._to_enu.transform(lon, lat, 0.0, radians=False)
            return float(east), float(north)
        lon, lat = self._fallback_lonlat(float(x), float(y))
        east = (
            math.radians(lon - self.origin_lon)
            * EARTH_RADIUS_M
            * math.cos(self._origin_lat_rad)
        )
        north = math.radians(lat - self.origin_lat) * EARTH_RADIUS_M
        return float(east), float(north)


def transform_polygons(
    polygons: list[list[list[tuple[float, float]]]],
    transformer: GpkgToLocalENU,
) -> list[list[list[tuple[float, float]]]]:
    return [
        [[transformer.transform(x, y) for x, y in ring] for ring in polygon]
        for polygon in polygons
    ]


def rasterize_keepout(
    polygons: list[list[list[tuple[float, float]]]],
    *,
    resolution_m: float,
    padding_m: float,
) -> tuple[bytearray, int, int, float, float]:
    if not math.isfinite(resolution_m) or resolution_m <= 0.0:
        raise ValueError("resolution_m must be finite and positive")
    if not math.isfinite(padding_m) or padding_m < 0.0:
        raise ValueError("padding_m must be finite and nonnegative")
    all_points = [point for polygon in polygons for ring in polygon for point in ring]
    if not all_points:
        raise ValueError("drivable area has no polygon points")
    min_x = min(point[0] for point in all_points) - padding_m
    min_y = min(point[1] for point in all_points) - padding_m
    max_x = max(point[0] for point in all_points) + padding_m
    max_y = max(point[1] for point in all_points) + padding_m
    width = max(1, int(math.ceil((max_x - min_x) / resolution_m)))
    height = max(1, int(math.ceil((max_y - min_y) / resolution_m)))
    pixels = bytearray(width * height)

    segments = []
    for polygon in polygons:
        for ring in polygon:
            if len(ring) < 3:
                continue
            closed = ring if ring[0] == ring[-1] else ring + [ring[0]]
            segments.extend(zip(closed, closed[1:]))

    for row in range(height):
        y = max_y - (row + 0.5) * resolution_m
        intersections = []
        for start, end in segments:
            if (start[1] > y) == (end[1] > y):
                continue
            ratio = (y - start[1]) / (end[1] - start[1])
            intersections.append(start[0] + ratio * (end[0] - start[0]))
        intersections.sort()
        for left, right in zip(intersections[0::2], intersections[1::2]):
            start_col = max(0, int(math.ceil((left - min_x) / resolution_m - 0.5)))
            end_col = min(width - 1, int(math.floor((right - min_x) / resolution_m - 0.5)))
            if end_col < start_col:
                continue
            offset = row * width + start_col
            pixels[offset : offset + end_col - start_col + 1] = bytes(
                [254]
            ) * (end_col - start_col + 1)
    return pixels, width, height, min_x, min_y


def compile_gpkg_keepout(
    gpkg_path: str | Path,
    output_dir: str | Path,
    *,
    origin_lat: float,
    origin_lon: float,
    origin_alt: float = 0.0,
    layer_name: str | None = None,
    resolution_m: float = 0.10,
    padding_m: float = 2.0,
) -> dict:
    polygons, srs_id, resolved_layer = read_gpkg_polygons(gpkg_path, layer_name)
    transformer = GpkgToLocalENU(srs_id, origin_lat, origin_lon, origin_alt)
    local_polygons = transform_polygons(polygons, transformer)
    pixels, width, height, origin_x, origin_y = rasterize_keepout(
        local_polygons,
        resolution_m=resolution_m,
        padding_m=padding_m,
    )

    output = Path(output_dir).expanduser()
    output.mkdir(parents=True, exist_ok=True)
    pgm_path = output / "road_keepout.pgm"
    yaml_path = output / "road_keepout.yaml"
    with open(pgm_path, "wb") as pgm_file:
        pgm_file.write(f"P5\n{width} {height}\n255\n".encode("ascii"))
        pgm_file.write(pixels)
    with open(yaml_path, "w", encoding="utf-8") as yaml_file:
        yaml.safe_dump(
            {
                "image": pgm_path.name,
                "mode": "trinary",
                "resolution": float(resolution_m),
                "origin": [round(origin_x, 6), round(origin_y, 6), 0.0],
                "negate": 0,
                "occupied_thresh": 0.65,
                "free_thresh": 0.25,
            },
            yaml_file,
            sort_keys=False,
        )
    return {
        "source": str(Path(gpkg_path).expanduser()),
        "source_layer": resolved_layer,
        "source_srs_id": srs_id,
        "keepout_map_yaml": yaml_path.name,
        "keepout_map_image": pgm_path.name,
        "resolution_m": float(resolution_m),
        "width_cells": width,
        "height_cells": height,
    }
