"""Evidence-first road map and active observation policy.

This module intentionally has no access to a simulator truth map. Truth is an
evaluator-only concern in the replay harness.
"""
from __future__ import annotations

from dataclasses import dataclass, asdict
from enum import Enum
import json
import math
from pathlib import Path
from typing import Iterable


class EvidenceState(str, Enum):
    UNOBSERVED = "UNOBSERVED"
    OBSERVED_GEOMETRY = "OBSERVED_GEOMETRY"
    TRAVERSED = "TRAVERSED"
    BLOCKED_EVIDENCE = "BLOCKED_EVIDENCE"
    UNCERTAIN = "UNCERTAIN"


@dataclass(frozen=True)
class GeoTransform:
    crs: str
    datum: str
    origin_x_m: float
    origin_y_m: float
    pixel_size_x_m: float
    pixel_size_y_m: float
    axis_order: str = "x_east_y_north"

    def pixel_to_local(self, col: float, row: float) -> tuple[float, float]:
        return (self.origin_x_m + (col + 0.5) * self.pixel_size_x_m,
                self.origin_y_m - (row + 0.5) * self.pixel_size_y_m)

    def local_to_pixel(self, x_m: float, y_m: float) -> tuple[float, float]:
        """Inverse pixel-center mapping; CRS metadata remains attached."""
        if not all(math.isfinite(float(value)) for value in
                   (x_m, y_m, self.pixel_size_x_m, self.pixel_size_y_m)):
            raise ValueError("pixel transform requires finite values")
        if self.pixel_size_x_m == 0.0 or self.pixel_size_y_m == 0.0:
            raise ValueError("pixel size must be non-zero")
        return ((x_m - self.origin_x_m) / self.pixel_size_x_m - 0.5,
                (self.origin_y_m - y_m) / self.pixel_size_y_m - 0.5)


@dataclass(frozen=True)
class GeoTiffPrior:
    """Read-only GeoTIFF metadata boundary; pixels never become evidence by themselves."""
    path: str
    crs: str
    transform: tuple[float, float, float, float, float, float]
    width: int
    height: int

    @classmethod
    def load(cls, path: str | Path) -> "GeoTiffPrior":
        try:
            import rasterio
        except ImportError as exc:
            raise RuntimeError("rasterio is required to load GeoTIFF metadata; prior remains unavailable") from exc
        with rasterio.open(path) as dataset:
            if dataset.crs is None:
                raise ValueError("GeoTIFF CRS is required; refusing axis/datum guessing")
            transform = dataset.transform
            return cls(str(path), dataset.crs.to_string(),
                       (transform.a, transform.b, transform.c, transform.d, transform.e, transform.f),
                       dataset.width, dataset.height)


@dataclass(frozen=True)
class MaGRoadPrior:
    """Minimal source-grounded road graph loader (GeoJSON, read-only)."""
    path: str
    model_version: str
    crs: str
    edges: tuple[dict, ...]

    @classmethod
    def load_geojson(cls, path: str | Path, *, expected_crs: str, model_version: str) -> "MaGRoadPrior":
        payload = json.loads(Path(path).read_text())
        declared = payload.get("crs", {}).get("properties", {}).get("name", expected_crs)
        if declared != expected_crs:
            raise ValueError(f"MaGRoad CRS mismatch: {declared} != {expected_crs}")
        edges = []
        for feature in payload.get("features", []):
            if feature.get("geometry", {}).get("type") != "LineString":
                continue
            coords = feature["geometry"].get("coordinates", [])
            if len(coords) < 2:
                continue
            edges.append({"id": str(feature.get("properties", {}).get("id", len(edges))),
                          "coordinates": [tuple(point[:2]) for point in coords],
                          "properties": feature.get("properties", {})})
        return cls(str(path), model_version, expected_crs, tuple(edges))


@dataclass
class RoadEvidence:
    evidence_id: str
    geometry_xy: list[tuple[float, float]]
    state: EvidenceState
    stamp: float
    source: str
    local_submap_id: str
    pose_uncertainty_m: float
    observed_length_m: float
    valid_depth_m: tuple[float, float] | None = None


@dataclass
class RoadEvent:
    event_id: str
    kind: str
    location_xy: tuple[float, float]
    observed_length_m: float
    unknown_length_m: float
    state: EvidenceState
    impact: float
    blocked_by: str = ""


@dataclass(frozen=True)
class ObservationCandidate:
    candidate_id: str
    event_id: str
    reachable: bool
    safe: bool
    pose_trustworthy: bool
    sensor_valid: bool
    impact: float
    observable_fraction: float
    cost: float
    reason: str = ""

    @property
    def score(self) -> float:
        if not (self.reachable and self.safe and self.pose_trustworthy and self.sensor_valid):
            return float("-inf")
        return self.impact * max(0.0, min(1.0, self.observable_fraction)) / (self.cost + 1e-6)


class EvidenceStore:
    def __init__(self, transform: GeoTransform, map_version: str):
        self.transform = transform
        self.map_version = map_version
        self._evidence: dict[str, RoadEvidence] = {}

    def add(self, evidence: RoadEvidence) -> bool:
        """Add once by UUID; replaying the same observation is idempotent."""
        if evidence.evidence_id in self._evidence:
            return False
        self._evidence[evidence.evidence_id] = evidence
        return True

    def evidence(self) -> list[RoadEvidence]:
        return list(self._evidence.values())

    def save(self, path: str | Path) -> None:
        payload = {"schema": 1, "map_version": self.map_version, "transform": asdict(self.transform),
                   "evidence": [{**asdict(e), "state": e.state.value} for e in self.evidence()]}
        Path(path).write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")

    @classmethod
    def load(cls, path: str | Path) -> "EvidenceStore":
        payload = json.loads(Path(path).read_text())
        store = cls(GeoTransform(**payload["transform"]), payload["map_version"])
        for item in payload["evidence"]:
            item["state"] = EvidenceState(item["state"])
            item["geometry_xy"] = [tuple(p) for p in item["geometry_xy"]]
            store.add(RoadEvidence(**item))
        return store


def choose_observation(candidates: Iterable[ObservationCandidate]) -> ObservationCandidate | None:
    eligible = [c for c in candidates if math.isfinite(c.score)]
    return max(eligible, key=lambda c: (c.score, c.candidate_id), default=None)
