"""Evidence-first road map and active observation policy.

This module intentionally has no access to a simulator truth map. Truth is an
evaluator-only concern in the replay harness.
"""
from __future__ import annotations

from dataclasses import dataclass, asdict
from enum import Enum
import json
import math
import hashlib
import copy
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

    def __post_init__(self) -> None:
        if not self.crs or not self.datum or not self.axis_order:
            raise ValueError("CRS, datum and axis order are required")
        values = (self.origin_x_m, self.origin_y_m, self.pixel_size_x_m, self.pixel_size_y_m)
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("geotransform values must be finite")
        if self.pixel_size_x_m == 0.0 or self.pixel_size_y_m == 0.0:
            raise ValueError("pixel size must be non-zero")

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

    def pixel_to_crs(self, col: float, row: float):
        """Full affine pixel-center transform, including rotation/shear."""
        a,b,c,d,e,f = self.transform
        if not all(math.isfinite(v) for v in (*self.transform, col, row)):
            raise ValueError("finite GeoTIFF transform required")
        return a*(col+.5)+b*(row+.5)+c, d*(col+.5)+e*(row+.5)+f

    def crs_to_pixel(self, x: float, y: float):
        a,b,c,d,e,f = self.transform
        determinant = a*e-b*d
        if not all(math.isfinite(v) for v in (*self.transform, x, y)) or abs(determinant) < 1e-15:
            raise ValueError("finite invertible GeoTIFF transform required")
        return ((e*(x-c)-b*(y-f))/determinant-.5,
                (-d*(x-c)+a*(y-f))/determinant-.5)

    def cropped(self, col_offset: int, row_offset: int, width: int, height: int):
        if (col_offset < 0 or row_offset < 0 or width <= 0 or height <= 0 or
                col_offset+width > self.width or row_offset+height > self.height):
            raise ValueError("crop must lie within the original GeoTIFF")
        a,b,c,d,e,f = self.transform
        return GeoTiffPrior(self.path, self.crs, (a,b,c+a*col_offset+b*row_offset,
                            d,e,f+d*col_offset+e*row_offset), width, height)

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
        declared = payload.get("crs", {}).get("properties", {}).get("name")
        if not declared:
            raise ValueError("MaGRoad CRS is required; refusing to infer CRS")
        if declared != expected_crs:
            raise ValueError(f"MaGRoad CRS mismatch: {declared} != {expected_crs}")
        if not model_version:
            raise ValueError("MaGRoad model version is required")
        edges = []
        for feature in payload.get("features", []):
            if feature.get("geometry", {}).get("type") != "LineString":
                continue
            coords = feature["geometry"].get("coordinates", [])
            if len(coords) < 2:
                continue
            if any(len(point) < 2 or not all(math.isfinite(float(value)) for value in point[:2])
                   for point in coords):
                raise ValueError("MaGRoad coordinates must be finite XY points")
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
        if not all(math.isfinite(float(value)) for value in
                   (self.impact, self.observable_fraction, self.cost)):
            return float("-inf")
        if self.impact < 0.0 or not 0.0 <= self.observable_fraction <= 1.0 or self.cost < 0.0:
            return float("-inf")
        return self.impact * self.observable_fraction / max(self.cost, 1e-6)


class EvidenceStore:
    def __init__(self, transform: GeoTransform, map_version: str):
        if not str(map_version).strip() or str(map_version).strip().upper() == "UNKNOWN":
            raise ValueError("a persisted map version must be known")
        self.transform = transform
        self.map_version = str(map_version).strip()
        self.prior_version = self.map_version
        self._evidence: dict[str, RoadEvidence] = {}
        self.submap_anchors = {}
        self.graph_updates = {}
        self.rolled_back_graph_updates = {}

    def add(self, evidence: RoadEvidence) -> bool:
        """Add once by UUID; replaying the same observation is idempotent."""
        self._validate_evidence(evidence)
        if evidence.evidence_id in self._evidence:
            if asdict(self._evidence[evidence.evidence_id]) != asdict(evidence):
                raise ValueError("an evidence UUID cannot refer to different measurements")
            return False
        self._evidence[evidence.evidence_id] = copy.deepcopy(evidence)
        self._update_version()
        return True

    def _update_version(self):
        content = {"prior_version": self.prior_version, "transform": asdict(self.transform),
                   "evidence": [asdict(self._evidence[key]) for key in sorted(self._evidence)]}
        if self.submap_anchors:
            content["submap_anchors"] = self.submap_anchors
        if self.graph_updates:
            content["graph_updates"]=self.graph_updates
        if self.rolled_back_graph_updates:
            content["rolled_back_graph_updates"]=self.rolled_back_graph_updates
        self.map_version = hashlib.sha256(json.dumps(content,sort_keys=True).encode()).hexdigest()

    def anchor_submap(self, submap_id, map_from_local_xyyaw, *, stamp, uncertainty_m, authority_valid):
        if (authority_valid is not True or not submap_id or len(map_from_local_xyyaw)!=3 or
                not all(math.isfinite(value) for value in (*map_from_local_xyyaw,stamp,uncertainty_m)) or
                stamp<=0 or uncertainty_m<0):
            raise ValueError("verified current authority, acquisition time and explicit anchor uncertainty required")
        self.submap_anchors[submap_id] = {"target_frame":"map", "map_from_local_xyyaw":list(map_from_local_xyyaw),
            "stamp":stamp,"uncertainty_m":uncertainty_m,"state":"ANCHORED",
            "source":"selected_rtk_map_odom_authority"}
        self._update_version()

    def mark_anchors_stale(self):
        changed=False
        for anchor in self.submap_anchors.values():
            if anchor["state"]!="STALE":anchor["state"]="STALE";changed=True
        if changed:self._update_version()
        return changed

    def geometry_in_map(self,evidence_id):
        evidence=self._evidence[evidence_id]
        anchor=self.submap_anchors.get(evidence.local_submap_id)
        if not anchor or anchor["state"]!="ANCHORED":return None
        tx,ty,yaw=anchor["map_from_local_xyyaw"];c,s=math.cos(yaw),math.sin(yaw)
        return [(tx+c*x-s*y,ty+s*x+c*y) for x,y in evidence.geometry_xy]

    def add_graph_update(self,update):
        if update['prior_version']!=self.prior_version or not update['update_id']:
            raise ValueError('graph increment must bind the immutable prior')
        if not update['start_node_id'] or not update['end_node_id'] or update['start_node_id']==update['end_node_id']:
            raise ValueError('distinct graph endpoints required')
        geometry=[tuple(p) for p in update['geometry_xy']]
        length=sum(math.dist(a,b) for a,b in zip(geometry,geometry[1:]))
        self._validate_evidence(RoadEvidence(update['update_id'],geometry,EvidenceState.OBSERVED_GEOMETRY,
            update['stamp'],update['source'],update['local_submap_id'],update['pose_uncertainty_m'],length))
        if not math.isfinite(update['supported_width_m']) or update['supported_width_m']<=0:
            raise ValueError('positive measured-support width required')
        if not update['evidence_ids'] or any(identity not in self._evidence or
            self._evidence[identity].local_submap_id!=update['local_submap_id'] for identity in update['evidence_ids']):
            raise ValueError('graph increment needs persisted measurement UUIDs from its submap')
        if update['update_id'] in self.graph_updates:
            if json.dumps(self.graph_updates[update['update_id']],sort_keys=True)!=json.dumps(update,sort_keys=True):
                raise ValueError('graph update UUID cannot refer to different observations')
            return False
        self.graph_updates[update['update_id']]=copy.deepcopy(update);self._update_version();return True

    def rollback_graph_update(self,update_id,*,reason,stamp):
        if update_id not in self.graph_updates or not reason or not math.isfinite(stamp):
            raise ValueError("existing graph increment, reason and finite rollback time required")
        self.rolled_back_graph_updates[update_id]={"reason":str(reason),"stamp":stamp}
        self._update_version()

    def graph_updates_in_map(self):
        result=[]
        for update in self.graph_updates.values():
            if update["update_id"] in self.rolled_back_graph_updates:continue
            anchor=self.submap_anchors.get(update['local_submap_id'])
            if not anchor or anchor['state']!='ANCHORED':continue
            tx,ty,yaw=anchor['map_from_local_xyyaw'];c,s=math.cos(yaw),math.sin(yaw)
            transformed=copy.deepcopy(update)
            transformed['geometry_xy']=[(tx+c*x-s*y,ty+s*x+c*y) for x,y in update['geometry_xy']]
            transformed['anchor_uncertainty_m']=anchor['uncertainty_m'];result.append(transformed)
        return result

    @staticmethod
    def _validate_evidence(evidence: RoadEvidence) -> None:
        if not evidence.evidence_id or not evidence.source or not evidence.local_submap_id:
            raise ValueError("evidence id, source and local_submap_id are required")
        if not isinstance(evidence.state, EvidenceState):
            raise ValueError("evidence state must use the controlled vocabulary")
        if len(evidence.geometry_xy) < 2:
            raise ValueError("evidence geometry requires at least two points")
        if any(len(point) != 2 or not all(math.isfinite(float(value)) for value in point)
               for point in evidence.geometry_xy):
            raise ValueError("evidence geometry must be finite XY points")
        if not math.isfinite(evidence.stamp):
            raise ValueError("evidence stamp must be finite")
        if not math.isfinite(evidence.pose_uncertainty_m) or evidence.pose_uncertainty_m < 0.0:
            raise ValueError("pose uncertainty must be finite and non-negative")
        if not math.isfinite(evidence.observed_length_m) or evidence.observed_length_m < 0.0:
            raise ValueError("observed length must be finite and non-negative")
        geometry_length = sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a,b in
                              zip(evidence.geometry_xy, evidence.geometry_xy[1:]))
        if evidence.observed_length_m > geometry_length + 1e-6:
            raise ValueError("observed length cannot extend beyond measured geometry")
        if evidence.valid_depth_m is not None:
            if (len(evidence.valid_depth_m) != 2 or
                    not all(math.isfinite(float(value)) for value in evidence.valid_depth_m) or
                    evidence.valid_depth_m[0] <= 0.0 or
                    evidence.valid_depth_m[0] > evidence.valid_depth_m[1]):
                raise ValueError("valid depth interval must be positive, finite and ordered")

    def evidence(self) -> list[RoadEvidence]:
        return copy.deepcopy(list(self._evidence.values()))

    def save(self, path: str | Path) -> None:
        payload = {"schema": 1, "map_version": self.map_version, "prior_version": self.prior_version,
                   "transform": asdict(self.transform),
                   "submap_anchors":self.submap_anchors,
                   "graph_updates":self.graph_updates,
                   "rolled_back_graph_updates":self.rolled_back_graph_updates,
                   "evidence": [{**asdict(e), "state": e.state.value} for e in self.evidence()]}
        Path(path).write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")

    @classmethod
    def load(cls, path: str | Path) -> "EvidenceStore":
        payload = json.loads(Path(path).read_text())
        if payload.get("schema") != 1:
            raise ValueError("unsupported evidence store schema")
        if not isinstance(payload.get("evidence"), list):
            raise ValueError("evidence store list is required")
        store = cls(GeoTransform(**payload["transform"]), payload.get("prior_version",payload["map_version"]))
        for submap_id,anchor in payload.get("submap_anchors",{}).items():
            if anchor.get("target_frame")!="map" or anchor.get("state") not in ("ANCHORED","STALE"):
                raise ValueError("invalid persisted submap anchor")
            store.anchor_submap(submap_id,anchor["map_from_local_xyyaw"],stamp=anchor["stamp"],
                uncertainty_m=anchor["uncertainty_m"],authority_valid=True)
            store.submap_anchors[submap_id]["state"]=anchor["state"]
        if store.submap_anchors:store._update_version()
        for item in payload["evidence"]:
            item["state"] = EvidenceState(item["state"])
            item["geometry_xy"] = [tuple(p) for p in item["geometry_xy"]]
            if item.get("valid_depth_m") is not None:
                item["valid_depth_m"] = tuple(item["valid_depth_m"])
            if not store.add(RoadEvidence(**item)):
                raise ValueError("duplicate evidence_id in persisted store")
        for update in payload.get('graph_updates',{}).values():store.add_graph_update(update)
        for identity,record in payload.get('rolled_back_graph_updates',{}).items():
            store.rollback_graph_update(identity,reason=record['reason'],stamp=record['stamp'])
        if "prior_version" in payload and store.map_version != payload["map_version"]:
            raise ValueError("persisted evidence content/version mismatch")
        if "prior_version" not in payload:
            # Legacy schema-1 stores had an externally assigned version.
            # Keep it readable; the next measured update produces a hash.
            store.map_version = payload["map_version"]
        return store


def choose_observation(candidates: Iterable[ObservationCandidate]) -> ObservationCandidate | None:
    eligible = [c for c in candidates if math.isfinite(c.score)]
    return max(eligible, key=lambda c: (c.score, c.candidate_id), default=None)
