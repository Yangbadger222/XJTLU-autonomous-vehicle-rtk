"""Explicit pixel/depth to local-map geometry for active-road evidence.

The functions require measured intrinsics and rigid transforms. They do not
invent a camera model, CRS transform, or vehicle extrinsic when one is absent.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence


def _finite(values: Sequence[float]) -> bool:
    return all(math.isfinite(float(value)) for value in values)


def _qnormalize(q: Sequence[float]) -> tuple[float, float, float, float]:
    if len(q) != 4 or not _finite(q):
        raise ValueError("quaternion must contain four finite values")
    norm = math.sqrt(sum(float(value) * float(value) for value in q))
    if norm <= 1e-12:
        raise ValueError("quaternion norm must be non-zero")
    return tuple(float(value) / norm for value in q)  # type: ignore[return-value]


def _qmul(a: Sequence[float], b: Sequence[float]) -> tuple[float, float, float, float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def _qconj(q: Sequence[float]) -> tuple[float, float, float, float]:
    return (-q[0], -q[1], -q[2], q[3])


def _qrotate(q: Sequence[float], point: Sequence[float]) -> tuple[float, float, float]:
    normalized = _qnormalize(q)
    rotated = _qmul(_qmul(normalized, (point[0], point[1], point[2], 0.0)), _qconj(normalized))
    return rotated[:3]


@dataclass(frozen=True)
class CameraIntrinsics:
    """Measured pinhole intrinsics and the declared depth convention."""

    fx_px: float
    fy_px: float
    cx_px: float
    cy_px: float
    depth_unit_m: float = 1.0
    depth_mode: str = "optical_z"

    def __post_init__(self) -> None:
        values = (self.fx_px, self.fy_px, self.cx_px, self.cy_px, self.depth_unit_m)
        if not _finite(values) or self.fx_px <= 0.0 or self.fy_px <= 0.0 or self.depth_unit_m <= 0.0:
            raise ValueError("camera intrinsics must be finite with positive focal lengths/unit")
        if self.depth_mode not in {"optical_z", "ray_range"}:
            raise ValueError("depth_mode must be optical_z or ray_range")


@dataclass(frozen=True)
class RigidTransform:
    """A measured transform from the source frame into the destination frame."""

    translation_m: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]

    def __post_init__(self) -> None:
        if len(self.translation_m) != 3 or not _finite(self.translation_m):
            raise ValueError("transform translation must be three finite values")
        _qnormalize(self.quaternion_xyzw)

    def apply(self, point: Sequence[float]) -> tuple[float, float, float]:
        if len(point) != 3 or not _finite(point):
            raise ValueError("point must contain three finite values")
        rotated = _qrotate(self.quaternion_xyzw, point)
        return tuple(rotated[index] + self.translation_m[index] for index in range(3))


def pixel_to_camera(u_px: float, v_px: float, depth: float,
                    intrinsics: CameraIntrinsics) -> tuple[float, float, float]:
    """Convert one valid pixel/depth sample into the optical camera frame."""
    if not _finite((u_px, v_px, depth)) or depth <= 0.0:
        raise ValueError("depth sample must be finite and strictly positive")
    depth_m = float(depth) * intrinsics.depth_unit_m
    x_over_z = (float(u_px) - intrinsics.cx_px) / intrinsics.fx_px
    y_over_z = (float(v_px) - intrinsics.cy_px) / intrinsics.fy_px
    if intrinsics.depth_mode == "optical_z":
        return (x_over_z * depth_m, y_over_z * depth_m, depth_m)
    ray = (x_over_z, y_over_z, 1.0)
    norm = math.sqrt(sum(component * component for component in ray))
    return tuple(component * depth_m / norm for component in ray)


def pixel_to_odom(u_px: float, v_px: float, depth: float,
                  intrinsics: CameraIntrinsics,
                  camera_to_base: RigidTransform,
                  base_to_odom: RigidTransform) -> tuple[float, float, float]:
    """Apply the acquisition-time camera→base→odom chain exactly once."""
    camera_point = pixel_to_camera(u_px, v_px, depth, intrinsics)
    return base_to_odom.apply(camera_to_base.apply(camera_point))
