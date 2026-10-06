"""Final research command gate. It preserves the vehicle authority boundary."""
from __future__ import annotations

from dataclasses import dataclass
import math
import time


@dataclass(frozen=True)
class AuthorityState:
    motion_allowed: bool
    authority_stamp: float
    now: float
    health: str = "OK"
    tf_ok: bool = True
    map_ok: bool = True
    trajectory_ok: bool = True
    stop_override: bool = False
    manual_stop: bool = False
    max_linear_speed_mps: float | None = None


@dataclass(frozen=True)
class SafetyCommand:
    linear_x: float
    angular_z: float
    allowed: bool
    reason: str


class SafetyGate:
    """Apply stop-first policy before any command reaches the original guard."""

    def __init__(self, authority_timeout_s: float = 0.50):
        self.authority_timeout_s = authority_timeout_s

    def command(self, requested_linear_x: float, requested_angular_z: float, state: AuthorityState) -> SafetyCommand:
        reasons: list[str] = []
        if not state.motion_allowed:
            reasons.append("rtk_authority_false")
        if state.now - state.authority_stamp > self.authority_timeout_s or state.now < state.authority_stamp:
            reasons.append("authority_stale")
        if state.health.upper() != "OK":
            reasons.append("lio_health_not_ok")
        if not state.tf_ok:
            reasons.append("tf_invalid")
        if not state.map_ok:
            reasons.append("map_invalid")
        if not state.trajectory_ok:
            reasons.append("trajectory_invalid")
        if state.stop_override:
            reasons.append("stop_override")
        if state.manual_stop:
            reasons.append("manual_stop")
        if not math.isfinite(requested_linear_x) or not math.isfinite(requested_angular_z):
            reasons.append("non_finite_command")
        if reasons:
            return SafetyCommand(0.0, 0.0, False, ";".join(reasons))
        linear_x = requested_linear_x
        if state.max_linear_speed_mps is not None:
            linear_x = max(-state.max_linear_speed_mps, min(state.max_linear_speed_mps, linear_x))
        return SafetyCommand(linear_x, requested_angular_z, True, "allowed")


def format_serial(command: SafetyCommand, angular_z_scale: float = 1.0) -> bytes:
    """Mirror the locked serial contract, including the newline."""
    return f"vcx={command.linear_x:g},wc={command.angular_z * angular_z_scale:g}\n".encode("ascii")
