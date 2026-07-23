from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class GuardedCommand:
    linear_x: float
    angular_z: float
    allowed: bool
    reason: str


class CorridorCommandGuard:
    def __init__(
        self,
        *,
        straight_max_mps: float = 0.85,
        turn_product_limit: float = 0.25,
        min_turn_rate_radps: float = 0.05,
        command_timeout_s: float = 0.25,
        heartbeat_timeout_s: float = 0.50,
        require_speed_limit: bool = False,
        road_rejoin_max_mps: float = 0.35,
    ) -> None:
        values = (
            straight_max_mps,
            turn_product_limit,
            min_turn_rate_radps,
            command_timeout_s,
            heartbeat_timeout_s,
            road_rejoin_max_mps,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError("guard limits and timeouts must be finite and positive")
        self.straight_max_mps = straight_max_mps
        self.turn_product_limit = turn_product_limit
        self.min_turn_rate_radps = min_turn_rate_radps
        self.command_timeout_s = command_timeout_s
        self.heartbeat_timeout_s = heartbeat_timeout_s
        self.require_speed_limit = bool(require_speed_limit)
        self.road_rejoin_max_mps = road_rejoin_max_mps
        self._command: tuple[float, float] | None = None
        self._command_received_s: float | None = None
        self._authority_allowed: bool | None = None
        self._authority_received_s: float | None = None
        self._stop_override: bool | None = None
        self._stop_received_s: float | None = None
        self._speed_limit_mps: float | None = None
        self._speed_limit_received_s: float | None = None
        self._road_rejoin_active = False

    def update_command(
        self, linear_x: float, angular_z: float, *, received_s: float
    ) -> None:
        self._command = (float(linear_x), float(angular_z))
        self._command_received_s = received_s

    def update_authority(self, allowed: bool, *, received_s: float) -> None:
        self._authority_allowed = bool(allowed)
        self._authority_received_s = received_s

    def update_stop_override(self, stop: bool, *, received_s: float) -> None:
        self._stop_override = bool(stop)
        self._stop_received_s = received_s

    def update_speed_limit(self, max_linear_speed_mps: float, *, received_s: float) -> None:
        self._speed_limit_mps = float(max_linear_speed_mps)
        self._speed_limit_received_s = received_s

    def update_road_rejoin(self, active: bool) -> None:
        self._road_rejoin_active = bool(active)

    @staticmethod
    def _age(now_s: float, received_s: float | None) -> float:
        if received_s is None or not all(
            math.isfinite(value) for value in (now_s, received_s)
        ):
            return math.inf
        return now_s - received_s

    @staticmethod
    def _zero(reason: str) -> GuardedCommand:
        return GuardedCommand(0.0, 0.0, False, reason)

    def evaluate(self, *, now_s: float) -> GuardedCommand:
        if self._command is None:
            return self._zero("COMMAND_UNAVAILABLE")
        if self._authority_allowed is None:
            return self._zero("AUTHORITY_UNAVAILABLE")
        if self._stop_override is None:
            return self._zero("STOP_OVERRIDE_UNAVAILABLE")
        if self._age(now_s, self._command_received_s) > self.command_timeout_s:
            return self._zero("COMMAND_STALE")
        if self._age(now_s, self._authority_received_s) > self.heartbeat_timeout_s:
            return self._zero("AUTHORITY_STALE")
        if self._age(now_s, self._stop_received_s) > self.heartbeat_timeout_s:
            return self._zero("STOP_OVERRIDE_STALE")
        if not self._authority_allowed:
            return self._zero("MOTION_AUTHORITY_FALSE")
        if self._stop_override:
            return self._zero("STOP_OVERRIDE_TRUE")
        if self.require_speed_limit and self._speed_limit_mps is None:
            return self._zero("SPEED_LIMIT_UNAVAILABLE")
        if self.require_speed_limit and (
            self._age(now_s, self._speed_limit_received_s) > self.heartbeat_timeout_s
        ):
            return self._zero("SPEED_LIMIT_STALE")
        if self._speed_limit_mps is not None:
            if not math.isfinite(self._speed_limit_mps) or self._speed_limit_mps < 0.0:
                return self._zero("SPEED_LIMIT_INVALID")
            if self._speed_limit_mps == 0.0:
                return self._zero("SPEED_LIMIT_ZERO")

        linear_x, angular_z = self._command
        if not all(math.isfinite(value) for value in (linear_x, angular_z)):
            return self._zero("NONFINITE_COMMAND")
        v_limit = min(
            self.straight_max_mps,
            self.turn_product_limit
            / max(abs(angular_z), self.min_turn_rate_radps),
        )
        if self._speed_limit_mps is not None:
            v_limit = min(v_limit, self._speed_limit_mps)
        if self._road_rejoin_active:
            v_limit = min(v_limit, self.road_rejoin_max_mps)
        limited_x = math.copysign(min(abs(linear_x), v_limit), linear_x)
        return GuardedCommand(limited_x, angular_z, True, "ALLOWED")
