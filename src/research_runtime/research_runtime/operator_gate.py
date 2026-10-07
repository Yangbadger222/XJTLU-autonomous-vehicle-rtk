"""Fresh, ordered human consent shared by observer and final tracker.

Receipt time uses a steady clock. Duplicate messages cannot renew a lease.
The sole publisher is checked by the ROS adapters, outside this interface.
"""
from dataclasses import dataclass
import math
from .physical_parameter_lock import AUTHORITY_HEARTBEAT_TIMEOUT_S


@dataclass(frozen=True)
class OperatorConsent:
    session_id: str
    sequence: int
    mode: str
    map_version: str
    state: str
    motion_requested: bool
    lease_active: bool = False


class OperatorGate:
    TIMEOUT_S = AUTHORITY_HEARTBEAT_TIMEOUT_S

    def __init__(self):
        self._consent = None
        self._received = -math.inf
        self._sessions = {}  # Keep high-water marks across session changes.
        self._latched = True
        self._armed_session = ""

    def receive(self, consent, received_at):
        if (not isinstance(consent, OperatorConsent) or not consent.session_id or
                len(consent.session_id) > 128 or type(consent.sequence) is not int or
                consent.sequence <= self._sessions.get(consent.session_id, 0) or
                not math.isfinite(received_at)):
            return False
        if len(self._sessions) >= 100 and consent.session_id not in self._sessions:
            return False  # Restart an exhausted isolated test session safely.
        if received_at-self._received > self.TIMEOUT_S or (self._consent and consent.session_id!=self._consent.session_id):
            self._latched=True
        self._sessions[consent.session_id] = consent.sequence
        self._consent, self._received = consent, received_at
        return True

    def allowed(self, now, *, mode, map_version, sole_publisher=True):
        c = self._consent
        transport=bool(sole_publisher and c and 0<=now-self._received<=self.TIMEOUT_S and
                       mode=="live" and c.mode==mode and c.lease_active is True)
        if not transport:
            self._latched=True
            return False
        # Evidence versions advance during a task. Deny a mismatched snapshot
        # until both inputs agree; this cannot renew a lost operator lease.
        if map_version in ("","UNKNOWN") or c.map_version!=map_version:return False
        if c.state=="READY" and c.motion_requested is False:
            self._latched=False
            self._armed_session=c.session_id
            return False
        if c.state!="AUTONOMOUS" or c.motion_requested is not True:
            self._latched=True
            return False
        return not self._latched and c.session_id==self._armed_session

    def editing_allowed(self, now, *, mode, map_version, sole_publisher=True):
        c = self._consent
        return bool(sole_publisher and c and 0 <= now-self._received <= self.TIMEOUT_S and
                    c.mode == mode and c.map_version == map_version and map_version not in ("", "UNKNOWN") and
                    c.state in ("VIEW_ONLY", "READY", "PAUSED", "TAKEOVER_WAIT", "STOP_LATCHED") and not c.motion_requested and c.lease_active is True)


def consent_from_message(msg):
    return OperatorConsent(msg.session_id, msg.sequence, msg.execution_mode,
                           msg.map_version, msg.state, msg.motion_requested, msg.lease_active)
