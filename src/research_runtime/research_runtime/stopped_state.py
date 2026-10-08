"""The original measured-rate stopping definition, with acquisition continuity.

This labels a measured state inside the original tolerance; it never overwrites
raw odometry or claims an independently measured physical stop.
"""
import math
from .physical_parameter_lock import STOP_CONFIRMATION


class StopConfirmation:
    def __init__(self, maximum_gap_s):
        if not math.isfinite(maximum_gap_s) or maximum_gap_s <= 0:
            raise ValueError("invalid acquisition gap")
        self.maximum_gap_s = maximum_gap_s
        self.invalidate()

    def invalidate(self):
        self.since = None
        self.previous = None
        self.continuity_valid = False

    def update(self, stamp_s, translation_speed, yaw_rate):
        if (not all(math.isfinite(float(v)) for v in (stamp_s, translation_speed, yaw_rate)) or
                stamp_s <= 0 or translation_speed < 0):
            self.invalidate()
            return False
        if self.previous is not None and not 0 < stamp_s-self.previous <= self.maximum_gap_s:
            self.invalidate()
            return False
        self.continuity_valid = True
        quiet = translation_speed <= STOP_CONFIRMATION[0] and abs(yaw_rate) <= STOP_CONFIRMATION[1]
        if not quiet:
            self.since = None
        elif self.since is None:
            self.since = stamp_s
        self.previous = stamp_s
        return self.since is not None and stamp_s-self.since >= STOP_CONFIRMATION[2]
