"""Component slew limits copied from the effective corridor smoother."""
import math


def slew_command(previous, requested, dt, limits, fraction=1.0, *, in_place_rotation=False):
    if not math.isfinite(fraction) or not 0 < fraction <= 1:
        raise ValueError("normal slew fraction must lie within (0,1]")
    if not all(math.isfinite(v) for v in (*previous, *requested, dt)) or dt <= 0:
        return (0.0, 0.0)
    if in_place_rotation and (abs(previous[0])>1e-12 or abs(requested[0])>1e-12):
        return (0.,0.)
    # A stalled timer must not accumulate a large permitted step at recovery.
    dt = min(dt, 0.10)
    dv = max(-limits.max_decel_mps2 * dt * fraction, min(limits.max_accel_mps2 * dt * fraction, requested[0] - previous[0]))
    dw = max(-limits.max_yaw_decel_rps2 * dt * fraction, min(limits.max_yaw_accel_rps2 * dt * fraction, requested[1] - previous[1]))
    if not in_place_rotation and limits.max_curvature_1pm is not None:
        # The forward curvature cone |w|<=k*v is convex. Use one fraction of
        # the command segment so independent axis slew cannot leave this cone.
        delta_v,delta_w=requested[0]-previous[0],requested[1]-previous[1]
        scale=min(1.,abs(dv/delta_v) if abs(delta_v)>1e-12 else 1.,
                    abs(dw/delta_w) if abs(delta_w)>1e-12 else 1.)
        dv,dw=scale*delta_v,scale*delta_w
    return previous[0] + dv, previous[1] + dw
