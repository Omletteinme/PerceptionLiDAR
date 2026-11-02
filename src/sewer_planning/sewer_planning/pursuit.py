"""Pure-pursuit target selection along a route polyline.

CUSTOM code (ours), pure NumPy. Plain pure pursuit aims at a point `lookahead` metres ahead, which lets the
robot cut a corner: in a 1 m corridor that can bring it within a few centimetres of the inner wall corner
(seen in the Day 9 live runs). Here, when a sharp corner of the route lies within the lookahead distance, the
robot aims at the corner point itself until it is within `corner_tol` of it, then carries on along the next leg.
"""
import numpy as np


def pursuit_target(pts, x, y, lookahead=0.5, corner_angle=0.5, corner_tol=0.2):
    """Point to steer toward. pts: (N, 2) dense route polyline; (x, y): current pose estimate."""
    pts = np.asarray(pts, float)
    if len(pts) < 2:
        return pts[-1]
    seg = np.diff(pts, axis=0)
    seg_len = np.hypot(seg[:, 0], seg[:, 1])
    k = int(np.argmin(np.hypot(pts[:, 0] - x, pts[:, 1] - y)))
    heading = np.arctan2(seg[:, 1], seg[:, 0])
    turn = np.abs(np.arctan2(np.sin(np.diff(heading)), np.cos(np.diff(heading))))
    for c in np.flatnonzero(turn > corner_angle) + 1:     # vertex c joins segments c-1 and c
        if c < k:
            continue
        if np.sum(seg_len[k:c]) > lookahead:
            break                                          # corners are ordered along the route
        if np.hypot(pts[c, 0] - x, pts[c, 1] - y) > corner_tol:
            return pts[c]
    remaining = lookahead
    for i in range(k, len(seg_len)):
        if remaining <= seg_len[i]:
            return pts[i] + seg[i] * (remaining / max(seg_len[i], 1e-9))
        remaining -= seg_len[i]
    return pts[-1]


def health_speed_factor(state, critical_for, wait=8.0, creep=0.3):
    """Fraction of the normal speed for a localization-health state (RELIABLE / DEGRADED / CRITICAL).

    RELIABLE drives normally, DEGRADED at half speed, CRITICAL stops and waits for the conditions to pass. A robot that
    waits forever is no use, so after `wait` seconds of continuous CRITICAL it creeps at `creep` of the normal speed.
    """
    if state == 'CRITICAL':
        return creep if critical_for >= wait else 0.0
    return 0.5 if state == 'DEGRADED' else 1.0
