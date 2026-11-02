"""Per-beam measurement reliability from temporal and geometric consistency.

CUSTOM code (ours): this is the research contribution. Neither AMCL nor Nav2 does this.
Uses ONLY the scan and the odometry/IMU motion estimate. It never looks at the map, the particles
or ground truth, so it is an independent judge of how far each range reading can be trusted.

  geometric consistency g_i : is beam i's endpoint on a locally straight surface formed by its
      neighbours? Isolated spikes (spray, floating debris) are not. We test the endpoint against
      three lines (two one-sided, one centred) and keep the best fit, so legitimate corners and
      range discontinuities are not punished.
  temporal consistency t_i  : does beam i agree with the previous scan after compensating the
      robot's motion? Reflections off moving water change from scan to scan, even when they look
      locally smooth, so they fail here while passing the geometric test.

  r_i = clip(g_i * t_i, floor, 1)   for valid returns, 0 otherwise.
  scan reliability R = mean of r_i over valid returns (used for monitoring and uncertainty).
"""
import math
from dataclasses import dataclass

import numpy as np

from .map_utils import valid_returns


@dataclass
class ReliabilityParams:
    lidar_x: float = 0.05
    rmin: float = 0.12
    rmax: float = 8.0
    sigma_g0: float = 0.03       # geometric tolerance: sigma_g = g0 + g1 * range
    sigma_g1: float = 0.01
    sigma_t0: float = 0.05       # temporal tolerance: sigma_t = t0 + t1*range + t_rot*|dtheta|*range
    sigma_t1: float = 0.01
    sigma_t_rot: float = 0.1
    neutral_g: float = 0.5       # no usable neighbours: weak evidence
    neutral_t: float = 0.8       # nothing to compare against (new area, first scan): do not punish
    floor: float = 0.02
    max_gap: float = math.radians(2.5)   # max bearing gap to interpolate the predicted scan over
    max_dr: float = 0.4                  # max range jump to interpolate over


class ReliabilityScorer:
    def __init__(self, angles, params=None):
        self.angles = np.asarray(angles, float)
        self.p = params or ReliabilityParams()
        self.prev = None

    def reset(self):
        self.prev = None

    def _valid(self, z):
        return valid_returns(z, self.p.rmin, self.p.rmax)

    def geometric(self, z):
        p, a = self.p, self.angles
        n = len(z)
        v = self._valid(z)
        zz = np.where(v, z, 0.0)
        px, py = zz * np.cos(a), zz * np.sin(a)
        idx = np.arange(n)
        res = np.full(n, np.inf)
        for oa, ob in ((-2, -1), (1, 2), (-1, 1)):
            ia, ib = idx + oa, idx + ob
            in_b = (ia >= 0) & (ia < n) & (ib >= 0) & (ib < n)
            ia, ib = np.clip(ia, 0, n - 1), np.clip(ib, 0, n - 1)
            ok = in_b & v[ia] & v[ib] & v
            abx, aby = px[ib] - px[ia], py[ib] - py[ia]
            length = np.hypot(abx, aby)
            cross = abx * (py - py[ia]) - aby * (px - px[ia])
            d = np.where(length > 1e-3, np.abs(cross) / np.maximum(length, 1e-3),
                         np.hypot(px - px[ia], py - py[ia]))
            res = np.where(ok, np.minimum(res, d), res)
        sigma = p.sigma_g0 + p.sigma_g1 * zz
        g = np.where(np.isfinite(res), np.exp(-0.5 * (res / sigma) ** 2), p.neutral_g)
        return np.where(v, g, 0.0)

    def temporal(self, z, motion):
        """motion = (dx, dy, dtheta) of base_link since the previous scan, in the previous base frame."""
        p, a = self.p, self.angles
        v = self._valid(z)
        neutral = np.where(v, p.neutral_t, 0.0)
        if self.prev is None or motion is None:
            return neutral
        dx, dy, dth = motion
        zp = self.prev
        vp = self._valid(zp)
        if vp.sum() < 2:
            return neutral
        # previous endpoints: previous sensor frame -> previous base -> current base -> current sensor
        ex = p.lidar_x + zp[vp] * np.cos(a[vp])
        ey = zp[vp] * np.sin(a[vp])
        ux, uy = ex - dx, ey - dy
        c, s = math.cos(dth), math.sin(dth)
        cx = c * ux + s * uy - p.lidar_x
        cy = -s * ux + c * uy
        bear, rng = np.arctan2(cy, cx), np.hypot(cx, cy)
        order = np.argsort(bear)
        bear, rng = bear[order], rng[order]
        m = len(bear)
        k = np.searchsorted(bear, a)
        lo, hi = k - 1, k
        ok = (lo >= 0) & (hi < m)
        lo, hi = np.clip(lo, 0, m - 1), np.clip(hi, 0, m - 1)
        gap = bear[hi] - bear[lo]
        ok &= (gap > 1e-9) & (gap <= p.max_gap) & (np.abs(rng[hi] - rng[lo]) <= p.max_dr)
        frac = (a - bear[lo]) / np.maximum(gap, 1e-9)
        zhat = rng[lo] + frac * (rng[hi] - rng[lo])
        zz = np.where(v, z, 0.0)
        sigma = p.sigma_t0 + p.sigma_t1 * zz + p.sigma_t_rot * abs(dth) * zz
        t = np.where(ok, np.exp(-0.5 * ((zz - zhat) / sigma) ** 2), p.neutral_t)
        return np.where(v, t, 0.0)

    def score(self, z, motion):
        """Return (r, g, t, R) for this scan and remember it as the reference for the next one."""
        z = np.asarray(z, float)
        v = self._valid(z)
        g = self.geometric(z)
        t = self.temporal(z, motion)
        r = np.where(v, np.clip(g * t, self.p.floor, 1.0), 0.0)
        self.prev = z.copy()
        R = float(r[v].mean()) if v.any() else 0.0
        return r, g, t, R
