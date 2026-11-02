"""Particle filter for 2D pose tracking in the sewer map.

CUSTOM code (ours), pure NumPy.

  ParticleFilter            BASELINE. Standard PF: odometry/IMU motion model + AMCL-style likelihood
                            field range model (same functional form as Nav2 AMCL's
                            likelihood_field), every beam trusted equally.
  ReliabilityAwareFilter    PROPOSED. Identical in every respect except that each beam's log-likelihood
                            is multiplied by its reliability r_i (see reliability.py).

Both use the same motion model, resampler and RNG stream, so under the same seed and the same data any
difference in the estimate comes from the reliability weighting alone.
"""
import math
from dataclasses import dataclass

import numpy as np

from .map_utils import valid_returns
from .reliability import ReliabilityParams, ReliabilityScorer


def wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))


def fuse_heading(dth_wheel, dth_gyro, gyro_weight=0.8):
    """Complementary blend of wheel-odometry and IMU-gyro heading increments."""
    return gyro_weight * dth_gyro + (1.0 - gyro_weight) * dth_wheel


@dataclass
class PFParams:
    n: int = 500
    lidar_x: float = 0.05
    rmin: float = 0.12
    rmax: float = 8.0
    beam_stride: int = 6          # use every 6th beam (6 deg apart: neighbouring beams are correlated)
    z_hit: float = 0.8            # likelihood field: z_hit*exp(-d^2/2s^2) + z_rand/rmax
    z_rand: float = 0.2
    sigma_hit: float = 0.15
    kappa: float = 0.25           # tempering: scales the summed log-likelihood
    a_ds: float = 0.03            # motion noise: translation, relative
    s_ds: float = 0.003           # translation floor [m / step]
    a_dth: float = 0.05           # rotation, relative
    a_dth_ds: float = 0.03        # rotation noise per metre travelled [rad / m]
    s_dth: float = 0.002          # rotation floor [rad / step]
    resample_frac: float = 0.5    # resample when N_eff < frac * N
    init_std_xy: float = 0.1
    init_std_th: float = 0.05
    gyro_weight: float = 0.8
    wall_block: float = 0.10      # a move that would end within this distance of an obstacle is rejected (m)
    rough_k: float = 0.2          # roughening after resampling: jitter = k * spread * N^(-1/3) (Gordon et al.)
    rough_floor_xy: float = 0.005   # ... but at least 5 mm
    rough_floor_th: float = 0.0017  # ... and 0.1 deg. A 2 cm / 0.5 deg floor lost track under disturbance (Day 11
                                    # tuning seeds: baseline RMSE 0.057 -> 0.291 m): the jitter is applied at nearly
                                    # every step there and random-walks the estimate along the weak corridor axis.


class ParticleFilter:
    def __init__(self, omap, params, rng, init_pose):
        self.map, self.p, self.rng = omap, params, rng
        n = params.n
        self.x = init_pose[0] + rng.normal(0, params.init_std_xy, n)
        self.y = init_pose[1] + rng.normal(0, params.init_std_xy, n)
        self.th = wrap(init_pose[2] + rng.normal(0, params.init_std_th, n))
        self.w = np.full(n, 1.0 / n)
        self.neff = float(n)
        self._moving = True
        # Health signals, refreshed by every update() (read by sewer_localization.health.HealthMonitor):
        self.correction = 0.0          # how far the last scan pulled the estimate [m]
        self.last_travel = 0.0         # distance the robot moved since the previous scan [m]
        self.fit_residual = float('nan')   # how far the scan's beam endpoints are from a wall at the particles [m]
        self.n_used = 0                # beams that took part in the last update
        self._travel = 0.0

    def predict(self, ds, dth):
        """Propagate with a fused odometry increment: ds [m] along the heading, dth [rad]."""
        p, n = self.p, self.p.n
        # The noise floors apply only while the robot reports motion: with no motion there is nothing to be
        # uncertain about, and a constant floor would random-walk a parked robot along weakly observable
        # axes (measured Day 13: 0.2-0.3 m in 90 s in a straight corridor).
        moving = abs(ds) > 1e-3 or abs(dth) > 1e-3
        self._moving = moving
        self._travel += abs(ds)
        ds_n = ds + self.rng.normal(0, p.a_ds * abs(ds) + (p.s_ds if moving else 0.0), n)
        dth_n = dth + self.rng.normal(
            0, p.a_dth * abs(dth) + p.a_dth_ds * abs(ds) + (p.s_dth if moving else 0.0), n)
        mid = self.th + 0.5 * dth_n
        nx, ny = self.x + ds_n * np.cos(mid), self.y + ds_n * np.sin(mid)
        # Walls block the particles: the robot cannot be inside one. Without this, odometry that overshoots (wheel
        # slip) pushes particles through a wall, and a particle inside a wall has every beam endpoint at distance 0
        # from an obstacle, i.e. the best possible likelihood, so it hijacks the estimate (found in the Day 11 live
        # runs with disturbed odometry: the estimate ended 0.5 m inside the wall behind J4).
        free = self.map.dist(nx, ny) >= p.wall_block
        self.x, self.y = np.where(free, nx, self.x), np.where(free, ny, self.y)
        self.th = wrap(self.th + dth_n)

    def update(self, ranges, angles, reliability=None):
        """Measurement update. `reliability` (per beam, 0..1) scales each beam's log-likelihood."""
        p = self.p
        ranges = np.asarray(ranges, float)
        sel = np.zeros(len(ranges), bool)
        sel[::p.beam_stride] = True
        sel &= valid_returns(ranges, p.rmin, p.rmax)
        self.last_travel, self._travel = self._travel, 0.0
        if not sel.any():
            self.correction, self.fit_residual, self.n_used = 0.0, float('nan'), 0
            return
        z, b = ranges[sel], np.asarray(angles)[sel]
        r = 1.0 if reliability is None else np.asarray(reliability)[sel]
        lx = self.x + p.lidar_x * np.cos(self.th)
        ly = self.y + p.lidar_x * np.sin(self.th)
        ang = self.th[:, None] + b[None, :]
        d = self.map.dist(lx[:, None] + z[None, :] * np.cos(ang), ly[:, None] + z[None, :] * np.sin(ang))
        lik = p.z_hit * np.exp(-0.5 * (d / p.sigma_hit) ** 2) + p.z_rand / p.rmax
        logw = p.kappa * (np.log(lik) * r).sum(axis=1)
        x0, y0 = float(self.w @ self.x), float(self.w @ self.y)      # the estimate before this measurement
        self.w *= np.exp(logw - logw.max())
        self.w /= self.w.sum()
        self.neff = float(1.0 / np.sum(self.w ** 2))
        self.correction = math.hypot(float(self.w @ self.x) - x0, float(self.w @ self.y) - y0)
        self.fit_residual = float(self.w @ np.minimum(d, 0.5).mean(axis=1))
        self.n_used = int(sel.sum())
        if self.neff < p.resample_frac * p.n:
            self._resample()

    def _resample(self):
        n = self.p.n
        u = (self.rng.random() + np.arange(n)) / n  # systematic resampling
        cs = np.cumsum(self.w)
        cs[-1] = 1.0
        idx = np.searchsorted(cs, u)
        self.x, self.y, self.th = self.x[idx], self.y[idx], self.th[idx]
        self.w = np.full(n, 1.0 / n)
        if self._moving:
            self._roughen()

    def _roughen(self):
        """Jitter the resampled particles by an amount tied to the cloud spread (Gordon et al. roughening).

        Resampling copies particles, so without this the cloud collapses onto a few of them and the reported
        sigma understates the true error. Skipped while the robot is parked: nothing moved, so there is nothing
        to be uncertain about, and the jitter would random-walk the estimate along weakly observable axes.
        """
        p = self.p
        k = p.rough_k * p.n ** (-1.0 / 3.0)
        mean_th = math.atan2(float(np.mean(np.sin(self.th))), float(np.mean(np.cos(self.th))))
        jx = max(k * float(np.ptp(self.x)), p.rough_floor_xy)
        jy = max(k * float(np.ptp(self.y)), p.rough_floor_xy)
        jt = max(k * float(np.ptp(wrap(self.th - mean_th))), p.rough_floor_th)
        self.x = self.x + self.rng.normal(0, jx, p.n)
        self.y = self.y + self.rng.normal(0, jy, p.n)
        self.th = wrap(self.th + self.rng.normal(0, jt, p.n))

    def estimate(self):
        """Weighted mean pose and 2x2 position covariance."""
        w = self.w
        x, y = float(w @ self.x), float(w @ self.y)
        th = float(math.atan2(w @ np.sin(self.th), w @ np.cos(self.th)))
        dx, dy = self.x - x, self.y - y
        cov = np.array([[w @ (dx * dx), w @ (dx * dy)], [w @ (dx * dy), w @ (dy * dy)]])
        return x, y, th, cov

    @property
    def neff_ratio(self):
        return self.neff / self.p.n

    @property
    def std_xy(self):
        """Largest standard deviation of the particle cloud [m]: a simple uncertainty estimate."""
        return float(math.sqrt(max(np.linalg.eigvalsh(self.estimate()[3]))))


class ReliabilityAwareFilter(ParticleFilter):
    """PROPOSED: same filter, but each beam is weighted by its temporal+geometric reliability."""

    def __init__(self, omap, params, rng, init_pose, angles, rel_params=None):
        super().__init__(omap, params, rng, init_pose)
        self.scorer = ReliabilityScorer(angles, rel_params or ReliabilityParams(
            lidar_x=params.lidar_x, rmin=params.rmin, rmax=params.rmax))
        self._acc = [0.0, 0.0, 0.0]   # motion since the previous scan, in the previous base frame
        self.r = self.g = self.t = None
        self.R = 1.0

    def predict(self, ds, dth):
        super().predict(ds, dth)
        ax, ay, ath = self._acc
        self._acc = [ax + ds * math.cos(ath + 0.5 * dth), ay + ds * math.sin(ath + 0.5 * dth), ath + dth]

    def update(self, ranges, angles, reliability=None):
        self.r, self.g, self.t, self.R = self.scorer.score(ranges, tuple(self._acc))
        self._acc = [0.0, 0.0, 0.0]
        super().update(ranges, angles, self.r if reliability is None else reliability)
