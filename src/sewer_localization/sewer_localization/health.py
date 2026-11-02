"""Localization-health monitor: RELIABLE / DEGRADED / CRITICAL.

CUSTOM code (ours), pure Python, no ROS imports. The localizers say where the robot is; this module says how hostile the
conditions are right now, so the robot can slow down or stop instead of driving at full speed on a doubtful estimate.

It is a conditions detector, not a divergence predictor: sandbox calibration (PLAN.md, M3 findings) showed that a lost
particle filter is confidently wrong, so no signal the filter produces predicts that it has lost track. What these
signals do show is that the scans or the odometry are being corrupted:
  R     smoothed scan reliability (reliability-aware filter only; low = many beams inconsistent)
  fit   mean distance of the scan's beam endpoints to a wall at the particles [m] (high = the map does not explain it)
  gyro  |wheel yaw rate - gyro yaw rate| [rad/s] (high = wheel slip or a biased gyro)
Each is smoothed, then turned into a stress: 0 = healthy ... 1 = as bad as the CRITICAL boundary. The state follows the
worst stress, with hysteresis (moving up needs the stress to persist for `up_time`, moving down needs `down_time` below
a lower threshold) so it does not chatter. Thresholds were set on sandbox tuning seeds (see PLAN.md).
"""
import math
from dataclasses import dataclass

RELIABLE, DEGRADED, CRITICAL = 'RELIABLE', 'DEGRADED', 'CRITICAL'
LEVEL = {RELIABLE: 0, DEGRADED: 1, CRITICAL: 2}


@dataclass
class HealthParams:
    tau: float = 1.0               # smoothing time constant of every signal [s]
    warmup: float = 3.0            # seconds after start during which the state stays RELIABLE (the cloud is still settling)
    # (ok, bad): a signal at `ok` has stress 0, at `bad` stress 1 (works in both directions)
    fit: tuple = (0.12, 0.18)      # clean tuning runs peak at 0.115
    R: tuple = (0.90, 0.45)        # clean tuning runs never go below 0.93
    gyro: tuple = (0.06, 0.15)     # clean tuning runs peak at 0.054
    degrade_at: float = 0.5        # stress needed to leave RELIABLE
    critical_at: float = 1.0       # stress needed to reach CRITICAL
    down_margin: float = 0.8       # to step down, the stress must fall below margin * threshold
    up_time: float = 0.5           # seconds the stress must persist to step up
    down_time: float = 2.0         # seconds below the lower threshold to step down


def stress(x, ok, bad):
    if x is None or math.isnan(x):
        return 0.0
    return max(0.0, (x - ok) / (bad - ok))


class HealthMonitor:
    def __init__(self, params=None):
        self.p = params or HealthParams()
        self.state = RELIABLE
        self.score = 0.0
        self.stresses = {}
        self._ema = {}
        self._t = 0.0
        self._up = 0.0
        self._down = 0.0

    def _smooth(self, name, value, dt):
        if value is None or math.isnan(value):
            return self._ema.get(name)
        a = 1.0 - math.exp(-dt / self.p.tau)
        prev = self._ema.get(name, value)
        self._ema[name] = prev + a * (value - prev)
        return self._ema[name]

    def update(self, dt, fit=None, R=None, gyro=None):
        """One health update every `dt` seconds (None = signal not available). Returns the state."""
        p = self.p
        self._t += dt
        self.stresses = {
            'fit': stress(self._smooth('fit', fit, dt), *p.fit),
            'R': stress(self._smooth('R', R, dt), *p.R),
            'gyro': stress(self._smooth('gyro', gyro, dt), *p.gyro),
        }
        self.score = max(self.stresses.values())
        if self._t < p.warmup:
            return self.state
        target = CRITICAL if self.score >= p.critical_at else DEGRADED if self.score >= p.degrade_at else RELIABLE
        lower = (CRITICAL if self.score >= p.critical_at * p.down_margin
                 else DEGRADED if self.score >= p.degrade_at * p.down_margin else RELIABLE)
        if LEVEL[target] > LEVEL[self.state]:
            self._up += dt
            self._down = 0.0
            if self._up >= p.up_time:
                self.state, self._up = target, 0.0
        elif LEVEL[lower] < LEVEL[self.state]:
            self._down += dt
            self._up = 0.0
            if self._down >= p.down_time:
                self.state, self._down = lower, 0.0
        else:
            self._up = self._down = 0.0
        return self.state

    def details(self):
        return dict(state=self.state, score=round(self.score, 3), **{k: round(v, 3) for k, v in self.stresses.items()})
