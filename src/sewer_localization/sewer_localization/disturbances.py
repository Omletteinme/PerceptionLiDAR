"""LiDAR disturbance models: spray, water reflections, dropouts, noise bursts.

CUSTOM code (ours). Pure NumPy. Shared by the 2D sandbox and the ROS scan_disturbance_node, so both
inject exactly the same, seeded, repeatable corruption. Gazebo only supplies baseline sensor noise.

  spurious : each beam replaced w.p. p by a short random return (spray, floating debris)
  water    : a sector of beams sees a reflection off a moving water surface: ranges scaled by a factor
             that random-walks from scan to scan (smooth within a scan, inconsistent across scans)
  dropout  : each beam loses its return w.p. p (wet, absorbing surface)
  burst    : extra Gaussian range noise on all beams
"""
import math

import numpy as np

KIND = {'none': 0, 'spurious': 1, 'water': 2, 'dropout': 3, 'burst': 4}

# Base schedule (seconds since the robot starts moving), a design choice recorded in PLAN.md.
BASE_SCHEDULE = [
    dict(kind='spurious', t0=8.0, t1=16.0, p=0.35),
    dict(kind='water', t0=28.0, t1=38.0, center=math.pi / 2, width=math.radians(70)),
    dict(kind='dropout', t0=46.0, t1=56.0, p=0.5),
    dict(kind='spurious', t0=46.0, t1=56.0, p=0.2),
    dict(kind='water', t0=62.0, t1=72.0, center=-math.pi / 2, width=math.radians(70)),
    dict(kind='burst', t0=62.0, t1=72.0, sigma=0.1),
]


def make_schedule(severity=1.0):
    """BASE_SCHEDULE scaled by `severity`: more spurious/dropout beams, wider water sectors, more noise."""
    out = []
    for ev in BASE_SCHEDULE:
        e = dict(ev)
        if 'p' in e:
            e['p'] = min(0.9, e['p'] * severity)
        if 'width' in e:
            e['width'] = min(math.radians(330), e['width'] * severity)
        if 'sigma' in e:
            e['sigma'] *= severity
        out.append(e)
    return out


def apply_disturbances(z, t, schedule, rng, state, angles, rmax):
    """Corrupt one scan. Returns (z, kind[n] int8 label per beam, active). `state` keeps the water factors."""
    z = np.array(z, float)
    n = len(z)
    kind = np.zeros(n, np.int8)
    active = False
    for k, ev in enumerate(schedule):
        if not ev['t0'] <= t < ev['t1']:
            continue
        active = True
        if ev['kind'] == 'spurious':
            hi = np.where(np.isfinite(z), np.minimum(z, 3.0), 3.0)
            m = (rng.random(n) < ev['p']) & (hi > 0.25)
            z[m] = rng.uniform(0.2, hi[m])
            kind[m] = KIND['spurious']
        elif ev['kind'] == 'water':
            f = state.get(k, 1.5)
            f = float(np.clip(f + rng.normal(0, 0.12), 1.15, 2.2))
            state[k] = f
            diff = np.abs(np.arctan2(np.sin(angles - ev['center']), np.cos(angles - ev['center'])))
            m = (diff <= ev['width'] / 2) & np.isfinite(z)
            z[m] = z[m] * f + rng.normal(0, 0.04, m.sum())
            z[m & (z >= rmax)] = np.inf
            kind[m & np.isfinite(z)] = KIND['water']
        elif ev['kind'] == 'dropout':
            m = rng.random(n) < ev['p']
            z[m] = np.inf
            kind[m] = KIND['dropout']
        elif ev['kind'] == 'burst':
            m = np.isfinite(z)
            z[m] += rng.normal(0, ev['sigma'], m.sum())
            kind[m & (kind == 0)] = KIND['burst']
    return z, kind, active


# --------------------------------------------------------------------------------------------------------------
# Odometry / IMU error model. Gazebo's wheel odometry has no slip and drifts only ~1.5%, so with it the two filters
# look alike. This model makes the dead reckoning realistically bad. Shared by the sandbox and odom_disturbance_node.

# Wheel-slip ZONES (metres of route travelled since the robot started moving), on the same legs as the water windows:
# the wheels turn more than the robot moves, so wheel odometry over-reports distance and under-reports rotation. The
# gyro is unaffected. Slip belongs to the floor, not to the clock, so the zones are distances: slowing down or waiting
# does not shorten the exposure (the scan disturbances above are transient, so waiting those out does help).
BASE_SLIP = [dict(s0=10.0, s1=13.5), dict(s0=21.5, s1=25.5)]
SLIP_DS_PER_SEVERITY = 0.05     # extra distance reported per unit severity (x3 -> +15 %)
SLIP_DTH_PER_SEVERITY = -0.05   # change in rotation reported per unit severity (x3 -> -15 %)
SLIP_CAP = 0.3


def make_slip_schedule(severity=1.0):
    """Wheel-slip bursts scaled by `severity` (0 -> none)."""
    ds = min(SLIP_CAP, SLIP_DS_PER_SEVERITY * severity)
    dth = max(-SLIP_CAP, SLIP_DTH_PER_SEVERITY * severity)
    return [dict(e, ds_scale=ds, dth_scale=dth) for e in BASE_SLIP] if severity > 0 else []


class OdometryErrorModel:
    """Per-run odometry and gyro errors.

    Drawn once per run: distance scale error eps_s ~ N(0, 1 %), wheel-yaw scale error eps_r ~ N(0, 3 %), gyro bias
    ~ N(0, 1 mrad/s). Per step: gyro-bias random walk, small wheel noise (only while the robot moves: a parked robot's
    wheels report nothing), gyro noise, and the slip bursts. `scale` multiplies the three per-run errors.
    The random draws happen in a fixed order so a given seed always gives the same errors.
    """

    def __init__(self, rng, slips=(), scale=1.0):
        self.rng = rng
        self.eps_s = rng.normal(0, 0.01) * scale
        self.eps_r = rng.normal(0, 0.03) * scale
        self.bias = rng.normal(0, 0.001) * scale
        self.slips = list(slips)

    def step(self, ds, dth, dt, s=None):
        """One odometry step. ds [m] and dth [rad] are the TRUE increments over dt [s]; s = metres of route the robot
        has travelled so far (None: no slip). Returns (ds_wheel, dth_wheel, dth_gyro)."""
        rng = self.rng
        self.bias += rng.normal(0, 0.0002 * math.sqrt(dt))
        k = math.sqrt(dt / 0.1)                       # noise levels are defined per 0.1 s step
        n_s, n_w, n_g = rng.normal(0, 0.001 * k), rng.normal(0, 0.002 * k), rng.normal(0, 0.0003 * k)
        moving = abs(ds) > 1e-4 or abs(dth) > 1e-4
        ds_scale, dth_scale = self.eps_s, self.eps_r
        if s is not None:
            for e in self.slips:
                if e['s0'] <= s < e['s1']:
                    ds_scale += e['ds_scale']
                    dth_scale += e['dth_scale']
        return (ds * (1 + ds_scale) + (n_s if moving else 0.0),
                dth * (1 + dth_scale) + (n_w if moving else 0.0),
                dth + self.bias * dt + n_g)
