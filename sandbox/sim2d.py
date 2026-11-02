"""2D sandbox simulator: lets the filters run with no ROS and no Gazebo.

CUSTOM code (ours). It mirrors the Gazebo robot (unicycle, 360-beam 10 Hz LiDAR, noisy wheel odometry
and gyro) and uses the same map/world files that sewer_gazebo generated. Disturbances come from the shared
sewer_localization.disturbances module (the same code the ROS scan_disturbance_node uses), seeded so every
filter sees identical data.

The TRUE world is the prior map plus the unmapped debris from config/blockages_truth.yaml. Filters only
ever get the prior map. Ground truth is returned for evaluation only.
"""
import math
import os
import sys
from dataclasses import dataclass, field
from functools import lru_cache

import numpy as np
import yaml

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
for _pkg in ('sewer_localization', 'sewer_perception', 'sewer_planning', 'sewer_eval'):
    sys.path.insert(0, os.path.join(_ROOT, 'src', _pkg))
from sewer_localization import disturbances  # noqa: E402
from sewer_localization.disturbances import (KIND, OdometryErrorModel, make_schedule,  # noqa: E402,F401
                                              make_slip_schedule)
from sewer_localization.map_utils import OccupancyMap  # noqa: E402

GAZEBO_PKG = os.path.join(_ROOT, 'src', 'sewer_gazebo')
DT = 0.1                       # step = one LiDAR scan (10 Hz)
LIDAR_X, RMIN, RMAX, LIDAR_SIGMA = 0.05, 0.12, 8.0, 0.01
ANGLES = np.linspace(-3.14159, 3.14159, 360)   # same as the Gazebo sensor

# Same route as the Day 4-5 Gazebo run: the long loop A -> J1 -> J2 -> M -> J3 -> J4 beside debris B2.
ROUTE = [(4.0, 0.0), (8.0, 0.0), (8.2, 2.0), (8.2, 4.0), (8.0, 6.0), (12.0, 6.0),
         (16.0, 6.0), (16.0, 3.0), (16.0, 0.0)]


def apply_disturbances(z, t, schedule, rng, state):
    return disturbances.apply_disturbances(z, t, schedule, rng, state, ANGLES, RMAX)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


@lru_cache(maxsize=None)
def load_world(world='blocked'):
    """(prior_map, true_map). The true map has the unmapped debris when world == 'blocked'."""
    prior = OccupancyMap.load(os.path.join(GAZEBO_PKG, 'maps', 'sewer_map'))
    true = prior.copy()
    if world == 'blocked':
        with open(os.path.join(GAZEBO_PKG, 'config', 'blockages_truth.yaml')) as f:
            truth = yaml.safe_load(f)
        for b in truth['blockages'].values():
            for p in b['pieces']:
                if p[0] == 'box':
                    true.add_box(p[1], p[2], p[3], p[4])
                else:
                    true.add_circle(p[1], p[2], p[3])
    return prior, true


@lru_cache(maxsize=None)
def trajectory(route=tuple(ROUTE), vmax=0.4, tol=0.15):
    """Scripted drive along the route (true kinematics, independent of any sensor or seed).

    Returns (pose[N+1,3], ds[N], dth[N]) with one row per DT.
    """
    x = y = th = v = w = 0.0
    poses, ds, dth = [(x, y, th)], [], []
    for wx, wy in route:
        for _ in range(4000):
            dx, dy = wx - x, wy - y
            dist = math.hypot(dx, dy)
            if dist < tol:
                break
            err = wrap(math.atan2(dy, dx) - th)
            if abs(err) > 0.5:
                v_cmd, w_cmd = 0.0, max(-1.0, min(1.0, 1.5 * err))
            else:
                v_cmd = min(vmax, 0.5 * dist + 0.1) * math.cos(err)
                w_cmd = max(-1.0, min(1.0, 2.0 * err))
            v += max(-1.0 * DT, min(1.0 * DT, v_cmd - v))   # accel limits 1 m/s^2, 2 rad/s^2
            w += max(-2.0 * DT, min(2.0 * DT, w_cmd - w))
            x += v * DT * math.cos(th + 0.5 * w * DT)
            y += v * DT * math.sin(th + 0.5 * w * DT)
            th = wrap(th + w * DT)
            ds.append(v * DT)
            dth.append(w * DT)
            poses.append((x, y, th))
        else:
            raise RuntimeError(f'did not reach waypoint {(wx, wy)}')
    return np.array(poses), np.array(ds), np.array(dth)


def waypoint_times(route=tuple(ROUTE), tol=0.15):
    """Sim time at which the scripted drive reaches each waypoint (for choosing the schedule)."""
    poses = trajectory()[0]
    out, k = [], 0
    for wx, wy in route:
        while math.hypot(wx - poses[k, 0], wy - poses[k, 1]) >= tol:
            k += 1
        out.append(round(k * DT, 1))
    return out


@lru_cache(maxsize=None)
def true_ranges(world='blocked', route=tuple(ROUTE)):
    """Noise-free LiDAR ranges along the trajectory, [N, 360], inf for no return."""
    _, true = load_world(world)
    poses = trajectory(route)[0][1:]
    out = np.empty((len(poses), len(ANGLES)))
    for k, (x, y, th) in enumerate(poses):
        out[k] = true.raycast(x + LIDAR_X * math.cos(th), y + LIDAR_X * math.sin(th),
                              th + ANGLES, RMAX, RMIN)
    return out


@dataclass
class Recording:
    name: str
    seed: int
    t: np.ndarray
    pose: np.ndarray            # [N,3] TRUE pose (evaluation only)
    ds: np.ndarray              # wheel-odometry distance increment (noisy)
    dth_wheel: np.ndarray       # wheel-odometry heading increment (noisy)
    dth_gyro: np.ndarray        # IMU gyro heading increment (noisy, biased)
    scans: np.ndarray           # [N,360]
    kind: np.ndarray            # [N,360] injected-disturbance label per beam (evaluation only)
    active: np.ndarray          # [N] any disturbance active
    windows: list = field(default_factory=list)


def simulate(seed, scenario='disturbed', world='blocked', severity=1.0, route=None, odom_slip=True):
    """Record one run: true pose, noisy odometry/gyro, noisy and (optionally) disturbed scans.

    The odometry always carries the per-run scale errors and gyro bias; in the 'disturbed' scenario the wheel-slip
    bursts (scaled by `severity`) are added unless odom_slip is False. Same model as the ROS odom_disturbance_node.
    """
    route = tuple(route) if route is not None else tuple(ROUTE)
    poses, ds, dth = trajectory(route)
    n = len(ds)
    rng = np.random.default_rng(seed)
    schedule = make_schedule(severity) if scenario == 'disturbed' else []
    # odometry errors: per-run scale errors and gyro bias, gyro random walk, wheel-slip bursts (shared model)
    odo = OdometryErrorModel(rng, make_slip_schedule(severity) if scenario == 'disturbed' and odom_slip else ())
    ds_m, dthw_m, dthg_m = np.empty(n), np.empty(n), np.empty(n)
    travelled = 0.0                                  # true metres driven so far: slip zones are places on the route
    for k in range(n):
        ds_m[k], dthw_m[k], dthg_m[k] = odo.step(ds[k], dth[k], DT, travelled)
        travelled += abs(ds[k])
    base = true_ranges(world, route)
    scans, kinds, active, state = np.empty_like(base), np.zeros(base.shape, np.int8), np.zeros(n, bool), {}
    for k in range(n):
        z = base[k].copy()
        fin = np.isfinite(z)
        z[fin] += rng.normal(0, LIDAR_SIGMA, fin.sum())
        z, kinds[k], active[k] = apply_disturbances(z, (k + 1) * DT, schedule, rng, state)
        z[z >= RMAX] = np.inf
        scans[k] = z
    return Recording(scenario, seed, (np.arange(n) + 1) * DT, poses[1:], ds_m, dthw_m, dthg_m,
                     scans, kinds, active, [(e['t0'], e['t1']) for e in schedule])
