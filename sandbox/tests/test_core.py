"""Unit tests for the map, reliability scorer and particle filters (no ROS, no Gazebo)."""
import math

import numpy as np
import pytest

import sim2d
from sewer_localization.map_utils import valid_returns
from sewer_localization.particle_filter import (PFParams, ParticleFilter, ReliabilityAwareFilter,
                                                fuse_heading)
from sewer_localization.reliability import ReliabilityScorer

A = sim2d.ANGLES


def ScorerFor():
    return ReliabilityScorer(A)


# ---------------------------------------------------------------- map
def test_raycast_matches_known_geometry():
    prior, _ = sim2d.load_world('clear')
    r = prior.raycast(0.0, 0.0, np.array([0.0, math.pi / 2, math.pi]), rmax=20.0)
    assert 16.4 < r[0] < 16.65     # east end cap of the J1-J4 corridor, x = 16.5
    assert 0.45 < r[1] < 0.6       # north wall at y = 0.5
    assert 0.45 < r[2] < 0.6       # west end cap at x = -0.5


def test_distance_field_and_outside_is_solid():
    prior, _ = sim2d.load_world('clear')
    assert 0.45 < float(prior.dist(0.0, 0.0)) < 0.55
    assert float(prior.dist(-50.0, 0.0)) == 0.0
    assert bool(prior.is_solid(-50.0, 0.0))


def test_debris_only_in_true_map():
    prior, true = sim2d.load_world('blocked')
    assert not bool(prior.is_solid(12.0, 0.0))
    assert bool(true.is_solid(12.0, 0.0))


# ---------------------------------------------------------------- reliability
def clean_scan(pose, world='clear', seed=0):
    _, true = sim2d.load_world(world)
    x, y, th = pose
    z = true.raycast(x + sim2d.LIDAR_X * math.cos(th), y + sim2d.LIDAR_X * math.sin(th),
                     th + A, sim2d.RMAX, sim2d.RMIN)
    rng = np.random.default_rng(seed)
    fin = np.isfinite(z)
    z[fin] += rng.normal(0, sim2d.LIDAR_SIGMA, fin.sum())
    return z


def test_clean_scan_is_reliable():
    z = clean_scan((8.0, 0.0, 0.0))     # junction: walls, corners, openings
    sc = ScorerFor()
    _, g, t, R1 = sc.score(z, None)
    v = valid_returns(z, 0.12, 8.0)
    assert g[v].mean() > 0.9            # geometry alone: clean walls, corners and openings pass
    assert R1 == pytest.approx(0.8, abs=0.05)   # first scan: temporal term is neutral (0.8), by design
    r, g, t, R2 = sc.score(clean_scan((8.0, 0.0, 0.0), seed=1), (0.0, 0.0, 0.0))
    assert R2 > 0.85                    # second scan, independent noise, no motion: fully consistent
    assert np.median(r[v]) > 0.95


def test_isolated_spike_is_flagged_geometrically():
    z = clean_scan((8.0, 0.0, 0.0))
    v = valid_returns(z, 0.12, 8.0)
    i = int(np.flatnonzero(v)[len(np.flatnonzero(v)) // 2])
    z[i] = max(0.3, z[i] * 0.5)
    _, g, _, _ = ReliabilityScorer(A).score(z, None)
    assert g[i] < 0.05


def test_scan_to_scan_shift_is_flagged_temporally():
    pose = (8.0, 2.0, math.pi / 2)      # heading north inside the J1-J2 corridor
    z1 = clean_scan(pose)
    z2 = z1.copy()
    sector = (np.abs(np.arctan2(np.sin(A - math.pi / 2), np.cos(A - math.pi / 2))) < math.radians(35)) & np.isfinite(z2)
    z2[sector] *= 1.5                   # smooth, locally straight, but shifted vs. the previous scan
    sc = ReliabilityScorer(A)
    sc.score(z1, None)
    r, g, t, _ = sc.score(z2, (0.0, 0.0, 0.0))
    v = valid_returns(z2, 0.12, 8.0)
    inner = sector & v
    outer = ~sector & v
    assert t[inner].mean() < 0.3 and t[outer].mean() > 0.7
    assert g[inner].mean() > 0.6        # geometry alone cannot see it: that is why both terms exist


def test_temporal_term_compensates_motion():
    """Same clean world seen from two poses 4 cm apart: with the true motion given, scans agree."""
    z1 = clean_scan((8.0, 2.00, math.pi / 2))
    z2 = clean_scan((8.0, 2.04, math.pi / 2))
    sc = ReliabilityScorer(A)
    sc.score(z1, None)
    _, _, t, _ = sc.score(z2, (0.04, 0.0, 0.0))
    v = valid_returns(z2, 0.12, 8.0)
    assert t[v].mean() > 0.8


# ---------------------------------------------------------------- filters
@pytest.fixture(scope='module')
def rec_clean():
    return sim2d.simulate(seed=3, scenario='clean')


def run(rec, make, use_reliability_arg=None):
    prior, _ = sim2d.load_world('blocked')
    p = PFParams()
    f = make(prior, p, np.random.default_rng(7), (0.0, 0.0, 0.0))
    out = []
    for k in range(len(rec.t)):
        f.predict(rec.ds[k], fuse_heading(rec.dth_wheel[k], rec.dth_gyro[k], p.gyro_weight))
        if use_reliability_arg is None:
            f.update(rec.scans[k], A)
        else:
            f.update(rec.scans[k], A, use_reliability_arg)
        out.append(f.estimate()[:3])
    return np.array(out)


def baseline(prior, p, rng, init):
    return ParticleFilter(prior, p, rng, init)


def proposed(prior, p, rng, init):
    return ReliabilityAwareFilter(prior, p, rng, init, A)


def err(rec, est):
    return np.hypot(est[:, 0] - rec.pose[:, 0], est[:, 1] - rec.pose[:, 1])


def test_baseline_tracks_clean_run(rec_clean):
    e = err(rec_clean, run(rec_clean, baseline))
    assert e.max() < 0.35 and np.sqrt((e ** 2).mean()) < 0.15


def test_proposed_tracks_clean_run(rec_clean):
    e = err(rec_clean, run(rec_clean, proposed))
    assert e.max() < 0.35 and np.sqrt((e ** 2).mean()) < 0.15


def test_runs_are_deterministic(rec_clean):
    a = run(rec_clean, baseline)
    b = run(rec_clean, baseline)
    assert np.array_equal(a, b)


def run_with_sigma(rec, make):
    prior, _ = sim2d.load_world('blocked')
    p = PFParams()
    f = make(prior, p, np.random.default_rng(7), (0.0, 0.0, 0.0))
    est, std = [], []
    for k in range(len(rec.t)):
        f.predict(rec.ds[k], fuse_heading(rec.dth_wheel[k], rec.dth_gyro[k], p.gyro_weight))
        f.update(rec.scans[k], A)
        est.append(f.estimate()[:3])
        std.append(f.std_xy)
    return np.array(est), np.array(std)


@pytest.mark.parametrize('make', [baseline, proposed], ids=['baseline', 'proposed'])
def test_reported_sigma_is_credible(make, rec_clean):
    """The particle cloud's sigma must cover the real error: an honest 2D estimate has roughly 85-95% of errors within
    2 sigma and almost all within 3 (over five tuning seeds the clean 2-sigma coverage is 0.94). A single 780-step
    recording is noisy, because consecutive steps are correlated (seed 3, proposed filter: 0.849), so the bar here is
    0.80: it still fails for a collapsed, overconfident cloud, which the planner and the health state rely on not being."""
    est, std = run_with_sigma(rec_clean, make)
    e = err(rec_clean, est)
    assert np.mean(e <= 2 * std) >= 0.80
    assert np.mean(e <= 3 * std) >= 0.97
    assert np.median(std) < 0.1                  # and not trivially large


@pytest.mark.parametrize('make', [baseline, proposed], ids=['baseline', 'proposed'])
def test_parked_robot_does_not_drift(make):
    """Regression: constant motion-noise floors used to random-walk a parked robot (0.2-0.3 m in 90 s)."""
    _, true = sim2d.load_world('blocked')
    z0 = true.raycast(0.05, 0.0, A, sim2d.RMAX, sim2d.RMIN)
    rng = np.random.default_rng(0)
    prior, _ = sim2d.load_world('blocked')
    f = make(prior, PFParams(), np.random.default_rng(1000), (0.0, 0.0, 0.0))
    for _ in range(900):                      # 90 s stationary at the entry A
        z = z0.copy()
        fin = np.isfinite(z)
        z[fin] += rng.normal(0, sim2d.LIDAR_SIGMA, fin.sum())
        f.predict(0.0, 0.0)
        f.update(z, A)
    x, y, th, _ = f.estimate()
    assert math.hypot(x, y) < 0.06 and abs(th) < 0.05


@pytest.mark.parametrize('make', [baseline, proposed], ids=['baseline', 'proposed'])
def test_walls_block_the_particles(make):
    """Regression (Day 11): odometry that overshoots must not push particles through a wall. Drive 1.5 m west from the
    entry A into its end wall (x = -0.5): no particle may end inside it or within wall_block of it, and the estimate
    must stay in free space."""
    prior, _ = sim2d.load_world('blocked')
    p = PFParams()
    f = make(prior, p, np.random.default_rng(2), (0.0, 0.0, math.pi))     # at A, facing the end wall
    for _ in range(40):
        f.predict(0.04, 0.0)                                              # 40 steps x 4 cm = 1.6 m of 'wheel' motion
    assert float(prior.dist(f.x, f.y).min()) >= p.wall_block - 1e-9
    x, y, _, _ = f.estimate()
    assert not bool(prior.is_solid(x, y))


def test_unit_reliability_reduces_proposed_to_baseline(rec_clean):
    """Invariant: with every r_i = 1 the proposed filter IS the baseline (same RNG stream)."""
    a = run(rec_clean, baseline)
    b = run(rec_clean, proposed, use_reliability_arg=np.ones(len(A)))
    assert np.allclose(a, b)
