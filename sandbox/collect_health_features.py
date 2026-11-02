#!/usr/bin/env python3
"""Record the localization-health signals and the true error for both filters over many sandbox runs.

Output: ../logs/health_features.pkl = list of dicts (scenario, severity, seed, filter, array of rows). Row columns:
  t, err, std_xy, neff_ratio, correction, travel, fit_residual, R, gyro_disagree.
The error is for calibration/evaluation only. Runs in parallel (one process per recording).

Usage: python3 collect_health_features.py [n_seeds] [severity ...]      (tuning seeds 100+; 'clean' is always included)
"""
import multiprocessing as mp
import pickle
import sys

import numpy as np

import sim2d
from sewer_localization.particle_filter import (PFParams, ParticleFilter, ReliabilityAwareFilter,
                                                fuse_heading)

A = sim2d.ANGLES
COLS = ['t', 'err', 'std_xy', 'neff_ratio', 'correction', 'travel', 'fit_residual', 'R', 'gyro_disagree']


def collect(job):
    scen, sev, seed = job
    rec = sim2d.simulate(seed, scen, severity=sev)
    prior, _ = sim2d.load_world('blocked')
    p = PFParams()
    dth = fuse_heading(rec.dth_wheel, rec.dth_gyro, p.gyro_weight)
    out = []
    for name in ('baseline', 'reliability'):
        rng = np.random.default_rng(seed + 1000)
        f = (ParticleFilter(prior, p, rng, (0.0, 0.0, 0.0)) if name == 'baseline'
             else ReliabilityAwareFilter(prior, p, rng, (0.0, 0.0, 0.0), A))
        rows = []
        for k in range(len(rec.t)):
            f.predict(rec.ds[k], dth[k])
            f.update(rec.scans[k], A)
            x, y, _, _ = f.estimate()
            rows.append((rec.t[k], float(np.hypot(x - rec.pose[k, 0], y - rec.pose[k, 1])), f.std_xy, f.neff_ratio,
                         f.correction, f.last_travel, f.fit_residual, getattr(f, 'R', float('nan')) if name == 'reliability' else float('nan'),
                         abs(rec.dth_wheel[k] - rec.dth_gyro[k]) / sim2d.DT))
        out.append(dict(scenario=scen, severity=sev, seed=seed, filter=name, rows=np.array(rows)))
    return out


if __name__ == '__main__':
    n_seeds = int(sys.argv[1]) if len(sys.argv) > 1 else 5
    sevs = [float(s) for s in sys.argv[2:]] or [2.0, 3.0, 4.0, 5.0]
    jobs = [('clean', 1.0, s) for s in range(100, 100 + n_seeds)]
    jobs += [('disturbed', sev, s) for sev in sevs for s in range(100, 100 + n_seeds)]
    with mp.Pool(min(8, len(jobs))) as pool:
        results = [r for chunk in pool.imap_unordered(collect, jobs) for r in chunk]
    pickle.dump(dict(cols=COLS, runs=results), open('../logs/health_features.pkl', 'wb'))
    print(f'saved {len(results)} runs ({len(jobs)} recordings x 2 filters) to ../logs/health_features.pkl')
