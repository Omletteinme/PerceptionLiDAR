#!/usr/bin/env python3
"""How often do the filters actually fail in the sandbox at each disturbance severity (scans AND wheel slip)?

Prints, per severity and filter, the fraction of time the position error exceeds 0.15 m (recover threshold) and
0.5 m (failure threshold) and the worst error, on the tuning seeds (100+). Used to see whether the sandbox can
reproduce the live failure before calibrating the health monitor.

Usage: python3 failure_rates.py [n_seeds] [severity ...]
"""
import sys

import numpy as np

import run_experiment as R
import sim2d

n_seeds = int(sys.argv[1]) if len(sys.argv) > 1 else 5
sevs = [float(s) for s in sys.argv[2:]] or [3.0, 4.0, 5.0]
print(f'{"severity":>9s} {"filter":12s} {"time>0.15m":>11s} {"time>0.5m":>10s} {"worst [m]":>10s} {"rmse":>7s}  per-seed worst')
for sev in sevs:
    acc = {'baseline': [], 'reliability': [], 'dead_reckoning': []}
    for seed in range(100, 100 + n_seeds):
        rec = sim2d.simulate(seed, 'disturbed', severity=sev)
        res = R.run_filters(rec, seed)
        for name in acc:
            acc[name].append(R.pos_err(rec, res[name]['est']))
    for name, runs in acc.items():
        e = np.concatenate(runs)
        print(f'{"x%g" % sev:>9s} {name:14s} {np.mean(e > 0.15):11.2f} {np.mean(e > 0.5):10.2f} {e.max():10.2f} '
              f'{np.sqrt(np.mean(e ** 2)):7.3f}  ' + ' '.join(f'{r.max():.2f}' for r in runs), flush=True)
