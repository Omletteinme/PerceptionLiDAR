#!/usr/bin/env python3
"""Is the particle filter's reported uncertainty credible, and what does it cost in accuracy?

For each filter and scenario: how often is the true position error within 1, 2 and 3 of the reported sigma
(std_xy, the largest standard deviation of the particle cloud)? Plus the median sigma, the RMSE, and the per-seed
RMSE (a lost track shows up there). An honest 2D estimate covers roughly 40-60% at 1 sigma, 85-95% at 2 sigma
and > 97% at 3 sigma. Uses tuning seeds (100+), disjoint from the seeds reported in the evaluation.

Usage: python3 check_sigma.py [n_seeds] [rough_k rough_floor_xy rough_floor_th]
       (the optional roughening values override the PFParams defaults, to compare variants)
"""
import functools
import sys

import numpy as np

import run_experiment as R
import sim2d
from sewer_localization.particle_filter import PFParams

n_seeds = int(sys.argv[1]) if len(sys.argv) > 1 else 5
if len(sys.argv) > 4:
    k, fxy, fth = (float(v) for v in sys.argv[2:5])
    R.PFParams = functools.partial(PFParams, rough_k=k, rough_floor_xy=fxy, rough_floor_th=fth)
    print(f'roughening override: k={k} floor_xy={fxy} floor_th={fth}')
else:
    print(f'roughening defaults: k={PFParams.rough_k} floor_xy={PFParams.rough_floor_xy} floor_th={PFParams.rough_floor_th}')
print(f'{"scenario":14s} {"filter":12s} {"<=1sig":>7s} {"<=2sig":>7s} {"<=3sig":>7s} {"med sigma":>10s} {"rmse":>7s}  per-seed rmse')
for scen, sev in (('clean', 1.0), ('disturbed', 3.0)):
    acc = {'baseline': [], 'reliability': []}
    for seed in range(100, 100 + n_seeds):
        rec = sim2d.simulate(seed, scen, severity=sev)
        res = R.run_filters(rec, seed)
        for name in acc:
            acc[name].append((R.pos_err(rec, res[name]['est']), res[name]['std']))
    for name, runs in acc.items():
        e = np.concatenate([r[0] for r in runs])
        s = np.concatenate([r[1] for r in runs])
        c = [float(np.mean(e <= q * s)) for q in (1, 2, 3)]
        per = ' '.join(f'{np.sqrt(np.mean(r[0] ** 2)):.3f}' for r in runs)
        print(f'{scen + (" x%g" % sev if scen == "disturbed" else ""):14s} {name:12s} {c[0]:7.2f} {c[1]:7.2f} {c[2]:7.2f} '
              f'{np.median(s):10.3f} {np.sqrt(np.mean(e ** 2)):7.3f}  {per}', flush=True)
