#!/usr/bin/env python3
"""Calibrate and evaluate the localization-health monitor on the features recorded by collect_health_features.py.

1. Which signal separates a failing estimate (true error > 0.5 m) from a healthy one? (AUC per smoothed signal)
2. How does the HealthMonitor with its current parameters behave: time in each state per condition, false alarms
   on clean runs, how much of the failing time it flags CRITICAL, and the detection delay.

The true error is used only to calibrate and score. Usage: python3 analyze_health.py [features.pkl]
"""
import math
import os
import pickle
import sys
from collections import defaultdict

import numpy as np
from scipy.stats import rankdata

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'sewer_localization'))
from sewer_localization.health import CRITICAL, DEGRADED, HealthMonitor, HealthParams

DT = 0.1
path = sys.argv[1] if len(sys.argv) > 1 else '../logs/health_features.pkl'
data = pickle.load(open(path, 'rb'))
cols = {c: i for i, c in enumerate(data['cols'])}
runs = data['runs']


def ema(x, tau=1.0):
    a = 1.0 - math.exp(-DT / tau)
    out, v = np.empty(len(x)), None
    for i, xi in enumerate(x):
        if not math.isnan(xi):
            v = xi if v is None else v + a * (xi - v)
        out[i] = float('nan') if v is None else v
    return out


def signals(rows):
    """The smoothed signals the monitor sees (same definitions as HealthMonitor)."""
    g = lambda n: rows[:, cols[n]]   # noqa: E731
    corr, trav = ema(g('correction')), ema(g('travel'))
    innov = np.where(trav > 0.003, corr / np.maximum(trav, 1e-9), np.nan)
    return {'sigma': ema(g('std_xy')), 'innovation': ema(innov), 'fit': ema(g('fit_residual')), 'R': ema(g('R')),
            'gyro': ema(g('gyro_disagree')), '1-neff': ema(1.0 - g('neff_ratio'))}


def auc(pos, neg):
    """P(score of a failing step > score of a healthy step)."""
    pos, neg = pos[~np.isnan(pos)], neg[~np.isnan(neg)]
    if len(pos) < 20 or len(neg) < 20:
        return float('nan')
    r = rankdata(np.concatenate([pos, neg]))
    return (r[:len(pos)].sum() - len(pos) * (len(pos) + 1) / 2) / (len(pos) * len(neg))


print('=== (1) AUC of each smoothed signal for "error > 0.5 m" (1.0 = perfect, 0.5 = useless); warm-up (t < 3 s) excluded')
for fname in ('baseline', 'reliability'):
    pos, neg = defaultdict(list), defaultdict(list)
    for r in (x for x in runs if x['filter'] == fname):
        rows = r['rows']
        keep = rows[:, cols['t']] >= 3.0
        fail = rows[:, cols['err']] > 0.5
        for name, s in signals(rows).items():
            sign = -1.0 if name == 'R' else 1.0            # low reliability = bad
            pos[name].append(sign * s[keep & fail])
            neg[name].append(sign * s[keep & ~fail])
    n_fail = sum(len(p) for p in pos['sigma'])
    print(f'{fname:12s} failing steps: {n_fail:5d}  ' + '  '.join(
        f'{n}: {auc(np.concatenate(pos[n]), np.concatenate(neg[n])):.2f}' for n in pos))

print('\n=== (2) HealthMonitor with the current parameters')
params = HealthParams()
print(f'{"filter":12s} {"condition":14s} {"RELIABLE":>9s} {"DEGRADED":>9s} {"CRITICAL":>9s}   {"CRITICAL|err>0.5":>17s} {"err>0.5|CRITICAL":>17s} {"delay [s]":>10s}')
for fname in ('baseline', 'reliability'):
    cond = defaultdict(list)
    for r in (x for x in runs if x['filter'] == fname):
        cond[(r['scenario'], r['severity'])].append(r['rows'])
    for key in sorted(cond, key=lambda k: (k[0] != 'clean', k[1])):
        frac = np.zeros(3)
        crit_when_fail = [0, 0]
        crit_flag = [0, 0]
        delays = []
        for rows in cond[key]:
            mon = HealthMonitor(params)
            states = []
            for row in rows:
                g = lambda n: row[cols[n]]   # noqa: E731
                states.append(mon.update(DT, fit=g('fit_residual'), R=None if math.isnan(g('R')) else g('R'),
                                         gyro=g('gyro_disagree')))
            states = np.array(states)
            err = rows[:, cols['err']]
            for i, s in enumerate(('RELIABLE', DEGRADED, CRITICAL)):
                frac[i] += np.mean(states == s)
            fail = err > 0.5
            crit = states == CRITICAL
            crit_when_fail[0] += int((crit & fail).sum()); crit_when_fail[1] += int(fail.sum())
            crit_flag[0] += int((crit & fail).sum()); crit_flag[1] += int(crit.sum())
            if fail.any():
                t_fail = rows[np.argmax(fail), cols['t']]
                after = np.flatnonzero(crit & (rows[:, cols['t']] >= t_fail - 5.0))
                delays.append(rows[after[0], cols['t']] - t_fail if len(after) else float('nan'))
        frac /= len(cond[key])
        rec = crit_when_fail[0] / crit_when_fail[1] if crit_when_fail[1] else float('nan')
        prec = crit_flag[0] / crit_flag[1] if crit_flag[1] else float('nan')
        label = 'clean' if key[0] == 'clean' else f'severity x{key[1]:g}'
        print(f'{fname:12s} {label:14s} {frac[0]:9.2f} {frac[1]:9.2f} {frac[2]:9.2f}   {rec:17.2f} {prec:17.2f} '
              f'{np.nanmean(delays) if delays and not np.all(np.isnan(delays)) else float("nan"):10.1f}')
