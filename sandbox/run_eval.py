#!/usr/bin/env python3
"""M5 sandbox evaluation: dead reckoning vs baseline PF vs reliability-aware PF on identical recordings.

CUSTOM code (ours). 30 seeds per condition (reported seeds 0-29; the health/threshold tuning used seeds 100+, so there is no
overlap), clean plus disturbance severities 1-5 (scan disturbances and odometry wheel slip scale together). Every estimator
sees the same recording and the two filters share the particle-noise stream. Writes results.csv, tables.md and a plot.
Tables give mean ± 95 % confidence interval over seeds; the paired baseline-minus-reliability RMSE difference is also given.

Usage: python3 run_eval.py [--seeds 30] [--seed0 0] [--severities 1,2,3,4,5] [--procs 6] [--out ../logs/eval/sandbox]
"""
import argparse
import multiprocessing as mp
import os

import matplotlib
import numpy as np
import pandas as pd

import sim2d
from run_experiment import FILTERS, metrics, run_filters
from sewer_eval.metrics import fmt_ci, mean_ci

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

LABEL = {'dead_reckoning': 'dead reckoning', 'baseline': 'baseline PF', 'reliability': 'reliability-aware PF'}


def run_one(job):
    scen, sev, seed = job
    rec = sim2d.simulate(seed, scen, severity=sev)
    res = run_filters(rec, seed)
    return [dict(condition='clean' if scen == 'clean' else f'x{sev:g}', severity=0.0 if scen == 'clean' else sev,
                 seed=seed, filter=name, **m) for name, m in metrics(rec, res).items()]


def tables(df, conds):
    out = []
    cols = [('rmse', 'RMSE [m]'), ('max_err', 'max error [m]'), ('frac_above_fail', 'time above 0.5 m'),
            ('mean_recovery_s', 'recovery [s]')]
    out.append('### Localization error (mean ± 95 % CI over seeds)\n')
    out.append('| condition | estimator | ' + ' | '.join(c[1] for c in cols) + ' | runs that ever exceed 0.5 m |')
    out.append('|---|---|' + '---|' * (len(cols) + 1))
    for cond in conds:
        for f in FILTERS:
            d = df[(df.condition == cond) & (df['filter'] == f)]
            cells = [fmt_ci(d[c]) for c, _ in cols]
            out.append(f'| {cond} | {LABEL[f]} | ' + ' | '.join(cells) + f' | {int((d.max_err > 0.5).sum())}/{len(d)} |')
    out.append('\n### Paired difference, baseline minus reliability-aware RMSE (positive = reliability-aware better)\n')
    out.append('| condition | mean difference [m] ± 95 % CI | reliability-aware better in | median per-seed ratio (rel./base) |')
    out.append('|---|---|---|---|')
    for cond in conds:
        p = df[df.condition == cond].pivot(index='seed', columns='filter', values='rmse')
        diff = p['baseline'] - p['reliability']
        out.append(f'| {cond} | {fmt_ci(diff, 4)} | {int((diff > 0).sum())}/{len(diff)} seeds | '
                   f'{float(np.median(p["reliability"] / p["baseline"])):.2f} |')
    return '\n'.join(out) + '\n'


def plot(df, conds, path):
    fig, ax = plt.subplots(1, 2, figsize=(11, 4))
    x = np.arange(len(conds))
    for k, f in enumerate(('dead_reckoning', 'baseline', 'reliability')):
        for a, key, ttl in ((ax[0], 'rmse', 'position RMSE [m]'), (ax[1], 'max_err', 'maximum position error [m]')):
            ms = [mean_ci(df[(df.condition == c) & (df['filter'] == f)][key]) for c in conds]
            a.errorbar(x + (k - 1) * 0.12, [m[0] for m in ms], yerr=[0 if np.isnan(m[1]) else m[1] for m in ms],
                       marker='o', capsize=3, label=LABEL[f], color=sim2d_color(f))
            a.set_xticks(x, conds)
            a.set_ylabel(ttl)
            a.set_yscale('log')
            a.set_xlabel('disturbance severity (scan disturbances and wheel slip)')
    ax[0].legend()
    fig.suptitle('Sandbox localization error, mean ± 95 % CI over seeds')
    fig.tight_layout()
    fig.savefig(path, dpi=90)


def sim2d_color(f):
    return {'dead_reckoning': 'tab:gray', 'baseline': 'tab:blue', 'reliability': 'tab:red'}[f]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seeds', type=int, default=30)
    ap.add_argument('--seed0', type=int, default=0)
    ap.add_argument('--severities', default='1,2,3,4,5')
    ap.add_argument('--procs', type=int, default=6)
    ap.add_argument('--out', default=os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'logs', 'eval', 'sandbox'))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    sevs = [float(s) for s in a.severities.split(',')]
    seeds = range(a.seed0, a.seed0 + a.seeds)
    jobs = [('clean', 1.0, s) for s in seeds] + [('disturbed', v, s) for v in sevs for s in seeds]
    with mp.Pool(a.procs) as pool:
        rows = [r for chunk in pool.imap_unordered(run_one, jobs) for r in chunk]
    df = pd.DataFrame(rows).sort_values(['severity', 'seed', 'filter'])
    df.to_csv(os.path.join(a.out, 'results.csv'), index=False)
    conds = ['clean'] + [f'x{v:g}' for v in sevs]
    text = (f'# Sandbox evaluation: {a.seeds} seeds per condition (seeds {a.seed0}-{a.seed0 + a.seeds - 1})\n\n'
            + tables(df, conds))
    open(os.path.join(a.out, 'tables.md'), 'w', encoding='utf-8', newline='\n').write(text)
    plot(df, conds, os.path.join(a.out, 'rmse_vs_severity.png'))
    print(text)
    print('wrote', os.path.abspath(a.out))


if __name__ == '__main__':
    main()
