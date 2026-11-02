#!/usr/bin/env python3
"""M5: tables and plots for the Gazebo batch (tests/eval_batch.sh). Reads logs/day9/traj_<cond>-<sys>-<seed>{.csv,_summary.json}.

  cond:   clean | mild (scan x2, slip x2) | hard (scan x3, slip x3)
  system: rel   reliability-aware stack, health response on
          relnh reliability-aware stack, health response off
          base  baseline stack: baseline pose everywhere, ungated detector, constant R, no health response

Ground truth (the CSV's gt columns, the contact-sensor count) is used to SCORE only. Tables give mean ± 95 % CI over seeds.
Usage: python3 eval_gazebo_analyze.py [--logs ~/sewer_ws/logs] [--out ~/sewer_ws/logs/eval/gazebo]
"""
import argparse
import glob
import json
import math
import os
import re

import matplotlib
import numpy as np
import pandas as pd

from sewer_eval.metrics import fmt_ci, mean_ci, pos_error, track_metrics
from sewer_localization.disturbances import BASE_SCHEDULE

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

SYSTEMS = {'rel': 'reliability-aware + health response', 'relnh': 'reliability-aware, no health response',
           'base': 'baseline stack'}
CONDS = ['clean', 'mild', 'hard']
WINDOWS = sorted({(e['t0'], e['t1']) for e in BASE_SCHEDULE})
NAME = re.compile(r'traj_(clean|mild|hard)-(rel|relnh|base)-(\d+)_summary\.json$')


def load_run(path):
    cond, system, seed = NAME.search(path).groups()
    sm = json.load(open(path))
    df = pd.read_csv(path.replace('_summary.json', '.csv'))
    t = df['t'].to_numpy()
    gt = df[['gt_x', 'gt_y']].to_numpy()
    row = dict(cond=cond, system=system, seed=int(seed), goal=bool(sm['goal_reached']), mission_time=sm['mission_time'],
               path_length=sm['path_length'], collisions=sm['collisions']['total'], contact_time=sm['collisions']['contact_time'],
               min_clear=sm['min_clear'], route_cost=sm['route_cost_planned'],
               phantom=int(len(set(sm['blocked_first']) - {'J1-J4'})))
    row['success'] = row['goal'] and row['collisions'] == 0
    windows = WINDOWS if cond != 'clean' else []
    dt = float(np.median(np.diff(t)))
    for tag, cols in (('rel', ['est_x', 'est_y']), ('base', ['bl_x', 'bl_y']), ('dr', ['dr_x', 'dr_y'])):
        est = df[cols].to_numpy()
        ok = ~np.isnan(est).any(axis=1)
        if ok.sum() < 20:
            continue
        m = track_metrics(t[ok], pos_error(est[ok], gt[ok]), windows, dt=dt)
        row.update({f'{tag}_{k}': v for k, v in m.items()})
    own = 'base' if system == 'base' else 'rel'                      # the estimate that drove the robot
    for k in ('rmse', 'max_err', 'frac_above_fail', 'mean_recovery_s'):
        row[f'drv_{k}'] = row.get(f'{own}_{k}', float('nan'))
    h = df['health'].to_numpy()
    step = np.diff(t, append=t[-1])
    for k, name in enumerate(('reliable', 'degraded', 'critical')):
        row[f'time_{name}'] = float(step[h == k].sum())
    return row


def table(df, cols, header):
    out = ['| condition | system | n | ' + ' | '.join(h for _, h in cols) + ' |', '|---|---|---|' + '---|' * len(cols)]
    for c in CONDS:
        for s in SYSTEMS:
            d = df[(df.cond == c) & (df.system == s)]
            if len(d):
                out.append(f'| {c} | {SYSTEMS[s]} | {len(d)} | ' + ' | '.join(
                    f'{100 * d[k].mean():.0f} %' if k in ('success', 'goal') else fmt_ci(d[k], 2) for k, _ in cols) + ' |')
    return f'### {header}\n\n' + '\n'.join(out) + '\n'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--logs', default=os.path.expanduser('~/sewer_ws/logs'))
    ap.add_argument('--out', default=os.path.expanduser('~/sewer_ws/logs/eval/gazebo'))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    paths = sorted(p for p in glob.glob(os.path.join(a.logs, 'day9', 'traj_*_summary.json')) if NAME.search(p))
    df = pd.DataFrame([load_run(p) for p in paths])
    df.to_csv(os.path.join(a.out, 'runs.csv'), index=False)
    text = [f'# Gazebo evaluation: {len(df)} missions (mean ± 95 % CI over seeds; n = seeds per cell)\n',
            table(df, [('success', 'success (goal, 0 collisions)'), ('collisions', 'collisions'),
                       ('mission_time', 'mission time [s]'), ('path_length', 'path driven [m]'),
                       ('route_cost', 'planned route cost')], 'Mission outcome'),
            table(df, [('drv_rmse', 'RMSE [m]'), ('drv_max_err', 'max error [m]'), ('drv_frac_above_fail', 'time above 0.5 m'),
                       ('drv_mean_recovery_s', 'recovery [s]'), ('phantom', 'phantom blocked edges')],
                  'Localization error of the estimate that drove the robot'),
            table(df, [('time_reliable', 'RELIABLE [s]'), ('time_degraded', 'DEGRADED [s]'), ('time_critical', 'CRITICAL [s]')],
                  'Time in each localization-health state')]
    both = df[df.system.isin(['rel', 'relnh'])].dropna(subset=['rel_rmse', 'base_rmse'])
    if len(both):
        rows = ['| condition | n | reliability-aware RMSE [m] | baseline-filter RMSE [m] | dead reckoning RMSE [m] |', '|---|---|---|---|---|']
        for c in CONDS:
            d = both[both.cond == c]
            if len(d):
                rows.append(f'| {c} | {len(d)} | {fmt_ci(d.rel_rmse)} | {fmt_ci(d.base_rmse)} | {fmt_ci(d.dr_rmse)} |')
        text.append('### Paired localization comparison inside the reliability-aware missions (both filters ran on the identical '
                    'disturbed data; the robot was driven by the reliability-aware one)\n\n' + '\n'.join(rows) + '\n')
    open(os.path.join(a.out, 'tables.md'), 'w', encoding='utf-8', newline='\n').write('\n'.join(text))
    fig, ax = plt.subplots(1, 3, figsize=(13, 3.8))
    for k, (key, ttl) in enumerate((('success', 'success rate'), ('drv_rmse', 'RMSE of the driving estimate [m]'),
                                    ('mission_time', 'mission time [s]'))):
        for i, s in enumerate(SYSTEMS):
            ms = [mean_ci(df[(df.cond == c) & (df.system == s)][key].astype(float)) for c in CONDS]
            ax[k].errorbar(np.arange(3) + (i - 1) * 0.12, [m[0] for m in ms],
                           yerr=[0 if math.isnan(m[1]) else m[1] for m in ms], marker='o', capsize=3, label=SYSTEMS[s])
        ax[k].set_xticks(range(3), CONDS)
        ax[k].set_title(ttl)
    ax[0].legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(os.path.join(a.out, 'gazebo_summary.png'), dpi=90)
    print('\n'.join(text))
    print('wrote', a.out)


if __name__ == '__main__':
    main()
