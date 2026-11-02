#!/usr/bin/env python3
"""Run dead reckoning, the baseline PF and the reliability-aware PF on IDENTICAL recorded scenarios.

CUSTOM code (ours). For every (scenario, seed) the run is recorded once (true pose, noisy odometry/gyro,
noisy and disturbed scans) and all estimators consume that same recording. The two filters also share
the same particle-noise RNG seed. Ground truth is used only to compute errors.

Usage:  python3 run_experiment.py [--seeds 5] [--out ../logs/day6]
"""
import argparse
import math
import os

import matplotlib
import numpy as np
import pandas as pd

import sim2d
from sewer_eval.metrics import FAIL_M, RECOVER_M, pos_error, recovery_stats  # noqa: F401  (shared with the Gazebo analysis)
from sewer_localization.map_utils import valid_returns
from sewer_localization.particle_filter import (PFParams, ParticleFilter, ReliabilityAwareFilter,
                                                fuse_heading, wrap)

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

A = sim2d.ANGLES
FILTERS = ('dead_reckoning', 'baseline', 'reliability')
COLORS = {'dead_reckoning': 'tab:gray', 'baseline': 'tab:blue', 'reliability': 'tab:red'}


def run_filters(rec, seed):
    prior, _ = sim2d.load_world('blocked')
    p = PFParams()
    n = len(rec.t)
    dth_f = fuse_heading(rec.dth_wheel, rec.dth_gyro, p.gyro_weight)
    out = {}

    x = y = th = 0.0                                   # dead reckoning: fused odometry/IMU only
    dr = np.empty((n, 3))
    for k in range(n):
        x += rec.ds[k] * math.cos(th + 0.5 * dth_f[k])
        y += rec.ds[k] * math.sin(th + 0.5 * dth_f[k])
        th = wrap(th + dth_f[k])
        dr[k] = (x, y, th)
    out['dead_reckoning'] = dict(est=dr, std=np.full(n, np.nan))

    for name in ('baseline', 'reliability'):
        rng = np.random.default_rng(seed + 1000)       # same particle noise stream for both
        f = (ParticleFilter(prior, p, rng, (0.0, 0.0, 0.0)) if name == 'baseline'
             else ReliabilityAwareFilter(prior, p, rng, (0.0, 0.0, 0.0), A))
        est, std, neff = np.empty((n, 3)), np.empty(n), np.empty(n)
        R, rmat = np.full(n, np.nan), np.zeros((n, len(A)))
        for k in range(n):
            f.predict(rec.ds[k], dth_f[k])
            f.update(rec.scans[k], A)
            est[k] = f.estimate()[:3]
            std[k], neff[k] = f.std_xy, f.neff
            if name == 'reliability':
                R[k], rmat[k] = f.R, f.r
        out[name] = dict(est=est, std=std, neff=neff)
        if name == 'reliability':
            out[name].update(R=R, r=rmat)
    return out


def pos_err(rec, est):
    return pos_error(est[:, :2], rec.pose[:, :2])


def metrics(rec, res):
    rows = {}
    for name in FILTERS:
        est = res[name]['est']
        e = pos_err(rec, est)
        he = np.abs(wrap(est[:, 2] - rec.pose[:, 2]))
        exceeded, rec_t = recovery_stats(rec.t, e, rec.windows)
        d = rec.active
        rows[name] = dict(
            rmse=float(np.sqrt((e ** 2).mean())), max_err=float(e.max()), final_err=float(e[-1]),
            heading_rmse_deg=float(np.degrees(np.sqrt((he ** 2).mean()))),
            rmse_disturbed=float(np.sqrt((e[d] ** 2).mean())) if d.any() else float('nan'),
            frac_above_fail=float((e > FAIL_M).mean()),
            windows_exceeding=exceeded, mean_recovery_s=rec_t)
    return rows


def beam_class_means(rec, rmat):
    """Mean per-beam reliability by injected-disturbance class (valid returns only)."""
    v = valid_returns(rec.scans, sim2d.RMIN, sim2d.RMAX)
    return {name: (float(rmat[v & (rec.kind == code)].mean()) if (v & (rec.kind == code)).any() else float('nan'),
                   int((v & (rec.kind == code)).sum()))
            for name, code in sim2d.KIND.items()}


def shade(ax, rec):
    for t0, t1 in rec.windows:
        ax.axvspan(t0, t1, color='orange', alpha=0.15, lw=0)


def plot_errors(recs, results, out):
    fig, axes = plt.subplots(len(recs), 1, figsize=(11, 3.6 * len(recs)), squeeze=False)
    for ax, (scen, rec) in zip(axes[:, 0], recs.items()):
        shade(ax, rec)
        for name in FILTERS:
            ax.plot(rec.t, pos_err(rec, results[scen][name]['est']), color=COLORS[name], label=name,
                    lw=1.6 if name != 'dead_reckoning' else 1.0)
        ax.axhline(RECOVER_M, color='k', ls=':', lw=0.8)
        ax.axhline(FAIL_M, color='k', ls='--', lw=0.8)
        ax.set_yscale('symlog', linthresh=0.1)
        ax.set_ylabel('position error [m]')
        ax.set_title(f'{scen} (seed {rec.seed}); shaded = disturbance windows; dotted/dashed = recover/fail thresholds')
        ax.legend(loc='upper left')
    axes[-1, 0].set_xlabel('time [s]')
    fig.tight_layout()
    fig.savefig(out, dpi=80)


def plot_reliability(rec, res, class_means, out):
    fig, axes = plt.subplots(2, 1, figsize=(11, 7))
    ax = axes[0]
    shade(ax, rec)
    ax.plot(rec.t, res['reliability']['R'], color='tab:red', label='scan reliability R (mean r_i)')
    ax.set_ylim(0, 1.05)
    ax.set_xlabel('time [s]')
    ax.set_ylabel('R')
    ax.set_title(f'Scan reliability, disturbed scenario (seed {rec.seed}); shaded = disturbance windows')
    ax.legend(loc='lower left')
    ax = axes[1]
    names = ['none', 'burst', 'spurious', 'water']
    vals = [np.nanmean([cm[k][0] for cm in class_means]) for k in names]
    ax.bar(names, vals, color=['tab:green', 'tab:olive', 'tab:red', 'tab:purple'])
    for i, v in enumerate(vals):
        ax.text(i, v + 0.02, f'{v:.2f}', ha='center')
    ax.set_ylim(0, 1.05)
    ax.set_ylabel('mean per-beam reliability r_i')
    ax.set_title('Mean reliability by injected beam class (pooled over seeds): clean beams vs corrupted beams')
    fig.tight_layout()
    fig.savefig(out, dpi=80)


def plot_traj(rec, res, out):
    prior, _ = sim2d.load_world('blocked')
    fig, ax = plt.subplots(figsize=(9, 8))
    ax.imshow(~prior.solid, cmap='gray', origin='lower', vmin=0, vmax=1.5,
              extent=[prior.x0, prior.x0 + prior.w * prior.res, prior.y0, prior.y0 + prior.h * prior.res])
    ax.plot(rec.pose[:, 0], rec.pose[:, 1], 'g-', lw=3, label='true', alpha=0.6)
    for name in FILTERS:
        e = res[name]['est']
        ax.plot(e[:, 0], e[:, 1], color=COLORS[name], lw=1.2, label=name)
    ax.set_xlim(-1.5, 17.5)
    ax.set_ylim(-1.5, 7.5)
    ax.set_aspect('equal')
    ax.legend()
    ax.set_title(f'Trajectories, disturbed scenario (seed {rec.seed})')
    fig.savefig(out, dpi=80, bbox_inches='tight')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seeds', type=int, default=5)
    ap.add_argument('--severities', default='1,2,3,4', help='disturbance severity multipliers to sweep')
    ap.add_argument('--out', default=os.path.join(os.path.dirname(__file__), '..', 'logs', 'day6'))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    sevs = [float(s) for s in a.severities.split(',')]
    cases = [('clean', 1.0)] + [('disturbed', s) for s in sevs]
    label = lambda scen, s: 'clean' if scen == 'clean' else f'disturbed x{s:g}'  # noqa: E731
    rows, class_means, first = [], {}, {}
    for scen, sev in cases:
        for seed in range(a.seeds):
            rec = sim2d.simulate(seed, scen, severity=sev)
            res = run_filters(rec, seed)
            for name, m in metrics(rec, res).items():
                rows.append(dict(scenario=label(scen, sev), severity=sev if scen == 'disturbed' else 0.0,
                                 seed=seed, filter=name, **m))
            if scen == 'disturbed':
                class_means.setdefault(sev, []).append(beam_class_means(rec, res['reliability']['r']))
            if seed == 0:
                first[label(scen, sev)] = (rec, res)
            print(f'done {label(scen, sev)} seed {seed}', flush=True)
    df = pd.DataFrame(rows)
    df.to_csv(os.path.join(a.out, 'results.csv'), index=False)
    pd.set_option('display.width', 220)
    pd.set_option('display.max_columns', 20)
    pd.set_option('display.float_format', lambda v: f'{v:.3f}')
    order = [(label(s, v), f) for s, v in cases for f in FILTERS]
    cols = ['rmse', 'max_err', 'final_err', 'heading_rmse_deg', 'rmse_disturbed', 'frac_above_fail',
            'windows_exceeding', 'mean_recovery_s']
    print('\n=== mean over seeds ===')
    print(df.groupby(['scenario', 'filter'])[cols].mean().reindex(order))
    print('\n=== severity sweep: RMSE [m] mean over seeds, and seeds where reliability beats baseline ===')
    for s, v in cases:
        d = df[df.scenario == label(s, v)].pivot(index='seed', columns='filter', values='rmse')
        wins = int((d['reliability'] < d['baseline']).sum())
        print(f'{label(s, v):14s} baseline {d["baseline"].mean():.3f}  reliability {d["reliability"].mean():.3f}  '
              f'(rel. better in {wins}/{len(d)} seeds; max err baseline '
              f'{df[(df.scenario == label(s, v)) & (df["filter"] == "baseline")].max_err.mean():.2f} '
              f'reliability {df[(df.scenario == label(s, v)) & (df["filter"] == "reliability")].max_err.mean():.2f})')
    print('\n=== mean per-beam reliability by injected class (pooled over seeds) ===')
    for sev, cms in class_means.items():
        parts = []
        for k in ('none', 'burst', 'spurious', 'water'):
            vals = [cm[k][0] for cm in cms if cm[k][1] > 0]
            parts.append(f'{k} {np.mean(vals) if vals else float("nan"):.2f}')
        print(f'x{sev:g}: ' + '  '.join(parts))
    s_lo, s_hi = label('disturbed', sevs[0]), label('disturbed', sevs[-1])
    sub = {k: first[k] for k in ('clean', s_lo, s_hi) if k in first}
    plot_errors({k: v[0] for k, v in sub.items()}, {k: v[1] for k, v in sub.items()},
                os.path.join(a.out, 'error_vs_time.png'))
    plot_reliability(first[s_lo][0], first[s_lo][1], class_means[sevs[0]], os.path.join(a.out, 'reliability.png'))
    plot_traj(first[s_hi][0], first[s_hi][1], os.path.join(a.out, 'trajectories.png'))
    print('\nwrote', os.path.abspath(a.out))


if __name__ == '__main__':
    main()
