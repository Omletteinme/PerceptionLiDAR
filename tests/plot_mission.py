#!/usr/bin/env python3
"""Plot live missions recorded by mission_check.py: trajectories on the map, error and reliability vs time."""
import json
import os
import sys

import matplotlib
import numpy as np
import pandas as pd

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, os.path.expanduser('~/sewer_ws/src/sewer_localization'))
from sewer_localization.map_utils import OccupancyMap  # noqa: E402

L = os.path.expanduser('~/sewer_ws/logs/day9')
labels = sys.argv[1:] or ['clean', 'x3']
prior = OccupancyMap.load(os.path.expanduser('~/sewer_ws/src/sewer_gazebo/maps/sewer_map'))
fig, axes = plt.subplots(len(labels), 2, figsize=(16, 5.2 * len(labels)), squeeze=False,
                         gridspec_kw=dict(width_ratios=[1.3, 1]))
for row, label in zip(axes, labels):
    d = pd.read_csv(f'{L}/traj_{label}.csv')
    ev = json.load(open(f'{L}/traj_{label}_events.json'))
    ax = row[0]
    ax.imshow(~prior.solid, cmap='gray', origin='lower', vmin=0, vmax=1.5,
              extent=[prior.x0, prior.x0 + prior.w * prior.res, prior.y0, prior.y0 + prior.h * prior.res])
    ax.add_patch(plt.Rectangle((11.4, -0.5), 0.9, 1.0, color='orangered', alpha=0.6, label='B1 (unmapped debris)'))
    ax.add_patch(plt.Rectangle((7.5, 2.7), 0.4, 0.6, color='orange', alpha=0.7, label='B2 (partial)'))
    sc = ax.scatter(d.gt_x, d.gt_y, c=d.t, s=6, cmap='viridis', label='true path (colour = time)')
    if 'dr_x' in d and d.dr_x.notna().any():
        ax.plot(d.dr_x, d.dr_y, '--', color='gray', lw=1.0, label='dead reckoning (disturbed odometry)')
    if 'bl_x' in d and d.bl_x.notna().any():
        ax.plot(d.bl_x, d.bl_y, '-', color='tab:blue', lw=0.8, alpha=0.8, label='baseline filter')
    ax.plot(d.est_x, d.est_y, 'r-', lw=0.8, alpha=0.8, label='reliability-aware filter')
    for k, (t, x, y) in ev['blocked_first'].items():
        ax.plot(x, y, 'm*', ms=16, label=f'{k} flagged blocked (t={t:.0f}s)')
    ax.set_xlim(-1.5, 17.5)
    ax.set_ylim(-1.5, 7.5)
    ax.set_aspect('equal')
    ax.set_title(f'{label}: robot path on the prior map')
    ax.legend(loc='upper left', fontsize=7)
    ax2 = row[1]
    err = np.hypot(d.est_x - d.gt_x, d.est_y - d.gt_y)
    if 'dr_x' in d and d.dr_x.notna().any():
        ax2.plot(d.t, np.hypot(d.dr_x - d.gt_x, d.dr_y - d.gt_y), '--', color='gray', label='dead reckoning error')
    if 'bl_x' in d and d.bl_x.notna().any():
        ax2.plot(d.t, np.hypot(d.bl_x - d.gt_x, d.bl_y - d.gt_y), '-', color='tab:blue', lw=1.0, label='baseline error')
    ax2.plot(d.t, err, 'r-', label='reliability-aware error [m]')
    ax2.set_xlabel('time since the robot started moving [s]')
    ax2.set_ylabel('error [m]')
    for t, nodes, _, _ in ev['route_events']:
        ax2.axvline(t, color='k', ls=':', lw=0.7)
        ax2.text(t, ax2.get_ylim()[1] * 0.95, '-'.join(nodes[:2]) if nodes else '-', fontsize=6, rotation=90, va='top')
    ax3 = ax2.twinx()
    ax3.plot(d.t, d.R, 'b-', alpha=0.6, label='scan reliability R')
    ax3.set_ylim(0, 1.05)
    ax3.set_ylabel('R')
    ax2.set_title(f'{label}: error, reliability, replans (dotted lines = route changes)')
    ax2.legend(loc='upper left', fontsize=7)
    ax3.legend(loc='upper right', fontsize=7)
fig.tight_layout()
out = f'{L}/missions.png'
fig.savefig(out, dpi=80)
print('wrote', out)
