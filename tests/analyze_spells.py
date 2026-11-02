#!/usr/bin/env python3
"""Summarise recorded missions: duration, stationary spells, mean/max estimate error."""
import glob, os, sys
import numpy as np, pandas as pd
labels = sys.argv[1:]
for lab in labels:
    d = pd.read_csv(os.path.expanduser(f"~/sewer_ws/logs/day9/traj_{lab}.csv"))
    t = d.t.values
    sp = np.hypot(np.diff(d.gt_x), np.diff(d.gt_y)) / np.maximum(np.diff(t), 1e-6)
    rot = np.abs(np.diff(np.unwrap(d.gt_yaw.values))) / np.maximum(np.diff(t), 1e-6)
    still = t[1:][(sp < 0.03) & (rot < 0.1)]          # neither moving nor turning
    runs = []
    if len(still):
        a = b = still[0]
        for x in still[1:]:
            if x - b > 1.0:
                runs.append((a, b)); a = x
            b = x
        runs.append((a, b))
    err = np.hypot(d.est_x - d.gt_x, d.est_y - d.gt_y)
    turns = int(np.sum(np.abs(np.diff(np.sign(d.cmd_w.fillna(0).values))) > 1))
    print(f"{lab:6s} total {t[-1]:5.0f} s | idle spells>2.5s: {[(round(a), round(b)) for a, b in runs if b - a > 2.5]} | "
          f"cmd_w sign flips {turns} | est err mean {err.mean():.3f} max {err.max():.3f}")
