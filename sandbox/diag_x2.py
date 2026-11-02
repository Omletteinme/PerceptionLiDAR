"""Why is the reliability-aware PF 10 % worse than the baseline at scan severity 1-2? Split slip from scan disturbances."""
import sys

import numpy as np

sys.path.insert(0, '.')
import sim2d  # noqa: E402
from run_experiment import metrics, run_filters  # noqa: E402

for sev in (2.0,):
    for slip in (False, True):
        r = {'baseline': [], 'reliability': []}
        for seed in range(10):
            rec = sim2d.simulate(seed, 'disturbed', severity=sev, odom_slip=slip)
            m = metrics(rec, run_filters(rec, seed))
            for k in r:
                r[k].append(m[k]['rmse'])
        b, a = np.array(r['baseline']), np.array(r['reliability'])
        print(f'x{sev:g} slip={slip}: baseline {b.mean():.4f}  reliability {a.mean():.4f}  '
              f'rel better in {(a < b).sum()}/10 seeds', flush=True)
