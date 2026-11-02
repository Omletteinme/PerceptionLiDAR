"""Evaluation metrics shared by the 2D sandbox and the Gazebo analysis.

CUSTOM code (ours), pure NumPy. Ground truth is used here and only here (to score), never by a filter or planner.
"""
import math

import numpy as np

RECOVER_M = 0.15     # "recovered": position error below this for 1 s
FAIL_M = 0.50        # "failure": position error above this


def pos_error(est_xy, true_xy):
    """Euclidean position error per step. Both arguments are (N, 2) arrays."""
    d = np.asarray(est_xy, float) - np.asarray(true_xy, float)
    return np.hypot(d[:, 0], d[:, 1])


def recovery_stats(t, err, windows, recover_m=RECOVER_M, dt=0.1):
    """For each disturbance window where the error exceeded `recover_m`: seconds after the window ends until the error
    stays below `recover_m` for 1 s. Unrecovered windows are censored at the end of the run.
    Returns (windows where the error exceeded, mean recovery time or NaN)."""
    t, err = np.asarray(t, float), np.asarray(err, float)
    hold = max(1, round(1.0 / dt))
    times, exceeded = [], 0
    for t0, t1 in windows:
        inside = (t >= t0) & (t < t1)
        if not inside.any() or err[inside].max() <= recover_m:
            continue
        exceeded += 1
        t_rec = t[-1] - t1
        for i in np.flatnonzero(t >= t1):
            if i + hold <= len(err) and (err[i:i + hold] < recover_m).all():
                t_rec = t[i] - t1
                break
        times.append(t_rec)
    return exceeded, (float(np.mean(times)) if times else float('nan'))


def track_metrics(t, err, windows=(), active=None, dt=0.1):
    """Localization metrics of one run from its time stamps, position error and disturbance windows."""
    err = np.asarray(err, float)
    exceeded, rec_t = recovery_stats(t, err, windows, dt=dt)
    out = dict(rmse=float(np.sqrt((err ** 2).mean())), max_err=float(err.max()), final_err=float(err[-1]),
               frac_above_fail=float((err > FAIL_M).mean()), windows_exceeding=exceeded, mean_recovery_s=rec_t)
    if active is not None:
        a = np.asarray(active, bool)
        out['rmse_disturbed'] = float(np.sqrt((err[a] ** 2).mean())) if a.any() else float('nan')
    return out


# two-sided 95 % Student-t quantiles for 1..30 degrees of freedom (so no scipy is needed in the ROS image)
_T95 = [12.706, 4.303, 3.182, 2.776, 2.571, 2.447, 2.365, 2.306, 2.262, 2.228, 2.201, 2.179, 2.160, 2.145, 2.131,
        2.120, 2.110, 2.101, 2.093, 2.086, 2.080, 2.074, 2.069, 2.064, 2.060, 2.056, 2.052, 2.048, 2.045, 2.042]


def mean_ci(values):
    """(mean, half-width of the 95 % confidence interval of the mean, n) ignoring NaNs. Half-width is NaN for n < 2."""
    v = np.asarray(values, float)
    v = v[~np.isnan(v)]
    n = len(v)
    if n == 0:
        return float('nan'), float('nan'), 0
    if n == 1:
        return float(v[0]), float('nan'), 1
    t = _T95[n - 2] if n - 1 <= len(_T95) else 1.96
    return float(v.mean()), float(t * v.std(ddof=1) / math.sqrt(n)), n


def fmt_ci(values, digits=3):
    """'mean ± ci' text (or just the mean when there is a single value)."""
    m, h, n = mean_ci(values)
    if n == 0:
        return 'n/a'
    return f'{m:.{digits}f}' if math.isnan(h) else f'{m:.{digits}f} ± {h:.{digits}f}'
