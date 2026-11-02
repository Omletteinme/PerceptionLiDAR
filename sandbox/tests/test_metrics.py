"""Tests for the shared evaluation metrics."""
import math

import numpy as np
import pytest

from sewer_eval.metrics import fmt_ci, mean_ci, pos_error, recovery_stats, track_metrics


def test_pos_error_is_euclidean():
    e = pos_error([[3.0, 4.0], [1.0, 1.0]], [[0.0, 0.0], [1.0, 1.0]])
    assert e == pytest.approx([5.0, 0.0])


def test_recovery_time_counts_from_the_end_of_the_window_until_one_second_below_threshold():
    t = np.arange(1, 301) * 0.1                              # 30 s
    err = np.full(300, 0.02)
    err[(t >= 10) & (t < 16)] = 0.4                          # window 10-14 s, error stays high until 16 s
    n, rec = recovery_stats(t, err, [(10.0, 14.0)])
    assert n == 1 and rec == pytest.approx(2.1, abs=0.11)    # first full second below 0.15 m starts at t = 16.0


def test_window_without_exceedance_is_ignored_and_unrecovered_is_censored():
    t = np.arange(1, 201) * 0.1
    quiet = np.full(200, 0.05)
    assert recovery_stats(t, quiet, [(5.0, 8.0)]) == (0, pytest.approx(float('nan'), nan_ok=True))
    lost = np.where(t >= 5.0, 1.0, 0.05)
    n, rec = recovery_stats(t, lost, [(5.0, 8.0)])
    assert n == 1 and rec == pytest.approx(t[-1] - 8.0)


def test_track_metrics_summarises_error_and_failure_fraction():
    t = np.arange(1, 101) * 0.1
    err = np.concatenate([np.full(80, 0.1), np.full(20, 0.6)])
    m = track_metrics(t, err, windows=[(8.0, 9.0)], active=err > 0.5)
    assert m['rmse'] == pytest.approx(math.sqrt(0.8 * 0.01 + 0.2 * 0.36))
    assert m['max_err'] == 0.6 and m['final_err'] == 0.6 and m['frac_above_fail'] == pytest.approx(0.2)
    assert m['rmse_disturbed'] == pytest.approx(0.6)


def test_mean_ci_uses_student_t_and_skips_nan():
    m, h, n = mean_ci([1.0, 2.0, 3.0, 4.0, math.nan])
    assert (m, n) == (2.5, 4)
    assert h == pytest.approx(3.182 * np.std([1, 2, 3, 4], ddof=1) / 2)
    assert mean_ci([])[2] == 0 and math.isnan(mean_ci([5.0])[1])
    assert fmt_ci([1.0, 3.0], 2).startswith('2.00 ± ') and fmt_ci([]) == 'n/a'
