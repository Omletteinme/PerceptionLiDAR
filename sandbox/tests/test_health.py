"""Tests for the localization-health state machine (pure logic; thresholds are calibrated separately)."""
import math

import pytest

from sewer_localization.health import CRITICAL, DEGRADED, RELIABLE, HealthMonitor, HealthParams, stress

DT = 0.1
P = HealthParams()
GOOD = dict(fit=0.01, R=0.95, gyro=0.0)


def run(mon, seconds, **signals):
    state = None
    for _ in range(round(seconds / DT)):
        state = mon.update(DT, **signals)
    return state


def test_stress_is_zero_at_ok_one_at_bad_and_works_in_both_directions():
    assert stress(0.01, 0.03, 0.12) == 0.0
    assert stress(0.12, 0.03, 0.12) == 1.0
    assert stress(0.35, 0.85, 0.35) == 1.0                # R: lower is worse
    assert stress(0.95, 0.85, 0.35) == 0.0
    assert stress(None, 0.03, 0.12) == 0.0 and stress(math.nan, 0.03, 0.12) == 0.0


def test_healthy_signals_stay_reliable():
    assert run(HealthMonitor(), 20.0, **GOOD) == RELIABLE


def test_warmup_holds_the_state_reliable_even_with_bad_signals():
    mon = HealthMonitor()
    assert run(mon, P.warmup - 0.5, fit=1.0, R=0.0, gyro=1.0) == RELIABLE


def test_bad_signals_escalate_to_critical_and_good_ones_recover():
    mon = HealthMonitor()
    run(mon, 5.0, **GOOD)
    assert run(mon, 8.0, fit=0.5, R=0.1, gyro=0.5) == CRITICAL
    assert run(mon, 15.0, **GOOD) == RELIABLE


def test_each_signal_alone_can_raise_the_state():
    for bad in (dict(fit=0.5), dict(R=0.1), dict(gyro=0.5)):
        mon = HealthMonitor()
        run(mon, 5.0, **GOOD)
        assert run(mon, 8.0, **{**GOOD, **bad}) == CRITICAL, bad


def test_a_short_spike_does_not_change_the_state():
    mon = HealthMonitor()
    run(mon, 5.0, **GOOD)
    mon.update(DT, fit=0.5, R=0.1, gyro=0.5)               # one scan
    assert run(mon, 5.0, **GOOD) == RELIABLE


def test_hysteresis_stepping_down_needs_down_time():
    mon = HealthMonitor()
    run(mon, 5.0, **GOOD)
    run(mon, 8.0, fit=0.5, R=0.1, gyro=0.5)
    assert mon.state == CRITICAL
    mon.update(DT, **GOOD)                                 # smoothed stress is still high
    assert mon.state == CRITICAL
    states = []
    for _ in range(round(15.0 / DT)):
        states.append(mon.update(DT, **GOOD))
    first_non_critical = states.index(next(s for s in states if s != CRITICAL))
    assert first_non_critical * DT >= P.down_time          # it waited at least down_time before stepping down


def test_missing_signals_count_as_healthy():
    assert run(HealthMonitor(), 10.0, fit=None, R=None, gyro=None) == RELIABLE


def test_details_reports_state_score_and_every_stress():
    mon = HealthMonitor()
    run(mon, 5.0, **GOOD)
    d = mon.details()
    assert d['state'] == RELIABLE and set(d) == {'state', 'score', 'fit', 'R', 'gyro'}


def test_follower_speed_factor_slows_waits_then_creeps():
    from sewer_planning.pursuit import health_speed_factor as f
    assert f('RELIABLE', 0.0) == 1.0 and f('DEGRADED', 0.0) == 0.5
    assert f('CRITICAL', 0.0) == 0.0 and f('CRITICAL', 7.9) == 0.0       # waits for the conditions to pass
    assert f('CRITICAL', 8.0) == pytest.approx(0.3)                       # timeout: creep instead of waiting forever
