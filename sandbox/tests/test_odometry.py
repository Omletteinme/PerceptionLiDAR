"""Tests for the shared odometry/IMU error model (used by the sandbox and by the ROS odom_disturbance_node)."""
import numpy as np
import pytest

import sim2d
from sewer_localization.disturbances import OdometryErrorModel, make_slip_schedule

DT = 0.1


def drive(model, n, ds=0.04, dth=0.0, s0=0.0):
    """n steps of the true motion (ds, dth) starting s0 metres into the route; returns rows (wheel distance, wheel
    yaw, gyro yaw)."""
    return np.array([model.step(ds, dth, DT, s0 + k * abs(ds)) for k in range(n)])


def test_scale_error_is_a_drift_proportional_to_distance():
    m = OdometryErrorModel(np.random.default_rng(3))
    out = drive(m, 500)                                   # 20 m straight
    assert abs(out[:, 0].sum() - 20.0 * (1 + m.eps_s)) < 0.1


def test_slip_zone_over_reports_distance_inside_its_zone_only():
    slips = make_slip_schedule(3.0)                       # +15 % distance on route metres 10-13.5 and 21.5-25.5
    inside = drive(OdometryErrorModel(np.random.default_rng(5), slips), 80, s0=10.0)     # s = 10.0 .. 13.16 m
    before = drive(OdometryErrorModel(np.random.default_rng(5), slips), 80, s0=0.0)      # same errors, no slip
    assert inside[:, 0].sum() - before[:, 0].sum() == pytest.approx(0.15 * 0.04 * 80, abs=1e-9)
    after = drive(OdometryErrorModel(np.random.default_rng(5), slips), 80, s0=14.0)      # zone over
    assert after[:, 0].sum() == pytest.approx(before[:, 0].sum(), abs=1e-9)


def test_slip_depends_on_the_place_not_on_the_clock():
    """Driving the zone at half speed (twice the steps for the same metres) over-reports the same distance."""
    slips = make_slip_schedule(3.0)
    fast = drive(OdometryErrorModel(np.random.default_rng(5), slips), 80, ds=0.04, s0=10.0)
    slow = drive(OdometryErrorModel(np.random.default_rng(5), slips), 160, ds=0.02, s0=10.0)
    free_f = drive(OdometryErrorModel(np.random.default_rng(5), []), 80, ds=0.04)
    free_s = drive(OdometryErrorModel(np.random.default_rng(5), []), 160, ds=0.02)
    assert fast[:, 0].sum() - free_f[:, 0].sum() == pytest.approx(0.15 * 3.2, abs=0.02)
    assert slow[:, 0].sum() - free_s[:, 0].sum() == pytest.approx(0.15 * 3.2, abs=0.02)


def test_slip_under_reports_rotation_and_leaves_the_gyro_alone():
    slips = make_slip_schedule(3.0)
    turning = drive(OdometryErrorModel(np.random.default_rng(7), slips), 99, ds=0.0, dth=0.05, s0=10.0)
    free = drive(OdometryErrorModel(np.random.default_rng(7), slips), 99, ds=0.0, dth=0.05, s0=0.0)
    assert turning[:, 1].sum() - free[:, 1].sum() == pytest.approx(-0.15 * 0.05 * 99, abs=1e-9)
    assert np.array_equal(turning[:, 2], free[:, 2])      # the gyro does not slip


def test_parked_wheels_report_nothing_but_the_gyro_bias_keeps_drifting():
    m = OdometryErrorModel(np.random.default_rng(1))
    ds_w, dth_w, dth_g = m.step(0.0, 0.0, DT, None)
    assert ds_w == 0.0 and dth_w == 0.0
    assert dth_g != 0.0 and abs(dth_g) < 0.01             # bias * dt plus a little noise


def test_same_seed_gives_the_same_errors():
    a = drive(OdometryErrorModel(np.random.default_rng(11), make_slip_schedule(2.0)), 200, s0=8.0)
    b = drive(OdometryErrorModel(np.random.default_rng(11), make_slip_schedule(2.0)), 200, s0=8.0)
    assert np.array_equal(a, b)


def test_slip_schedule_scales_with_severity_and_is_capped():
    assert make_slip_schedule(0.0) == []
    s3 = make_slip_schedule(3.0)
    assert [(e['s0'], e['s1']) for e in s3] == [(10.0, 13.5), (21.5, 25.5)]
    assert s3[0]['ds_scale'] == pytest.approx(0.15) and s3[0]['dth_scale'] == pytest.approx(-0.15)
    huge = make_slip_schedule(100.0)
    assert huge[0]['ds_scale'] == pytest.approx(0.3) and huge[0]['dth_scale'] == pytest.approx(-0.3)


def test_sandbox_applies_slip_in_the_disturbed_scenario_only():
    clean = sim2d.simulate(0, 'clean')
    assert np.array_equal(clean.ds, sim2d.simulate(0, 'clean', odom_slip=True).ds)      # clean runs never slip
    slip = sim2d.simulate(0, 'disturbed', severity=3.0)
    noslip = sim2d.simulate(0, 'disturbed', severity=3.0, odom_slip=False)
    true_ds = sim2d.trajectory()[1]
    before = np.concatenate([[0.0], np.cumsum(np.abs(true_ds))[:-1]])                   # route metres before each step
    z = (before >= 10.0) & (before < 13.5)
    assert np.array_equal(slip.ds[before < 10.0], noslip.ds[before < 10.0])             # identical before the zone
    assert (slip.ds[z] - noslip.ds[z]).sum() == pytest.approx(0.15 * np.abs(true_ds[z]).sum(), abs=1e-6)  # +15 % in it
    assert np.array_equal(slip.dth_gyro, noslip.dth_gyro)                               # the gyro is unaffected
