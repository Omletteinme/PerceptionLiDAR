"""Tests for frontier exploration and coverage tracking (Account 1, Phase 1). No ROS, no Gazebo.

Unit tests drive the CoverageTracker / Explorer with hand-made positions and blockage statuses; the closed-loop test runs
the whole sandbox stack (particle filter -> blockage detector -> explorer -> pure pursuit) on the v2 network and scores
the result against the world truth, which only the evaluator reads.
"""
import os

import numpy as np
import pytest

import explore_sim
import sim2d
from sewer_localization.map_utils import OccupancyMap
from sewer_planning.coverage_tracker import BLOCKED, EXPLORED, UNEXPLORED, CoverageTracker
from sewer_planning.exploration import COMPLETE, EXPLORING, INCOMPLETE, RETURNING, Explorer, ExploreParams
from sewer_planning.graph import Graph
from sewer_planning.route_cost import RouteModel

GRAPH = Graph.load(os.path.join(sim2d.GAZEBO_PKG, 'config', 'network_graph_v2.yaml'))
PRIOR = OccupancyMap.load(os.path.join(sim2d.GAZEBO_PKG, 'maps', 'sewer_map_v2'))
BLOCK_J1J4 = {'J1-J4': dict(gap=0.0, blocked=True, s_min=3.55, s_max=4.4, offset=0.0, observed=0.5, cells=40)}


@pytest.fixture(scope='module')
def model():
    return RouteModel(GRAPH, PRIOR)


def drive(tracker, pts, step=0.1, sigma=0.05, trusted=True):
    """Mark coverage along a polyline of estimated positions."""
    for a, b in zip(pts[:-1], pts[1:]):
        a, b = np.asarray(a, float), np.asarray(b, float)
        n = max(1, int(np.ceil(np.linalg.norm(b - a) / step)))
        for k in range(n + 1):
            p = a + (b - a) * k / n
            tracker.update_pose(p[0], p[1], sigma, trusted)


# ------------------------------------------------------------------ coverage tracker
def test_fresh_tracker_has_no_coverage():
    s = CoverageTracker(GRAPH).summary()
    assert s['coverage'] == 0.0 and not s['complete']
    assert s['accessible_m'] == pytest.approx(s['total_known_m']) == pytest.approx(53.0)


def test_visiting_start_and_goal_is_not_full_coverage():
    """Teleporting between start and goal must not claim coverage: only the bins near those two points count."""
    t = CoverageTracker(GRAPH)
    for node in (GRAPH.start, GRAPH.goal):
        t.update_pose(*GRAPH.nodes[node])
    s = t.summary()
    assert 0.0 < s['coverage'] < 0.1 and not s['complete']


def test_driving_a_corridor_covers_exactly_that_corridor():
    t = CoverageTracker(GRAPH)
    drive(t, [GRAPH.nodes['A'], GRAPH.nodes['J1']])
    assert (t.bin_states(('A', 'J1')) == EXPLORED).all()
    assert (t.bin_states(('J1', 'J4')) == EXPLORED).sum() <= 2      # a glance into the side corridors at J1 only
    assert t.summary()['edges']['A-J1']['state'] == 'complete'


def test_untrusted_pose_marks_nothing():
    """Localization health CRITICAL -> trusted=False -> the doubtful pose must not create coverage."""
    t = CoverageTracker(GRAPH)
    drive(t, [GRAPH.nodes['A'], GRAPH.nodes['J1']], trusted=False)
    assert t.summary()['explored_m'] == 0.0


def test_low_confidence_coverage_is_reported():
    t = CoverageTracker(GRAPH)
    drive(t, [GRAPH.nodes['A'], GRAPH.nodes['J1']], sigma=0.6)
    s = t.summary()
    assert s['explored_m'] > 0 and s['confidence'] == 0.0


def test_blocked_edge_splits_into_two_stubs():
    t = CoverageTracker(GRAPH)
    t.set_status(BLOCK_J1J4)
    st = t.bin_states(('J1', 'J4'))
    c = t.centres[('J1', 'J4')]
    assert (st[(c > 3.3) & (c < 4.65)] == BLOCKED).all()
    assert (st[c < 3.2] == UNEXPLORED).all() and (st[c > 4.8] == UNEXPLORED).all()
    secs = [s for s in t.sections() if s[0] == ('J1', 'J4')]
    assert [(round(lo, 2), round(hi, 2), olo, ohi) for _, lo, hi, olo, ohi in secs] == [(0.0, 3.25, True, False),
                                                                                       (4.7, 8.0, False, True)]
    assert t.summary()['blocked_m'] > 0


def test_corridors_behind_blockages_become_unreachable():
    """Cut A-J1: nothing beyond J1 can be reached, so only A-J1's stub counts as accessible."""
    t = CoverageTracker(GRAPH)
    t.set_status({'A-J1': dict(gap=0.0, blocked=True, s_min=4.0, s_max=4.6, offset=0.0, observed=0.5, cells=30)})
    s = t.summary()
    assert s['accessible_m'] < 4.0
    assert s['segments']['unreachable'] == 9
    drive(t, [GRAPH.nodes['A'], (3.4, 0.0)])
    assert t.summary()['complete']


# ------------------------------------------------------------------ explorer decisions
def test_first_target_is_the_entrance_corridor(model):
    ex = Explorer(GRAPH, model)
    r = ex.step(GRAPH.nodes['A'], np.eye(2) * 1e-3, 0.95, {})
    assert r['state'] == EXPLORING
    assert r['target']['edge'] == 'A-J1'
    assert np.allclose(r['waypoints'][-1], GRAPH.nodes['J1'])


def test_dead_end_stub_stops_short_of_debris(model):
    """Robot at J1 with every other corridor already covered: the target is the J1-J4 stub, short of the debris."""
    ex = Explorer(GRAPH, model)
    ex.tracker.set_status(BLOCK_J1J4)
    for e, lo, hi, *_ in ex.tracker.sections():
        if e != ('J1', 'J4'):
            ex.tracker.explored[e][:] = True
    ex.tracker.explored[('J1', 'J4')][ex.tracker.centres[('J1', 'J4')] > 4.4] = True
    r = ex.step(GRAPH.nodes['J1'], None, 0.95, BLOCK_J1J4)
    assert r['state'] == EXPLORING and r['target']['edge'] == 'J1-J4'
    assert r['target']['s'] == pytest.approx(3.55 - 0.6, abs=0.01)
    assert r['frontiers'] == 1


def test_completion_returns_home_then_completes(model):
    ex = Explorer(GRAPH, model)
    for e in GRAPH.edges:
        ex.tracker.explored[e][:] = True
    r = ex.step(GRAPH.nodes['J3'], None, 0.95, {})
    assert r['state'] == RETURNING and r['reason'] == ''
    assert np.allclose(r['waypoints'][-1], GRAPH.nodes['A'])
    r = ex.step(GRAPH.nodes['A'], None, 0.95, {})
    assert r['state'] == COMPLETE and r['waypoints'] is None


def test_target_never_reachable_is_skipped_and_reported(model):
    """A target reached again and again without covering its bins must not loop forever: give up, report INCOMPLETE."""
    ex = Explorer(GRAPH, model, params=ExploreParams(patience=2))
    for e in GRAPH.edges:
        ex.tracker.explored[e][:] = True
    ex.tracker.explored[('J1', 'D')][-1] = False          # last bin of the dead end
    ex.tracker.p.sense_radius = 0.0                       # ...which the robot can never 'see'
    states = [ex.step(GRAPH.nodes['D'], None, 0.95, {}, trusted=True)['state'] for _ in range(4)]
    assert states[0] == EXPLORING and states[-1] == RETURNING
    s = ex.tracker.summary()
    assert s['skipped_m'] > 0 and not s['complete']
    r = ex.step(GRAPH.nodes['A'], None, 0.95, {})
    assert r['state'] == INCOMPLETE and 'skipped' in r['reason']


def test_hysteresis_keeps_the_current_target(model):
    ex = Explorer(GRAPH, model, params=ExploreParams(hysteresis=100.0))
    first = ex.step(GRAPH.nodes['J1'], None, 0.95, {})['target']
    again = ex.step((8.0, 0.4), None, 0.95, {})['target']      # moved a little toward J2
    assert (again['edge'], again['s']) == (first['edge'], first['s'])


# ------------------------------------------------------------------ closed loop (sandbox)
@pytest.fixture(scope='module')
def closed_loop():
    return explore_sim.run(seed=0, network='sewer_network_v2', world='blocked', t_max=600.0)


def test_closed_loop_explores_every_accessible_corridor(closed_loop):
    out = closed_loop
    s = out['summary']
    frac, missed = explore_sim.truth_coverage(out['graph'], out['truth'], out['true'][:, :2])
    print(f"\n[v2 seed 0] {out['result']['state']} in {out['sim_time']:.0f} s sim, tracker coverage {s['coverage']:.3f}, "
          f"truth-scored {frac:.3f}, blocked {out['blocked']}, min clearance {out['min_clearance']:.2f} m, "
          f"pose RMSE {out['pos_rmse']:.3f} m")
    assert out['result']['state'] == COMPLETE
    assert s['complete'] and s['coverage'] == 1.0
    assert frac >= 0.98, missed                          # scored against the world truth (evaluator only)
    assert out['blocked'] == ['J1-J4', 'M-J3']           # both full blockages of v2 found, no phantom ones


def test_closed_loop_returns_to_entrance_without_scraping(closed_loop):
    out = closed_loop
    end = out['true'][-1, :2]
    assert np.hypot(*(end - np.array(out['graph'].nodes['A']))) < 0.5
    assert out['min_clearance'] > explore_sim.ROBOT_HALF_W


# ------------------------------------------------------------------ Phase 5 regressions (disturbed sandbox runs)
B2_PHANTOM = {'J1-J2': dict(gap=0.3, blocked=True, s_min=2.8, s_max=3.6, offset=-0.2, observed=0.6, cells=30)}


def test_blockage_the_robot_drove_through_is_passable(model):
    """Heavy disturbance turned passable debris B2 into a 'full blockage' after the robot had driven past it."""
    ex = Explorer(GRAPH, model)
    for y in np.arange(0.5, 5.6, 0.1):                       # drove J1 -> J2 along the corridor
        ex.observe((8.0, y))
    eff = ex.effective_status(B2_PHANTOM)
    assert not eff['J1-J2']['blocked'] and eff['J1-J2']['traversed']
    ex2 = Explorer(GRAPH, model)
    for y in np.arange(0.5, 2.5, 0.1):                       # only reached the debris, never passed it
        ex2.observe((8.0, y))
    assert ex2.effective_status(B2_PHANTOM)['J1-J2']['blocked']
    ex3 = Explorer(GRAPH, model)
    for y in np.arange(0.5, 5.6, 0.1):
        ex3.observe((8.0, y), trusted=False)                 # doubtful pose: no evidence of passage
    assert ex3.effective_status(B2_PHANTOM)['J1-J2']['blocked']


def test_robot_cut_off_behind_a_blockage_still_counts_its_surroundings(model):
    """Accessibility from the entrance only made everything around a cut-off robot 'unreachable' (coverage 18/18 m)."""
    status = dict(B2_PHANTOM, **{'J1-J4': BLOCK_J1J4['J1-J4']})
    t = CoverageTracker(GRAPH)
    t.set_status(status)
    from_entrance = t.summary()['accessible_m']
    t.set_status(status, seeds=['J2'])
    assert t.summary()['accessible_m'] > from_entrance + 20


def test_no_route_home_is_not_complete(model):
    ex = Explorer(GRAPH, model)
    for e in GRAPH.edges:
        ex.tracker.explored[e][:] = True
    status = dict(B2_PHANTOM, **{'J1-J4': BLOCK_J1J4['J1-J4']})
    r = ex.step(GRAPH.nodes['M'], None, 0.95, status)       # robot never drove J1-J2 here: the cut is unexplained
    assert r['state'] == 'NO_ROUTE_HOME' and r['waypoints'] is None and 'entrance' in r['reason']
