"""Tests for blockage detection and the uncertainty-aware route cost (no ROS, no Gazebo)."""
import math
import os

import numpy as np
import pytest

import sim2d
from sewer_localization.particle_filter import PFParams, ReliabilityAwareFilter, fuse_heading
from sewer_perception.blockage import EvidenceGrid
from sewer_planning.graph import Graph
from sewer_planning.route_cost import CostParams, RouteModel

A = sim2d.ANGLES
GRAPH = Graph.load(os.path.join(sim2d.GAZEBO_PKG, 'config', 'network_graph.yaml'))
APPROACH = [(4.0, 0.0), (8.0, 0.0), (10.5, 0.0)]   # drive toward debris B1 and stop 1 m short of it
LOOP_PREFIX = sim2d.ROUTE                          # the long loop, past debris B2


def run_pipeline(rec, seed):
    """Reliability-aware localizer -> evidence grids fed with the ESTIMATED pose.

    Returns (g_default, g_plain): g_default is the shipped detector (only beams with reliability r >= 0.5 count);
    g_plain is the ablation where every valid beam counts. The live A/B (Day 11) showed the ablation fails under
    disturbance (phantom blockages, real debris erased), which the sandbox alone did not reveal.
    """
    prior, _ = sim2d.load_world('blocked')
    f = ReliabilityAwareFilter(prior, PFParams(), np.random.default_rng(seed + 1000), (0.0, 0.0, 0.0), A)
    g_default, g_plain = EvidenceGrid(prior), EvidenceGrid(prior)
    dth = fuse_heading(rec.dth_wheel, rec.dth_gyro, PFParams().gyro_weight)
    for k in range(len(rec.t)):
        f.predict(rec.ds[k], dth[k])
        f.update(rec.scans[k], A)
        x, y, th, _ = f.estimate()
        g_default.update((x, y, th), rec.scans[k], A, f.r)
        g_plain.update((x, y, th), rec.scans[k], A, None)
    return g_default, g_plain


def statuses(grid):
    return {GRAPH.key(e): grid.edge_status(GRAPH.nodes[e[0]], GRAPH.nodes[e[1]]) for e in GRAPH.edges}


def blocked(st):
    return sorted(k for k, v in st.items() if v['blocked'])


# ------------------------------------------------------------------ blockage detection
@pytest.fixture(scope='module')
def approach_clean():
    return statuses(run_pipeline(sim2d.simulate(0, 'clean', route=APPROACH), 0)[0])


def test_full_blockage_detected_on_approach(approach_clean):
    st = approach_clean
    assert blocked(st) == ['J1-J4']
    assert st['J1-J4']['gap'] < 0.3 and st['J1-J4']['s_min'] is not None


def test_full_blockage_detected_under_disturbances():
    st = statuses(run_pipeline(sim2d.simulate(1, 'disturbed', severity=3.0, route=APPROACH), 1)[0])
    assert blocked(st) == ['J1-J4']


def test_partial_obstruction_is_passable_with_offset():
    st = statuses(run_pipeline(sim2d.simulate(0, 'clean', route=LOOP_PREFIX), 0)[0])
    b2 = st['J1-J2']
    assert not b2['blocked']
    assert 0.5 <= b2['gap'] <= 0.75                # truth: 0.60 m free on the east side
    assert abs(b2['offset'] - (-0.2)) < 0.1        # drive 0.2 m off-centre toward +x (d is left of J1->J2)
    assert 'J1-J4' in blocked(st)


@pytest.mark.parametrize('seed', [0, 1])
def test_no_phantom_blockages_in_empty_network(seed):
    """Clear world, strong disturbances: the shipped (reliability-gated) detector must stay silent."""
    rec = sim2d.simulate(seed, 'disturbed', world='clear', severity=4.0, route=APPROACH)
    g_default, g_plain = run_pipeline(rec, seed)
    print(f'\n[seed {seed}] phantom blocked edges: default {blocked(statuses(g_default))}, '
          f'every-beam ablation {blocked(statuses(g_plain))}; obstacle cells {int((g_default.L >= 2).sum())} vs {int((g_plain.L >= 2).sum())}')
    assert blocked(statuses(g_default)) == []


# ------------------------------------------------------------------ route cost
@pytest.fixture(scope='module')
def model():
    prior, _ = sim2d.load_world('clear')
    return RouteModel(GRAPH, prior, CostParams())


def best(model, loc, status, R, goal='J4'):
    P0 = model.initial_covariance()
    return model.plan(loc, status, R, P0, goal)


def test_graph_locate():
    assert GRAPH.locate(8.1, 0.1) == ('node', 'J1')
    kind, edge, s, d = GRAPH.locate(8.0, 3.0)
    assert kind == 'edge' and edge == ('J1', 'J2') and abs(s - 3.0) < 1e-6 and abs(d) < 1e-6


def test_locate_picks_the_corridor_the_robot_is_in_beside_a_junction():
    """Regression (Day 9 live runs): ties in lateral offset beside J1 / M made the planner think the robot was
    on the dead-end spur or behind a node, so it turned back toward the node it had just left."""
    kind, edge, s, d = GRAPH.locate(7.8, 0.48)             # just north of J1, in the J1-J2 corridor
    assert (kind, edge) == ('edge', ('J1', 'J2')) and abs(s - 0.48) < 1e-6
    kind, edge, s, d = GRAPH.locate(12.4, 6.0)             # just east of the T-junction M, in M-J3
    assert (kind, edge) == ('edge', ('M', 'J3')) and abs(s - 0.4) < 1e-6


def test_route_never_doubles_back_to_the_node_just_passed(model):
    status = {'J1-J4': dict(gap=0.0, blocked=True, s_min=3.5, s_max=4.0, offset=0.0, observed=1.0, cells=50)}
    for xy, first, behind in (((7.8, 0.48), 'J2', (8.0, 0.0)), ((12.4, 6.0), 'J3', (12.0, 6.0))):
        c = best(model, GRAPH.locate(*xy), status, 0.95)[0]
        pts = model.waypoints(xy, c, status)
        assert c['nodes'][0] == first
        assert min(float(np.hypot(p[0] - behind[0], p[1] - behind[1])) for p in pts[1:]) > 0.3


def test_pursuit_aims_at_the_corner_instead_of_cutting_it():
    from sewer_planning.pursuit import pursuit_target
    a = np.column_stack([np.full(31, 8.0), np.linspace(0.0, 6.0, 31)])          # north along x = 8
    b = np.column_stack([np.linspace(8.0, 12.0, 21), np.full(21, 6.0)])[1:]     # then east along y = 6
    pts = np.vstack([a, b])
    t = pursuit_target(pts, 8.0, 5.55)                      # corner (8, 6) is 0.45 m ahead, inside the lookahead
    assert tuple(np.round(t, 3)) == (8.0, 6.0)
    t2 = pursuit_target(pts, 8.0, 6.0)                      # at the corner: carry on along the next leg
    assert t2[0] > 8.3 and abs(t2[1] - 6.0) < 1e-6
    t3 = pursuit_target(pts, 8.0, 3.0)                      # corner far away: plain lookahead along the leg
    assert tuple(np.round(t3, 3)) == (8.0, 3.5)


def test_direct_route_preferred_when_nothing_blocked(model):
    c = best(model, ('node', 'A'), {}, 0.95)
    assert c[0]['nodes'] == ['A', 'J1', 'J4']
    assert c[0]['total'] < c[1]['total']
    assert c[0]['length'] == pytest.approx(16.0, abs=0.01)


def test_blocked_edge_forces_the_loop(model):
    status = {'J1-J4': dict(gap=0.0, blocked=True, s_min=3.5, s_max=4.0, offset=0.0, observed=1.0, cells=50)}
    c = best(model, ('node', 'A'), status, 0.95)
    assert c[0]['nodes'] == ['A', 'J1', 'J2', 'M', 'J3', 'J4']
    assert len(c) == 1


def test_unreliable_sensor_raises_uncertainty_cost(model):
    hi = best(model, ('node', 'A'), {}, 0.95)[0]
    lo = best(model, ('node', 'A'), {}, 0.20)[0]
    assert hi['nodes'] == lo['nodes']
    assert lo['unc'] > hi['unc'] * 1.05 and lo['sigma_end'] > hi['sigma_end']


def test_uncertainty_grows_in_featureless_corridor_and_drops_at_a_bend(model):
    c = best(model, ('node', 'J1'), {}, 0.95)[0]           # J1 -> J4 along the straight 8 m corridor
    tr = dict(c['trace'])
    mid, end = tr[min(tr, key=lambda s: abs(s - 4.0))], tr[min(tr, key=lambda s: abs(s - 7.9))]
    start = tr[min(tr, key=lambda s: abs(s - 0.5))]
    assert mid > start                                     # drift accumulates between informative places
    assert end < mid                                       # the bend at J4 pins the position down again


def test_obstruction_waypoints_steer_into_the_free_gap(model):
    st = dict(gap=0.6, blocked=False, s_min=2.7, s_max=3.3, offset=-0.2, observed=1.0, cells=30)
    pts = model.obstruction_points(('J1', 'J2'), 0.0, 6.0, st)
    assert len(pts) == 4
    xs = [float(p[0]) for p in pts]
    assert xs[0] == pytest.approx(8.0) and xs[1] == pytest.approx(8.2) and xs[2] == pytest.approx(8.2)
    assert xs[3] == pytest.approx(8.0)
    assert [float(p[1]) for p in pts] == sorted(float(p[1]) for p in pts)   # in order of travel
