"""Tests for the Mission Manager, task prioritization and the proposed Account 1 <-> Account 2 interfaces (Phase 3).

No ROS, no Gazebo. Unit tests feed the MissionManager scripted inputs (the robot 'teleports' to the end of each route);
the closed-loop test runs the sandbox mission with the Account-2 STAND-INS (sandbox/mission_standins.py).
"""
import json
import os

import numpy as np
import pytest

import mission_sim
import sim2d
from sewer_localization.map_utils import OccupancyMap
from sewer_planning.graph import Graph
from sewer_planning.mission_interfaces import (Debris, DisposalStatus, ManipulationStatus, MissionTask, SchemaError,
                                               parse_debris, parse_defects, parse_manipulation)
from sewer_planning.mission_manager import (DISPOSAL_FAILED, EXPLORE, INITIALIZE, LOCALIZATION_RECOVERY,
                                            MISSION_COMPLETE, NAVIGATE_TO_DEBRIS, PICKUP, PICKUP_FAILED,
                                            RETURN_TO_DISPOSAL, SAFE_STOP, MissionManager, MissionParams)
from sewer_planning.route_cost import RouteModel
from sewer_planning.task_prioritizer import PriorityParams, TaskPrioritizer, carry_status

GRAPH = Graph.load(os.path.join(sim2d.GAZEBO_PKG, 'config', 'network_graph_v2.yaml'))
PRIOR = OccupancyMap.load(os.path.join(sim2d.GAZEBO_PKG, 'maps', 'sewer_map_v2'))
A = GRAPH.nodes['A']


@pytest.fixture(scope='module')
def model():
    return RouteModel(GRAPH, PRIOR)


def deb(id_, x, y, size, conf=0.9, stamp=0.0, **kw):
    return Debris(id=id_, x=x, y=y, size=size, confidence=conf, stamp=stamp, **kw)


def blocked(s_min, s_max):
    return dict(gap=0.0, blocked=True, s_min=s_min, s_max=s_max, offset=0.0, observed=0.5, cells=40)


# ------------------------------------------------------------------ interfaces
def test_debris_message_round_trip():
    msg = json.dumps(dict(frame_id='map', stamp=5.0, objects=[
        dict(id='D1', x=1, y=2, size=[0.1, 0.08, 0.05], confidence=0.8, graspable=True),
        dict(id='D2', x=1, y=2, size=[0.1, 0.08], confidence=0.8),               # bad size
        dict(id='D3', x=1, y=2, size=[0.1, 0.1, 0.1], confidence=1.4)]))         # bad confidence
    objs, errors = parse_debris(msg)
    assert [o.id for o in objs] == ['D1'] and objs[0].stamp == 5.0 and objs[0].graspable is True
    assert len(errors) == 2


def test_detections_outside_the_map_frame_are_refused():
    with pytest.raises(SchemaError):
        parse_debris(json.dumps(dict(frame_id='camera_link', stamp=1.0, objects=[])))
    objs, errors = parse_defects(json.dumps(dict(frame_id='map', stamp=1.0, defects=[
        dict(id='X', type='crack', x=0, y=0, severity=0.5, confidence=0.9, frame_id='base_link'),
        dict(id='Y', type='rust', x=0, y=0, severity=0.5, confidence=0.9)])))
    assert objs == [] and len(errors) == 2


def test_manipulation_status_and_task_schema():
    m = parse_manipulation(json.dumps(dict(state='HOLDING', stamp=3.0, object_id='D1', verified=True, task_id='T2')))
    assert m.verified and m.task_id == 'T2'
    with pytest.raises(SchemaError):
        parse_manipulation(json.dumps(dict(state='GRABBED', stamp=3.0)))
    t = json.loads(MissionTask('T1', 'PICKUP', 1.0, 'D1', dict(x=1, y=2, frame_id='map'), 'grasp_and_lift').to_json())
    assert t['type'] == 'PICKUP' and t['target']['frame_id'] == 'map'


# ------------------------------------------------------------------ prioritization
def test_largest_feasible_not_largest(model):
    """Objects at similar distances: L1 is the biggest but too wide for the gripper, so the largest FEASIBLE one (S4)
    must win over the smaller feasible ones."""
    tp = TaskPrioritizer(model)
    debris = [deb('S1', 4.0, 0.3, (0.14, 0.10, 0.08)), deb('S4', 5.0, -0.3, (0.14, 0.14, 0.14)),
              deb('S3', 4.5, 0.3, (0.06, 0.05, 0.04)), deb('L1', 5.5, 0.3, (0.35, 0.22, 0.20))]
    ev = tp.evaluate(debris, ('node', 'A'), {}, 0.95, model.initial_covariance(), 0.0, 'A')
    assert [r['id'] for r in ev['ranked']] == ['S4', 'S1', 'S3']
    assert {r['id']: r['reason'] for r in ev['rejected']} == {'L1': 'ungraspable'}
    best = ev['ranked'][0]
    assert best['terms']['size'] == 1.0
    t, p = best['terms'], tp.p                       # the score is exactly the documented weighted sum
    assert best['score'] == pytest.approx(p.w_size * t['size'] + p.w_obstruction * t['obstruction'] +
                                          p.w_access * t['accessibility'] + p.w_grasp * t['graspability'], abs=2e-3)


def test_closer_object_can_outrank_a_slightly_larger_far_one(model):
    """The rule is a weighted trade-off, not size alone: S1 next to the robot beats the slightly larger S2 22 m away."""
    tp = TaskPrioritizer(model)
    debris = [deb('S1', 4.0, 0.3, (0.14, 0.10, 0.08)), deb('S2', 12.3, 8.0, (0.12, 0.12, 0.10))]
    ev = tp.evaluate(debris, ('node', 'A'), {}, 0.95, model.initial_covariance(), 0.0, 'A')
    assert [r['id'] for r in ev['ranked']] == ['S1', 'S2']
    s1, s2 = ev['ranked']
    assert s2['terms']['size'] == 1.0 and s1['terms']['accessibility'] > 2 * s2['terms']['accessibility']


def test_hard_filters_record_reasons(model):
    tp = TaskPrioritizer(model)
    debris = [deb('lowconf', 4.0, 0.3, (0.1, 0.1, 0.05), conf=0.2), deb('old', 4.5, 0.3, (0.1, 0.1, 0.05), stamp=-100),
              deb('nogrip', 5.0, 0.3, (0.1, 0.1, 0.05), graspable=False), deb('tall', 5.5, 0.3, (0.1, 0.1, 0.3)),
              deb('tried', 6.0, 0.3, (0.1, 0.1, 0.05)), deb('cutoff', 12.0, 3.0, (0.1, 0.1, 0.05))]
    status = {'A-J1': blocked(7.0, 7.6)}           # nothing beyond the debris in A-J1 can be reached
    ev = tp.evaluate(debris, ('node', 'A'), status, 0.95, model.initial_covariance(), 0.0, 'A',
                     attempts={'tried': 2})
    got = {r['id']: r['reason'] for r in ev['rejected']}
    assert got == {'lowconf': 'confidence', 'old': 'stale', 'nogrip': 'ungraspable', 'tall': 'ungraspable',
                   'tried': 'attempts', 'cutoff': 'unreachable'}
    assert ev['ranked'] == []


def test_object_that_cannot_be_carried_back_is_rejected(model):
    """Return route forced through a 0.35 m gap: a 0.30 m robot + clearance cannot carry anything through it."""
    tp = TaskPrioritizer(model)
    status = {'J1-J4': blocked(3.6, 4.4),
              'J1-J2': dict(gap=0.35, blocked=False, s_min=2.8, s_max=3.5, offset=-0.2, observed=0.5, cells=20)}
    ev = tp.evaluate([deb('S2', 12.3, 8.0, (0.12, 0.12, 0.10))], ('node', 'M'), status, 0.95,
                     model.initial_covariance(), 0.0, 'A')
    assert [r['reason'] for r in ev['rejected']] == ['carry']


# ------------------------------------------------------------------ state machine (scripted inputs)
class Script:
    """Drives a MissionManager with scripted inputs. The robot jumps to the end of each route it is given."""

    def __init__(self, model, **params):
        self.mm = MissionManager(GRAPH, model, params=MissionParams(**params))
        self.t, self.xy, self.health = 0.0, np.array(A, float), 'RELIABLE'
        self.debris, self.manip, self.disposal = [], ManipulationStatus('IDLE', 0.0), None
        self.tasks, self.status = [], {}

    def tick(self, n=1, dt=0.5, move=True, pose=True):
        for _ in range(n):
            self.t += dt
            if self.manip is not None:
                self.manip.stamp = self.t
            if self.disposal is not None:
                self.disposal.stamp = self.t
            for d in self.debris:
                d.stamp = self.t
            out = self.mm.tick(self.t, tuple(self.xy), self.t if pose else self.t - 10, np.eye(2) * 1e-3, 0.95,
                               self.health, self.status, list(self.debris), [], self.manip, self.disposal)
            if out['task'] is not None:
                self.tasks.append(out['task'])
            if move and out['route']:
                self.xy = np.asarray(out['route'][-1], float)
        return out


def test_initialize_waits_for_manipulator_then_falls_back_to_inspection(model):
    s = Script(model, init_timeout=3.0)
    s.manip = None
    assert s.tick()['state'] == INITIALIZE
    out = s.tick(8)
    assert out['state'] == EXPLORE and not s.mm.manipulation_enabled
    assert 'inspection only' in s.mm.events[-1]['reason']


def test_localization_recovery_and_safe_stop(model):
    s = Script(model, critical_max=5.0)
    assert s.tick()['state'] == EXPLORE
    s.health = 'CRITICAL'
    out = s.tick()
    assert out['state'] == LOCALIZATION_RECOVERY and out['route'] is None      # follower's own stop/creep applies
    s.health = 'RELIABLE'
    assert s.tick()['state'] == EXPLORE
    s.health = 'CRITICAL'
    out = s.tick(14)
    assert out['state'] == SAFE_STOP and out['route'] == [] and s.tasks[-1].type == 'STOP'


def test_stale_pose_is_a_safe_stop(model):
    s = Script(model)
    s.tick()
    out = s.tick(pose=False)
    assert out['state'] == SAFE_STOP and 'pose' in out['reason']


def test_full_cycle_with_verified_pickup_and_disposal(model):
    s = Script(model)
    s.debris = [deb('S1', 4.0, 0.3, (0.14, 0.10, 0.08))]
    s.tick()
    out = s.tick(4)
    assert out['state'] == PICKUP and s.tasks[-1].type == 'PICKUP' and s.tasks[-1].object_id == 'S1'
    tid = s.tasks[-1].task_id
    s.manip = ManipulationStatus('HOLDING', s.t, 'S1', verified=True, task_id=tid)
    s.debris = []                                    # perception no longer sees it on the floor
    out = s.tick(3)
    assert out['state'] in (RETURN_TO_DISPOSAL, 'DISPOSE')
    out = s.tick(3)
    assert out['state'] == 'DISPOSE' and s.tasks[-1].type == 'DISPOSE'
    tid = s.tasks[-1].task_id
    s.manip = ManipulationStatus('RELEASED', s.t, 'S1', task_id=tid)
    s.disposal = DisposalStatus(s.t, 'S1', station_reached=True, released=True, verified=True, task_id=tid)
    s.tick(2)
    rep = s.mm.report(s.t)
    assert [r['id'] for r in rep['debris']['removed']] == ['S1']
    assert any(e.get('event') == 'disposed' for e in rep['events'])


def test_unverified_hold_is_not_a_pickup(model):
    """HOLDING without verification evidence must not count: retry, then give up and report."""
    s = Script(model, verify_timeout=1.0, max_pickup_attempts=2)
    s.debris = [deb('S1', 4.0, 0.3, (0.14, 0.10, 0.08))]
    s.tick(5)
    assert s.mm.state == PICKUP
    for _ in range(2):
        s.manip = ManipulationStatus('HOLDING', s.t, 'S1', verified=False, task_id=s.tasks[-1].task_id)
        s.tick(5)
    rep = s.mm.report(s.t)
    assert rep['debris']['removed'] == [] and [f['id'] for f in rep['debris']['failed']] == ['S1']
    assert any(e.get('to') == PICKUP_FAILED for e in rep['events'])
    assert s.mm.state in (EXPLORE, NAVIGATE_TO_DEBRIS, MISSION_COMPLETE)


def test_failed_disposal_while_holding_stops_safely(model):
    s = Script(model, max_dispose_attempts=2, dispose_timeout=2.0)
    s.debris = [deb('S1', 4.0, 0.3, (0.14, 0.10, 0.08))]
    s.tick(5)
    s.manip = ManipulationStatus('HOLDING', s.t, 'S1', verified=True, task_id=s.tasks[-1].task_id)
    s.debris = []
    s.tick(6)
    assert s.mm.state == 'DISPOSE'
    out = s.tick(12)                                  # nothing ever confirms the disposal
    assert out['state'] == SAFE_STOP and 'still held' in out['reason']
    rep = s.mm.report(s.t)
    assert rep['debris']['removed'] == [] and any(e.get('to') == DISPOSAL_FAILED for e in rep['events'])


# ------------------------------------------------------------------ closed loop (sandbox + stand-ins)
@pytest.fixture(scope='module')
def mission():
    return mission_sim.run(seed=0, fail={'S3': 1}, pf_n=250)


def test_closed_loop_mission_removes_every_feasible_object(mission):
    rep = mission['report']
    truth = {d.id: d.disposed for d in mission['scenario'].debris}
    print(f"\n[mission v2 seed 0] {rep['state']} in {mission['sim_time']:.0f} s sim, removed "
          f"{[r['id'] for r in rep['debris']['removed']]}, counts {rep['counts']}")
    assert rep['state'] == MISSION_COMPLETE
    assert sorted(r['id'] for r in rep['debris']['removed']) == ['S1', 'S2', 'S3']
    assert sorted(k for k, v in truth.items() if v) == ['S1', 'S2', 'S3']          # the simulator agrees
    assert rep['counts']['pickup_requests'] == 4                                  # S3's first grasp failed, retried
    assert {r['id']: r['reason'] for r in rep['debris']['rejected']} == {
        'L1': 'ungraspable', 'B1': 'ungraspable', 'B2': 'ungraspable', 'B3': 'ungraspable'}


def test_closed_loop_mission_still_covers_the_network(mission):
    rep = mission['report']
    assert rep['coverage']['complete'] and rep['coverage']['coverage'] == 1.0
    assert sorted(d['type'] for d in rep['defects']) == ['crack', 'damaged_joint', 'deformation', 'hole']
    end = mission['true'][-1, :2]
    assert np.hypot(*(end - np.array(A))) < 0.5 and mission['min_clearance'] > 0.15


# ------------------------------------------------------------------ Phase 5: carrying back to the station
NARROW_J1J2 = {'J1-J2': dict(gap=0.55, blocked=False, s_min=2.8, s_max=3.5, offset=-0.2, observed=0.5, cells=20)}


def test_carry_status_blocks_only_corridors_too_narrow_to_carry_through():
    cs = carry_status(dict(NARROW_J1J2, **{'J1-J4': blocked(3.6, 4.4)}), 0.60)
    assert cs['J1-J2']['blocked'] and cs['J1-J2']['carry_blocked']
    assert cs['J1-J4']['blocked'] and 'carry_blocked' not in cs['J1-J4']
    assert not carry_status(NARROW_J1J2, 0.40)['J1-J2']['blocked']


def test_return_with_debris_avoids_a_gap_it_could_drive_through_empty(model):
    """Wide carry (0.30 m robot + 2 x 0.15 m clearance = 0.60 m): J1-J2 has a 0.55 m gap, fine empty, too narrow loaded.
    The robot may reach S2 through J2, but must carry it back the long way round (J3-J4-J1)."""
    s = Script(model)
    s.mm.prio = TaskPrioritizer(model, params=PriorityParams(carry_clearance=0.15))
    s.status = dict(NARROW_J1J2)
    s.debris = [deb('S2', 12.3, 8.0, (0.12, 0.12, 0.10))]
    s.tick()
    assert s.mm.state == NAVIGATE_TO_DEBRIS and s.mm.target['id'] == 'S2'   # reachable and carryable (via J4)
    s.tick(4)
    assert s.mm.state == PICKUP
    s.manip = ManipulationStatus('HOLDING', s.t, 'S2', verified=True, task_id=s.tasks[-1].task_id)
    s.debris = []
    s.tick(1, move=False)
    assert s.mm.state == RETURN_TO_DISPOSAL
    assert 'J4' in s.mm.carry_route and 'J2' not in s.mm.carry_route


def test_report_records_resumption_after_disposal(model):
    s = Script(model)
    s.debris = [deb('S1', 4.0, 0.3, (0.14, 0.10, 0.08))]
    s.tick(5)
    s.manip = ManipulationStatus('HOLDING', s.t, 'S1', verified=True, task_id=s.tasks[-1].task_id)
    s.debris = []
    s.tick(6)
    tid = s.tasks[-1].task_id
    s.manip = ManipulationStatus('RELEASED', s.t, 'S1', task_id=tid)
    s.disposal = DisposalStatus(s.t, 'S1', station_reached=True, released=True, verified=True, task_id=tid)
    s.tick(2)
    rep = s.mm.report(s.t)
    assert len(rep['resumptions']) == 1 and rep['resumptions'][0]['explore_state'] == 'EXPLORING'
    assert rep['resumptions'][0]['next'] is not None and rep['holding'] is None


@pytest.mark.skipif(os.environ.get('SEWER_SLOW_TESTS') != '1', reason='about 3 min; set SEWER_SLOW_TESTS=1')
def test_disturbed_mission_never_claims_a_false_removal():
    """Scan disturbance x2 + wheel slip x2: the mission may finish or stop safely, but every removal it reports must
    be real (simulator agrees) and the health response must have been exercised."""
    out = mission_sim.run(seed=0, severity=2.0, slip=2.0, pf_n=250)
    rep = out['report']
    disposed = {d.id for d in out['scenario'].debris if d.disposed}
    print(f"\n[disturbed v2 seed 0] {rep['state']} ({rep['reason']}), removed {[r['id'] for r in rep['debris']['removed']]},"
          f" health {out['health_time']}, RMSE {out['pos_rmse']:.3f}")
    assert rep['state'] in (MISSION_COMPLETE, SAFE_STOP)
    assert {r['id'] for r in rep['debris']['removed']} <= disposed
    assert out['health_time'].get('DEGRADED', 0) + out['health_time'].get('CRITICAL', 0) > 0


def test_stranded_robot_waits_then_stops_safely_never_completes(model):
    s = Script(model, stranded_timeout=5.0)
    for e in GRAPH.edges:
        s.mm.explorer.tracker.explored[e][:] = True
    s.xy = np.array(GRAPH.nodes['M'], float)
    s.status = {'J1-J2': blocked(2.8, 3.6), 'J1-J4': blocked(3.6, 4.4)}
    out = s.tick(2)
    assert out['state'] == EXPLORE and out['route'] == [] and 'waiting' in out['reason']
    out = s.tick(12)
    assert out['state'] == SAFE_STOP and 'stranded' in out['reason']
    assert MISSION_COMPLETE not in [e.get('to') for e in s.mm.events]
