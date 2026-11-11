"""Closed-loop 2D mission run: explore, pick up the best feasible debris, dispose of it at the entrance, resume.

CUSTOM code (ours). Same robot loop as explore_sim.SimRobot (particle filter, blockage detector, pure pursuit on the
ESTIMATED pose); the MissionManager decides everything. Perception and the arm are the SANDBOX STAND-INS in
mission_standins.py (test doubles for Account 2's work); every message between them and the Mission Manager goes
through the JSON parsers of sewer_planning.mission_interfaces, exactly as the ROS node will.

  python3 sandbox/mission_sim.py --seed 0 [--fail S3=1] [--no-manipulator]
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
import explore_sim  # noqa: E402
from mission_standins import StandInManipulator, StandInPerception, scenario_v2  # noqa: E402
from sewer_localization.particle_filter import PFParams  # noqa: E402
from sewer_planning.mission_interfaces import (parse_debris, parse_defects, parse_disposal,  # noqa: E402
                                               parse_manipulation)
from sewer_planning.mission_manager import TERMINAL, MissionManager, MissionParams  # noqa: E402
from sewer_planning.report_render import decimate, write_report  # noqa: E402
from sewer_planning.route_cost import RouteModel  # noqa: E402

DT = explore_sim.DT


def run(seed=0, network='sewer_network_v2', t_max=1500.0, fail=None, manipulator=True, verbose=False, pf_n=500,
        severity=0.0, slip=0.0):
    prior, true, graph, truth = explore_sim.load_network(network, 'blocked')
    sc = scenario_v2(truth)
    for d in sc.debris:                         # loose objects taller than the scan plane are seen by the LiDAR too
        if d.category != 'rubble_pile' and d.size[2] > 0.13:
            true.add_box(d.x, d.y, d.size[0], d.size[1])
    robot = explore_sim.SimRobot(prior, true, graph, seed, PFParams(n=pf_n), severity=severity, slip=slip)
    mm = MissionManager(graph, RouteModel(graph, prior), params=MissionParams(require_manipulator=manipulator,
                                                                              init_timeout=3.0))
    rng = np.random.default_rng(seed + 7)
    perc = StandInPerception(sc, true, rng)
    arm = StandInManipulator(sc, graph.nodes[graph.start], fail=fail)
    path, res, states, tasks, contacts = None, None, [], [], set()
    t0 = time.time()
    for k in range(int(t_max / DT)):
        now = k * DT
        arm.step(now, robot.x, robot.y, robot.th)
        if k % 5 == 0:                           # mission manager at 2 Hz
            xe, ye, _, cov = robot.estimate()
            debris, _ = parse_debris(perc.debris_json(now, robot.x, robot.y))
            defects, _ = parse_defects(perc.defects_json(now, robot.x, robot.y))
            res = mm.tick(now, (xe, ye), now, cov[:2, :2], robot.f.R, robot.health.state, robot.edge_status(), debris,
                          defects, parse_manipulation(arm.manipulation_json(now)) if manipulator else None,
                          parse_disposal(arm.disposal_json(now)) if manipulator else None)
            if res['task'] is not None:
                tasks.append(json.loads(res['task'].to_json()))
                if manipulator:
                    arm.on_task(res['task'], now)
            if not states or states[-1][1] != res['state']:
                states.append((round(now, 1), res['state']))
                if verbose:
                    print(f"t={now:6.1f} {res['state']:20s} {res['reason']}")
            if res['state'] in TERMINAL:
                break
            if res['route'] is not None:
                path = explore_sim.densify(res['route']) if len(res['route']) else None
        robot.step(*robot.command(path))
        for d in sc.debris:                       # EVALUATION ONLY: did the TRUE robot run into loose debris?
            if not d.held and not d.disposed and d.category != 'rubble_pile' and                     math.hypot(d.x - robot.x, d.y - robot.y) < 0.5 * math.hypot(d.size[0], d.size[1]) + 0.15:
                contacts.add(d.id)
    rep = mm.report(k * DT)
    return dict(result=res, report=rep, states=states, tasks=tasks, scenario=sc, graph=graph, truth=truth,
                debris_contacts=sorted(contacts),
                wall_time=time.time() - t0, **robot.metrics())


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--t-max', type=float, default=1500.0)
    ap.add_argument('--fail', default='', help='e.g. S3=1,S1=2: grasp attempts that fail per object')
    ap.add_argument('--no-manipulator', action='store_true')
    ap.add_argument('--severity', type=float, default=0.0, help='scan disturbance severity (0 = clean)')
    ap.add_argument('--slip', type=float, default=0.0, help='wheel-slip severity (0 = none)')
    ap.add_argument('--report', default='', help='write the final mission report JSON here')
    ap.add_argument('--report-dir', default='', help='write report.json, report.md and map.png into this folder')
    a = ap.parse_args()
    fail = {kv.split('=')[0]: int(kv.split('=')[1]) for kv in a.fail.split(',') if kv}
    out = run(a.seed, t_max=a.t_max, fail=fail, manipulator=not a.no_manipulator, verbose=True, severity=a.severity,
              slip=a.slip)
    r = out['report']
    print(f"\nfinal {r['state']} ({r['reason']}) after {out['sim_time']:.0f} s sim, {out['wall_time']:.0f} s wall")
    print('coverage', r['coverage']['coverage'], 'counts', r['counts'])
    print('removed', r['debris']['removed'])
    print('failed', r['debris']['failed'])
    print('rejected', r['debris']['rejected'])
    print('defects', [(d['id'], d['type'], d['severity']) for d in r['defects']])
    print(f"min wall clearance {out['min_clearance']:.3f} m, pose RMSE {out['pos_rmse']:.3f} m "
          f"(max {out['max_pos_err']:.3f}), health time {out['health_time']}")
    print('true disposal:', {d.id: d.disposed for d in out['scenario'].debris})
    print('debris the TRUE robot ran into:', out['debris_contacts'], '| detoured', r['small_debris'])
    if a.report:
        with open(a.report, 'w') as f:
            json.dump(r, f, indent=1, default=str)
    if a.report_dir:
        prior = explore_sim.load_network('sewer_network_v2', 'blocked')[0]
        edges = {k: v['state'] for k, v in r['coverage']['edges'].items()}
        print('wrote', write_report(r, a.report_dir, out['graph'], edges, prior, decimate(out['est'][:, :2])))


if __name__ == '__main__':
    main()
