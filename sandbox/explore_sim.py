"""Closed-loop 2D exploration run: no ROS, no Gazebo.

CUSTOM code (ours). The robot is driven by the same pieces the ROS stack uses, in the same loop:
  true unicycle + noisy wheel odometry/gyro (shared OdometryErrorModel) + LiDAR ray-cast on the TRUE world
  -> ReliabilityAwareFilter (prior map only)  -> estimated pose
  -> EvidenceGrid blockage detector fed with the ESTIMATED pose -> per-edge status
  -> Explorer (frontier choice + coverage)     -> waypoints
  -> pure pursuit (pursuit.py, same command law as route_follower_node) -> wheel commands.
Ground truth (true pose, true world, blockages_truth*.yaml) is only used to simulate the sensors and, afterwards, by
`truth_coverage` to score the run. The algorithms never see it.

  python3 sandbox/explore_sim.py --network sewer_network_v2 --seed 0
"""
import argparse
import math
import os
import sys
import time

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(__file__))
import sim2d  # noqa: E402  (puts src/* on sys.path)
from sewer_localization.disturbances import OdometryErrorModel  # noqa: E402
from sewer_localization.map_utils import OccupancyMap  # noqa: E402
from sewer_localization.particle_filter import PFParams, ReliabilityAwareFilter, fuse_heading  # noqa: E402
from sewer_perception.blockage import EvidenceGrid  # noqa: E402
from sewer_planning.exploration import COMPLETE, INCOMPLETE, Explorer  # noqa: E402
from sewer_planning.graph import Graph  # noqa: E402
from sewer_planning.pursuit import pursuit_target  # noqa: E402
from sewer_planning.route_cost import RouteModel  # noqa: E402

DT, ANGLES = sim2d.DT, sim2d.ANGLES
ROBOT_HALF_W = 0.15            # wheel track 0.26 m + tyre: closer than this to a wall is a scrape


def load_network(network='sewer_network_v2', world='blocked'):
    """(prior map, TRUE map, graph, truth dict). The true map adds the unmapped debris (world='blocked')."""
    sfx = network[len('sewer_network'):]
    prior = OccupancyMap.load(os.path.join(sim2d.GAZEBO_PKG, 'maps', 'sewer_map' + sfx))
    graph = Graph.load(os.path.join(sim2d.GAZEBO_PKG, 'config', f'network_graph{sfx}.yaml'))
    with open(os.path.join(sim2d.GAZEBO_PKG, 'config', f'blockages_truth{sfx}.yaml')) as f:
        truth = yaml.safe_load(f)
    true = prior.copy()
    if world == 'blocked':
        for b in truth['blockages'].values():
            for p in b['pieces']:
                if p[0] == 'box':
                    true.add_box(p[1], p[2], p[3], p[4])
                else:
                    true.add_circle(p[1], p[2], p[3])
    else:
        truth = dict(truth, blockages={})
    return prior, true, graph, truth


def densify(pts, spacing=0.2):
    """Same densification as RoutePlannerNode.fill_path (the follower drives a dense polyline)."""
    dense = [np.asarray(pts[0], float)]
    for a, b in zip(pts[:-1], pts[1:]):
        a, b = np.asarray(a, float), np.asarray(b, float)
        n = max(1, int(math.ceil(np.linalg.norm(b - a) / spacing)))
        dense += [a + (b - a) * (k / n) for k in range(1, n + 1)]
    return np.array(dense)


class SimRobot:
    """One simulated robot: TRUE unicycle + sensors, and the robot's own localizer and blockage detector.

    The TRUE state (x, y, th) and the true map are only used to simulate the sensors (and by evaluators). Everything
    the robot decides with comes from `estimate()` and `edge_status()`.
    """

    def __init__(self, prior, true, graph, seed=0, pf_params=None):
        self.prior, self.true, self.g = prior, true, graph
        self.rng = np.random.default_rng(seed)
        self.x, self.y = graph.nodes[graph.start]
        self.th = self.v = self.w = 0.0
        self.pfp = pf_params or PFParams()
        self.f = ReliabilityAwareFilter(prior, self.pfp, np.random.default_rng(seed + 1000),
                                        (self.x, self.y, self.th), ANGLES)
        self.grid = EvidenceGrid(prior)
        self.odo = OdometryErrorModel(self.rng, ())   # per-run scale errors + gyro bias walk, no slip zones
        self.travelled, self.t = 0.0, 0.0
        self.log = dict(true=[], est=[], clearance=[])

    def estimate(self):
        return self.f.estimate()

    def edge_status(self):
        g = self.g
        return {Graph.key(e): self.grid.edge_status(g.nodes[e[0]], g.nodes[e[1]]) for e in g.edges}

    def command(self, path, vmax=0.35):
        """route_follower_node's command law (pure pursuit) on the ESTIMATED pose. path None/empty = stop."""
        xe, ye, the, _ = self.f.estimate()
        if path is None or len(path) == 0:
            return 0.0, 0.0
        goal = path[-1]
        dist_goal = math.hypot(goal[0] - xe, goal[1] - ye)
        if dist_goal < 0.2:
            return 0.0, 0.0
        tgt = pursuit_target(path, xe, ye, 0.5)
        alpha = sim2d.wrap(math.atan2(tgt[1] - ye, tgt[0] - xe) - the)
        if abs(alpha) > 0.6:
            return 0.0, max(-1.0, min(1.0, 1.5 * alpha))
        return min(vmax * max(0.3, 1.0 - abs(alpha)), 0.5 * dist_goal + 0.1), max(-1.0, min(1.0, 2.0 * alpha))

    def step(self, v_cmd, w_cmd):
        """Advance one LiDAR period: TRUE kinematics, odometry, scan, localization, blockage evidence."""
        x, y, th = self.x, self.y, self.th
        self.v += max(-1.0 * DT, min(1.0 * DT, v_cmd - self.v))     # accel limits as in sim2d.trajectory
        self.w += max(-2.0 * DT, min(2.0 * DT, w_cmd - self.w))
        v, w = self.v, self.w
        nx, ny = x + v * DT * math.cos(th + 0.5 * w * DT), y + v * DT * math.sin(th + 0.5 * w * DT)
        if self.true.dist(np.array([nx]), np.array([ny]))[0] < 0.05:   # hard wall: the body cannot enter it
            nx, ny, self.v = x, y, 0.0
        ds, dth = math.hypot(nx - x, ny - y) * (1 if v >= 0 else -1), w * DT
        self.x, self.y, self.th = nx, ny, sim2d.wrap(th + dth)
        ds_m, dthw_m, dthg_m = self.odo.step(ds, dth, DT, self.travelled)
        self.travelled += abs(ds)
        x, y, th = self.x, self.y, self.th
        z = self.true.raycast(x + sim2d.LIDAR_X * math.cos(th), y + sim2d.LIDAR_X * math.sin(th), th + ANGLES,
                              sim2d.RMAX, sim2d.RMIN)
        fin = np.isfinite(z)
        z[fin] += self.rng.normal(0, sim2d.LIDAR_SIGMA, fin.sum())
        z[z >= sim2d.RMAX] = np.inf
        self.f.predict(ds_m, fuse_heading(dthw_m, dthg_m, self.pfp.gyro_weight))
        self.f.update(z, ANGLES)
        xe, ye, the, _ = self.f.estimate()
        self.grid.update((xe, ye, the), z, ANGLES, self.f.r)
        self.t += DT
        self.log['true'].append((x, y, th))
        self.log['est'].append((xe, ye, the))
        self.log['clearance'].append(float(self.true.dist(np.array([x]), np.array([y]))[0]))

    def metrics(self):
        true_xy, est_xy = np.array(self.log['true'])[:, :2], np.array(self.log['est'])[:, :2]
        return dict(true=np.array(self.log['true']), est=np.array(self.log['est']), sim_time=len(self.log['true']) * DT,
                    min_clearance=min(self.log['clearance']) if self.log['clearance'] else float('inf'),
                    pos_rmse=float(np.sqrt(np.mean(np.sum((true_xy - est_xy) ** 2, axis=1)))))


def run(seed=0, network='sewer_network_v2', world='blocked', t_max=900.0, vmax=0.35, verbose=False):
    prior, true, graph, truth = load_network(network, world)
    robot = SimRobot(prior, true, graph, seed)
    ex = Explorer(graph, RouteModel(graph, prior))
    path, res, states = None, None, []
    t0 = time.time()
    for k in range(int(t_max / DT)):
        if k % 5 == 0:                         # explorer at 2 Hz, like the ROS node
            xe, ye, _, cov = robot.estimate()
            res = ex.step((xe, ye), cov[:2, :2], robot.f.R, robot.edge_status())
            if verbose and (k % 100 == 0 or res['state'] != (states[-1][1] if states else None)):
                print(f't={k * DT:6.1f} {res["state"]:10s} cov={ex.tracker.summary()["coverage"]:.3f} '
                      f'target={res["target"]} frontiers={res["frontiers"]}')
            states.append((k * DT, res['state']))
            if res['state'] in (COMPLETE, INCOMPLETE):
                break
            path = densify(res['waypoints']) if res['waypoints'] else None
        robot.step(*robot.command(path, vmax))
    status = robot.edge_status()
    return dict(result=res, summary=ex.tracker.summary(), graph=graph, truth=truth, status=status, states=states,
                wall_time=time.time() - t0, blocked=sorted(k for k, s in status.items() if s['blocked']),
                **robot.metrics())


def truth_coverage(graph, truth, true_xy, bin_len=0.5, sense=0.6, debris_margin=0.9):
    """EVALUATION ONLY. Fraction of the truly accessible corridor the robot's TRUE path came within `sense` of.

    Truly blocked corridors are the truth entries with blocks_route; their debris extent along the edge comes from the
    truth pieces. Bins within `debris_margin` of debris are excluded (the robot is meant to stop short of it).
    Returns (fraction, missed list of 'edge@s').
    """
    blk = {}
    for b in truth.get('blockages', {}).values():
        e = tuple(b['edge'])
        if e not in graph.edges:
            e = (e[1], e[0])
        A, a = np.array(graph.nodes[e[0]]), graph.direction(e)
        ss = []
        for p in b['pieces']:
            c = np.array(p[1:3])
            half = max(p[3], p[4]) / 2 if p[0] == 'box' else p[3]
            s = float((c - A) @ a)
            ss += [s - half, s + half]
        lo, hi = min(ss), max(ss)
        if e in blk:
            lo, hi = min(lo, blk[e][1]), max(hi, blk[e][2])
        blk[e] = (bool(b.get('blocks_route')), lo, hi)
    reach, todo = {graph.start}, [graph.start]
    while todo:
        u = todo.pop()
        for vv in graph.neighbours(u):
            e, _ = graph.edge_between(u, vv)
            if vv not in reach and not (e in blk and blk[e][0]):
                reach.add(vv)
                todo.append(vv)
    hit = total = 0
    missed = []
    for e in graph.edges:
        L = graph.length(e)
        n = max(1, int(math.ceil(L / bin_len - 1e-9)))
        for s in (np.arange(n) + 0.5) * (L / n):
            near_debris = e in blk and blk[e][1] - debris_margin <= s <= blk[e][2] + debris_margin
            if near_debris:
                continue
            if e in blk and blk[e][0]:
                ok = (s < blk[e][1] and e[0] in reach) or (s > blk[e][2] and e[1] in reach)
            else:
                ok = e[0] in reach or e[1] in reach
            if not ok:
                continue
            total += 1
            pt = graph.point(e, s)
            if np.min(np.hypot(true_xy[:, 0] - pt[0], true_xy[:, 1] - pt[1])) <= sense:
                hit += 1
            else:
                missed.append(f'{Graph.key(e)}@{s:.2f}')
    return (hit / total if total else 0.0), missed


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--network', default='sewer_network_v2')
    ap.add_argument('--world', default='blocked', choices=['blocked', 'clear'])
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--t-max', type=float, default=900.0)
    ap.add_argument('--plot', default='', help='write a PNG of the run here')
    a = ap.parse_args()
    out = run(a.seed, a.network, a.world, a.t_max, verbose=True)
    s = out['summary']
    frac, missed = truth_coverage(out['graph'], out['truth'], out['true'][:, :2])
    print(f"final state {out['result']['state']} {out['result']['reason']!r} after {out['sim_time']:.1f} s sim "
          f"({out['wall_time']:.1f} s wall)")
    print(f"tracker coverage {s['coverage']:.3f} (confidence {s['confidence']:.3f}), explored {s['explored_m']} / "
          f"accessible {s['accessible_m']} m, blocked {s['blocked_m']} m, segments {s['segments']}")
    print(f"truth-scored coverage {frac:.3f}, missed bins {missed}")
    print(f"blocked edges detected {out['blocked']}, min wall clearance {out['min_clearance']:.3f} m, "
          f"pose RMSE {out['pos_rmse']:.3f} m")
    if a.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        prior = load_network(a.network, a.world)[0]
        fig, ax = plt.subplots(figsize=(8, 7))
        ext = (prior.x0, prior.x0 + prior.w * prior.res, prior.y0, prior.y0 + prior.h * prior.res)
        ax.imshow(prior.solid, origin='lower', extent=ext, cmap='Greys', alpha=0.6)
        ax.plot(out['true'][:, 0], out['true'][:, 1], 'b-', lw=1, label='true path')
        ax.plot(out['est'][:, 0], out['est'][:, 1], 'r--', lw=0.8, label='estimate')
        for k, n in out['graph'].nodes.items():
            ax.annotate(k, n, color='green')
        ax.set_title(f"{a.network} seed {a.seed}: {out['result']['state']}, coverage {s['coverage']:.2f}, "
                     f"truth {frac:.2f}")
        ax.legend()
        ax.set_aspect('equal')
        fig.savefig(a.plot, dpi=120)
        print('wrote', a.plot)


if __name__ == '__main__':
    main()
