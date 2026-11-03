#!/usr/bin/env python3
"""Observe a live mission in Gazebo: blockage detection, replanning, driving, clearance. Controls nothing.

Start robot_sim.launch.py, localization.launch.py and then mission.launch.py; this script only listens.
Ground truth (/ground_truth/odom, blockages_truth.yaml) is used for EVALUATION ONLY: where the robot really
is, and how close it comes to walls and the real debris.

Usage: mission_check.py [timeout_sim_s]
"""
import json
import math
import sys
import time

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32, String

from sewer_localization.map_utils import OccupancyMap

GZ = '/home/jeevi/sewer_ws/src/sewer_gazebo'
ROBOT_RADIUS = 0.20     # circumscribed radius incl. wheels
NETWORK = sys.argv[3] if len(sys.argv) > 3 else 'sewer_network'    # sewer_network | sewer_network_v2
SFX = NETWORK[len('sewer_network'):]


def true_map():
    m = OccupancyMap.load(GZ + f'/maps/sewer_map{SFX}')
    with open(GZ + f'/config/blockages_truth{SFX}.yaml') as f:
        for b in yaml.safe_load(f)['blockages'].values():
            for p in b['pieces']:
                m.add_box(p[1], p[2], p[3], p[4]) if p[0] == 'box' else m.add_circle(p[1], p[2], p[3])
    return m


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Watch(Node):
    def __init__(self):
        super().__init__('mission_check', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.truth = true_map()
        self.route_events = []        # (t, nodes, total, R)
        self.blocked_first = {}       # edge -> (t, gt x, gt y)
        self.gaps = {}
        self.min_clear, self.min_clear_at = 9.0, None
        self.gt = None
        self.status = 'WAITING'
        self.t_move = None
        self.gt_traj = []
        self.est = (float('nan'), float('nan'))
        self.est_yaw = float('nan')
        self.cmd = (float('nan'), float('nan'))
        self.create_subscription(Twist, '/cmd_vel', lambda m: setattr(self, 'cmd', (m.linear.x, m.angular.z)), 10)
        self.R = float('nan')
        self.health = 'RELIABLE'
        self.collisions = dict(total=0, wall=0, debris=0, other=0, contact_time=0.0, events=[])
        self.create_subscription(String, '/collisions', lambda m: setattr(self, 'collisions', json.loads(m.data)), 10)
        self.health_log = []          # (t, state) at every change
        self.create_subscription(String, '/reliability/localization_health', self.on_health, 10)
        self.bl = (float('nan'), float('nan'))      # baseline filter estimate (if /baseline/* runs)
        self.dr = (float('nan'), float('nan'))      # dead reckoning from /odom_disturbed (if the odometry is disturbed)
        self.create_subscription(PoseWithCovarianceStamped, '/baseline/estimated_pose',
                                 lambda m: setattr(self, 'bl', (m.pose.pose.position.x, m.pose.pose.position.y)), 10)
        self.create_subscription(Odometry, '/odom_disturbed',
                                 lambda m: setattr(self, 'dr', (m.pose.pose.position.x, m.pose.pose.position.y)), 10)
        self.create_subscription(PoseWithCovarianceStamped, '/reliability/estimated_pose', self.on_est, 10)
        self.create_subscription(Float32, '/reliability/scan_reliability_R', lambda m: setattr(self, 'R', m.data), 10)
        self.create_subscription(String, '/route_info', self.on_info, 10)
        self.create_subscription(String, '/edge_gaps', self.on_gaps, 10)
        self.create_subscription(String, '/mission_status', lambda m: setattr(self, 'status', m.data), 10)
        self.create_subscription(Odometry, '/ground_truth/odom', self.on_gt, 10)

    def on_health(self, m):
        state = json.loads(m.data)['state']
        if state != self.health:
            self.health_log.append((round(self.t(), 1), state))
        self.health = state

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def t(self):
        return self.now() - (self.t_move if self.t_move is not None else self.now())

    def on_gt(self, m):
        p = m.pose.pose.position
        self.gt = (p.x, p.y)
        if self.t_move is None and abs(m.twist.twist.linear.x) > 0.05:
            self.t_move = self.now()
        self.gt_traj.append((self.t(), p.x, p.y, self.est[0], self.est[1], self.R,
                             self.est_yaw, yaw_of(m.pose.pose.orientation), self.cmd[0], self.cmd[1],
                             self.bl[0], self.bl[1], self.dr[0], self.dr[1], ('RELIABLE', 'DEGRADED', 'CRITICAL').index(self.health)))
        clear = float(self.truth.dist(np.array([p.x]), np.array([p.y]))[0]) - ROBOT_RADIUS
        if clear < self.min_clear:
            self.min_clear, self.min_clear_at = clear, (round(self.t(), 1), round(p.x, 2), round(p.y, 2))

    def on_est(self, m):
        self.est = (m.pose.pose.position.x, m.pose.pose.position.y)
        self.est_yaw = yaw_of(m.pose.pose.orientation)

    def on_info(self, m):
        d = json.loads(m.data)
        nodes = d.get('route')
        if not self.route_events or self.route_events[-1][1] != nodes:
            self.route_events.append((self.t(), nodes, d.get('cost', {}).get('total'), d.get('R')))

    def on_gaps(self, m):
        self.gaps = json.loads(m.data)
        for k, v in self.gaps.items():
            if v['blocked'] and k not in self.blocked_first and self.gt is not None:
                self.blocked_first[k] = (self.t(), self.gt[0], self.gt[1])


def main():
    timeout = float(sys.argv[1]) if len(sys.argv) > 1 else 220.0
    rclpy.init()
    n = Watch()
    wall0 = time.monotonic()
    t_last_status = 0
    while n.status != 'GOAL_REACHED':
        rclpy.spin_once(n, timeout_sec=0.1)
        if n.t_move is not None and n.t() > timeout:
            break
        if time.monotonic() - wall0 > 700:
            break
    for _ in range(10):
        rclpy.spin_once(n, timeout_sec=0.1)
    if len(sys.argv) > 2:
        with open(sys.argv[2], 'w') as f:
            f.write('t,gt_x,gt_y,est_x,est_y,R,est_yaw,gt_yaw,cmd_v,cmd_w,bl_x,bl_y,dr_x,dr_y,health\n')
            f.writelines(f'{t:.2f},{x:.3f},{y:.3f},{ex:.3f},{ey:.3f},{R:.3f},{eyaw:.3f},{gyaw:.3f},{cv:.3f},{cw:.3f},'
                         f'{bx:.3f},{by:.3f},{dx:.3f},{dy:.3f},{hs}\n'
                         for t, x, y, ex, ey, R, eyaw, gyaw, cv, cw, bx, by, dx, dy, hs in n.gt_traj[::2])
        with open(sys.argv[2].replace('.csv', '_events.json'), 'w') as f:
            json.dump(dict(route_events=n.route_events, blocked_first=n.blocked_first), f)
        rows = np.array(n.gt_traj, float)
        path_len = float(np.hypot(np.diff(rows[:, 1]), np.diff(rows[:, 2])).sum()) if len(rows) > 1 else float('nan')
        loop = next((e for e in reversed(n.route_events) if e[1]), None)          # the route of the last replan
        with open(sys.argv[2].replace('.csv', '_summary.json'), 'w') as f:
            json.dump(dict(status=n.status, goal_reached=n.status == 'GOAL_REACHED', mission_time=round(n.t(), 2),
                           t_move=n.t_move,
                           path_length=round(path_len, 2), collisions=n.collisions, min_clear=round(n.min_clear, 3),
                           health_log=n.health_log, blocked_first=n.blocked_first,
                           route_cost_planned=(loop[2] if loop else (n.route_events[0][2] if n.route_events else None)),
                           route_nodes=(loop[1] if loop else (n.route_events[-1][1] if n.route_events else None)),
                           final_gaps={k: v['gap'] for k, v in n.gaps.items()}), f)
    fails = 0

    def check(name, ok, detail):
        nonlocal fails
        print(f"{'PASS' if ok else 'FAIL'}  {name}: {detail}", flush=True)
        fails += 0 if ok else 1

    print('-- route timeline (sim s since the robot started moving): t, route, total cost, smoothed R')
    for t, nodes, tot, R in n.route_events:
        print(f'   {t:6.1f}  {"-".join(nodes) if nodes else "NO ROUTE":22s} cost {tot}  R {R}')
    gx, gy = n.gt if n.gt else (float('nan'), float('nan'))
    print(f'-- end: status {n.status}, gt ({gx:.2f},{gy:.2f}), sim time {n.t():.0f} s, min clearance to walls/debris '
          f'{n.min_clear:.2f} m at {n.min_clear_at}')
    rows = np.array(n.gt_traj, float)
    if len(rows):
        def err_stats(ix, iy):
            e = np.hypot(rows[:, ix] - rows[:, 1], rows[:, iy] - rows[:, 2])
            e = e[np.isfinite(e)]
            return f'rmse {np.sqrt(np.mean(e ** 2)):.3f} max {e.max():.3f} final {e[-1]:.3f} m' if len(e) else 'n/a'
        print(f'-- position error vs ground truth: dead reckoning ({err_stats(12, 13)}) | baseline filter ({err_stats(10, 11)}) '
              f'| reliability-aware filter ({err_stats(3, 4)})')
    print('-- blocked edges first flagged (t, robot gt x, y):', {k: tuple(round(v, 1) for v in val) for k, val in n.blocked_first.items()})
    with open(f'{GZ}/config/blockages_truth{SFX}.yaml') as f:
        truth = yaml.safe_load(f)['blockages']
    with open(f'{GZ}/config/network_graph{SFX}.yaml') as f:
        graph = yaml.safe_load(f)
    blocking = {'-'.join(b['edge']): name for name, b in truth.items() if b['blocks_route']}
    partial = {'-'.join(b['edge']): name for name, b in truth.items() if not b['blocks_route']}
    clusters = {k: (round(v['gap'], 2), round(v['s_min'], 1), round(v['s_max'], 1), v['cells'])
                for k, v in n.gaps.items() if v['s_min'] is not None}
    print(f'-- network {NETWORK}: real blocking debris on {sorted(blocking)}, passable debris on {sorted(partial)}')
    print('-- obstruction clusters at the end (edge: gap, s_min, s_max, cells):', clusters)
    print('-- final gaps (m):', {k: round(v['gap'], 2) for k, v in n.gaps.items() if k in blocking or k in partial})

    if len(rows):
        dt_rows = np.diff(rows[:, 0], append=rows[-1, 0])
        share = [float(dt_rows[rows[:, 14] == k].sum()) for k in range(3)]
        print(f'-- localization health: time RELIABLE {share[0]:.0f} s, DEGRADED {share[1]:.0f} s, CRITICAL {share[2]:.0f} s; '
              f'changes (t, state): {n.health_log}')

    gx_goal, gy_goal = graph['nodes'][graph['goal']]['x'], graph['nodes'][graph['goal']]['y']
    edges_of = lambda nodes: {f'{u}-{v}' for u, v in zip(nodes, nodes[1:])} | {f'{v}-{u}' for u, v in zip(nodes, nodes[1:])}  # noqa: E731
    first = next((nodes for _, nodes, _, _ in n.route_events if nodes), None)
    check('mission completed', n.status == 'GOAL_REACHED' and math.hypot(gx - gx_goal, gy - gy_goal) < 0.5,
          f"{n.status}, gt ({gx:.2f},{gy:.2f}) vs goal {graph['goal']} ({gx_goal:g},{gy_goal:g})")
    # the observer may start a moment after the robot leaves A, so the first route seen may begin at J1
    check('started on a route through the (still unknown) blocking debris', bool(first) and bool(edges_of(first) & set(blocking)),
          f'{first}')
    for edge, name in sorted(blocking.items()):
        pcs = truth[name]['pieces']
        cx, cy = float(np.mean([p[1] for p in pcs])), float(np.mean([p[2] for p in pcs]))
        det = n.blocked_first.get(edge)
        check(f'blockage {name} (edge {edge}) detected', det is not None, f'{det}')
        if det is None:
            continue
        check(f'{name} detected from at least 2 m away', math.hypot(det[1] - cx, det[2] - cy) >= 2.0,
              f'robot was {math.hypot(det[1] - cx, det[2] - cy):.1f} m from it')
        new = next(((t, nodes) for t, nodes, _, _ in n.route_events if t >= det[0] and nodes and edge not in edges_of(nodes)), None)
        check(f'replanned around {name} within 3 s of the detection', new is not None and new[0] - det[0] < 3.0,
              f'{new[0] - det[0]:.1f} s after detection, new route {new[1]}' if new else 'never left the blocked edge')
    check('no phantom blockage (only the real blocking edges flagged)', set(n.blocked_first) <= set(blocking),
          f'{sorted(n.blocked_first)}')
    for edge, name in sorted(partial.items()):
        g = n.gaps.get(edge, {})
        check(f'partial obstruction {name} ({edge}) seen as passable and narrow',
              0.5 <= g.get('gap', 9) < 0.8 and not g.get('blocked', True), f"gap {g.get('gap')}")
    c = n.collisions
    check('no collision (Gazebo contact sensors: walls, debris)', c['total'] == 0,
          f"{c['total']} counted (walls {c['wall']}, debris {c['debris']}, other {c['other']}), contact time {c['contact_time']} s, "
          f"events {c['events']}; secondary: ground-truth clearance {n.min_clear:.2f} m")
    print(f'RESULT  {"ALL PASS" if fails == 0 else str(fails) + " FAILED"}', flush=True)
    rclpy.shutdown()
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
