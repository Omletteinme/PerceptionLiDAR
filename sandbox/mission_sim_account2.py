#!/usr/bin/env python3
"""End-to-end mission offline: Account 1's Mission Manager + Account 2's REAL perception and arm logic.

CUSTOM code (Account 2), sandbox only. Same closed loop as Account 1's sandbox/mission_sim.py (their SimRobot: particle
filter, blockage detector, pure pursuit on the ESTIMATED pose; their MissionManager decides everything), but the
Account 2 stand-ins are replaced by the code the ROS nodes run:
  perception    sewer_perception.inspection_core.InspectionCore fed with rendered depth/RGB frames (render3d.py) and
                LiDAR scans of the inspection world, at the ESTIMATED pose
  arm           sewer_manipulation.manipulation_bridge.ManipulationBridge + ManipulationController
Every message between the two sides is the documented JSON (docs/mission_interfaces.md), parsed by Account 1's own
parsers. What is still a stand-in: ArmWorld below replaces Gazebo's physics for the arm (joints follow commands at a
fixed rate; an object is caught only if its footprint lies between the open fingers (and is pushed to the middle
as they close), contact sensors report the squeezed object, a held object moves
with the fingers, a released one drops). The SimRobot drives through small debris (no 3D physics in the sandbox).

  python3 sandbox/mission_sim_account2.py [--seed 0] [--t-max 1800] [--rgb-every 40] [--no-sim-pose]
                                          [--severity 2 --slip 2]      # Account 1's disturbed setting
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import explore_sim  # noqa: E402
import render3d  # noqa: E402
import sim2d  # noqa: E402
from inspection_scoring import load_truth, score_debris, score_defects  # noqa: E402

sc = render3d.sc
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'sewer_manipulation'))
from sewer_localization.particle_filter import PFParams  # noqa: E402
from sewer_manipulation.arm_kinematics import FINGERS, JOINTS, fk  # noqa: E402
from sewer_manipulation.manipulation_bridge import ManipulationBridge  # noqa: E402
from sewer_perception import interfaces as itf  # noqa: E402
from sewer_perception.camera_model import CameraModel  # noqa: E402
from sewer_perception.inspection_core import InspectionCore  # noqa: E402
from sewer_planning.mission_interfaces import (parse_debris, parse_defects, parse_disposal,  # noqa: E402
                                               parse_manipulation)
from sewer_planning.mission_manager import TERMINAL, MissionManager, MissionParams  # noqa: E402
from sewer_planning.route_cost import RouteModel  # noqa: E402

DT = explore_sim.DT
G = sim2d.GAZEBO_PKG


class RecordingMap:
    """The robot's true LiDAR world; remembers the last ray cast so perception gets the same scan as the localizer."""

    def __init__(self, m):
        self._m, self.last = m, None

    def raycast(self, *a, **k):
        self.last = self._m.raycast(*a, **k)
        return self.last

    def __getattr__(self, name):
        return getattr(self._m, name)


class ArmWorld:
    """Kinematic stand-in for Gazebo's arm physics (see module docstring). Objects in world coordinates."""

    def __init__(self, geo, scene):
        self.geo, self.scene = geo, scene
        self.q = {j: 0.0 for j in JOINTS}
        self.q.update({f: 0.03 for f in FINGERS})
        self.obj = {}
        for s in sc.SMALL_DEBRIS:
            L, W, H = s['size']
            self.obj[f"debris_{s['id']}"] = dict(id=s['id'], x=s['x'], y=s['y'], z=H / 2, h=H, sx=L, sy=W,
                                                 yaw=s['yaw'], round=s['kind'] == 'cyl')
        self.held = None
        self.narrow = {}

    def tip_world(self, pose):
        f = fk(self.geo, [self.q[j] for j in JOINTS])
        t = f['tip']
        c, s = math.cos(pose[2]), math.sin(pose[2])
        rot = lambda v: np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])
        return pose[0] + c * t[0] - s * t[1], pose[1] + s * t[0] + c * t[1], t[2] + self.geo.base_z, \
            rot(f['fwd']), rot(f['side'])

    @staticmethod
    def half_extent(o, u):
        """Half the object's extent along the horizontal unit vector u."""
        if o['round']:
            return 0.5 * o['sx']
        a = math.atan2(u[1], u[0]) - o['yaw']
        return 0.5 * (o['sx'] * abs(math.cos(a)) + o['sy'] * abs(math.sin(a)))

    def step(self, dt, pose, cmd, now, sense):
        for j in JOINTS:
            self.q[j] += float(np.clip(cmd[j] - self.q[j], -1.5 * dt, 1.5 * dt))
        tx, ty, tz, fwd, side = self.tip_world(pose)
        g = self.geo
        between = None
        if self.held is not None:
            between = self.held                         # held: moves with the fingers
        else:
            # an object is caught when its footprint lies between the OPEN fingers (across: inside the opening; along
            # the fingers: overlapping the 7 cm pads by at least 1 cm) at the right height
            for m, o in self.obj.items():
                d = np.array([o['x'] - tx, o['y'] - ty])
                e_side, e_fwd = self.half_extent(o, side), self.half_extent(o, fwd)
                q_open = min(self.q[f] for f in FINGERS)
                if (abs(float(d @ side)) + e_side <= q_open and abs(float(d @ fwd)) < 0.5 * g.finger_len + e_fwd - 0.01
                        and abs(tz - o['z']) < 0.03 and cmd[FINGERS[0]] < q_open - 1e-3):
                    between = m
                    # closing fingers push the object to the middle of the gripper
                    o['x'], o['y'] = o['x'] - float(d @ side) * side[0], o['y'] - float(d @ side) * side[1]
                    self.narrow[m] = 2 * e_side
                    break
        for f in FINGERS:
            nq = self.q[f] + float(np.clip(cmd[f] - self.q[f], -0.1 * dt, 0.1 * dt))
            if between is not None and nq < self.narrow[between] / 2:
                nq = self.narrow[between] / 2
            self.q[f] = nq
        squeezed = between is not None and all(self.q[f] <= self.narrow[between] / 2 + 1e-3 for f in FINGERS)
        if squeezed:
            self.held = between
            for side in ('left', 'right'):
                sense(side, f'sewer_robot::{side}_finger_link::{side}_finger_link_collision', f'{between}::link::collision', now)
        elif self.held is not None:
            self.held = None
        moved = []
        if self.held is not None:
            o = self.obj[self.held]
            o['x'], o['y'], o['z'] = tx, ty, tz
            moved.append(self.held)
        for m, o in self.obj.items():
            if m != self.held and o['z'] > o['h'] / 2 + 1e-3:
                o['z'] = o['h'] / 2                                    # released: falls straight down
                moved.append(m)
        for m in moved:
            self._sync(m)

    def _sync(self, model):
        o = self.obj[model]
        for k, p in enumerate(self.scene.solids):
            if p['model'] == model:
                p['c'] = (o['x'], o['y'], o['z'])
                self.scene._c[k] = p['c']


def in_tray(o, station):
    s = station['tray']
    return s['x_min'] <= o['x'] <= s['x_max'] and s['y_min'] <= o['y'] <= s['y_max'] and o['z'] < 0.15


def run(seed=0, t_max=1800.0, rgb_every=40, use_sim_pose=True, verbose=True, pf_n=500, severity=0.0, slip=0.0):
    prior, _, graph, truth = explore_sim.load_network('sewer_network_v2', 'blocked')
    true_map = RecordingMap(sc.true_lidar_map(prior))               # blockages + wall defects, 1 cm grid
    robot = explore_sim.SimRobot(prior, true_map, graph, seed, PFParams(n=pf_n), severity=severity, slip=slip)
    # wheel odometry as the robot measures it (Account 1's noisy/slipping odometry model), integrated like /odom
    odom = [0.0, 0.0, 0.0]
    odo_step = robot.odo.step

    def odo_record(ds, dth, dt, travelled):
        out = odo_step(ds, dth, dt, travelled)
        dsm, dthm = out[0], out[1]
        odom[0] += dsm * math.cos(odom[2] + 0.5 * dthm)
        odom[1] += dsm * math.sin(odom[2] + 0.5 * dthm)
        odom[2] = math.atan2(math.sin(odom[2] + dthm), math.cos(odom[2] + dthm))
        return out
    robot.odo.step = odo_record
    mm = MissionManager(graph, RouteModel(graph, prior), params=MissionParams(require_manipulator=True, init_timeout=5.0))
    # Account 2's real gripper limits (what docs/mission_interfaces.md asks Account 2 to provide; mission.yaml keys
    # gripper_max_width / gripper_max_length / gripper_max_height). max_width is the finger opening: the Mission
    # Manager adds the size uncertainty itself; Account 2's own `graspable` keeps 1 cm per side in reserve.
    mm.prio.lim.max_width, mm.prio.lim.max_length, mm.prio.lim.max_height = 0.11, 0.32, 0.15
    with open(os.path.join(G, 'config', 'inspection_station.yaml')) as f:
        station = yaml.safe_load(f)
    with open(os.path.join(G, 'config', 'network_graph_v2.yaml')) as f:
        gy = yaml.safe_load(f)
    nodes = {k: (v['x'], v['y']) for k, v in gy['nodes'].items()}
    edges = [(e['from'], e['to']) for e in gy['edges']]
    scene = render3d.Scene()
    dcam, rcam = CameraModel(320, 180), CameraModel(640, 360)
    core = InspectionCore(prior, nodes, edges, station,
                          save_evidence=lambda rgb, t, boxes, tag, circles: f'{tag}_frame_{t:.1f}')
    logs = []
    bridge = ManipulationBridge(station, log=lambda s: logs.append(s))
    arm = ArmWorld(bridge.ctl.geo, scene)
    rng = np.random.default_rng(seed + 11)
    path, res, states, tasks = None, None, [], []
    t0 = time.time()
    k = 0
    for k in range(int(t_max / DT)):
        now = k * DT
        xe, ye, the, cov = robot.estimate()
        est = (xe, ye, the)
        true_pose = (robot.x, robot.y, robot.th)
        # ---- arm at 20 Hz (two control steps per robot step)
        bridge.on_pose(est)
        bridge.on_odom(now, tuple(odom))
        for sub in (0.0, 0.5 * DT):
            bridge.on_joints(list(arm.q), list(arm.q.values()))
            if use_sim_pose:
                for m, o in arm.obj.items():
                    bridge.on_sim_pose(m, (o['x'], o['y'], o['z']))
            cmd = bridge.tick(now + sub, use_sim_pose)
            if cmd is not None:
                arm.step(0.5 * DT, true_pose, cmd, now + sub,
                         lambda side, a, b, t: bridge.on_contact(side, a, b, t))
        # ---- perception
        if true_map.last is not None and k % 2 == 0:
            core.on_scan(now, est, true_map.last, sim2d.ANGLES, robot.f.r)
        if k % 5 == 0:
            depth, _, _ = scene.render(dcam, true_pose)
            core.on_depth(now, est, depth)
            bridge.tray_count = bridge.tray_points(dcam, depth, est, now) if bridge.wants_tray() else None
            if rgb_every and k % rgb_every == 0:
                _, rgb, _ = scene.render(rcam, true_pose, noise=0.02, rng=rng)
                core.on_rgb(now, est, (rgb * 255).astype(np.uint8))
        # ---- Mission Manager at 2 Hz, through the documented JSON messages
        if k % 5 == 0:
            dmsg, fmsg = core.messages(now)
            djson = itf.dumps(dmsg)
            bridge.on_debris(djson)
            status, disp = bridge.status_json(now), bridge.disposal_json(now)
            core.on_manipulation(json.loads(status))
            core.on_disposal(json.loads(disp))
            debris, derr = parse_debris(djson)
            defects, ferr = parse_defects(itf.dumps(fmsg))
            assert not derr and not ferr, (derr, ferr)
            res = mm.tick(now, (xe, ye), now, cov[:2, :2], robot.f.R, robot.health.state, robot.edge_status(), debris,
                          defects, parse_manipulation(status), parse_disposal(disp))
            if res['task'] is not None:
                tj = res['task'].to_json()
                tasks.append(json.loads(tj))
                bridge.on_task(tj, now)
            if not states or states[-1][1] != res['state']:
                states.append((round(now, 1), res['state']))
                if verbose:
                    print(f"t={now:7.1f} {res['state']:20s} {res['reason']}  [{time.time() - t0:.0f}s wall]", flush=True)
            if res['state'] in TERMINAL:
                break
            if res['route'] is not None:
                path = explore_sim.densify(res['route']) if len(res['route']) else None
        robot.step(*robot.command(path))
    rep = mm.report(k * DT)
    final_d, final_f = core.messages(k * DT)
    # score every object perception found, including the ones since picked up and removed (no longer published)
    for tr in core.tracker.tracks:
        if tr.state in ('held', 'removed'):
            g_ = tr.geometry()
            final_d['objects'].append(dict(id=tr.id, x=g_['x'], y=g_['y'], size=[g_['length'], g_['width'], g_['height']],
                                           graspable=True, state='confirmed', feasibility=dict(reasons=[])))
    truth3 = load_truth()
    disposed_truth = sorted(o['id'] for o in arm.obj.values() if in_tray(o, station))
    return dict(report=rep, states=states, tasks=tasks, arm_log=logs, disposed_in_tray=disposed_truth,
                perception=dict(debris=score_debris(final_d, truth3), defects=score_defects(final_f, truth3)),
                wall_time=round(time.time() - t0, 1), **{k_: v for k_, v in robot.metrics().items()
                                                         if k_ not in ('true', 'est')})


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--t-max', type=float, default=1800.0)
    ap.add_argument('--rgb-every', type=int, default=40, help='robot steps between RGB frames (0 = no cracks)')
    ap.add_argument('--no-sim-pose', action='store_true', help='verify disposal from the camera only')
    ap.add_argument('--severity', type=float, default=0.0, help='scan disturbance severity (0 = clean)')
    ap.add_argument('--slip', type=float, default=0.0, help='wheel-slip severity (0 = none)')
    ap.add_argument('--out', default=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'results',
                                                  'mission_account2.json'))
    a = ap.parse_args()
    out = run(a.seed, a.t_max, a.rgb_every, not a.no_sim_pose, severity=a.severity, slip=a.slip)
    r = out['report']
    print(f"\nfinal {r['state']} ({r['reason']}); sim {out['sim_time']:.0f} s, wall {out['wall_time']:.0f} s")
    print('coverage', r['coverage']['coverage'], 'counts', r['counts'])
    print('removed (mission report):', list(r['debris']['removed']))
    print('in the tray (truth):     ', out['disposed_in_tray'])
    print('failed:', r['debris']['failed'])
    print('rejected:', {x['id']: x.get('detail', x.get('reason')) for x in r['debris']['rejected']} if isinstance(
        r['debris']['rejected'], list) else r['debris']['rejected'])
    p = out['perception']
    print('perception: debris P/R', p['debris']['precision'], p['debris']['recall'], 'defects P/R',
          p['defects']['precision'], p['defects']['recall'])
    print(f"pose RMSE {out['pos_rmse']:.3f} m, min wall clearance {out['min_clearance']:.3f} m")
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    with open(a.out, 'w') as f:
        json.dump(out, f, indent=1, default=str)


if __name__ == '__main__':
    main()
