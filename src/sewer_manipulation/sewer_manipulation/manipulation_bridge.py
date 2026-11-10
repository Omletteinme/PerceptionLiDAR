"""Glue between the Mission Manager's messages and the arm controller (Account 2). CUSTOM code, pure Python, no ROS.

Used by manipulation_node.py (ROS) and by the offline end-to-end mission test (sandbox/mission_sim_account2.py), so
both run exactly the same logic:
  /mission/task PICKUP  -> look the object up in /perception/debris, express it in base_link, controller pickup
  /mission/task DISPOSE -> tray centre (station config) in base_link from the ESTIMATED pose, controller release
  /mission/task STOP    -> controller stop (freeze, keep holding)
  finger contacts, joint positions, depth-camera tray count, optional simulator object poses -> controller tick
  controller state -> /manipulation/status and /disposal/status JSON (docs/mission_interfaces.md)
Tasks are matched by task_id; a task delivered twice (latched topic) or older than `max_task_age` is ignored.
A PICKUP or DISPOSE starts only once the robot base has come to rest (the Mission Manager sends it when the robot is
within its arrival tolerance, possibly still rolling); the object / tray is located again at that moment. If the base
moves while the arm is working, the action is aborted and reported (`robot moved during the grasp`).
"""
import json
import math

import numpy as np

from sewer_perception import interfaces as itf
from sewer_perception.camera_model import map_to_base

from .arm_kinematics import FINGERS, JOINTS, ik
from .manipulation_controller import ManipulationController


class ManipulationBridge:
    def __init__(self, station=None, max_task_age=10.0, max_obs_age=1.0, log=print):
        self.ctl = ManipulationController()
        self.station = station                       # inspection_station.yaml contents (a-priori configuration)
        if station:
            s = station['tray']
            self.ctl.tray = (s['x_min'], s['x_max'], s['y_min'], s['y_max'])
        self.max_task_age, self.max_obs_age, self.log = max_task_age, max_obs_age, log
        self.q, self.sense, self.sim = {}, {'left': {}, 'right': {}}, {}
        self.pose = None
        self.odom = []                          # (t, x, y, th) wheel odometry, last 60 s
        self.debris = {}
        self.tray_count = None
        self.tray_seen = None                   # (t, tray centre x, y in base_link) located by the depth camera
        self.seen_tasks = set()
        self.started = False
        self.disposal_task = None
        self.disposal_object = ''               # the object the CURRENT /disposal/status is about
        self.pending = None                     # (task, t_request) waiting for the base to stop
        self.poses = []                         # recent (t, pose) to tell whether the base is at rest
        self.work_pose = None                   # base pose when the arm started an action
        self.max_odom_carry_age, self.max_odom_carry_dist = 30.0, 1.5
        self.target_source = ''
        self.still_time, self.still_tol, self.wait_max, self.move_tol = 0.6, (0.01, 0.01), 8.0, (0.08, 0.10)

    # ------------------------------------------------------------------ inputs
    def on_pose(self, pose):
        self.pose = tuple(pose)

    def on_odom(self, t, pose):
        """Wheel odometry (/odom): drifts over long distances but is smooth and accurate over the last metre, so it
        carries the camera's last direct sighting of an object forward once the object is out of the camera's view."""
        self.odom.append((float(t),) + tuple(float(v) for v in pose))
        self.odom = [o for o in self.odom if t - o[0] <= 60.0]

    def odom_at(self, t, tol=0.25):
        if not self.odom:
            return None
        o = min(self.odom, key=lambda o: abs(o[0] - t))
        return o[1:] if abs(o[0] - t) <= tol else None

    def _record_pose(self, now):
        if self.pose is not None:
            self.poses.append((now, self.pose))
            self.poses = [p for p in self.poses if now - p[0] <= 2.0]


    def base_at_rest(self, now):
        """The estimated pose has not moved (1 cm, 0.01 rad) during the last still_time seconds."""
        recent = [p for t, p in self.poses if now - t <= self.still_time]
        if not self.poses or now - self.poses[0][0] < self.still_time or len(recent) < 2:
            return False
        xs, ys, ths = zip(*recent)
        return (max(xs) - min(xs) < self.still_tol[0] and max(ys) - min(ys) < self.still_tol[0]
                and max(ths) - min(ths) < self.still_tol[1])

    def on_joints(self, names, positions):
        for n, v in zip(names, positions):
            self.q[n] = float(v)

    def on_contact(self, side, collision1, collision2, now):
        """One contact from a finger's contact sensor (ros_gz_interfaces/Contact collision names)."""
        other = collision2 if collision1.startswith('sewer_robot::') else collision1
        if not other.startswith('sewer_robot::'):
            self.sense[side][other.split('::')[0]] = now

    def on_sim_pose(self, name, xyz):
        """SIMULATED STATE (optional): Gazebo's pose of a debris model, extra evidence for lift/disposal only."""
        name = name.split('::')[-1]
        if name.startswith('debris_'):
            self.sim[name] = tuple(xyz)

    def on_debris(self, text):
        try:
            msg = json.loads(text)
        except ValueError:
            return
        self.debris = {o['id']: o for o in msg.get('objects', [])}

    def ready(self):
        return all(j in self.q for j in JOINTS + FINGERS)

    # ------------------------------------------------------------------ geometry
    def object_in_base(self, oid, now, target=None):
        """Object geometry in base_link, in order of preference:
        1. the latest direct camera observation (perception's fused footprint centre relative to the wall-registered
           camera pose of that frame: no localization error in it), if fresh;
        2. the latest direct observation carried forward with wheel odometry: the robot often stops with the object
           beside it, out of the camera's view, and the localization estimate can drift several cm while it stands
           there; odometry over the last metre does not (found by the end-to-end mission test);
        3. its map pose (perception's, else the task target) transformed with the current estimate."""
        o = self.debris.get(oid)
        if o is None and target and 'x' in target:
            # the ID may have changed (perception merged two tracks of the same object): use the object perception
            # now reports at the target position, if there is exactly one within 0.25 m
            near = [d for d in self.debris.values() if math.hypot(d['x'] - target['x'], d['y'] - target['y']) < 0.25]
            o = near[0] if len(near) == 1 else None
        if self.pose is None or o is None:
            return None
        L, W, H = o['size']
        rb = o.get('last_seen_base')
        o_then, o_now = (self.odom_at(rb['stamp']), self.odom_at(now)) if rb else (None, None)
        if rb and now - rb['stamp'] <= self.max_obs_age:
            x, y = rb['x'], rb['y']
            src = 'camera'
        elif o_then is not None and o_now is not None and now - rb['stamp'] <= self.max_odom_carry_age \
                and math.hypot(o_now[0] - o_then[0], o_now[1] - o_then[1]) <= self.max_odom_carry_dist:
            # base_then -> odom -> base_now
            bx, by = rb['x'], rb['y']
            c, s_ = math.cos(o_then[2]), math.sin(o_then[2])
            ox, oy = o_then[0] + c * bx - s_ * by, o_then[1] + s_ * bx + c * by
            x, y = map_to_base([(ox, oy)], o_now)[0]
            src = 'camera+odometry'
        else:
            x, y = map_to_base([(o['x'], o['y'])], self.pose)[0]
            src = 'map+localization'
        self.target_source = src
        yaw = o['yaw'] - self.pose[2]
        return dict(x=float(x), y=float(y), yaw=math.atan2(math.sin(yaw), math.cos(yaw)), width=W, length=L, height=H)

    def drop_point(self, now=None):
        """Tray centre in base_link: where the depth camera located the tray in the last second (no localization error
        in it), else from the a-priori station config and the ESTIMATED pose; None without either."""
        if self.tray_seen is not None and now is not None and now - self.tray_seen[0] <= self.max_obs_age:
            return self.tray_seen[1], self.tray_seen[2]
        if not self.station or self.pose is None:
            return None
        s = self.station['tray']
        return tuple(map_to_base([((s['x_min'] + s['x_max']) / 2, (s['y_min'] + s['y_max']) / 2)], self.pose)[0])

    def station_reached(self, now=None):
        """The robot is 'at the station' when the tray centre is within the arm's reach from where it stands."""
        d = self.drop_point(now)
        if d is None:
            return False
        g = self.ctl.geo
        return ik(g, (d[0], d[1], 0.12)) is not None and ik(g, (d[0], d[1], 0.0)) is not None

    # ------------------------------------------------------------------ tasks
    def on_task(self, text, now):
        try:
            task = itf.parse_task(text)
        except (ValueError, itf.SchemaError) as e:
            self.log(f'bad /mission/task: {e}')
            return None
        if task is None or task['task_id'] in self.seen_tasks:
            return None
        self.seen_tasks.add(task['task_id'])
        if task['stamp'] is not None and now - float(task['stamp']) > self.max_task_age:
            self.log(f"ignoring stale task {task['task_id']} ({now - float(task['stamp']):.1f} s old)")
            return None
        if task['type'] in ('pickup', 'release'):
            # wait for the base to come to rest; tick() starts the action
            self.pending = (task, now)
            self.ctl.task_id, self.ctl.object_id = task['task_id'], (task['object_id'] or self.ctl.object_id)
            if task['type'] == 'pickup':
                self.ctl.state, self.ctl.reason = 'approaching', 'waiting for the robot to stop'
                self.ctl.steps = []
            else:
                self.disposal_task = task['task_id']
                self.disposal_object = self.ctl.object_id or ''
                self.ctl.disposal = dict(station_reached=False, object_released=False, disposal_verified=False,
                                         reason='')
        else:
            self.pending = None
            self.ctl.request(task, now)
        self.log(f"task {task['task_id']} {task['mission_type']} {task['object_id']} -> {self.ctl.state} "
                 f"{self.ctl.reason or self.ctl.disposal.get('reason', '')}")
        return task

    # ------------------------------------------------------------------ control
    def start_pending(self, now):
        task, t_req = self.pending
        if not self.base_at_rest(now):
            if now - t_req > self.wait_max:
                self.pending = None
                if task['type'] == 'pickup':
                    self.ctl._fail('robot did not come to rest for the pickup', retreat=False)
                else:
                    self.ctl.disposal['reason'] = 'failed: robot did not come to rest at the station'
            return
        self.pending = None
        self.work_pose = self.pose
        if task['type'] == 'pickup':
            obj = self.object_in_base(task['object_id'], now, task['target'])
            if obj is None:
                self.ctl._fail('object unknown to perception or no pose estimate', retreat=False)
                return
            self.ctl.state = 'idle'
            self.ctl.request(task, now, obj)
        else:
            self.ctl.request(task, now, station_reached=self.station_reached(now), drop=self.drop_point(now))
        self.log(f"start {task['task_id']} {task['mission_type']} ({self.target_source if task['type'] == 'pickup' else 'tray'})"
                 f" -> {self.ctl.state} "
                 f"{self.ctl.reason or self.ctl.disposal.get('reason', '')}")

    def base_moved_while_working(self):
        if self.work_pose is None or self.pose is None:
            return False
        dx = math.hypot(self.pose[0] - self.work_pose[0], self.pose[1] - self.work_pose[1])
        dth = abs(math.atan2(math.sin(self.pose[2] - self.work_pose[2]), math.cos(self.pose[2] - self.work_pose[2])))
        return dx > self.move_tol[0] or dth > self.move_tol[1]

    def tick(self, now, use_sim_pose=True):
        """One control step; returns the joint commands, or None until the joint states are known."""
        self._record_pose(now)
        if not self.ready():
            return None
        if not self.started:
            self.ctl.request({'type': 'stow', 'task_id': ''}, now)
            self.started = True
        if self.pending is not None:
            self.start_pending(now)
        if self.ctl.state in ('approaching', 'grasping') and self.pending is None and self.base_moved_while_working():
            self.ctl._fail('robot moved during the grasp')
            self.work_pose = None
        if self.ctl.state in ('holding', 'idle', 'failed', 'released'):
            self.work_pose = None
        obj_pose = (lambda name: self.sim.get(name)) if use_sim_pose else None
        return self.ctl.tick(now, self.q, self.sense, obj_pose, self.tray_count)

    def status_json(self, now):
        st = self.ctl.status()
        v = dict(st['verification'])
        if st['state'] in ('lifting', 'holding', 'releasing') and self.ctl.obj:
            v['held_dims'] = {k: self.ctl.obj[k] for k in ('length', 'width', 'height')}
        return itf.dumps(itf.manipulation_msg(st['state'], now, st['task_id'], st['object_id'], v, st['reason'],
                                              st['arm_in_scan_plane']))

    def disposal_json(self, now):
        d = self.ctl.disposal
        # the disposal status keeps naming the object it was about, even after the arm moved on to the next pickup
        return itf.dumps(itf.disposal_msg(now, self.disposal_task, self.disposal_object,
                                          d['station_reached'], d['object_released'], d['disposal_verified'],
                                          d.get('reason', ''), d.get('evidence', [])))

    def wants_tray(self):
        """The tray check costs ~0.1 s per frame: run it only around a disposal."""
        return ((self.pending is not None and self.pending[0]['type'] == 'release')
                or self.ctl.state == 'releasing'
                or (self.ctl.state == 'released' and bool(self.ctl.steps)))

    def tray_points(self, cam, depth, pose, now=None):
        """Depth-camera check of the tray (sensor evidence for disposal). The tray is first LOCATED from its rims
        (a 4 cm high rectangle): the expected rectangle from the station config is shifted / turned until the most rim
        points line up, so a localization error of up to 0.3 m does not move the check off the tray (found by the
        end-to-end mission with disturbed localization: points beside the tray were counted as inside it).
        Returns the number of points standing above the tray floor inside the located tray, or None if the rims
        were not found (tray not in view, or not where it should be)."""
        if not self.station:
            return None
        pc, _, _ = cam.depth_to_points(depth, 2)
        if len(pc) == 0:
            return None
        pw = cam.to_world(pc, pose)
        s = self.station['tray']
        t = s.get('rim_t', 0.01)
        cx, cy = 0.5 * (s['x_min'] + s['x_max']), 0.5 * (s['y_min'] + s['y_max'])
        hx, hy = 0.5 * (s['x_max'] - s['x_min']) + 0.5 * t, 0.5 * (s['y_max'] - s['y_min']) + 0.5 * t
        near = pw[(np.hypot(pw[:, 0] - cx, pw[:, 1] - cy) < 0.6) & (pw[:, 2] < 0.25)]
        # drop the foot of walls (anything with points higher than a rim or a debris object in the same 2 cm cell)
        cell = lambda p: set(zip(np.floor(p[:, 0] / 0.02).astype(int), np.floor(p[:, 1] / 0.02).astype(int)))
        tall = {(i + a, j + b) for i, j in cell(near[near[:, 2] > 0.17]) for a in (-1, 0, 1) for b in (-2, -1, 0, 1, 2)}
        if tall:
            keys = zip(np.floor(near[:, 0] / 0.02).astype(int), np.floor(near[:, 1] / 0.02).astype(int))
            near = near[np.array([k not in tall for k in keys], bool)]
        rim = near[(near[:, 2] > 0.012) & (near[:, 2] < s.get('rim_h', 0.04) + 0.015)]
        if len(rim) < 20:
            self.tray_seen = None
            return None
        def search(dxs, dys, dths, best, tol):
            for dth in dths:
                c, sn = math.cos(dth), math.sin(dth)
                for dx in dxs:
                    for dy in dys:
                        # rim points in the frame of the candidate tray (centre cx+dx, cy+dy, turned by dth)
                        px, py = rim[:, 0] - cx - dx, rim[:, 1] - cy - dy
                        u, v = c * px + sn * py, -sn * px + c * py
                        on_x = (np.abs(np.abs(u) - hx) < tol) & (np.abs(v) < hy + tol)
                        on_y = (np.abs(np.abs(v) - hy) < tol) & (np.abs(u) < hx + tol)
                        sides = [int((on_x & (u > 0)).sum()), int((on_x & (u < 0)).sum()),
                                 int((on_y & (v > 0)).sum()), int((on_y & (v < 0)).sum())]
                        score = sum(sides)
                        if sum(k >= 6 for k in sides) >= 3 and (best is None or score > best[0]):
                            best = (score, float(dx), float(dy), float(dth))
            return best

        best = search(np.arange(-0.32, 0.321, 0.04), np.arange(-0.32, 0.321, 0.04), np.arange(-0.12, 0.121, 0.04), None, 0.025)
        if best is not None:                                  # refine around the coarse match
            _, bx0, by0, bt0 = best
            best = search(bx0 + np.arange(-0.03, 0.031, 0.01), by0 + np.arange(-0.03, 0.031, 0.01),
                          bt0 + np.arange(-0.03, 0.031, 0.01), None, 0.012)
        if best is None:
            self.tray_seen = None
            return None
        _, dx, dy, dth = best
        c, sn = math.cos(dth), math.sin(dth)
        px, py = near[:, 0] - cx - dx, near[:, 1] - cy - dy
        u, v = c * px + sn * py, -sn * px + c * py
        m = 0.02
        inside = ((np.abs(u) < hx - 0.5 * t - m) & (np.abs(v) < hy - 0.5 * t - m) & (near[:, 2] > 0.02))
        if now is not None:
            bx, by = map_to_base([(cx + dx, cy + dy)], pose)[0]
            self.tray_seen = (now, float(bx), float(by))
        self.tray_offset = (round(float(dx), 2), round(float(dy), 2), dth)
        return int(inside.sum())
