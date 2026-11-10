"""Pick / carry / release sequencing with evidence-based verification. CUSTOM code (Account 2), pure Python, no ROS.

This is NOT the mission state machine (that is Account 1's Mission Manager). It executes one requested action at a
time (pickup, release, stow, stop), reports its own status and never moves the robot base. The Mission Manager's
/mission/task types map to it as PICKUP -> pickup, DISPOSE -> release (into the tray), STOP -> stop
(docs/mission_interfaces.md; the translation lives in manipulation_bridge.py).

Verification rules (a command acknowledgement is never treated as success):
  grasp   both finger contact sensors touch the SAME object (not a wall, the floor or the robot) AND the fingers
          stalled at a gap matching the expected object width (they did not close on nothing)
  lift    while lifting and carrying, both contacts and the finger gap persist; if the simulator's object pose is
          available, the object must also have risen by >= 0.08 m (simulated state, allowed by the build plan)
  hold    the fingers close past the object's width, or a finger reports no contact with it for > 1 s, while
          holding -> failed ('dropped'); Gazebo's contact messages flicker, so short gaps are tolerated
  release fingers open and neither finger touches the object for 0.5 s
  disposal released AND robot at the station AND the object is seen in the tray (depth-camera points inside the tray
          increase after the release, a sensor check) or, if available, the simulator's object pose is inside the tray
Objects are never teleported or attached by script: they are held by finger friction in the physics engine.
"""
import math
from dataclasses import dataclass

import numpy as np

from .arm_kinematics import ArmGeometry, FINGERS, JOINTS, fk, grasp_height, ik, radial_heading

IGNORE_CONTACT = ('sewer_robot', 'walls', 'floor', 'ground', 'decor')


@dataclass
class CtlParams:
    speed: float = 0.10              # finger-centre speed [m/s]
    joint_tol: float = 0.05          # arm joints converged when |q - cmd| < tol [rad]
    move_margin: float = 4.0         # timeout = path time + margin [s]
    above: float = 0.12              # the open gripper is lowered onto the object from this far above it
    lift: float = 0.12
    squeeze: float = 0.02            # close each finger this far past the expected surface (grip force)
    finger_open_margin: float = 0.02
    gap_tol_lo: float = 0.02
    gap_tol_hi: float = 0.03
    close_timeout: float = 3.0
    stall_vel: float = 0.003         # finger speed below this [m/s] counts as stalled
    stall_time: float = 0.4
    contact_hold: float = 0.5        # a contact counts as present if seen within this time [s]
    drop_time: float = 1.0           # no contact message from a finger for this long while holding -> lost
    hold_grace: float = 0.3          # a failed hold check must persist this long before it counts
    release_wait: float = 0.8
    min_rise: float = 0.08
    max_width: float = 0.09          # object extent across the fingers (inner opening 0.11 m minus 1 cm each side)


class ManipulationController:
    def __init__(self, geo=None, params=None):
        self.geo = geo or ArmGeometry()
        self.p = params or CtlParams()
        self.state = 'idle'
        self.reason = ''
        self.task_id = None
        self.object_id = None
        self.model = None                 # simulator model name of the held object, learned from the CONTACT sensors
        self.verification = {}
        self.disposal = dict(station_reached=False, object_released=False, disposal_verified=False, reason='')
        self.cmd = {j: 0.0 for j in JOINTS + FINGERS}
        self.tip = None
        self.heading = 0.0
        self.steps = []
        self.step_data = {}
        self.obj = None
        self.t = 0.0

    # ------------------------------------------------------------------ requests
    def request(self, task, t, obj=None, station_reached=False, drop=None):
        """task: dict(type, task_id[, object_id]) with type pickup / release / stow / stop.
        obj (pickup): dict(x, y, yaw, width, length, height) in base_link.
        drop (release): (x, y) in base_link where the object should go (the tray centre); None = straight ahead."""
        typ = task['type']
        self.task_id = task.get('task_id')
        if typ in ('stop', 'abort'):
            # freeze the arm where it is and keep holding whatever is held (Mission Manager SAFE_STOP)
            self.steps, self.step_data = [], {}
            if self.state not in ('lifting', 'holding', 'releasing'):
                self.state = 'idle' if self.state != 'failed' else 'failed'
            elif self.state in ('lifting', 'releasing'):
                self.state = 'holding' if self.model is not None else self.state
            self.reason = 'stopped'
            return True
        if typ == 'stow':
            self.state, self.reason = 'idle', 'stowing'
            self.steps = [('move', tuple(self.geo.stow))]
            self.step_data = {}
            return True
        if typ == 'pickup':
            if self.state == 'holding':
                self.reason = 'rejected: already_holding'      # keep holding; the request is simply refused
                return False
            self.object_id = task.get('object_id')
            self.model = None
            self.verification = {}
            return self._plan_pickup(obj)
        if typ == 'release':
            self.disposal = dict(station_reached=bool(station_reached), object_released=False,
                                 disposal_verified=False, reason='')
            return self._plan_release(drop)
        return self._fail(f'unknown task {typ}', retreat=False)

    def _fail(self, reason, retreat=True):
        """Report the failure (state stays 'failed' with the reason until the next task) and, unless aborted, fold
        the arm safely out of the LiDAR plane: open the fingers, lift, stow. Retries are the Mission Manager's call."""
        retreating = self.state == 'failed'          # a failure during the retreat itself: just stop
        self.state, self.reason = 'failed', reason
        if not retreat or reason == 'aborted' or retreating or self.tip is None:
            self.steps = []
        else:
            up = (float(self.tip[0]), float(self.tip[1]), float(self.tip[2]) + self.p.above, self.heading)
            self.steps = [('fingers_to', self.geo.finger_max), ('move', up), ('move', tuple(self.geo.stow))]
        self.step_data = {}
        return False

    def _plan_pickup(self, obj):
        g, p = self.geo, self.p
        if obj is None:
            return self._fail('no_object_geometry', retreat=False)
        self.obj = dict(obj)
        x, y = obj['x'], obj['y']
        radial = radial_heading(g, (x, y))
        # the fingers point along the gripper heading and close across it: point them along the object's long side
        # (so they squeeze its width) or, if the length fits too, along its short side; prefer the heading closest to
        # straight out from the turret (smallest wrist twist) that the arm can reach
        options = [(obj['yaw'], obj['width']), (obj['yaw'] + math.pi, obj['width']),
                   (obj['yaw'] + math.pi / 2, obj['length']), (obj['yaw'] - math.pi / 2, obj['length'])]
        zg = grasp_height(g, obj['height']) - g.base_z
        best = None
        for h, w in options:
            if w > p.max_width:
                continue
            h = math.atan2(math.sin(h), math.cos(h))
            twist = abs(math.atan2(math.sin(h - radial), math.cos(h - radial)))
            if all(ik(g, (x, y, z), h) is not None for z in (zg, zg + p.above, zg + p.lift)):
                if best is None or twist < best[0]:
                    best = (twist, h, w)
        if best is None:
            if min(obj['length'], obj['width']) > p.max_width:
                return self._fail(f'too_wide_for_gripper ({min(obj["length"], obj["width"]):.3f} m)', retreat=False)
            return self._fail('unreachable (no gripper heading reaches the object)', retreat=False)
        _, heading, w = best
        self.obj['grasp_width'] = w
        # squeezing a slightly rotated object usually turns it flat between the fingers, so the gap may end anywhere
        # between its narrow side and the width measured by perception
        self.obj['min_width'] = min(obj['length'], obj['width'], w)
        above = (x, y, zg + p.above, heading)
        grasp = (x, y, zg, heading)
        lift = (x, y, zg + p.lift, heading)
        q_open = min(g.finger_max, w / 2 + p.finger_open_margin)
        q_close = max(0.0, w / 2 - p.squeeze)
        self.state, self.reason = 'approaching', ''
        self.steps = [('fingers_to', q_open), ('move', above), ('move', grasp),
                      ('state', 'grasping'), ('close', q_close), ('verify_grasp', None),
                      ('state', 'lifting'), ('move', lift), ('move', tuple(g.stow)), ('verify_lift', None),
                      ('state', 'holding')]
        self.step_data = {}
        return True

    def _plan_release(self, drop=None):
        g, p = self.geo, self.p
        h = self.obj['height'] if self.obj else 0.06
        zd = 0.04 + h / 2 + 0.03 - g.base_z                # drop with the object's bottom 3 cm above the tray rim
        dx, dy = drop if drop is not None else (g.grasp_x, 0.0)
        above, target = (dx, dy, zd + p.above), (dx, dy, zd)
        if ik(g, above) is None or ik(g, target) is None:
            # keep holding and do not move; the Mission Manager may reposition and retry
            self.disposal['reason'] = f'failed: drop point out of reach ({math.hypot(dx, dy):.2f} m)'
            return False
        self.state, self.reason = 'releasing', ''
        self.steps = [('mark_tray', None), ('move', above), ('move', target),
                      ('fingers_to', g.finger_max), ('wait', p.release_wait), ('verify_release', None),
                      ('state', 'released'), ('move', above), ('move', tuple(g.stow)),
                      ('verify_disposal', None)]
        self.step_data = {}
        return True

    # ------------------------------------------------------------------ helpers
    def _contacts(self, sense):
        """Models touched by each finger recently: (set_left, set_right)."""
        t, hold = self.t, self.p.contact_hold
        def recent(d):                                         # noqa: E306
            return {m for m, ts in d.items() if t - ts <= hold and m.split('::')[0] not in IGNORE_CONTACT}
        return recent(sense.get('left', {})), recent(sense.get('right', {}))

    def gap(self, q):
        return q.get(FINGERS[0], 0.0) + q.get(FINGERS[1], 0.0)

    def _arm_converged(self, q):
        return all(abs(q.get(j, 0.0) - self.cmd[j]) < self.p.joint_tol for j in JOINTS)

    def _hold_check(self, q, sense):
        """Is the object still between the fingers? (ok, info)

        Main signal: the finger gap. If the object slips out, the fingers close past its width. Second signal: the
        finger contact sensors. Gazebo only publishes contact messages on steps with contact, and friction contacts
        flicker, so a finger only counts as 'lost' after drop_time without any message naming the object."""
        ages = []
        for side in ('left', 'right'):
            ts = [v for m, v in sense.get(side, {}).items() if self.model is not None and m.split('::')[0] == self.model]
            ages.append(self.t - max(ts) if ts else math.inf)
        w = self.obj['grasp_width'] if self.obj else 0.0
        wmin = self.obj.get('min_width', w) if self.obj else 0.0
        gp = self.gap(q)
        gap_ok = (wmin - self.p.gap_tol_lo) <= gp <= (w + self.p.gap_tol_hi)
        ok = gap_ok and max(ages) <= self.p.drop_time
        info = dict(finger_gap=round(gp, 4), gap_ok=gap_ok, contact_age_left=round(min(ages[0], 99.0), 2),
                    contact_age_right=round(min(ages[1], 99.0), 2))
        return ok, info

    def _hold_failed(self, q, sense, sd, reason):
        """Run the hold check; fail with `reason` (and the evidence) if it has been failing for hold_grace."""
        ok, info = self._hold_check(q, sense)
        if ok:
            sd.pop('bad_since', None)
            return False
        sd.setdefault('bad_since', self.t)
        if self.t - sd['bad_since'] >= self.p.hold_grace:
            self.verification['hold_failure'] = info
            self._fail(reason)
            return True
        return False

    # ------------------------------------------------------------------ tick
    def tick(self, t, q, sense, obj_pose=None, tray_count=None):
        """Advance by one control step.

        q: joint positions (arm + fingers); sense: {'left': {model: last_contact_time}, 'right': {...}};
        obj_pose(model) -> (x, y, z) world or None (simulated state, optional);
        tray_count: number of depth points seen above the tray floor right now, or None.
        Returns the joint command dict.
        """
        self.t = t
        if self.tip is None:
            p0 = fk(self.geo, [q.get(j, 0.0) for j in JOINTS])
            self.tip, self.heading = np.array(p0['tip']), p0['heading']
            for j in JOINTS:
                self.cmd[j] = q.get(j, 0.0)
        if self.state == 'holding' and not self.steps:
            if self._hold_failed(q, sense, self.step_data, 'dropped'):
                self.verification['dropped_at'] = round(t, 2)
        while self.steps:
            was_failed = self.state == 'failed'
            n_before = len(self.steps)
            done = self._run_step(self.steps[0], q, sense, obj_pose, tray_count)
            if self.state == 'failed' and not was_failed:
                break                                    # steps were replaced by the safe retreat
            if not done or len(self.steps) != n_before:
                break
            self.steps.pop(0)
            self.step_data = {'last_ok': t}
        return dict(self.cmd)

    def _run_step(self, step, q, sense, obj_pose, tray_count):
        kind, arg = step
        p, g, sd, t = self.p, self.geo, self.step_data, self.t
        if kind == 'state':
            self.state = arg
            return True
        if kind == 'fingers_to':
            self.cmd[FINGERS[0]] = self.cmd[FINGERS[1]] = float(arg)
            return True
        if kind == 'wait':
            sd.setdefault('t0', t)
            return t - sd['t0'] >= arg
        if kind == 'move':
            target = np.array(arg[:3], float)
            h1 = arg[3] if len(arg) > 3 and arg[3] is not None else radial_heading(g, target)
            cx, cy = g.turret_xyz[0], g.turret_xyz[1]
            if 't0' not in sd:
                # move along a path around the turret axis (turret angle, radius and height interpolated together):
                # a straight line from the folded pose to an object beside the robot passes over the turret axis,
                # where the arm cannot reach
                def cyl(v):
                    return math.atan2(v[1] - cy, v[0] - cx), math.hypot(v[0] - cx, v[1] - cy), float(v[2])
                a0, r0, z0 = cyl(self.tip)
                a1, r1, z1 = cyl(target)
                da = a1 - a0                     # no wrap-around: the turret never turns through the back
                sd['cyl'] = (a0, r0, z0, da, r1 - r0, z1 - z0)
                sd['len'] = max(math.sqrt((0.5 * (r0 + r1) * da) ** 2 + (r1 - r0) ** 2 + (z1 - z0) ** 2), 1e-6)
                sd['t0'], sd['s'] = t, 0.0
                sd['timeout'] = sd['len'] / p.speed + p.move_margin
                sd['h0'] = self.heading
                sd['dh'] = math.atan2(math.sin(h1 - self.heading), math.cos(h1 - self.heading))
            if self.state in ('lifting', 'holding') and self._hold_failed(q, sense, sd, 'object_slipped'):
                return False
            step_len = p.speed * max(1e-3, t - sd.get('t_prev', t - 0.05))
            sd['t_prev'] = t
            sd['s'] = min(sd['len'], sd['s'] + step_len)
            prog = sd['s'] / sd['len']
            a0, r0, z0, da, dr, dz = sd['cyl']
            a, r = a0 + da * prog, r0 + dr * prog
            nxt = target if prog >= 1.0 else np.array([cx + r * math.cos(a), cy + r * math.sin(a), z0 + dz * prog])
            h = sd['h0'] + sd['dh'] * prog                     # turn the gripper smoothly along the way
            sol = ik(g, nxt, h)
            if sol is None:
                return self._fail('path_unreachable') and False
            self.tip, self.heading = nxt, h
            for j, v in zip(JOINTS, sol):
                self.cmd[j] = float(v)
            if prog >= 1.0 and self._arm_converged(q):
                return True
            if t - sd['t0'] > sd['timeout']:
                return self._fail('controller: arm_timeout (joints did not reach the commanded pose)') and False
            return False
        if kind == 'close':
            if 't0' not in sd:
                sd['t0'], sd['g_prev'], sd['t_prev'], sd['still'] = t, self.gap(q), t, 0.0
                self.cmd[FINGERS[0]] = self.cmd[FINGERS[1]] = float(arg)
                return False
            dt = max(1e-3, t - sd['t_prev'])
            vel = abs(self.gap(q) - sd['g_prev']) / dt
            sd['still'] = sd['still'] + dt if vel < p.stall_vel else 0.0
            sd['g_prev'], sd['t_prev'] = self.gap(q), t
            return sd['still'] >= p.stall_time or t - sd['t0'] >= p.close_timeout
        if kind == 'verify_grasp':
            L, R = self._contacts(sense)
            lm = {m.split('::')[0] for m in L}
            rm = {m.split('::')[0] for m in R}
            common = sorted(lm & rm)
            w = self.obj['grasp_width']
            gp = self.gap(q)
            closed_target = 2 * self.cmd[FINGERS[0]]
            self.verification = dict(contacts_left=sorted(lm), contacts_right=sorted(rm), finger_gap=round(gp, 4),
                                     expected_width=round(w, 4), grasp_verified=False, lift_verified=False,
                                     verification_source='contact+joint')
            if not common:
                return self._fail('no_finger_contact' if not (lm or rm) else 'one_sided_contact') and False
            if gp <= closed_target + 0.004:
                return self._fail('closed_on_nothing') and False
            wmin = self.obj.get('min_width', w)
            if not (wmin - p.gap_tol_lo <= gp <= w + p.gap_tol_hi):
                return self._fail(f'gap_mismatch ({gp:.3f} m, expected {wmin:.3f}-{w:.3f})') and False
            self.model = common[0]
            self.verification['grasp_verified'] = True
            self.verification['object_model'] = self.model
            if obj_pose is not None:
                pz = obj_pose(self.model)
                if pz is not None:
                    self.verification['object_z_at_grasp'] = round(pz[2], 4)
            return True
        if kind == 'verify_lift':
            ok, info = self._hold_check(q, sense)
            self.verification['hold_at_lift'] = info
            if not ok:
                if self._hold_failed(q, sense, sd, 'object_slipped'):
                    return False
                return False                                   # wait (up to hold_grace) for the check to recover
            if obj_pose is not None and 'object_z_at_grasp' in self.verification:
                pz = obj_pose(self.model)
                if pz is not None:
                    rise = pz[2] - self.verification['object_z_at_grasp']
                    self.verification['object_lift_m'] = round(rise, 4)
                    self.verification['verification_source'] = 'contact+joint+sim_pose'
                    if rise < p.min_rise:
                        return self._fail(f'object_not_lifted ({rise:.3f} m)') and False
            self.verification['lift_verified'] = True
            return True
        if kind == 'mark_tray':
            self.disposal['tray_before'] = tray_count
            return True
        if kind == 'verify_release':
            L, R = self._contacts(sense)
            touching = self.model is not None and any(m.split('::')[0] == self.model for m in L | R)
            if touching:
                sd.setdefault('t0', t)
                if t - sd['t0'] > 3.0:
                    self.disposal['reason'] = 'failed: object still in the gripper'
                    return self._fail('release_failed') and False
                return False
            sd.setdefault('free_since', t)
            if t - sd['free_since'] < p.contact_hold + 0.2:
                return False
            self.disposal['object_released'] = True
            self.verification['release_verified'] = True
            return True
        if kind == 'verify_disposal':
            sd.setdefault('t0', t)
            if t - sd['t0'] < 1.0:                               # let the camera see the tray after the arm folds
                return False
            ev = []
            before = self.disposal.get('tray_before')
            if tray_count is not None and before is not None and tray_count - before >= 20:
                ev.append('camera_tray_points')
            sim_outside = False
            if obj_pose is not None and self.model is not None:
                pz = obj_pose(self.model)
                if pz is not None and self._in_tray(pz):
                    ev.append('sim_pose_in_tray')
                elif pz is not None:
                    sim_outside = True                       # the simulator says it is NOT in the tray: never verify
            self.disposal['evidence'] = ev
            self.disposal['tray_after'] = tray_count
            if not self.disposal['station_reached']:
                self.disposal['reason'] = 'failed: released away from the station'
            elif sim_outside:
                self.disposal['reason'] = 'failed: object landed outside the tray (simulated object pose)'
            elif not ev:
                self.disposal['reason'] = 'failed: no evidence the object is in the tray'
            else:
                self.disposal['disposal_verified'] = True
            self.model = None
            return True
        raise ValueError(kind)

    tray = None                                              # (x_min, x_max, y_min, y_max) world, set by the node

    def _in_tray(self, pz):
        if self.tray is None:
            return False
        x0, x1, y0, y1 = self.tray
        return x0 <= pz[0] <= x1 and y0 <= pz[1] <= y1 and pz[2] < 0.15

    def finger_q(self):
        return self.cmd[FINGERS[0]]

    def status(self):
        q = [self.cmd[j] for j in JOINTS]
        from .arm_kinematics import crosses_scan_plane
        held = self.obj if self.state in ('lifting', 'holding', 'releasing') else None
        return dict(state=self.state, task_id=self.task_id, object_id=self.object_id, reason=self.reason,
                    verification=dict(self.verification),
                    arm_in_scan_plane=crosses_scan_plane(self.geo, q, self.finger_q(), held))
