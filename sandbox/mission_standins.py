"""SANDBOX-ONLY STAND-INS for Account 2's perception and manipulation. TEST DOUBLES, NOT ALGORITHMS.

Account 2 owns debris/defect perception and the arm/gripper. Until that work exists, these stand-ins let the Mission
Manager (Account 1) be tested end to end in the 2D sandbox. They play the role of the SIMULATOR plus an idealised
Account-2 stack, so they read the scenario's ground truth, exactly like the sandbox's LiDAR ray-caster does:
  - StandInPerception reports objects/defects within sensor range and line of sight, with noise, as JSON in the
    PROPOSED /perception/debris and /perception/defects format (docs/mission_interfaces.md);
  - StandInManipulator answers PICKUP / DISPOSE tasks: it only succeeds when the TRUE robot is within reach and the
    object physically fits, needs time, can be told to fail, and moves the object with the robot while held. Its
    `verified` flag mirrors the physical state (held / inside the disposal zone), not the request.
None of this is used by the ROS stack. The Mission Manager only ever sees the JSON messages.
"""
import json
import math
from dataclasses import dataclass, field

import numpy as np


@dataclass
class TrueDebris:
    id: str
    x: float
    y: float
    size: tuple                  # (length, width, height)
    category: str = 'debris'
    obstruction: float = 0.1
    held: bool = False
    disposed: bool = False


@dataclass
class TrueDefect:
    id: str
    type: str
    x: float
    y: float
    severity: float


@dataclass
class Scenario:
    debris: list = field(default_factory=list)
    defects: list = field(default_factory=list)


def scenario_v2(truth=None):
    """Debris and defects for the v2 network (sandbox only; the Gazebo world does not contain them yet).

    S1-S3 fit the PROPOSED gripper; L1 is the largest loose object but too wide to grasp; the blockage piles from the
    world truth (B1, B2, B3) are added as large rubble objects.
    """
    sc = Scenario(
        debris=[TrueDebris('S1', 4.0, 0.30, (0.14, 0.10, 0.08), 'brick', 0.10),
                TrueDebris('S2', 12.30, 8.0, (0.12, 0.12, 0.10), 'bottle_pack', 0.15),
                TrueDebris('S3', 8.30, -3.0, (0.06, 0.05, 0.04), 'can', 0.05),
                TrueDebris('L1', 14.0, 9.65, (0.35, 0.22, 0.20), 'tyre_piece', 0.35)],
        defects=[TrueDefect('C1', 'crack', 6.0, -0.5, 0.4), TrueDefect('H1', 'hole', 16.5, 3.0, 0.8),
                 TrueDefect('J1d', 'damaged_joint', 7.5, 2.0, 0.6), TrueDefect('F1', 'deformation', 11.5, 9.0, 0.5)])
    for name, b in (truth or {}).get('blockages', {}).items():
        xs, ys = [], []
        for p in b['pieces']:
            hx, hy = (p[3] / 2, p[4] / 2) if p[0] == 'box' else (p[3], p[3])
            xs += [p[1] - hx, p[1] + hx]
            ys += [p[2] - hy, p[2] + hy]
        sc.debris.append(TrueDebris(name, 0.5 * (min(xs) + max(xs)), 0.5 * (min(ys) + max(ys)),
                                    (max(xs) - min(xs), max(ys) - min(ys), 0.5), 'rubble_pile',
                                    1.0 if b.get('blocks_route') else 0.5))
    return sc


class StandInPerception:
    def __init__(self, scenario, true_map, rng, rng_range=3.0, defect_range=2.0, pos_sigma=0.03, size_sigma=0.01):
        self.sc, self.map, self.rng = scenario, true_map, rng
        self.r_debris, self.r_defect = rng_range, defect_range
        self.pos_sigma, self.size_sigma = pos_sigma, size_sigma

    def _visible(self, rx, ry, x, y, rmax, half=0.0):
        d = math.hypot(x - rx, y - ry)
        if d > rmax:
            return False
        if d < 0.15:
            return True
        hit = self.map.raycast(rx, ry, [math.atan2(y - ry, x - rx)], d, 0.0)[0]
        return not np.isfinite(hit) or hit >= d - half - 0.1    # the object's own surface may be in the map

    def debris_json(self, now, rx, ry):
        objs = []
        for d in self.sc.debris:
            if d.held or d.disposed or not self._visible(rx, ry, d.x, d.y, self.r_debris, 0.5 * max(d.size[:2])):
                continue
            n = self.rng.normal
            objs.append(dict(id=d.id, x=d.x + n(0, self.pos_sigma), y=d.y + n(0, self.pos_sigma), z=0.0,
                             size=[max(0.01, s + n(0, self.size_sigma)) for s in d.size],
                             size_sigma=[self.size_sigma] * 3, pos_sigma=self.pos_sigma, category=d.category,
                             confidence=0.9, obstruction=d.obstruction, stamp=now))
        return json.dumps(dict(frame_id='map', stamp=now, objects=objs))

    def defects_json(self, now, rx, ry):
        out = [dict(id=d.id, type=d.type, x=d.x, y=d.y, severity=d.severity, confidence=0.85, stamp=now,
                    evidence=f'sandbox_frame_{now:.1f}')
               for d in self.sc.defects if math.hypot(d.x - rx, d.y - ry) <= self.r_defect]
        return json.dumps(dict(frame_id='map', stamp=now, defects=out))


class StandInManipulator:
    """Answers /mission/task with /manipulation/status and /disposal/status JSON. Physical checks use the TRUE pose."""

    def __init__(self, scenario, station_xy, reach=0.65, max_width=0.15, grasp_time=3.0, release_time=2.0,
                 station_radius=0.6, fail=None):
        self.sc, self.station = scenario, np.asarray(station_xy, float)
        self.reach, self.max_width = reach, max_width
        self.grasp_time, self.release_time, self.station_radius = grasp_time, release_time, station_radius
        self.fail = dict(fail or {})            # object id -> number of grasp attempts that fail
        self.state, self.obj, self.task, self.t_busy = 'IDLE', None, None, None
        self.reason, self.disposal = '', dict(object_id='', station_reached=False, released=False, verified=False,
                                              reason='', task_id='')

    def on_task(self, task, now):
        self.task, self.t_busy, self.reason = task, now, ''
        if task.type == 'PICKUP':
            self.state = 'GRASPING'
        elif task.type == 'DISPOSE':
            self.state = 'RELEASING'
            self.disposal = dict(object_id=task.object_id, station_reached=False, released=False, verified=False,
                                 reason='', task_id=task.task_id)
        elif task.type == 'STOP':
            self.state = 'IDLE' if self.obj is None else 'HOLDING'

    def step(self, now, rx, ry, rth):
        if self.obj is not None:                       # a held object travels with the gripper
            self.obj.x, self.obj.y = rx + 0.25 * math.cos(rth), ry + 0.25 * math.sin(rth)
        t = self.task
        if self.state == 'GRASPING' and now - self.t_busy >= self.grasp_time:
            d = next((o for o in self.sc.debris if o.id == t.object_id), None)
            if d is None or d.disposed:
                self.state, self.reason = 'FAILED', 'no such object at the target'
            elif math.hypot(d.x - rx, d.y - ry) > self.reach:
                self.state, self.reason = 'FAILED', f'out of reach ({math.hypot(d.x - rx, d.y - ry):.2f} m)'
            elif sorted(d.size[:2])[0] > self.max_width:
                self.state, self.reason = 'FAILED', 'object wider than the gripper'
            elif self.fail.get(d.id, 0) > 0:
                self.fail[d.id] -= 1
                self.state, self.reason = 'FAILED', 'grasp slipped (no contact on lift)'
            else:
                d.held, self.obj, self.state = True, d, 'HOLDING'
        if self.state == 'RELEASING' and now - self.t_busy >= self.release_time:
            at = math.hypot(self.station[0] - rx, self.station[1] - ry) <= self.station_radius
            self.disposal['station_reached'] = bool(at)
            if self.obj is not None and at:
                self.obj.held, self.obj.disposed = False, True
                self.obj.x, self.obj.y = float(self.station[0]), float(self.station[1])
                self.obj, self.state = None, 'RELEASED'
                self.disposal.update(released=True, verified=True)
            else:
                self.state = 'HOLDING' if self.obj is not None else 'IDLE'
                self.disposal.update(reason='failed: not inside the disposal zone' if not at else 'failed: nothing held')

    def manipulation_json(self, now):
        oid = self.obj.id if self.obj is not None else (self.task.object_id if self.task else '')
        return json.dumps(dict(state=self.state, stamp=now, object_id=oid, verified=self.state == 'HOLDING' and
                               self.obj is not None and self.obj.held, reason=self.reason,
                               task_id=self.task.task_id if self.task else ''))

    def disposal_json(self, now):
        return json.dumps(dict(self.disposal, stamp=now))
