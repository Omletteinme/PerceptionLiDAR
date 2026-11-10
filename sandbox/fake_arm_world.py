"""A tiny kinematic stand-in for Gazebo's arm physics, to test the manipulation controller offline (CUSTOM, tests only).

It is NOT a physics simulation: joints move toward their commands at a fixed rate, fingers stop at the object's width
when the object is between them, and both finger 'contact sensors' report the object while it is squeezed. A held
object follows the finger centre; released, it falls straight down. Options create the failure cases (no object,
slipping, one-sided contact). The real behaviour (friction, contact noise) must be checked in Gazebo.
"""
import math
import os
import sys

import numpy as np

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(_ROOT, 'src', 'sewer_manipulation'))
from sewer_manipulation.arm_kinematics import FINGERS, JOINTS, fk  # noqa: E402
from sewer_manipulation.manipulation_controller import ManipulationController  # noqa: E402

TRAY = (-0.44, -0.24, -0.15, 0.15)


class FakeArmWorld:
    def __init__(self, robot=(2.12, 0.28, 0.0), obj=(2.5, 0.28), size=(0.06, 0.06, 0.06), present=True,
                 slip_after=None, one_sided=False, dt=0.05, flicker=0):
        self.ctl = ManipulationController()
        self.ctl.tray = TRAY
        self.geo = self.ctl.geo
        self.robot = list(robot)
        self.obj = dict(x=obj[0], y=obj[1], z=size[2] / 2, w=size[1], l=size[0], h=size[2], present=present)
        self.obj_yaw = 0.0                                  # object's long-side direction in the world
        self.q = {j: 0.0 for j in JOINTS}
        self.q.update({f: 0.03 for f in FINGERS})
        self.sense = {'left': {}, 'right': {}}
        self.t, self.dt = 0.0, dt
        self.slip_after, self.one_sided = slip_after, one_sided
        self.flicker, self.k = flicker, 0   # flicker=N: contact messages only on every N-th step (like Gazebo)
        self.tray_count = 0
        self.states = []
        self.held_since = None

    def base_of(self, px, py):
        c, s = math.cos(self.robot[2]), math.sin(self.robot[2])
        dx, dy = px - self.robot[0], py - self.robot[1]
        return c * dx + s * dy, -s * dx + c * dy

    def world_of(self, bx, by):
        c, s = math.cos(self.robot[2]), math.sin(self.robot[2])
        return self.robot[0] + c * bx - s * by, self.robot[1] + s * bx + c * by

    def obj_in_base(self):
        bx, by = self.base_of(self.obj['x'], self.obj['y'])
        return dict(x=bx, y=by, yaw=self.obj_yaw - self.robot[2], width=self.obj['w'], length=self.obj['l'],
                    height=self.obj['h'])

    def tray_in_base(self):
        x0, x1, y0, y1 = TRAY
        return self.base_of((x0 + x1) / 2, (y0 + y1) / 2)

    def obj_pose(self, model):
        return (self.obj['x'], self.obj['y'], self.obj['z']) if model == 'debris_S1' else None

    def move_robot(self, pose):
        """Drive the base (the Mission Manager's job); a held object rides along with the gripper."""
        tip = fk(self.geo, [self.q[j] for j in JOINTS])['tip']
        self.robot = list(pose)
        if self.held_since is not None:
            self.obj['x'], self.obj['y'] = self.world_of(tip[0], tip[1])

    def step(self, n=1):
        for _ in range(n):
            self.t += self.dt
            cmd = self.ctl.tick(self.t, self.q, self.sense, self.obj_pose, self.tray_count)
            for j in JOINTS:
                self.q[j] += float(np.clip(cmd[j] - self.q[j], -1.5 * self.dt, 1.5 * self.dt))
            tip = fk(self.geo, [self.q[j] for j in JOINTS])['tip']
            ox, oy = self.base_of(self.obj['x'], self.obj['y'])
            between = (self.obj['present'] and abs(tip[0] - ox) < 0.03 and abs(tip[1] - oy) < 0.03
                       and abs(tip[2] + self.geo.base_z - self.obj['z']) < 0.03)
            for f in FINGERS:
                nq = self.q[f] + float(np.clip(cmd[f] - self.q[f], -0.1 * self.dt, 0.1 * self.dt))
                if between and nq < min(self.obj['w'], self.obj['l']) / 2:    # squeezed on its narrow side
                    nq = min(self.obj['w'], self.obj['l']) / 2
                self.q[f] = nq
            squeezed = between and all(self.q[f] <= min(self.obj['w'], self.obj['l']) / 2 + 1e-3 for f in FINGERS)
            if squeezed and self.held_since is None:
                self.held_since = self.t
            slipped = self.slip_after is not None and self.held_since is not None and \
                self.t - self.held_since > self.slip_after
            touching = squeezed and not slipped
            self.k += 1
            if touching:
                self.obj['x'], self.obj['y'] = self.world_of(tip[0], tip[1])
                self.obj['z'] = tip[2] + self.geo.base_z
                if self.flicker and self.k % self.flicker:
                    continue
                for side in (('left',) if self.one_sided else ('left', 'right')):
                    self.sense[side]['debris_S1'] = self.t
            else:
                if self.held_since is not None and slipped:
                    self.obj['present'] = False                         # fell out of reach
                self.held_since = None if not touching else self.held_since
                if self.obj['z'] > self.obj['h'] / 2 + 1e-3:
                    self.obj['z'] = self.obj['h'] / 2
                    x0, x1, y0, y1 = TRAY
                    if x0 < self.obj['x'] < x1 and y0 < self.obj['y'] < y1:
                        self.tray_count = 150                           # the depth camera would now see it
            if not self.states or self.states[-1] != self.ctl.state:
                self.states.append(self.ctl.state)

    def run_until(self, states, max_s=60.0):
        n = int(max_s / self.dt)
        for _ in range(n):
            self.step()
            if self.ctl.state in states and not self.ctl.steps:
                return True
        return False
