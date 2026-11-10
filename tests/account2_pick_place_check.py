#!/usr/bin/env python3
"""Live Gazebo check of the Account 2 pipeline: detect -> pick up -> carry -> drop in the tray -> verify (TEST DRIVER).

This script stands in for Account 1's Mission Manager so Account 2's part can be verified alone. It behaves like it:
it stops on the corridor centre line 0.25 m before the object's near face, facing along the corridor, sends the
documented /mission/task messages (PICKUP, then DISPOSE at the entrance node A), and accepts success only from the
`verified` flags (docs/mission_interfaces.md). Driving uses a simple waypoint controller on the ESTIMATED pose (a
test driver, not a product feature). Ground truth is only printed.

Steps: debris near S1 (2.5, 0.28) confirmed and graspable -> stand-off point -> PICKUP -> HOLDING + verified ->
drive to A -> DISPOSE -> disposal released + verified. Also writes the final /perception/* messages to out_dir.
Usage: account2_pick_place_check.py <timeout_s> <out_dir>
"""
import json
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import String

TARGET = (2.5, 0.28)          # where S1 is in the world (used only to pick which detection to test)
STATION = (0.05, 0.0, math.pi)       # within the Mission Manager's 0.25 m of the entrance node A, facing the end wall


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Check(Node):
    def __init__(self, out_dir):
        super().__init__('account2_check')
        self.out = out_dir
        self.est = self.true = None
        self.debris = self.defects = None
        self.manip = {}
        self.disposal = {}
        self.last_task = ''
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.task = self.create_publisher(String, '/mission/task',
                                          QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(PoseWithCovarianceStamped, '/reliability/estimated_pose', self.on_est,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, '/ground_truth/odom', self.on_true, qos_profile_sensor_data)
        self.create_subscription(String, '/perception/debris', lambda m: setattr(self, 'debris', json.loads(m.data)), 10)
        self.create_subscription(String, '/perception/defects', lambda m: setattr(self, 'defects', json.loads(m.data)), 10)
        self.create_subscription(String, '/manipulation/status', lambda m: setattr(self, 'manip', json.loads(m.data)), 10)
        self.create_subscription(String, '/disposal/status', lambda m: setattr(self, 'disposal', json.loads(m.data)), 10)

    def on_est(self, m):
        p = m.pose.pose
        q = p.orientation
        self.est = (p.position.x, p.position.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def on_true(self, m):
        p = m.pose.pose
        q = p.orientation
        self.true = (p.position.x, p.position.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def spin_for(self, s):
        t0 = time.time()
        while time.time() - t0 < s:
            rclpy.spin_once(self, timeout_sec=0.05)

    def stop(self):
        self.cmd.publish(Twist())

    def drive_to(self, x, y, tol=0.04, timeout=90):
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.est is None:
                continue
            ex, ey, th = self.est
            dx, dy = x - ex, y - ey
            d = math.hypot(dx, dy)
            if d < tol:
                self.stop()
                return True
            err = wrap(math.atan2(dy, dx) - th)
            tw = Twist()
            if abs(err) > 0.35:
                tw.angular.z = max(-0.6, min(0.6, 1.5 * err))
            else:
                tw.linear.x = min(0.15, 0.6 * d + 0.03)
                tw.angular.z = max(-0.6, min(0.6, 2.0 * err))
            self.cmd.publish(tw)
        self.stop()
        return False

    def turn_to(self, yaw, tol=0.03, timeout=30):
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.est is None:
                continue
            err = wrap(yaw - self.est[2])
            if abs(err) < tol:
                self.stop()
                return True
            tw = Twist()
            tw.angular.z = max(-0.5, min(0.5, 1.5 * err)) if abs(err) > 0.1 else 0.15 * math.copysign(1, err)
            self.cmd.publish(tw)
        self.stop()
        return False

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def send(self, typ, tid, obj=None, target=None, action=''):
        """A /mission/task exactly as the Mission Manager publishes it (MissionTask.to_json)."""
        m = dict(task_id=tid, type=typ, stamp=self.now(), object_id=obj or '', target=target or {}, action=action)
        self.task.publish(String(data=json.dumps(m)))

    def wait_state(self, states, timeout):
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.manip.get('state') in states and self.manip.get('task_id') == self.last_task:
                return self.manip
        return self.manip

    def dump(self):
        for name, msg in (('debris', self.debris), ('defects', self.defects)):
            if msg is not None:
                with open(os.path.join(self.out, f'final_{name}.json'), 'w') as f:
                    json.dump(msg, f, indent=1)


def main():
    timeout = float(sys.argv[1]) if len(sys.argv) > 1 else 400
    out = sys.argv[2] if len(sys.argv) > 2 else os.path.expanduser('~/sewer_ws/logs/account2')
    os.makedirs(out, exist_ok=True)
    rclpy.init()
    n = Check(out)
    t_start = time.time()
    result = dict(steps=[])

    def step(name, ok, **info):
        result['steps'].append(dict(step=name, ok=bool(ok), t=round(time.time() - t_start, 1), **info))
        print(f"[{'PASS' if ok else 'FAIL'}] {name} {info}", flush=True)
        return ok

    try:
        n.spin_for(5)
        # 1. drive slowly up the corridor until the object near S1 is confirmed and graspable
        obj = None
        for wx in (0.6, 1.0, 1.4):
            n.drive_to(wx, 0.0)
            n.spin_for(3)
            for o in (n.debris or {}).get('objects', []):
                if math.hypot(o['x'] - TARGET[0], o['y'] - TARGET[1]) < 0.3:
                    obj = o
            if obj and obj['graspable']:
                break
        if not step('detect_S1', obj is not None and obj['graspable'],
                    obj=None if obj is None else {k: obj[k] for k in ('id', 'x', 'y', 'yaw', 'size', 'graspable',
                                                                       'confidence', 'feasibility')}):
            raise SystemExit
        # 2. stand-off point like the Mission Manager: centre line, 0.25 m before the near face, facing +x
        sx = obj['x'] - 0.5 * max(obj['size'][:2]) - 0.25
        ok = n.drive_to(sx, 0.0, tol=0.05) and n.turn_to(0.0)
        step('reach_stand_off', ok, est=n.est, true=n.true, stand_off=(round(sx, 3), 0.0))
        n.spin_for(2)
        # 3. PICKUP
        n.last_task = 'T1'
        n.send('PICKUP', 'T1', obj['id'], dict(x=obj['x'], y=obj['y'], yaw=0.0, frame_id='map'), 'grasp_and_lift')
        st = n.wait_state(('HOLDING', 'FAILED'), 60)
        if not step('pickup_verified', st.get('state') == 'HOLDING' and st.get('verified'),
                    state=st.get('state'), reason=st.get('reason'), verification=st.get('verification')):
            n.spin_for(15)
            raise SystemExit
        # 4. carry to the entrance node A
        n.spin_for(1)
        ok = n.drive_to(1.0, 0.0) and n.drive_to(STATION[0] + 0.3, 0.0) and n.turn_to(STATION[2]) and \
            n.drive_to(STATION[0], STATION[1], tol=0.05) and n.turn_to(STATION[2])
        st = n.wait_state(('HOLDING', 'FAILED'), 1)
        step('carry_to_station', ok and st.get('state') == 'HOLDING', state=st.get('state'), est=n.est, true=n.true)
        n.spin_for(2)
        # 5. DISPOSE + verification
        n.last_task = 'T2'
        n.send('DISPOSE', 'T2', obj['id'], dict(x=0.0, y=0.0, frame_id='map'), 'release_in_zone')
        st = n.wait_state(('RELEASED', 'FAILED'), 60)
        n.spin_for(6)
        d = n.disposal
        step('disposal_verified', st.get('state') == 'RELEASED' and d.get('released') and d.get('verified'),
             state=st.get('state'), reason=st.get('reason'), disposal=d)
    except SystemExit:
        pass
    finally:
        n.stop()
        while time.time() - t_start < min(timeout, 30):
            rclpy.spin_once(n, timeout_sec=0.1)
        n.spin_for(2)
        n.dump()
        result['all_passed'] = all(s['ok'] for s in result['steps']) and len(result['steps']) == 5
        with open(os.path.join(out, 'pick_place_result.json'), 'w') as f:
            json.dump(result, f, indent=1)
        print('RESULT', 'PASS' if result['all_passed'] else 'FAIL', flush=True)
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
