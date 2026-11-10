#!/usr/bin/env python3
"""Watch a full Gazebo mission (Account 1's Mission Manager + Account 2's nodes) and record the outcome (EVALUATION).

Waits until /mission/state reaches MISSION_COMPLETE or SAFE_STOP (or the timeout), then writes to out_dir:
  mission_report.json     the latched /mission/report
  final_debris.json, final_defects.json   the last /perception/* messages (score with sandbox/inspection_scoring.py)
  mission_check.json      states seen, tasks, every /manipulation/status and /disposal/status change, and which
                          debris models really lie in the tray at the end (from /sim/dynamic_poses: SIMULATED STATE,
                          evaluation only)
Usage: account2_mission_check.py <timeout_s> <out_dir>
"""
import json
import os
import sys
import time

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

TERMINAL = ('MISSION_COMPLETE', 'SAFE_STOP')


class Watch(Node):
    def __init__(self):
        super().__init__('account2_mission_check')
        latched = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state, self.report, self.debris, self.defects = None, None, None, None
        self.states, self.tasks, self.manip, self.disposal, self.sim = [], [], [], [], {}
        self.create_subscription(String, '/mission/state', self.on_state, 10)
        self.create_subscription(String, '/mission/report', lambda m: setattr(self, 'report', json.loads(m.data)),
                                 latched)
        self.create_subscription(String, '/mission/task', lambda m: self.tasks.append(json.loads(m.data)), latched)
        self.create_subscription(String, '/perception/debris', lambda m: setattr(self, 'debris', json.loads(m.data)), 10)
        self.create_subscription(String, '/perception/defects', lambda m: setattr(self, 'defects', json.loads(m.data)), 10)
        self.create_subscription(String, '/manipulation/status', lambda m: self._change(self.manip, m), 10)
        self.create_subscription(String, '/disposal/status', lambda m: self._change(self.disposal, m), 10)
        try:
            from tf2_msgs.msg import TFMessage
            self.create_subscription(TFMessage, '/sim/dynamic_poses', self.on_sim, 10)
        except ImportError:
            pass

    def on_state(self, m):
        s = json.loads(m.data)
        self.state = s.get('state')
        if not self.states or self.states[-1][0] != self.state:
            self.states.append((self.state, s.get('reason', '')))
            print(f"[mission] {self.state}: {s.get('reason', '')}", flush=True)

    def _change(self, log, m):
        s = json.loads(m.data)
        key = {k: s.get(k) for k in ('state', 'object_id', 'verified', 'task_id', 'reason', 'released')}
        if not log or {k: log[-1].get(k) for k in key} != key:
            log.append(s)
            print(f"[arm] {json.dumps(key)}", flush=True)

    def on_sim(self, m):
        for tr in m.transforms:
            name = tr.child_frame_id.split('::')[-1]
            if name.startswith('debris_'):
                v = tr.transform.translation
                self.sim[name] = (v.x, v.y, v.z)


def main():
    timeout = float(sys.argv[1]) if len(sys.argv) > 1 else 1800
    out = sys.argv[2] if len(sys.argv) > 2 else os.path.expanduser('~/sewer_ws/logs/account2')
    os.makedirs(out, exist_ok=True)
    rclpy.init()
    n = Watch()
    t0 = time.time()
    while time.time() - t0 < timeout and n.state not in TERMINAL:
        rclpy.spin_once(n, timeout_sec=0.2)
    end = time.time()
    while time.time() - end < 5:                       # final report and messages
        rclpy.spin_once(n, timeout_sec=0.2)
    station = None
    try:
        from ament_index_python.packages import get_package_share_directory
        with open(os.path.join(get_package_share_directory('sewer_gazebo'), 'config', 'inspection_station.yaml')) as f:
            station = yaml.safe_load(f)['tray']
    except Exception:  # noqa: BLE001
        pass
    in_tray = sorted(k for k, (x, y, z) in n.sim.items() if station and station['x_min'] <= x <= station['x_max']
                     and station['y_min'] <= y <= station['y_max'])
    for name, msg in (('mission_report', n.report), ('final_debris', n.debris), ('final_defects', n.defects)):
        if msg is not None:
            with open(os.path.join(out, name + '.json'), 'w') as f:
                json.dump(msg, f, indent=1)
    res = dict(final_state=n.state, wall_time_s=round(end - t0, 1), states=n.states, tasks=n.tasks,
               manipulation=n.manip, disposal=n.disposal, debris_in_tray_sim_state=in_tray,
               sim_pose_topic_seen=bool(n.sim))
    with open(os.path.join(out, 'mission_check.json'), 'w') as f:
        json.dump(res, f, indent=1, default=str)
    r = n.report or {}
    print('\nfinal state:', n.state, '| wall time', round(end - t0), 's')
    print('removed (mission report):', [d.get('id') for d in (r.get('debris', {}) or {}).get('removed', [])]
          if isinstance((r.get('debris', {}) or {}).get('removed'), list) else (r.get('debris', {}) or {}).get('removed'))
    print('failed:', (r.get('debris', {}) or {}).get('failed'))
    print('really in the tray (Gazebo state):', in_tray if n.sim else 'unknown (/sim/dynamic_poses not received)')
    print('coverage:', (r.get('coverage') or {}).get('coverage'))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
