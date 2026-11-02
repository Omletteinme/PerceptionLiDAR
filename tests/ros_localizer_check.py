#!/usr/bin/env python3
"""Live check of the ROS localizers against the Gazebo sewer network (loop route A -> ... -> J4).

Expects robot_sim.launch.py (world sewer_network.sdf, spawn x=0 y=0 yaw=0) and
localization.launch.py filter:=both already running. The driver steers on WHEEL ODOMETRY only. Ground truth
(/ground_truth/odom) is used for evaluation only, never by the localizers.

Usage: ros_localizer_check.py <max_rmse_m>
"""
import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformListener

from sewer_localization.disturbances import BASE_SCHEDULE

ROUTE = [(4.0, 0.0), (8.0, 0.0), (8.2, 2.0), (8.2, 4.0), (8.0, 6.0), (12.0, 6.0),
         (16.0, 6.0), (16.0, 3.0), (16.0, 0.0)]
WINDOWS = sorted({(e['t0'], e['t1']) for e in BASE_SCHEDULE})


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Check(Node):
    def __init__(self):
        super().__init__('ros_localizer_check', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.odom = self.gt = self.est_b = self.est_r = self.R = None
        self.n_est = {'b': 0, 'r': 0}
        self.t_move = None
        self.rows = []
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(Odometry, '/ground_truth/odom', lambda m: setattr(self, 'gt', m), 10)
        self.create_subscription(PoseWithCovarianceStamped, '/baseline/estimated_pose', self.on_b, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/reliability/estimated_pose', self.on_r, 10)
        self.create_subscription(Float32, '/reliability/scan_reliability_R', lambda m: setattr(self, 'R', m.data), 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tfbuf = Buffer()
        self.tfl = TransformListener(self.tfbuf, self)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, m):
        self.odom = m
        if self.t_move is None and abs(m.twist.twist.linear.x) > 0.05:
            self.t_move = self.now()

    def on_b(self, m):
        self.est_b = m
        self.n_est['b'] += 1

    def on_r(self, m):
        self.est_r = m
        self.n_est['r'] += 1

    @staticmethod
    def xyz(m):
        p = m.pose.pose
        return p.position.x, p.position.y, yaw_of(p.orientation)

    def step(self):
        rclpy.spin_once(self, timeout_sec=0.02)
        if None not in (self.gt, self.odom, self.est_b, self.est_r, self.R):
            self.rows.append((self.now(), *self.xyz(self.gt), *self.xyz(self.odom),
                              *self.xyz(self.est_b), *self.xyz(self.est_r), self.R))

    def cmd(self, v, w):
        m = Twist()
        m.linear.x, m.angular.z = float(v), float(w)
        self.pub.publish(m)

    def goto(self, wx, wy, vmax=0.4, tol=0.15, timeout=120.0):
        t0 = self.now()
        while self.now() - t0 < timeout:
            self.step()
            x, y, yaw = self.xyz(self.odom)
            dx, dy = wx - x, wy - y
            dist = math.hypot(dx, dy)
            if dist < tol:
                self.cmd(0, 0)
                return True
            err = wrap(math.atan2(dy, dx) - yaw)
            if abs(err) > 0.5:
                self.cmd(0.0, max(-1.0, min(1.0, 1.5 * err)))
            else:
                self.cmd(min(vmax, 0.5 * dist + 0.1) * math.cos(err), max(-1.0, min(1.0, 2.0 * err)))
        self.cmd(0, 0)
        return False

    def settle(self, secs):
        t0 = self.now()
        while self.now() - t0 < secs:
            self.cmd(0, 0)
            self.step()


def main():
    max_rmse = float(sys.argv[1]) if len(sys.argv) > 1 else 0.3
    rclpy.init()
    n = Check()
    t_wait = time.monotonic()
    while any(v is None for v in (n.odom, n.gt, n.est_b, n.est_r, n.R)):
        n.step()
        if time.monotonic() - t_wait > 90:   # wall clock: DDS discovery can be slow under WSL
            missing = [k for k in ('odom', 'gt', 'est_b', 'est_r', 'R') if getattr(n, k) is None]
            print(f'FAIL  no data from: {missing}')
            return 1
    n.settle(1.0)
    n.rows.clear()
    n.n_est = {'b': 0, 'r': 0}
    t_start = n.now()
    reached = all(n.goto(wx, wy) for wx, wy in ROUTE)
    n.settle(1.5)
    dur = n.now() - t_start
    fails = 0

    def check(name, ok, detail):
        nonlocal fails
        print(f"{'PASS' if ok else 'FAIL'}  {name}: {detail}", flush=True)
        fails += 0 if ok else 1

    a = np.array(n.rows)
    t = a[:, 0] - (n.t_move if n.t_move is not None else a[0, 0])
    gt, od, eb, er, R = a[:, 1:4], a[:, 4:7], a[:, 7:10], a[:, 10:13], a[:, 13]
    err = {k: np.hypot(v[:, 0] - gt[:, 0], v[:, 1] - gt[:, 1]) for k, v in
           (('wheel odom', od), ('baseline', eb), ('reliability', er))}
    inw = np.zeros(len(t), bool)
    for t0, t1 in WINDOWS:
        inw |= (t >= t0) & (t < t1)
    check('route completed', reached, f'{dur:.0f} sim s')
    print('  estimator      rmse   max   final | rmse in windows / outside')
    for k, e in err.items():
        print(f'  {k:12s} {np.sqrt((e ** 2).mean()):6.3f} {e.max():5.2f} {e[-1]:6.3f} | '
              f'{np.sqrt((e[inw] ** 2).mean()) if inw.any() else float("nan"):.3f} / {np.sqrt((e[~inw] ** 2).mean()):.3f}')
    print(f'  scan reliability R: mean in windows {R[inw].mean() if inw.any() else float("nan"):.2f}, '
          f'outside {R[~inw].mean():.2f}, min {R.min():.2f}')
    for k in ('baseline', 'reliability'):
        e = err[k]
        check(f'{k} rmse < {max_rmse} m', np.sqrt((e ** 2).mean()) < max_rmse, f'{np.sqrt((e ** 2).mean()):.3f} m')
    # TF: map -> base_footprint through map->odom (localizer) and odom->base_footprint (Gazebo)
    try:
        tf = n.tfbuf.lookup_transform('map', 'base_footprint', Time())
        d = math.hypot(tf.transform.translation.x - er[-1, 0], tf.transform.translation.y - er[-1, 1])
        check('TF map->base_footprint matches /reliability estimate', d < 0.05, f'diff {d:.3f} m')
    except Exception as ex:  # noqa: BLE001
        check('TF map->base_footprint available', False, str(ex)[:120])
    rate_b, rate_r = n.n_est['b'] / dur, n.n_est['r'] / dur
    check('estimate publish rate ~10 Hz', 8 < rate_b < 12 and 8 < rate_r < 12, f'baseline {rate_b:.1f} Hz, reliability {rate_r:.1f} Hz')
    n.cmd(0, 0)
    print(f'RESULT  {"ALL PASS" if fails == 0 else str(fails) + " FAILED"}', flush=True)
    rclpy.shutdown()
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
