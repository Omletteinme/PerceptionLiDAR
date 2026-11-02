#!/usr/bin/env python3
"""Day 4-5 check: drive the robot through the sewer network and compare the live LiDAR with the map.

Usage: day4_network_check.py <clear_direct|blocked_direct|blocked_alt>   (robot spawned at A=(0,0), yaw 0)

The driver steers on WHEEL ODOMETRY only. Ground truth (/ground_truth/odom) is used for evaluation only:
final position error and ray-casting the a-priori map from the true pose to compare with the live scan.
"""
import math
import sys

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

MAP = '/home/jeevi/sewer_ws/src/sewer_gazebo/maps/sewer_map'
LIDAR_X = 0.05  # lidar is 0.05 m ahead of base_link

ROUTES = {
    # J1 -> J4 along the short direct corridor
    'clear_direct': [(4.0, 0.0), (8.0, 0.0), (12.0, 0.0), (16.0, 0.0)],
    # stop before debris B1 (blocked: front-most debris surface is at x ~ 11.55)
    'blocked_direct': [(4.0, 0.0), (8.0, 0.0), (10.5, 0.0)],
    # long loop J1 -> J2 -> M -> J3 -> J4, threading the 0.6 m gap beside B2 (gap centre x = 8.2)
    'blocked_alt': [(4.0, 0.0), (8.0, 0.0), (8.2, 2.0), (8.2, 4.0), (8.0, 6.0), (12.0, 6.0),
                    (16.0, 6.0), (16.0, 3.0), (16.0, 0.0)],
}
GOAL = {'clear_direct': (16.0, 0.0), 'blocked_direct': (10.5, 0.0), 'blocked_alt': (16.0, 0.0)}


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Map:
    def __init__(self, base):
        meta = yaml.safe_load(open(base + '.yaml'))
        raw = open(base + '.pgm', 'rb').read()
        # header: P5\nW H\n255\n
        parts = raw.split(b'\n', 3)
        w, h = (int(v) for v in parts[1].split())
        self.img = np.frombuffer(parts[3], np.uint8).reshape(h, w)
        self.res, (self.x0, self.y0, _) = meta['resolution'], meta['origin']
        self.h, self.w = h, w

    def occupied(self, x, y):
        i = np.floor((x - self.x0) / self.res).astype(int)
        j = self.h - 1 - np.floor((y - self.y0) / self.res).astype(int)
        ok = (i >= 0) & (i < self.w) & (j >= 0) & (j < self.h)
        occ = np.ones(x.shape, bool)           # outside the map counts as solid
        occ[ok] = self.img[j[ok], i[ok]] < 250  # anything but free is solid
        return occ

    def raycast(self, px, py, angles, rmax=8.0, rmin=0.12, step=0.02):
        r = np.arange(rmin, rmax, step)
        xs = px + r[None, :] * np.cos(angles)[:, None]
        ys = py + r[None, :] * np.sin(angles)[:, None]
        hit = self.occupied(xs, ys)
        first = np.argmax(hit, axis=1)
        out = np.where(hit.any(axis=1), r[first], np.inf)
        return out


class Driver(Node):
    def __init__(self):
        super().__init__('day4_network_check', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.odom = self.gt = self.scan = None
        self.create_subscription(Odometry, '/odom', lambda m: setattr(self, 'odom', m), 10)
        self.create_subscription(Odometry, '/ground_truth/odom', lambda m: setattr(self, 'gt', m), 10)
        self.create_subscription(LaserScan, '/scan', lambda m: setattr(self, 'scan', m), 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.map = Map(MAP)
        self.min_range = float('inf')
        self.path_len = 0.0
        self.stuck = False

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def pose(self, m):
        p = m.pose.pose
        return p.position.x, p.position.y, yaw_of(p.orientation)

    def cmd(self, v, w):
        m = Twist()
        m.linear.x, m.angular.z = float(v), float(w)
        self.pub.publish(m)

    def goto(self, wx, wy, vmax=0.4, tol=0.15, timeout=120.0):
        t0, last = self.now(), self.pose(self.odom)
        while self.now() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.02)
            x, y, yaw = self.pose(self.odom)
            self.path_len += math.hypot(x - last[0], y - last[1])
            last = (x, y, yaw)
            if self.scan is not None:
                fin = [r for r in self.scan.ranges if math.isfinite(r)]
                if fin:
                    self.min_range = min(self.min_range, min(fin))
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
        self.stuck = True
        return False

    def settle(self, secs=0.8):
        t0 = self.now()
        while self.now() - t0 < secs:
            self.cmd(0, 0)
            rclpy.spin_once(self, timeout_sec=0.02)

    def map_match(self, tol=0.15):
        """Fraction of LiDAR rays agreeing with the a-priori map ray-cast from the TRUE pose (eval only)."""
        s = self.scan
        gx, gy, gyaw = self.pose(self.gt)
        lx, ly = gx + LIDAR_X * math.cos(gyaw), gy + LIDAR_X * math.sin(gyaw)
        ang = s.angle_min + np.arange(len(s.ranges)) * s.angle_increment
        exp = self.map.raycast(lx, ly, gyaw + ang, rmax=s.range_max)
        meas = np.array(s.ranges, float)
        meas[~np.isfinite(meas)] = np.inf
        meas[meas >= s.range_max - 0.05] = np.inf
        both_inf = np.isinf(meas) & np.isinf(exp)
        with np.errstate(invalid='ignore'):  # inf - inf
            ok = both_inf | (np.isfinite(meas) & np.isfinite(exp) & (np.abs(meas - exp) < tol))
        front = np.abs(wrap_arr(ang)) < math.radians(20)
        return ok.mean(), ok[front].mean(), exp[np.argmin(np.abs(ang))], meas[np.argmin(np.abs(ang))]


def wrap_arr(a):
    return np.arctan2(np.sin(a), np.cos(a))


def main():
    scen = sys.argv[1]
    rclpy.init()
    n = Driver()
    while n.odom is None or n.gt is None or n.scan is None:
        rclpy.spin_once(n, timeout_sec=0.2)
    t_start = n.now()
    matches, fails = [], 0

    def check(name, ok, detail):
        nonlocal fails
        print(f"{'PASS' if ok else 'FAIL'}  {name}: {detail}", flush=True)
        fails += 0 if ok else 1

    for k, (wx, wy) in enumerate(ROUTES[scen]):
        reached = n.goto(wx, wy)
        n.settle()
        gx, gy, _ = n.pose(n.gt)
        ox, oy, _ = n.pose(n.odom)
        m_all, m_front, e0, r0 = n.map_match()
        matches.append(m_all)
        print(f'  wp{k} ({wx:.1f},{wy:.1f}) reached={reached} gt=({gx:.2f},{gy:.2f}) '
              f'odom_err={math.hypot(ox - gx, oy - gy):.3f} map_match={m_all:.0%} front20={m_front:.0%} '
              f'range@0deg meas={r0:.2f} map={e0:.2f}', flush=True)
        if not reached:
            break

    gx, gy, _ = n.pose(n.gt)
    gwx, gwy = GOAL[scen]
    ox, oy, _ = n.pose(n.odom)
    print(f'INFO  sim time {n.now() - t_start:.0f} s, odom path {n.path_len:.1f} m, '
          f'closest LiDAR return {n.min_range:.2f} m, final odom drift vs gt {math.hypot(ox - gx, oy - gy):.3f} m',
          flush=True)
    check('not stuck', not n.stuck, 'all waypoints reached' if not n.stuck else 'timeout on a waypoint')
    check('final position near goal (gt)', math.hypot(gx - gwx, gy - gwy) < 0.4,
          f'gt=({gx:.2f},{gy:.2f}) goal=({gwx},{gwy})')
    if scen == 'clear_direct':
        check('world matches map (mean ray agreement >= 95%)', float(np.mean(matches)) >= 0.95,
              f'mean={np.mean(matches):.1%} min={np.min(matches):.1%}')
    elif scen == 'blocked_direct':
        m_all, m_front, e0, r0 = n.map_match()
        check('blockage B1 visible ahead (range << map)', r0 < 1.5 and e0 > 4.0, f'measured={r0:.2f} map predicts={e0:.2f}')
        check('blockage = mismatch in forward sector', m_front < 0.6, f'front +-20deg agreement={m_front:.0%}')
    else:
        check('world mostly matches map on the loop (>= 85%)', float(np.mean(matches)) >= 0.85,
              f'mean={np.mean(matches):.1%} min={np.min(matches):.1%}')
    n.cmd(0, 0)
    print(f'RESULT  {"ALL PASS" if fails == 0 else str(fails) + " FAILED"}', flush=True)
    rclpy.shutdown()
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
