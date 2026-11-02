#!/usr/bin/env python3
"""Day 2-3 robot check against the live Gazebo sim (test_arena, robot spawned at x=-2, y=0, yaw=0).

Uses the ground-truth topic for EVALUATION ONLY. Prints PASS/FAIL lines and exits non-zero on failure.
"""
import math
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Check(Node):
    def __init__(self):
        super().__init__('day2_robot_check', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.odom = self.gt = self.scan = None
        self.imu_az = []
        self.create_subscription(Odometry, '/odom', lambda m: setattr(self, 'odom', m), 10)
        self.create_subscription(Odometry, '/ground_truth/odom', lambda m: setattr(self, 'gt', m), 10)
        self.create_subscription(LaserScan, '/scan', lambda m: setattr(self, 'scan', m), 10)
        self.create_subscription(Imu, '/imu', lambda m: self.imu_az.append(m.linear_acceleration.z), 100)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.fails = 0

    def check(self, name, ok, detail):
        print(f"{'PASS' if ok else 'FAIL'}  {name}: {detail}", flush=True)
        self.fails += 0 if ok else 1

    def spin_for(self, sim_seconds, v=0.0, w=0.0):
        t0 = self.get_clock().now()
        while (self.get_clock().now() - t0).nanoseconds * 1e-9 < sim_seconds:
            msg = Twist()
            msg.linear.x, msg.angular.z = v, w
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.02)

    def pose(self, m):
        return m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation)

    def range_at(self, angle):
        s = self.scan
        i = int(round((angle - s.angle_min) / s.angle_increment)) % len(s.ranges)
        r = [s.ranges[(i + k) % len(s.ranges)] for k in (-2, -1, 0, 1, 2)]
        return float(np.median(r))


def main():
    rclpy.init()
    n = Check()
    while n.odom is None or n.gt is None or n.scan is None or not n.imu_az:
        rclpy.spin_once(n, timeout_sec=0.2)

    # --- 1. At rest: IMU gravity, upright, LiDAR vs known wall distances ---
    n.imu_az.clear()
    n.spin_for(3.0)
    az = float(np.mean(n.imu_az))
    n.check('IMU az at rest ~ +9.8', abs(az - 9.8) < 0.3, f'mean az={az:.3f} (n={len(n.imu_az)})')
    gx, gy, gyaw = n.pose(n.gt)
    n.check('start pose', abs(gx + 2.0) < 0.05 and abs(gy) < 0.05, f'gt=({gx:.3f},{gy:.3f})')
    q = n.gt.pose.pose.orientation
    n.check('upright', abs(q.x) < 0.02 and abs(q.y) < 0.02, f'qx={q.x:.4f} qy={q.y:.4f}')
    # lidar sits 0.05 m ahead of base; walls at x=+-2.95, y=+-1.95 (inner faces)
    exp = {0.0: 2.95 - (gx + 0.05), math.pi / 2: 1.95 - gy, -math.pi / 2: 1.95 + gy,
           math.pi: (gx + 0.05) + 2.95}
    for a, e in exp.items():
        r = n.range_at(a)
        n.check(f'LiDAR range at {math.degrees(a):+.0f} deg', abs(r - e) < 0.06, f'measured={r:.3f} expected={e:.3f}')

    # --- 2. Rotate in place +90 deg then back: wheel odom yaw vs ground truth ---
    _, _, gy0 = n.pose(n.gt)
    _, _, oy0 = n.pose(n.odom)
    n.spin_for(math.pi / 2 / 0.5, w=0.5)
    n.spin_for(1.0)
    _, _, gy1 = n.pose(n.gt)
    _, _, oy1 = n.pose(n.odom)
    gt_turn, od_turn = wrap(gy1 - gy0), wrap(oy1 - oy0)
    n.check('rotate +90deg: gt yaw change', abs(gt_turn - math.pi / 2) < 0.15, f'{math.degrees(gt_turn):.1f} deg')
    n.check('rotate: odom vs gt yaw', abs(gt_turn - od_turn) < 0.1, f'odom={math.degrees(od_turn):.1f} gt={math.degrees(gt_turn):.1f}')
    n.spin_for(math.pi / 2 / 0.5, w=-0.5)
    n.spin_for(1.0)

    # --- 3. Straight line 0.2 m/s for 3 s: wheel odom vs ground truth ---
    gxa, gya, _ = n.pose(n.gt)
    oxa, oya, _ = n.pose(n.odom)
    n.spin_for(3.0, v=0.2)
    n.spin_for(1.0)
    gxb, gyb, _ = n.pose(n.gt)
    oxb, oyb, _ = n.pose(n.odom)
    gd, od = math.hypot(gxb - gxa, gyb - gya), math.hypot(oxb - oxa, oyb - oya)
    n.check('straight: distance travelled ~0.6 m', abs(gd - 0.6) < 0.1, f'gt={gd:.3f} odom={od:.3f}')
    n.check('straight: odom vs gt distance', abs(gd - od) < 0.05, f'diff={abs(gd - od):.3f} m')

    # --- 4. Collision: drive into the east wall for a long time ---
    zmax = 0.0
    for _ in range(40):
        n.spin_for(0.5, v=0.4)
        zmax = max(zmax, n.gt.pose.pose.position.z)
    gx, gy, _ = n.pose(n.gt)
    ox, oy, _ = n.pose(n.odom)
    front_face = gx + 0.15
    n.check('collision: stopped by wall (front face <= wall at 2.95)', front_face < 2.97, f'front face x={front_face:.3f}')
    n.check('collision: did reach the wall', front_face > 2.85, f'front face x={front_face:.3f}')
    n.check('collision: no climbing', zmax < 0.05, f'max z={zmax:.3f}')
    print(f'INFO  wheel odom x={ox:.3f} vs gt x={gx:.3f} (slip against wall -> odom error {ox - gx:+.3f} m)', flush=True)

    n.pub.publish(Twist())
    print(f'RESULT  {"ALL PASS" if n.fails == 0 else str(n.fails) + " FAILED"}', flush=True)
    rclpy.shutdown()
    return 1 if n.fails else 0


if __name__ == '__main__':
    sys.exit(main())
