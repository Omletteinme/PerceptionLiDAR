#!/usr/bin/env python3
"""Compare live /scan frames with the range the prior map predicts from the TRUE pose (/ground_truth/odom, evaluation only).

Phantom obstacles (a visual that reaches into the scan plane) show up as beams SHORTER than predicted; run it in a world
without debris (<world>_clear.sdf). Usage: scan_vs_map.py <network: sewer_network|sewer_network_v2> [n_scans]
Prints, per run, the share of beams that are > 0.15 m too short (phantoms) and > 0.30 m too long (misses), plus the pose.
"""
import math
import sys

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from sewer_localization.map_utils import OccupancyMap

net = sys.argv[1] if len(sys.argv) > 1 else 'sewer_network'
n_scans = int(sys.argv[2]) if len(sys.argv) > 2 else 20
sfx = net[len('sewer_network'):]
omap = OccupancyMap.load(f'/home/jeevi/sewer_ws/src/sewer_gazebo/maps/sewer_map{sfx}')
LIDAR_X = 0.05

rclpy.init()
node = Node('scan_vs_map')
pose = []
scans = []
node.create_subscription(Odometry, '/ground_truth/odom', lambda m: pose.append(
    (m.pose.pose.position.x, m.pose.pose.position.y,
     2 * math.atan2(m.pose.pose.orientation.z, m.pose.pose.orientation.w))), 10)
node.create_subscription(LaserScan, '/scan', lambda m: scans.append((m, pose[-1] if pose else None)), qos_profile_sensor_data)
for _ in range(400):
    rclpy.spin_once(node, timeout_sec=0.1)
    if len(scans) >= n_scans:
        break
short, long_, tot = 0, 0, 0
bearings = {}
for m, p in scans[2:]:
    if p is None:
        continue
    r = np.array(m.ranges, float)
    ang = m.angle_min + np.arange(len(r)) * m.angle_increment
    x, y, th = p
    pred = omap.raycast(x + LIDAR_X * math.cos(th), y + LIDAR_X * math.sin(th), th + ang, m.range_max, m.range_min)
    ok = np.isfinite(r) & np.isfinite(pred)
    d = r[ok] - pred[ok]
    bad = np.flatnonzero(ok)[d < -0.15]
    for k in bad:
        bearings[round(math.degrees(ang[k]) / 5) * 5] = bearings.get(round(math.degrees(ang[k]) / 5) * 5, 0) + 1
    short += int((d < -0.15).sum())
    long_ += int((d > 0.30).sum())
    tot += int(ok.sum())
x, y, th = pose[-1]
print(f'pose ({x:.2f},{y:.2f}) yaw {math.degrees(th):.1f} deg; {len(scans) - 2} scans, {tot} beams: '
      f'phantoms {100 * short / max(tot, 1):.1f} %, misses {100 * long_ / max(tot, 1):.1f} %')
print('phantom beams by bearing [deg]:', dict(sorted(bearings.items())))
ok = short / max(tot, 1) < 0.005
print('PASS  no phantom returns' if ok else 'FAIL  decoration shows up in the scan plane')
rclpy.shutdown()
sys.exit(0 if ok else 1)
