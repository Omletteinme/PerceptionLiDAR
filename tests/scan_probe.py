#!/usr/bin/env python3
"""Read one /scan with the robot parked at the entry A and check the robot does not see itself (the beacon mast)."""
import math
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

rclpy.init()
n = Node('scan_probe')
got = []
n.create_subscription(LaserScan, '/scan', got.append, qos_profile_sensor_data)
for _ in range(100):
    rclpy.spin_once(n, timeout_sec=0.2)
    if got:
        break
if not got:
    print('FAIL  no /scan received')
    sys.exit(1)
m = got[0]
r = np.array(m.ranges, float)
ang = m.angle_min + np.arange(len(r)) * m.angle_increment
back = np.abs(np.arctan2(np.sin(ang - math.pi), np.cos(ang - math.pi))) < math.radians(8)
fin = np.isfinite(r)
close = int(np.sum(fin & (r < 0.35)))
print(f'rear range (end wall expected ~0.55 m): median {np.median(r[back & fin]):.2f} m; '
      f'closest return anywhere {r[fin].min():.2f} m; beams closer than 0.35 m: {close}')
ok = abs(np.median(r[back & fin]) - 0.55) < 0.1 and r[fin].min() > 0.3
print('PASS  LiDAR does not see the robot itself' if ok else 'FAIL  something near the robot is in the scan plane')
rclpy.shutdown()
sys.exit(0 if ok else 1)
