#!/usr/bin/env python3
"""Diagnostic: log yaw (gt, odom) and wheel speeds while rotating in place at w rad/s for T s."""
import math
import sys

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState

W = float(sys.argv[1]) if len(sys.argv) > 1 else 0.5
T = float(sys.argv[2]) if len(sys.argv) > 2 else 3.14


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


rclpy.init()
n = Node('diag', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
st = {}
n.create_subscription(Odometry, '/odom', lambda m: st.__setitem__('o', m), 10)
n.create_subscription(Odometry, '/ground_truth/odom', lambda m: st.__setitem__('g', m), 10)
n.create_subscription(JointState, '/joint_states', lambda m: st.__setitem__('j', m), 10)
pub = n.create_publisher(Twist, '/cmd_vel', 10)
while not all(k in st for k in 'ogj'):
    rclpy.spin_once(n, timeout_sec=0.2)

def t():
    return n.get_clock().now().nanoseconds * 1e-9

t0 = t()
y0g, y0o = yaw_of(st['g'].pose.pose.orientation), yaw_of(st['o'].pose.pose.orientation)
jp0 = list(st['j'].position)
nxt = 0.0
print(f'cmd w={W} for {T}s')
print(' t    gt_yaw  odom_yaw  gt_wz  odom_wz  wheel_vel(L,R)  wheel_dpos(L,R)')
while t() - t0 < T + 2.0:
    m = Twist()
    if t() - t0 < T:
        m.angular.z = W
    pub.publish(m)
    rclpy.spin_once(n, timeout_sec=0.02)
    if t() - t0 >= nxt:
        nxt += 0.5
        g, o, j = st['g'], st['o'], st['j']
        dg = math.degrees(math.atan2(math.sin(yaw_of(g.pose.pose.orientation) - y0g), math.cos(yaw_of(g.pose.pose.orientation) - y0g)))
        do = math.degrees(math.atan2(math.sin(yaw_of(o.pose.pose.orientation) - y0o), math.cos(yaw_of(o.pose.pose.orientation) - y0o)))
        print(f'{t()-t0:4.1f} {dg:8.1f} {do:8.1f} {g.twist.twist.angular.z:6.3f} {o.twist.twist.angular.z:6.3f}  '
              f'({j.velocity[0]:6.2f},{j.velocity[1]:6.2f})  ({j.position[0]-jp0[0]:6.2f},{j.position[1]-jp0[1]:6.2f})  names={list(j.name)}')
rclpy.shutdown()
