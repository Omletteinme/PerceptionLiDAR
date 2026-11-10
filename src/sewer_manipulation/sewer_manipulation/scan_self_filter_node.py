"""LiDAR self-filter (Account 2). CUSTOM node.

While the arm is lowered in front of the robot it crosses the 2D LiDAR's scan plane, and the robot would 'see' its own
arm as an obstacle (phantom debris for the blockage detector, wrong ranges for the localizers). This node republishes
the scan with those beams set to +inf (no return), computed from the current joint positions and the arm geometry
(arm_kinematics.scan_self_mask). With the arm folded, the scan passes through unchanged.

  in_topic  (default /scan)  ->  out_topic (default /scan_self_filtered)
Using it means pointing the localizers / scan_disturbance_node at out_topic: a shared launch change that needs the
lead's approval (ACCOUNT2_README.md, integration step 3).
"""
import json

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import String

from .arm_kinematics import FINGERS, JOINTS, ArmGeometry, scan_self_mask


class ScanSelfFilter(Node):
    def __init__(self):
        super().__init__('scan_self_filter')
        d = self.declare_parameter
        self.geo = ArmGeometry()
        self.q = {}
        self.held = None
        self.pub = self.create_publisher(LaserScan, d('out_topic', '/scan_self_filtered').value, qos_profile_sensor_data)
        self.create_subscription(LaserScan, d('in_topic', '/scan').value, self.on_scan, qos_profile_sensor_data)
        self.create_subscription(JointState, '/joint_states', self.on_joints, qos_profile_sensor_data)
        self.create_subscription(String, '/manipulation/status', self.on_status, 10)
        self.masked = 0

    def on_joints(self, m):
        for n, v in zip(m.name, m.position):
            self.q[n] = float(v)

    def on_status(self, m):
        try:
            s = json.loads(m.data)
        except ValueError:
            return
        st = s.get('detail_state', str(s.get('state', '')).lower())
        self.held = s.get('verification', {}).get('held_dims') if st in ('lifting', 'holding', 'releasing') else None

    def on_scan(self, m):
        if not all(j in self.q for j in JOINTS):
            self.pub.publish(m)
            return
        ranges = np.array(m.ranges, float)
        angles = m.angle_min + np.arange(len(ranges)) * m.angle_increment
        q = [self.q[j] for j in JOINTS]
        mask = scan_self_mask(self.geo, q, self.q.get(FINGERS[0], 0.0), angles, ranges, self.held)
        if mask.any():
            ranges[mask] = np.inf
            m.ranges = ranges.astype(np.float32).tolist()
        self.masked = int(mask.sum())
        self.pub.publish(m)


def main():
    rclpy.init()
    node = ScanSelfFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
