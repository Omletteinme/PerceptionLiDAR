"""ROS 2 node that republishes /scan as /scan_disturbed with seeded, repeatable corruption.

CUSTOM node (ours). Uses the same disturbance models as the sandbox (sewer_localization.disturbances).
The schedule clock starts when the robot first moves (wheel-odometry speed > 0.05 m/s), so the windows fall
on the same legs of the route from run to run. Before that, scans pass through untouched.

  /scan_disturbed          sensor_msgs/LaserScan      the corrupted scan the localizers should use
  /scan_disturbance_kind   std_msgs/Int8MultiArray    per-beam injected class: EVALUATION ONLY
"""
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int8MultiArray

from .disturbances import apply_disturbances, make_schedule


class ScanDisturbanceNode(Node):
    def __init__(self):
        super().__init__('scan_disturbance')
        d = self.declare_parameter
        scan_topic = d('scan_topic', '/scan').value
        odom_topic = d('odom_topic', '/odom').value
        severity = float(d('severity', 1.0).value)
        seed = int(d('seed', 0).value)
        self.schedule = make_schedule(severity)
        self.rng = np.random.default_rng(seed)
        self.state = {}
        self.t0 = None
        self.create_subscription(Odometry, odom_topic, self.on_odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, scan_topic, self.on_scan, qos_profile_sensor_data)
        self.pub = self.create_publisher(LaserScan, 'scan_disturbed', 10)
        self.kind_pub = self.create_publisher(Int8MultiArray, 'scan_disturbance_kind', 10)
        self.get_logger().info(f'scan disturbances severity={severity} seed={seed}')

    def on_odom(self, m):
        if self.t0 is None and abs(m.twist.twist.linear.x) > 0.05:
            self.t0 = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            self.get_logger().info(f'robot moving: disturbance clock starts at t={self.t0:.2f}')

    def on_scan(self, m):
        out = LaserScan()
        out.header, out.angle_min, out.angle_max = m.header, m.angle_min, m.angle_max
        out.angle_increment, out.time_increment, out.scan_time = m.angle_increment, m.time_increment, m.scan_time
        out.range_min, out.range_max, out.intensities = m.range_min, m.range_max, m.intensities
        z = np.array(m.ranges, float)
        kind = np.zeros(len(z), np.int8)
        if self.t0 is not None:
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 - self.t0
            angles = m.angle_min + np.arange(len(z)) * m.angle_increment
            z[z >= m.range_max] = np.inf
            z, kind, _ = apply_disturbances(z, t, self.schedule, self.rng, self.state, angles, m.range_max)
            z[z >= m.range_max] = np.inf
        out.ranges = [float(v) for v in z]
        self.pub.publish(out)
        self.kind_pub.publish(Int8MultiArray(data=[int(v) for v in kind]))


def main():
    rclpy.init()
    node = ScanDisturbanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
