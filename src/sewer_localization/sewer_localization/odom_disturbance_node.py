"""ROS 2 node that makes the wheel odometry and the IMU gyro realistically bad, seeded and repeatable.

CUSTOM node (ours). Gazebo's wheel odometry has no slip and drifts only ~1.5%, so with it the two filters look alike.
This node reads the TRUE /odom and /imu and publishes disturbed versions built with the same OdometryErrorModel the
sandbox uses (sewer_localization.disturbances): per-run distance and yaw scale errors, gyro bias with a random walk,
and wheel-slip zones (places on the route, in metres travelled, where the wheels turn more than the robot moves).

  /odom_disturbed  nav_msgs/Odometry  dead-reckoning pose integrated from the erroneous increments (frame odom)
  /imu_disturbed   sensor_msgs/Imu    the gyro z rate plus the (drifting) bias

Run the localizers with odom_topic:=/odom_disturbed imu_topic:=/imu_disturbed tf_odom_topic:=/odom: the motion model
reads the disturbed data while map->odom is still composed against the true odometry that Gazebo's TF uses.
"""
import copy
import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from .disturbances import OdometryErrorModel, make_slip_schedule
from .particle_filter import wrap


def stamp_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class OdomDisturbanceNode(Node):
    def __init__(self):
        super().__init__('odom_disturbance')
        d = self.declare_parameter
        odom_topic = d('odom_topic', '/odom').value
        imu_topic = d('imu_topic', '/imu').value
        scale = float(d('scale', 1.0).value)                  # multiplies the per-run scale errors and gyro bias
        slip_severity = float(d('slip_severity', 1.0).value)  # 0 switches the slip bursts off
        seed = int(d('seed', 0).value)
        self.model = OdometryErrorModel(np.random.default_rng(seed), make_slip_schedule(slip_severity), scale)
        self.travelled = 0.0       # true metres driven so far (the slip zones are distances)
        self.prev = None           # previous TRUE odometry (t, x, y, yaw)
        self.pose = None           # disturbed dead-reckoning pose [x, y, yaw]
        self.create_subscription(Odometry, odom_topic, self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Imu, imu_topic, self.on_imu, qos_profile_sensor_data)
        self.odom_pub = self.create_publisher(Odometry, 'odom_disturbed', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu_disturbed', 10)
        self.get_logger().info(f'odometry disturbances: scale={scale} slip_severity={slip_severity} seed={seed} '
                               f'eps_s={self.model.eps_s:+.4f} eps_r={self.model.eps_r:+.4f} bias={self.model.bias:+.5f}')

    def on_odom(self, m):
        t, p = stamp_sec(m.header.stamp), m.pose.pose
        yaw = yaw_of(p.orientation)
        if self.prev is None:
            self.prev = (t, p.position.x, p.position.y, yaw)
            self.pose = [p.position.x, p.position.y, yaw]
            return
        pt, px, py, pyaw = self.prev
        dt = t - pt
        if dt <= 1e-6:
            return
        dx, dy, dth = p.position.x - px, p.position.y - py, wrap(yaw - pyaw)
        heading = pyaw + 0.5 * dth
        ds = math.hypot(dx, dy) * (1.0 if dx * math.cos(heading) + dy * math.sin(heading) >= 0 else -1.0)
        self.prev = (t, p.position.x, p.position.y, yaw)
        dt = min(dt, 0.1)
        ds_m, dth_w, _ = self.model.step(ds, dth, dt, self.travelled)
        self.travelled += abs(ds)
        x, y, th = self.pose
        mid = th + 0.5 * dth_w
        self.pose = [x + ds_m * math.cos(mid), y + ds_m * math.sin(mid), wrap(th + dth_w)]
        out = Odometry()
        out.header, out.child_frame_id = m.header, m.child_frame_id
        out.pose.pose.position.x, out.pose.pose.position.y = self.pose[0], self.pose[1]
        out.pose.pose.orientation.z, out.pose.pose.orientation.w = math.sin(self.pose[2] / 2), math.cos(self.pose[2] / 2)
        out.twist.twist.linear.x, out.twist.twist.angular.z = ds_m / dt, dth_w / dt
        self.odom_pub.publish(out)

    def on_imu(self, m):
        out = copy.deepcopy(m)
        out.angular_velocity.z = m.angular_velocity.z + self.model.bias
        self.imu_pub.publish(out)


def main():
    rclpy.init()
    node = OdomDisturbanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
