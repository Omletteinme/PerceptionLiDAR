"""ROS 2 wrapper around the pure-NumPy particle filters.

CUSTOM node (ours). Inputs come from Gazebo through ros_gz_bridge: /scan (LaserScan), /odom (wheel
odometry) and /imu. The filter itself (sewer_localization.particle_filter) has no ROS imports.

Parameter `filter` picks the algorithm: 'baseline' (standard PF) or 'reliability' (proposed).
Publishes (relative names, so a namespace gives /baseline/estimated_pose etc.):
  estimated_pose        geometry_msgs/PoseWithCovarianceStamped   in the map frame
  particle_cloud        geometry_msgs/PoseArray
  scan_reliability      std_msgs/Float32MultiArray   per-beam r_i (reliability filter only)
  scan_reliability_R    std_msgs/Float32             scan-level reliability R (reliability filter only)
  reliability_scan      sensor_msgs/LaserScan        the scan the filter used, intensities = r_i (for RViz)
  localization_health   std_msgs/String   JSON: state RELIABLE/DEGRADED/CRITICAL, score and per-signal stress
  status_marker         visualization_msgs/Marker    text next to the pose: state, sigma (and R); for RViz
  map -> odom TF        only if publish_tf is true: the ACTIVE localizer, never two at once

Ground truth is never read here.
"""
import json
import math
from collections import deque

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose, PoseArray, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float32, Float32MultiArray, String
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker

from .health import CRITICAL, DEGRADED, HealthMonitor
from .map_utils import OccupancyMap
from .particle_filter import PFParams, ParticleFilter, ReliabilityAwareFilter, fuse_heading, wrap


def stamp_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def nearest(buf, t):
    return min(buf, key=lambda e: abs(e[0] - t)) if buf else None


class LocalizerNode(Node):
    def __init__(self):
        super().__init__('localizer')
        d = self.declare_parameter
        self.kind = d('filter', 'reliability').value
        map_path = d('map_path', '').value
        scan_topic = d('scan_topic', '/scan').value
        odom_topic = d('odom_topic', '/odom').value
        imu_topic = d('imu_topic', '/imu').value
        self.map_frame = d('map_frame', 'map').value
        self.odom_frame = d('odom_frame', 'odom').value
        self.init_pose = (d('init_x', 0.0).value, d('init_y', 0.0).value, d('init_yaw', 0.0).value)
        self.n_particles = int(d('n_particles', 500).value)
        self.seed = int(d('seed', 1000).value)
        self.publish_tf = bool(d('publish_tf', True).value)
        if self.kind not in ('baseline', 'reliability'):
            raise ValueError("parameter 'filter' must be 'baseline' or 'reliability'")
        if not map_path:
            raise ValueError("parameter 'map_path' (base path of the .pgm/.yaml map) is required")
        self.omap = OccupancyMap.load(map_path)
        # The motion model reads odom_topic (possibly the DISTURBED odometry). map->odom must be composed against the
        # odometry that the TF tree really uses (Gazebo's /odom), or TF map->base_footprint would not equal the estimate.
        tf_odom_topic = d('tf_odom_topic', '').value or odom_topic

        self.odom_buf = deque(maxlen=200)      # (t, x, y, yaw) of wheel odometry
        self.tf_buf = self.odom_buf if tf_odom_topic == odom_topic else deque(maxlen=200)
        self.gyro_buf = deque(maxlen=1000)     # (t, cumulative integrated gyro yaw)
        self._gyro_cum, self._gyro_t = 0.0, None
        self.filter = None
        self.prev = None                       # (odom pose, gyro cumulative) at the previous scan
        self.prev_t = None                     # time of the previous scan
        self.health = HealthMonitor()
        self.map_odom = None                   # (x, y, yaw) of map -> odom

        self.create_subscription(Odometry, odom_topic, self.on_odom, qos_profile_sensor_data)
        if self.tf_buf is not self.odom_buf:
            self.create_subscription(Odometry, tf_odom_topic, self.on_tf_odom, qos_profile_sensor_data)
        self.create_subscription(Imu, imu_topic, self.on_imu, qos_profile_sensor_data)
        self.create_subscription(LaserScan, scan_topic, self.on_scan, qos_profile_sensor_data)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'estimated_pose', 10)
        self.cloud_pub = self.create_publisher(PoseArray, 'particle_cloud', 10)
        self.rel_pub = self.create_publisher(Float32MultiArray, 'scan_reliability', 10)
        self.R_pub = self.create_publisher(Float32, 'scan_reliability_R', 10)
        self.viz_scan_pub = self.create_publisher(LaserScan, 'reliability_scan', 10)
        self.marker_pub = self.create_publisher(Marker, 'status_marker', 10)
        self.health_pub = self.create_publisher(String, 'localization_health', 10)
        self.tf = TransformBroadcaster(self)
        self.create_timer(0.05, self.publish_map_odom)
        self.get_logger().info(f'localizer filter={self.kind} n={self.n_particles} seed={self.seed} '
                               f'publish_tf={self.publish_tf} scan={scan_topic}')

    # ------------------------------------------------------------------ inputs
    def on_odom(self, m):
        p = m.pose.pose
        self.odom_buf.append((stamp_sec(m.header.stamp), p.position.x, p.position.y, yaw_of(p.orientation)))

    def on_tf_odom(self, m):
        p = m.pose.pose
        self.tf_buf.append((stamp_sec(m.header.stamp), p.position.x, p.position.y, yaw_of(p.orientation)))

    def on_imu(self, m):
        t = stamp_sec(m.header.stamp)
        if self._gyro_t is not None:
            self._gyro_cum += m.angular_velocity.z * min(max(t - self._gyro_t, 0.0), 0.1)
        self._gyro_t = t
        self.gyro_buf.append((t, self._gyro_cum))

    def on_scan(self, m):
        t = stamp_sec(m.header.stamp)
        o, g, o_tf = nearest(self.odom_buf, t), nearest(self.gyro_buf, t), nearest(self.tf_buf, t)
        if o is None or o_tf is None:
            return
        ranges = np.array(m.ranges, float)
        angles = m.angle_min + np.arange(len(ranges)) * m.angle_increment
        if self.filter is None:
            self._start(m, angles, o_tf)
            self.prev, self.prev_t = (o, g), t
            return
        (po, pg), dth_gyro = self.prev, None
        dx, dy, dth_wheel = o[1] - po[1], o[2] - po[2], wrap(o[3] - po[3])
        heading = po[3] + 0.5 * dth_wheel
        ds = math.hypot(dx, dy) * (1.0 if dx * math.cos(heading) + dy * math.sin(heading) >= 0 else -1.0)
        dth = dth_wheel
        if g is not None and pg is not None:
            dth = fuse_heading(dth_wheel, g[1] - pg[1], self.filter.p.gyro_weight)
        dt = min(max(t - self.prev_t, 1e-3), 0.5)
        gyro_gap = abs(dth_wheel - (g[1] - pg[1])) / dt if g is not None and pg is not None else None
        self.prev, self.prev_t = (o, g), t
        self.filter.predict(ds, dth)
        self.filter.update(ranges, angles)
        self.health.update(dt, fit=self.filter.fit_residual, R=self.filter.R if self.kind == 'reliability' else None,
                           gyro=gyro_gap)
        self.publish_estimate(m, o_tf)

    def _start(self, m, angles, o):
        p = PFParams(n=self.n_particles, rmin=m.range_min, rmax=m.range_max)
        rng = np.random.default_rng(self.seed)
        if self.kind == 'baseline':
            self.filter = ParticleFilter(self.omap, p, rng, self.init_pose)
        else:
            self.filter = ReliabilityAwareFilter(self.omap, p, rng, self.init_pose, angles)
        self._set_map_odom(self.init_pose, o)

    # ------------------------------------------------------------------ outputs
    def _set_map_odom(self, pose_map, o):
        """map->odom = T_map_base * inverse(T_odom_base)  (2D rigid transforms)."""
        a = pose_map[2] - o[3]
        c, s = math.cos(a), math.sin(a)
        self.map_odom = (pose_map[0] - (c * o[1] - s * o[2]), pose_map[1] - (s * o[1] + c * o[2]), a)

    def publish_estimate(self, scan, o):
        stamp = scan.header.stamp
        f = self.filter
        x, y, th, cov = f.estimate()
        self._set_map_odom((x, y, th), o)
        msg = PoseWithCovarianceStamped()
        msg.header.stamp, msg.header.frame_id = stamp, self.map_frame
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(th / 2), math.cos(th / 2)
        c = [0.0] * 36
        c[0], c[1], c[6], c[7] = cov[0, 0], cov[0, 1], cov[1, 0], cov[1, 1]
        c[35] = float(f.w @ wrap(f.th - th) ** 2)
        msg.pose.covariance = c
        self.pose_pub.publish(msg)
        cloud = PoseArray()
        cloud.header = msg.header
        for px, py, pth in zip(f.x, f.y, f.th):
            q = Pose()
            q.position.x, q.position.y = float(px), float(py)
            q.orientation.z, q.orientation.w = math.sin(pth / 2), math.cos(pth / 2)
            cloud.poses.append(q)
        self.cloud_pub.publish(cloud)
        if self.kind == 'reliability':
            r = Float32MultiArray()
            r.data = [float(v) for v in f.r]
            self.rel_pub.publish(r)
            self.R_pub.publish(Float32(data=float(f.R)))
            viz = LaserScan()
            viz.header, viz.angle_min, viz.angle_max = scan.header, scan.angle_min, scan.angle_max
            viz.angle_increment, viz.range_min, viz.range_max = scan.angle_increment, scan.range_min, scan.range_max
            viz.ranges = scan.ranges
            viz.intensities = r.data
            self.viz_scan_pub.publish(viz)
        self.health_pub.publish(String(data=json.dumps(self.health.details())))
        self.publish_status(stamp, x, y)

    def publish_status(self, stamp, x, y):
        """Text next to the pose: the baseline's text sits below it, the reliability-aware one above."""
        f = self.filter
        mk = Marker()
        mk.header.stamp, mk.header.frame_id = stamp, self.map_frame
        mk.ns, mk.id, mk.type, mk.action = self.kind, 0, Marker.TEXT_VIEW_FACING, Marker.ADD
        mk.pose.orientation.w = 1.0
        mk.pose.position.x, mk.pose.position.z = x, 0.9
        mk.scale.z = 0.2
        mk.lifetime = Duration(sec=0, nanosec=500_000_000)
        mk.color.a = 1.0
        if self.kind == 'reliability':
            mk.pose.position.y = y + 0.8
            mk.text = f'reliability-aware  {self.health.state}  R={f.R:.2f}  sigma={f.std_xy:.2f} m'
            mk.color.r, mk.color.g, mk.color.b = ((1.0, 0.25, 0.2) if self.health.state == CRITICAL else
                                                  (1.0, 0.85, 0.1) if self.health.state == DEGRADED else (0.2, 1.0, 0.2))
        else:
            mk.pose.position.y = y - 0.8
            mk.text = f'baseline  {self.health.state}  sigma={f.std_xy:.2f} m'
            mk.color.r, mk.color.g, mk.color.b = 0.4, 0.7, 1.0
        self.marker_pub.publish(mk)

    def publish_map_odom(self):
        if not self.publish_tf or self.map_odom is None:
            return
        x, y, a = self.map_odom
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = (
            self.get_clock().now().to_msg(), self.map_frame, self.odom_frame)
        t.transform.translation.x, t.transform.translation.y = x, y
        t.transform.rotation.z, t.transform.rotation.w = math.sin(a / 2), math.cos(a / 2)
        self.tf.sendTransform(t)


def main():
    rclpy.init()
    node = LocalizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
