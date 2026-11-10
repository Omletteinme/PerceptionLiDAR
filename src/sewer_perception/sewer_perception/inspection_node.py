"""ROS 2 wrapper: object-level debris + structural defects (Account 2). CUSTOM node; the pipeline is
inspection_core.InspectionCore (pure NumPy, also used offline by the evaluation and the end-to-end mission test).

Inputs (ESTIMATED pose only, never ground truth):
  pose_topic   /reliability/estimated_pose     PoseWithCovarianceStamped
  scan_topic   /reliability/reliability_scan   LaserScan, intensities = per-beam reliability (as the blockage node)
  depth_topic  /robot_depth                    Image 32FC1, planar depth [m] (depth camera, SIMULATED by Gazebo)
  rgb_topic    /robot_cam/image                Image rgb8 (SIMULATED by Gazebo)
  /manipulation/status, /disposal/status       String JSON (sewer_manipulation): held / removed objects, arm exclusion
Outputs (docs/mission_interfaces.md):
  /perception/debris    String JSON {frame_id: map, stamp, objects: [...]}
  /perception/defects   String JSON {frame_id: map, stamp, defects: [...]}
  /perception/markers   MarkerArray (RViz: debris boxes green = graspable / red = not, defects with labels)
  evidence images       PNG files under evidence_dir (crack frames with boxes, LiDAR defects marked in a camera frame)
The existing blockage_detector_node is untouched and keeps publishing /edge_gaps for route planning.
"""
import json
import math
import os

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from sewer_localization.map_utils import OccupancyMap
from . import interfaces as itf
from .inspection_core import InspectionCore
from .ros_utils import PoseHistory, image_to_numpy, stamp_sec, yaw_of


class InspectionNode(Node):
    def __init__(self):
        super().__init__('inspection')
        d = self.declare_parameter
        map_path = d('map_path', '').value
        graph_path = d('graph_path', '').value
        station_path = d('station_path', '').value
        self.use_reliability = bool(d('use_reliability', True).value)
        self.evidence_dir = os.path.expanduser(d('evidence_dir', '~/sewer_ws/logs/evidence').value)
        if not map_path or not graph_path:
            raise ValueError("parameters 'map_path' and 'graph_path' are required")
        with open(graph_path) as f:
            g = yaml.safe_load(f)
        nodes = {k: (v['x'], v['y']) for k, v in g['nodes'].items()}
        edges = [(e['from'], e['to']) for e in g['edges']]
        station = None
        if station_path:
            with open(station_path) as f:
                station = yaml.safe_load(f)
        os.makedirs(self.evidence_dir, exist_ok=True)
        self.core = InspectionCore(OccupancyMap.load(map_path), nodes, edges, station, self.save_evidence,
                                   int(d('evidence_frames', 40).value), crack_every=int(d('crack_every', 2).value))
        self.poses = PoseHistory()

        self.create_subscription(PoseWithCovarianceStamped, d('pose_topic', '/reliability/estimated_pose').value,
                                 self.on_pose, qos_profile_sensor_data)
        self.create_subscription(LaserScan, d('scan_topic', '/reliability/reliability_scan').value, self.on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, d('depth_topic', '/robot_depth').value, self.on_depth, qos_profile_sensor_data)
        self.create_subscription(Image, d('rgb_topic', '/robot_cam/image').value, self.on_rgb, qos_profile_sensor_data)
        self.create_subscription(String, '/manipulation/status', lambda m: self._json(m, self.core.on_manipulation), 10)
        self.create_subscription(String, '/disposal/status', lambda m: self._json(m, self.core.on_disposal), 10)
        self.debris_pub = self.create_publisher(String, '/perception/debris', 10)
        self.defects_pub = self.create_publisher(String, '/perception/defects', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/perception/markers', 10)
        self.create_timer(float(d('publish_period', 0.5).value), self.publish)
        self.get_logger().info(f'inspection node up; evidence -> {self.evidence_dir}')

    @staticmethod
    def _json(m, fn):
        try:
            fn(json.loads(m.data))
        except ValueError:
            pass

    def on_pose(self, m):
        p = m.pose.pose
        self.poses.add(stamp_sec(m.header.stamp), (p.position.x, p.position.y, yaw_of(p.orientation)))

    def on_scan(self, m):
        t = stamp_sec(m.header.stamp)
        pose = self.poses.at(t, 0.06)
        if pose is None:
            return
        ranges = np.array(m.ranges, float)
        angles = m.angle_min + np.arange(len(ranges)) * m.angle_increment
        r = np.array(m.intensities, float) if self.use_reliability and len(m.intensities) == len(ranges) else None
        self.core.on_scan(t, pose, ranges, angles, r)

    def on_depth(self, m):
        t = stamp_sec(m.header.stamp)
        pose = self.poses.at(t)
        if pose is not None:
            self.core.on_depth(t, pose, image_to_numpy(m))

    def on_rgb(self, m):
        t = stamp_sec(m.header.stamp)
        pose = self.poses.at(t)
        if pose is not None:
            self.core.on_rgb(t, pose, image_to_numpy(m))

    def save_evidence(self, rgb, t, boxes, tag, circles=()):
        try:
            from PIL import Image as PILImage, ImageDraw
        except ImportError:
            return None
        im = PILImage.fromarray(np.asarray(rgb, np.uint8))
        dr = ImageDraw.Draw(im)
        for b in boxes:
            dr.rectangle([b[0] - 3, b[1] - 3, b[2] + 3, b[3] + 3], outline=(255, 40, 40), width=2)
        for u, v, txt in circles:
            dr.ellipse([u - 14, v - 14, u + 14, v + 14], outline=(255, 200, 0), width=2)
            dr.text((u + 16, v - 6), txt, fill=(255, 200, 0))
        path = os.path.join(self.evidence_dir, f'{tag}_{t:09.2f}.png')
        im.save(path)
        return path

    def publish(self):
        t_pose, _ = self.poses.latest()
        stamp = t_pose if t_pose is not None else self.get_clock().now().nanoseconds * 1e-9
        dmsg, fmsg = self.core.messages(stamp)
        self.debris_pub.publish(String(data=itf.dumps(dmsg)))
        self.defects_pub.publish(String(data=itf.dumps(fmsg)))
        self.marker_pub.publish(self.markers(dmsg['objects'], fmsg['defects']))

    def markers(self, objs, defects):
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        now = self.get_clock().now().to_msg()
        k = 0

        def mk(ns, typ):
            nonlocal k
            m = Marker()
            m.header.stamp, m.header.frame_id = now, 'map'
            m.ns, m.id, m.type, m.action = ns, k, typ, Marker.ADD
            m.pose.orientation.w = 1.0
            k += 1
            return m
        for o in objs:
            L, W, H = o['size']
            b = mk('debris', Marker.CUBE)
            b.pose.position.x, b.pose.position.y, b.pose.position.z = o['x'], o['y'], H / 2
            b.pose.orientation.z, b.pose.orientation.w = math.sin(o['yaw'] / 2), math.cos(o['yaw'] / 2)
            b.scale.x, b.scale.y, b.scale.z = max(L, 0.02), max(W, 0.02), max(H, 0.02)
            ok = o['graspable']
            b.color.r, b.color.g, b.color.b, b.color.a = (0.1, 0.9, 0.2, 0.8) if ok else (0.9, 0.15, 0.1, 0.8)
            arr.markers.append(b)
            tx = mk('debris_label', Marker.TEXT_VIEW_FACING)
            tx.pose.position.x, tx.pose.position.y, tx.pose.position.z = o['x'], o['y'], 0.45
            tx.scale.z = 0.12
            tx.text = f"{o['id']} {L:.2f}x{W:.2f}x{H:.2f} {'GRASPABLE' if ok else 'no-grasp'}"
            tx.color.r = tx.color.g = tx.color.b = tx.color.a = 1.0
            arr.markers.append(tx)
        colours = {'crack': (1.0, 0.9, 0.1), 'hole': (0.2, 0.6, 1.0), 'damaged_joint': (0.8, 0.3, 1.0),
                   'deformation': (1.0, 0.5, 0.0)}
        for dfc in defects:
            s = mk('defect', Marker.SPHERE)
            s.pose.position.x, s.pose.position.y, s.pose.position.z = dfc['x'], dfc['y'], dfc['z']
            s.scale.x = s.scale.y = s.scale.z = 0.12
            s.color.r, s.color.g, s.color.b = colours[dfc['type']]
            s.color.a = 0.9
            arr.markers.append(s)
            tx = mk('defect_label', Marker.TEXT_VIEW_FACING)
            tx.pose.position.x, tx.pose.position.y, tx.pose.position.z = dfc['x'], dfc['y'], 1.15
            tx.scale.z = 0.12
            tx.text = f"{dfc['id']} {dfc['type']} {dfc['severity_label']}"
            tx.color.r = tx.color.g = tx.color.b = tx.color.a = 1.0
            arr.markers.append(tx)
        return arr


def main():
    rclpy.init()
    node = InspectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
