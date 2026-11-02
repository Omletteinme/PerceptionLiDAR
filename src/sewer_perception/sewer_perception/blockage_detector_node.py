"""ROS 2 wrapper around sewer_perception.blockage (CUSTOM node).

Inputs (from the reliability-aware localizer, so pose and scan share one time stamp):
  /reliability/estimated_pose   PoseWithCovarianceStamped   the ESTIMATED pose (never ground truth)
  /reliability/reliability_scan LaserScan                   the scan the filter used, intensities = per-beam r_i
Outputs:
  edge_gaps   std_msgs/String   JSON {"A-B": {gap, blocked, s_min, s_max, offset, observed, cells}, ...}
  blockages   visualization_msgs/MarkerArray   unmapped-obstacle cells and a label per obstructed corridor
Parameter use_reliability (default true): only beams with per-beam reliability r_i >= 0.5 count as evidence, both
obstacle hits and free-space rays. Live A/B (Day 11, 3x disturbance, two disturbance seeds): with every valid beam
counted, the detector flagged phantom blockages, lost the real debris (water returns are longer than the truth and
their rays erase evidence) and the mission failed in 2 of 2 runs; gated, it passed 2 of 2. The earlier sandbox
ablation had missed this because it only counted phantom blockages. use_reliability:=false is the ablation.
"""
import json
import math

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from sewer_localization.map_utils import OccupancyMap
from .blockage import EvidenceGrid


def stamp_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class BlockageDetectorNode(Node):
    def __init__(self):
        super().__init__('blockage_detector')
        d = self.declare_parameter
        map_path = d('map_path', '').value
        graph_path = d('graph_path', '').value
        pose_topic = d('pose_topic', '/reliability/estimated_pose').value
        scan_topic = d('scan_topic', '/reliability/reliability_scan').value
        self.use_reliability = bool(d('use_reliability', True).value)
        if not map_path or not graph_path:
            raise ValueError("parameters 'map_path' and 'graph_path' are required")
        self.grid = EvidenceGrid(OccupancyMap.load(map_path))
        with open(graph_path) as f:
            g = yaml.safe_load(f)
        self.nodes = {k: (v['x'], v['y']) for k, v in g['nodes'].items()}
        self.edges = [(e['from'], e['to']) for e in g['edges']]
        self.pose = None
        self.create_subscription(PoseWithCovarianceStamped, pose_topic, self.on_pose, qos_profile_sensor_data)
        self.create_subscription(LaserScan, scan_topic, self.on_scan, qos_profile_sensor_data)
        self.gaps_pub = self.create_publisher(String, 'edge_gaps', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'blockages', 10)
        self.create_timer(0.5, self.publish)
        self.get_logger().info(f'blockage detector: use_reliability={self.use_reliability} scan={scan_topic}')

    def on_pose(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (stamp_sec(m.header.stamp), m.pose.pose.position.x, m.pose.pose.position.y, yaw)

    def on_scan(self, m):
        if self.pose is None or abs(self.pose[0] - stamp_sec(m.header.stamp)) > 0.06:
            return   # pose and scan must belong together
        ranges = np.array(m.ranges, float)
        angles = m.angle_min + np.arange(len(ranges)) * m.angle_increment
        r = np.array(m.intensities, float) if self.use_reliability and len(m.intensities) == len(ranges) else None
        self.grid.update(self.pose[1:], ranges, angles, r)

    def publish(self):
        status = {f'{a}-{b}': self.grid.edge_status(self.nodes[a], self.nodes[b]) for a, b in self.edges}
        self.gaps_pub.publish(String(data=json.dumps(status)))
        now = self.get_clock().now().to_msg()
        arr = MarkerArray()
        cx, cy = self.grid.obstacle_cells()
        cells = Marker()
        cells.header.stamp, cells.header.frame_id = now, 'map'
        cells.ns, cells.id, cells.type, cells.action = 'obstacle_cells', 0, Marker.CUBE_LIST, Marker.ADD
        cells.pose.orientation.w = 1.0
        cells.scale.x = cells.scale.y = self.grid.map.res
        cells.scale.z = 0.5
        cells.color.r, cells.color.g, cells.color.b, cells.color.a = 1.0, 0.45, 0.0, 0.9
        for x, y in zip(cx[::2], cy[::2]):
            p = Marker().pose.position
            p.x, p.y, p.z = float(x), float(y), 0.25
            cells.points.append(p)
        arr.markers.append(cells)
        for k, ((a, b), st) in enumerate(zip(self.edges, status.values())):
            if st['s_min'] is None:
                continue
            ax, ay = self.nodes[a]
            bx, by = self.nodes[b]
            length = math.hypot(bx - ax, by - ay)
            s = 0.5 * (st['s_min'] + st['s_max']) / length
            t = Marker()
            t.header.stamp, t.header.frame_id = now, 'map'
            t.ns, t.id, t.type, t.action = 'label', k, Marker.TEXT_VIEW_FACING, Marker.ADD
            t.pose.position.x, t.pose.position.y, t.pose.position.z = ax + s * (bx - ax) + 0.9, ay + s * (by - ay), 1.2
            t.pose.orientation.w = 1.0
            t.scale.z = 0.3
            t.text = f"{a}-{b}: gap {st['gap']:.2f} m  {'BLOCKED' if st['blocked'] else 'passable'}"
            t.color.r, t.color.g, t.color.b, t.color.a = ((1.0, 0.2, 0.2, 1.0) if st['blocked'] else (1.0, 0.8, 0.1, 1.0))
            arr.markers.append(t)
        self.marker_pub.publish(arr)


def main():
    rclpy.init()
    node = BlockageDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
