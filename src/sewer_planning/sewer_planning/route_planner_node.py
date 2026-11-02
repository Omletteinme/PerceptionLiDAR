"""ROS 2 route planner node (CUSTOM): uncertainty-aware replanning on the sewer graph.

Inputs:  /reliability/estimated_pose (pose + covariance), /reliability/scan_reliability_R, /edge_gaps (JSON from
         the blockage detector). Ground truth is never read.
Outputs: route       nav_msgs/Path        dense polyline from the robot to the goal, in the map frame
         route_info  std_msgs/String      JSON: chosen route, cost breakdown, alternatives, R, blocked edges
Replans at 2 Hz. Hysteresis: keep the current route unless another is cheaper by more than `hysteresis` metres.
"""
import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32, String

from sewer_localization.map_utils import OccupancyMap
from .graph import Graph
from .route_cost import CostParams, RouteModel


class RoutePlannerNode(Node):
    def __init__(self):
        super().__init__('route_planner')
        d = self.declare_parameter
        graph_path, map_path = d('graph_path', '').value, d('map_path', '').value
        pose_topic = d('pose_topic', '/reliability/estimated_pose').value
        r_topic = d('r_topic', '/reliability/scan_reliability_R').value
        w_unc = float(d('w_unc', 2.0).value)
        self.hysteresis = float(d('hysteresis', 1.0).value)
        if not graph_path or not map_path:
            raise ValueError("parameters 'graph_path' and 'map_path' are required")
        self.graph = Graph.load(graph_path)
        goal = d('goal', '').value
        self.goal = goal or self.graph.goal
        self.model = RouteModel(self.graph, OccupancyMap.load(map_path), CostParams(w_unc=w_unc))
        self.pose, self.cov, self.R, self.status, self.current = None, None, 0.9, {}, None
        self.create_subscription(PoseWithCovarianceStamped, pose_topic, self.on_pose, qos_profile_sensor_data)
        self.create_subscription(Float32, r_topic, self.on_r, qos_profile_sensor_data)
        self.create_subscription(String, 'edge_gaps', self.on_gaps, 10)
        self.path_pub = self.create_publisher(Path, 'route', 10)
        self.info_pub = self.create_publisher(String, 'route_info', 10)
        self.create_timer(0.5, self.step)
        self.get_logger().info(f'route planner: goal={self.goal} w_unc={w_unc}')

    def on_pose(self, m):
        q, c = m.pose.pose.orientation, m.pose.covariance
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y)
        self.cov = np.array([[c[0], c[1]], [c[6], c[7]]])

    def on_r(self, m):
        self.R = 0.9 * self.R + 0.1 * float(m.data)    # about 1 s smoothing at 10 Hz

    def on_gaps(self, m):
        self.status = json.loads(m.data)

    def step(self):
        if self.pose is None:
            return
        x, y = self.pose
        loc = self.graph.locate(x, y)
        cands = self.model.plan(loc, self.status, self.R, self.model.initial_covariance(self.cov), self.goal)
        stamp = self.get_clock().now().to_msg()
        info = dict(t=stamp.sec + stamp.nanosec * 1e-9, goal=self.goal, R=round(self.R, 3),
                    at=list(loc[:2]) if loc[0] == 'node' else [Graph.key(loc[1]), round(loc[2], 2)],
                    blocked=sorted(k for k, v in self.status.items() if v['blocked']),
                    narrow=sorted(k for k, v in self.status.items() if not v['blocked'] and v['gap'] < self.model.p.narrow_gap))
        path = Path()
        path.header.stamp, path.header.frame_id = stamp, 'map'
        if not cands:
            info.update(route=None, status='NO_ROUTE')
            self.current = None
        else:
            best = cands[0]
            chosen = best
            if self.current is not None:
                for c in cands:
                    if c['nodes'] == self.current and c['total'] <= best['total'] + self.hysteresis:
                        chosen = c
                        break
            self.current = chosen['nodes']
            pts = self.model.waypoints((x, y), chosen, self.status)
            if not chosen['legs']:
                pts.append(np.array(self.graph.nodes[chosen['nodes'][0]]))
            self.fill_path(path, pts)
            fmt = lambda c: dict(nodes=c['nodes'], total=round(c['total'], 2), length=round(c['length'], 2),  # noqa: E731
                                 unc=round(c['unc'], 3), narrow=round(c['narrow'], 2), sigma_end=round(c['sigma_end'], 3))
            info.update(route=chosen['nodes'], cost=fmt(chosen), alternatives=[fmt(c) for c in cands[:4]],
                        status='OK')
        self.path_pub.publish(path)
        self.info_pub.publish(String(data=json.dumps(info)))

    @staticmethod
    def fill_path(path, pts, spacing=0.2):
        dense = [pts[0]]
        for a, b in zip(pts[:-1], pts[1:]):
            n = max(1, int(math.ceil(np.linalg.norm(b - a) / spacing)))
            dense += [a + (b - a) * (k / n) for k in range(1, n + 1)]
        for k, p in enumerate(dense):
            q = dense[min(k + 1, len(dense) - 1)] - dense[max(k - 1, 0)] if len(dense) > 1 else np.array([1.0, 0.0])
            yaw = math.atan2(q[1], q[0])
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x, ps.pose.position.y = float(p[0]), float(p[1])
            ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            path.poses.append(ps)


def main():
    rclpy.init()
    node = RoutePlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
