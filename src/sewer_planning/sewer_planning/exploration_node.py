"""ROS 2 exploration node (CUSTOM): frontier exploration + coverage tracking on the sewer graph.

Thin wrapper around sewer_planning.exploration / coverage_tracker (pure NumPy, shared with sandbox/explore_sim.py).
It replaces route_planner_node in exploration missions: instead of one fixed goal it keeps choosing the cheapest
uncovered corridor, and when every accessible corridor is covered it routes back to the entrance.

Inputs:  /reliability/estimated_pose        PoseWithCovarianceStamped  ESTIMATED pose (never ground truth)
         /reliability/scan_reliability_R    Float32                    scan reliability R (route-cost uncertainty)
         edge_gaps                          String (JSON)              blockage detector per-edge status
         /reliability/localization_health   String (JSON)              optional; CRITICAL -> no coverage is marked
Outputs: route                nav_msgs/Path               waypoints to the current target, map frame (route_follower_node
                                                          drives it; this node never publishes /cmd_vel)
         exploration/status   std_msgs/String (JSON)      state INIT/EXPLORING/RETURNING/COMPLETE/INCOMPLETE, reason,
                                                          frontier count, active segment, target, candidate costs,
                                                          localization health
         coverage/status      std_msgs/String (JSON)      CoverageTracker.summary(): coverage %, confidence, explored /
                                                          accessible / blocked / total metres, per-corridor states
         coverage/markers     visualization_msgs/MarkerArray  corridor bins coloured by state (RViz)
"""
import json

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import ColorRGBA, Float32, String
from visualization_msgs.msg import Marker, MarkerArray

from sewer_localization.map_utils import OccupancyMap
from .coverage_tracker import BLOCKED, EXPLORED, SKIPPED, UNEXPLORED, CoverageParams, CoverageTracker
from .exploration import COMPLETE, INCOMPLETE, INIT, Explorer, ExploreParams
from .graph import Graph
from .route_cost import CostParams, RouteModel
from .route_planner_node import RoutePlannerNode

COLOURS = {EXPLORED: (0.2, 0.9, 0.3), UNEXPLORED: (0.6, 0.6, 0.6), BLOCKED: (1.0, 0.2, 0.2), SKIPPED: (1.0, 0.6, 0.1)}


class ExplorationNode(Node):
    def __init__(self):
        super().__init__('exploration')
        d = self.declare_parameter
        graph_path, map_path = d('graph_path', '').value, d('map_path', '').value
        pose_topic = d('pose_topic', '/reliability/estimated_pose').value
        r_topic = d('r_topic', '/reliability/scan_reliability_R').value
        health_topic = d('health_topic', '').value                 # '' = ignore the localization health
        if not graph_path or not map_path:
            raise ValueError("parameters 'graph_path' and 'map_path' are required")
        self.graph = Graph.load(graph_path)
        model = RouteModel(self.graph, OccupancyMap.load(map_path), CostParams(w_unc=float(d('w_unc', 2.0).value)))
        tracker = CoverageTracker(self.graph, CoverageParams(bin_len=float(d('bin_len', 0.5).value),
                                                             sense_radius=float(d('sense_radius', 0.6).value)))
        params = ExploreParams(stop_short=float(d('stop_short', 0.6).value),
                               return_home=bool(d('return_home', True).value))
        self.explorer = Explorer(self.graph, model, tracker, params, home=d('home', '').value or None)
        self.pose, self.cov, self.R, self.status, self.health = None, None, 0.9, {}, 'RELIABLE'
        self.last = dict(state=INIT)
        self.create_subscription(PoseWithCovarianceStamped, pose_topic, self.on_pose, qos_profile_sensor_data)
        self.create_subscription(Float32, r_topic, self.on_r, qos_profile_sensor_data)
        self.create_subscription(String, 'edge_gaps', self.on_gaps, 10)
        if health_topic:
            self.create_subscription(String, health_topic, self.on_health, 10)
        self.path_pub = self.create_publisher(Path, 'route', 10)
        self.status_pub = self.create_publisher(String, 'exploration/status', 10)
        self.cov_pub = self.create_publisher(String, 'coverage/status', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'coverage/markers', 10)
        self.create_timer(0.5, self.step)
        self.create_timer(1.0, self.publish_coverage)
        self.get_logger().info(f'exploration: start={self.graph.start} home={self.explorer.home} health={health_topic or "off"}')

    def on_pose(self, m):
        c = m.pose.covariance
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y)
        self.cov = np.array([[c[0], c[1]], [c[6], c[7]]])

    def on_r(self, m):
        self.R = 0.9 * self.R + 0.1 * float(m.data)    # same smoothing as route_planner_node

    def on_gaps(self, m):
        self.status = json.loads(m.data)

    def on_health(self, m):
        self.health = json.loads(m.data).get('state', 'RELIABLE')

    def step(self):
        stamp = self.get_clock().now().to_msg()
        info = dict(t=stamp.sec + stamp.nanosec * 1e-9, localization=self.health)
        if self.pose is None:
            info.update(state=INIT, reason='waiting for the estimated pose')
            self.status_pub.publish(String(data=json.dumps(info)))
            return
        if self.last.get('state') in (COMPLETE, INCOMPLETE):    # finished: keep reporting, stop routing
            info.update({k: v for k, v in self.last.items() if k != 'waypoints'})
            self.status_pub.publish(String(data=json.dumps(info)))
            return
        trusted = self.health != 'CRITICAL'
        res = self.explorer.step(self.pose, self.cov, self.R, self.status, trusted)
        self.last = res
        loc = self.graph.locate(*self.pose)
        info.update({k: v for k, v in res.items() if k != 'waypoints'},
                    active_segment=Graph.key(loc[1]) if loc[0] == 'edge' else loc[1],
                    coverage_marking=trusted)
        if res['waypoints']:
            path = Path()
            path.header.stamp, path.header.frame_id = stamp, 'map'
            RoutePlannerNode.fill_path(path, [np.asarray(p, float) for p in res['waypoints']])
            self.path_pub.publish(path)
        if res['state'] in (COMPLETE, INCOMPLETE):
            self.get_logger().info(f"exploration {res['state']}: {res['reason'] or 'all accessible corridors covered'}")
            self.publish_coverage()
        self.status_pub.publish(String(data=json.dumps(info)))

    def publish_coverage(self):
        stamp = self.get_clock().now().to_msg()
        summ = self.explorer.tracker.summary()
        summ.update(t=stamp.sec + stamp.nanosec * 1e-9, frame_id='map')
        self.cov_pub.publish(String(data=json.dumps(summ)))
        arr = MarkerArray()
        tr = self.explorer.tracker
        for i, e in enumerate(self.graph.edges):
            states = tr.bin_states(e)
            half = 0.5 * self.graph.length(e) / len(tr.centres[e])
            mk = Marker()
            mk.header.stamp, mk.header.frame_id = stamp, 'map'
            mk.ns, mk.id, mk.type, mk.action = 'coverage', i, Marker.LINE_LIST, Marker.ADD
            mk.scale.x = 0.12
            mk.pose.orientation.w = 1.0
            for s, st in zip(tr.centres[e], states):
                col = COLOURS.get(st, (0.3, 0.3, 0.3))
                for q in (self.graph.point(e, s - half), self.graph.point(e, s + half)):
                    mk.points.append(Point(x=float(q[0]), y=float(q[1]), z=0.02))
                    mk.colors.append(self._rgba(col))
            arr.markers.append(mk)
        self.marker_pub.publish(arr)

    @staticmethod
    def _rgba(c):
        return ColorRGBA(r=float(c[0]), g=float(c[1]), b=float(c[2]), a=0.9)


def main():
    rclpy.init()
    node = ExplorationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
