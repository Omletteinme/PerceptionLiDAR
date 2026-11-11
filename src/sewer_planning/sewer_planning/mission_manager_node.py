"""ROS 2 Mission Manager node (CUSTOM): the single authority that sequences the mission.

Thin wrapper around sewer_planning.mission_manager (pure NumPy, shared with sandbox/mission_sim.py). It includes the
explorer, so in a mission it replaces both route_planner_node and exploration_node. route_follower_node remains the
only node that publishes /cmd_vel.

Inputs:
  /reliability/estimated_pose       PoseWithCovarianceStamped  ESTIMATED pose (never ground truth); header stamp = age
  /reliability/scan_reliability_R   Float32                    scan reliability R (route cost)
  /reliability/localization_health  String (JSON)              RELIABLE / DEGRADED / CRITICAL
  edge_gaps                         String (JSON)              blockage detector
  /perception/debris                String (JSON)              Account 2, PROPOSED schema (docs/mission_interfaces.md)
  /perception/defects               String (JSON)              Account 2, PROPOSED schema
  /manipulation/status              String (JSON)              Account 2, PROPOSED schema
  /disposal/status                  String (JSON)              Account 2, PROPOSED schema
Outputs:
  route            nav_msgs/Path            what the follower drives; an EMPTY path means stop
  /mission/state   std_msgs/String (JSON)   state, reason, target (id, approach point, score), localization health
  /mission/task    std_msgs/String (JSON)   MissionTask (PICKUP / DISPOSE / STOP ...), published once per request
  /mission/report  std_msgs/String (JSON)   final-report structure, every 2 s and once on MISSION_COMPLETE / SAFE_STOP
  coverage/status  std_msgs/String (JSON)   CoverageTracker.summary()
Malformed or non-map-frame messages are dropped and listed under input_errors in the report.
"""
import json
import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import Float32, String

from sewer_localization.map_utils import OccupancyMap
from .graph import Graph
from .mission_interfaces import SchemaError, parse_debris, parse_defects, parse_disposal, parse_manipulation
from .mission_manager import TERMINAL, MissionManager, MissionParams
from .report_render import decimate, write_report
from .route_cost import CostParams, RouteModel
from .route_planner_node import RoutePlannerNode
from .task_prioritizer import GripperLimits, PriorityParams, TaskPrioritizer


def sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class MissionManagerNode(Node):
    def __init__(self):
        super().__init__('mission_manager')
        d = self.declare_parameter
        graph_path, map_path = d('graph_path', '').value, d('map_path', '').value
        pose_topic = d('pose_topic', '/reliability/estimated_pose').value
        r_topic = d('r_topic', '/reliability/scan_reliability_R').value
        health_topic = d('health_topic', '/reliability/localization_health').value
        if not graph_path or not map_path:
            raise ValueError("parameters 'graph_path' and 'map_path' are required")
        self.graph = Graph.load(graph_path)
        self.omap = OccupancyMap.load(map_path)
        self.report_dir = os.path.expanduser(d('report_dir', '~/sewer_ws/logs/mission_reports').value)
        self.path_xy = []
        model = RouteModel(self.graph, self.omap, CostParams(w_unc=float(d('w_unc', 2.0).value)))
        lim = GripperLimits(max_width=float(d('gripper_max_width', 0.11).value),
                            max_length=float(d('gripper_max_length', 0.32).value),
                            max_height=float(d('gripper_max_height', 0.15).value))
        prio = TaskPrioritizer(model, lim, PriorityParams(w_size=float(d('w_size', 0.4).value),
                                                          w_obstruction=float(d('w_obstruction', 0.3).value),
                                                          w_access=float(d('w_access', 0.2).value),
                                                          w_grasp=float(d('w_grasp', 0.1).value)))
        station = d('station_xy', [0.0, 0.0]).value if d('use_station_xy', False).value else None
        self.mm = MissionManager(self.graph, model, prioritizer=prio,
                                 params=MissionParams(require_manipulator=bool(d('require_manipulator', True).value),
                                                      init_timeout=float(d('init_timeout', 20.0).value)),
                                 station_xy=station)
        self.pose = self.pose_stamp = self.cov = None
        self.R, self.health, self.status = 0.9, 'RELIABLE', {}
        self.new_debris, self.new_defects, self.manip, self.disposal = [], [], None, None
        self.create_subscription(PoseWithCovarianceStamped, pose_topic, self.on_pose, qos_profile_sensor_data)
        self.create_subscription(Float32, r_topic, self.on_r, qos_profile_sensor_data)
        if health_topic:
            self.create_subscription(String, health_topic, self.on_health, 10)
        self.create_subscription(String, 'edge_gaps', self.on_gaps, 10)
        self.create_subscription(String, '/perception/debris', self.on_debris, 10)
        self.create_subscription(String, '/perception/defects', self.on_defects, 10)
        self.create_subscription(String, '/manipulation/status', self.on_manip, 10)
        self.create_subscription(String, '/disposal/status', self.on_disposal, 10)
        latched = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, 'route', 10)
        self.state_pub = self.create_publisher(String, '/mission/state', 10)
        self.task_pub = self.create_publisher(String, '/mission/task', latched)
        self.report_pub = self.create_publisher(String, '/mission/report', latched)
        self.cov_pub = self.create_publisher(String, 'coverage/status', 10)
        self.create_timer(0.5, self.step)
        self.create_timer(2.0, self.publish_report)
        self.get_logger().info(f'mission manager: home={self.mm.home} station={self.mm.station_xy.tolist()}')

    # ------------------------------------------------------------------ inputs
    def _bad(self, what, err):
        self.mm.input_errors.append(f'{what}: {err}')
        self.get_logger().warn(f'dropped {what}: {err}', throttle_duration_sec=5.0)

    def on_pose(self, m):
        c = m.pose.covariance
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y)
        self.pose_stamp = sec(m.header.stamp)
        if not self.path_xy or (self.pose[0] - self.path_xy[-1][0]) ** 2 + (self.pose[1] - self.path_xy[-1][1]) ** 2 > 0.04:
            self.path_xy.append(self.pose)
        self.cov = np.array([[c[0], c[1]], [c[6], c[7]]])

    def on_r(self, m):
        self.R = 0.9 * self.R + 0.1 * float(m.data)

    def on_health(self, m):
        self.health = json.loads(m.data).get('state', 'RELIABLE')

    def on_gaps(self, m):
        self.status = json.loads(m.data)

    def on_debris(self, m):
        try:
            objs, errs = parse_debris(m.data)
            self.new_debris += objs
            for e in errs:
                self._bad('/perception/debris', e)
        except (SchemaError, ValueError) as e:
            self._bad('/perception/debris', e)

    def on_defects(self, m):
        try:
            objs, errs = parse_defects(m.data)
            self.new_defects += objs
            for e in errs:
                self._bad('/perception/defects', e)
        except (SchemaError, ValueError) as e:
            self._bad('/perception/defects', e)

    def on_manip(self, m):
        try:
            self.manip = parse_manipulation(m.data)
        except (SchemaError, ValueError) as e:
            self._bad('/manipulation/status', e)

    def on_disposal(self, m):
        try:
            self.disposal = parse_disposal(m.data)
        except (SchemaError, ValueError) as e:
            self._bad('/disposal/status', e)

    # ------------------------------------------------------------------ loop
    def step(self):
        now_msg = self.get_clock().now().to_msg()
        now = sec(now_msg)
        was_terminal = self.mm.state in TERMINAL
        out = self.mm.tick(now, self.pose, self.pose_stamp, self.cov, self.R, self.health, self.status,
                           self.new_debris, self.new_defects, self.manip, self.disposal)
        self.new_debris, self.new_defects = [], []
        if out['task'] is not None:
            self.task_pub.publish(String(data=out['task'].to_json()))
            self.get_logger().info(f"task {out['task'].task_id}: {out['task'].type} {out['task'].object_id}")
        if out['route'] is not None and not (was_terminal and not out['route']):
            path = Path()
            path.header.stamp, path.header.frame_id = now_msg, 'map'
            if out['route']:
                RoutePlannerNode.fill_path(path, [np.asarray(p, float) for p in out['route']])
            self.path_pub.publish(path)
        self.state_pub.publish(String(data=json.dumps(dict(t=now, state=out['state'], reason=out['reason'],
                                                           target=out['target'], localization=self.health))))
        if out['state'] in TERMINAL and not was_terminal:
            self.get_logger().info(f"mission {out['state']}: {out['reason']}")
            self.publish_report()
            self.save_report(now)

    def save_report(self, now):
        """report.json, report.md and map.png under report_dir/<wall-clock time>/."""
        out = os.path.join(self.report_dir, time.strftime('%Y%m%d_%H%M%S'))
        try:
            rep = self.mm.report(now)
            edges = {k: v['state'] for k, v in rep['coverage'].get('edges', {}).items()}
            files = write_report(rep, out, self.graph, edges, self.omap, decimate(self.path_xy))
            self.get_logger().info('mission report written: ' + ', '.join(files))
        except Exception as e:                     # never let reporting take the node down
            self.get_logger().error(f'could not write the mission report to {out}: {e}')

    def publish_report(self):
        now = sec(self.get_clock().now().to_msg())
        self.report_pub.publish(String(data=json.dumps(self.mm.report(now), default=str)))
        self.cov_pub.publish(String(data=json.dumps(self.mm.explorer.tracker.summary())))


def main():
    rclpy.init()
    node = MissionManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
