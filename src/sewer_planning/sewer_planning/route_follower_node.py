"""ROS 2 route follower (CUSTOM): pure pursuit along /route using the ESTIMATED pose.

Publishes /cmd_vel (geometry_msgs/Twist, as the Gazebo bridge expects) and /mission_status
(std_msgs/String: WAITING, RUNNING, GOAL_REACHED, NO_ROUTE). Ground truth is never read, so any
localization error shows up in how well the robot drives. The robot stops when it reaches the end of the
route (the goal node). With `health_topic` set it also reads the localizer's health state and slows (DEGRADED) or
waits (CRITICAL, then creeps) instead of driving at full speed on a doubtful estimate. This is deliberately a small controller: Nav2's grid planners and costmaps are not used.
"""
import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

from .pursuit import health_speed_factor, pursuit_target


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class RouteFollowerNode(Node):
    def __init__(self):
        super().__init__('route_follower')
        d = self.declare_parameter
        pose_topic = d('pose_topic', '/reliability/estimated_pose').value
        self.vmax = float(d('vmax', 0.35).value)
        self.lookahead = float(d('lookahead', 0.5).value)
        self.goal_tol = float(d('goal_tol', 0.2).value)
        health_topic = d('health_topic', '').value                # '' = ignore the localization health
        # Exploration missions send a new goal after each one is reached: with rearm_on_new_goal the follower accepts a
        # route whose end point differs from the reached goal. Default False keeps the original single-goal behaviour.
        self.rearm = bool(d('rearm_on_new_goal', False).value)
        self.health, self.critical_since = 'RELIABLE', None
        self.pose, self.path, self.reached, self.reached_at = None, None, False, None
        self.create_subscription(PoseWithCovarianceStamped, pose_topic, self.on_pose, qos_profile_sensor_data)
        self.create_subscription(Path, 'route', self.on_path, 10)
        if health_topic:
            self.create_subscription(String, health_topic, self.on_health, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, 'mission_status', 10)
        self.create_timer(0.05, self.control)
        self.create_timer(0.5, lambda: self.status_pub.publish(String(data=self.status())))

    def on_pose(self, m):
        q = m.pose.pose.orientation
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y,
                     math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def on_health(self, m):
        state = json.loads(m.data)['state']
        if state == 'CRITICAL' and self.health != 'CRITICAL':
            self.critical_since = self.get_clock().now()
        self.health = state

    def on_path(self, m):
        self.path = np.array([[p.pose.position.x, p.pose.position.y] for p in m.poses]) if m.poses else None
        if self.rearm and self.reached and self.path is not None and self.reached_at is not None and \
                math.hypot(*(self.path[-1] - self.reached_at)) > self.goal_tol:
            self.reached = False

    def status(self):
        if self.reached:
            return 'GOAL_REACHED'
        if self.pose is None or self.path is None:
            return 'WAITING' if self.pose is None else 'NO_ROUTE'
        return 'RUNNING'

    def stop(self):
        self.cmd_pub.publish(Twist())

    def control(self):
        if self.reached or self.pose is None or self.path is None:
            return self.stop()
        x, y, yaw = self.pose
        pts = self.path
        goal = pts[-1]
        dist_goal = math.hypot(goal[0] - x, goal[1] - y)
        if dist_goal < self.goal_tol:
            self.reached, self.reached_at = True, np.array(goal, float)
            self.get_logger().info('goal reached')
            return self.stop()
        target = pursuit_target(pts, x, y, self.lookahead)
        alpha = wrap(math.atan2(target[1] - y, target[0] - x) - yaw)
        cmd = Twist()
        waited = ((self.get_clock().now() - self.critical_since).nanoseconds * 1e-9
                  if self.health == 'CRITICAL' and self.critical_since else 0.0)
        k = health_speed_factor(self.health, waited)
        if abs(alpha) > 0.6:
            cmd.angular.z = max(-1.0, min(1.0, 1.5 * alpha))
        else:
            cmd.linear.x = min(self.vmax * max(0.3, 1.0 - abs(alpha)), 0.5 * dist_goal + 0.1)
            cmd.angular.z = max(-1.0, min(1.0, 2.0 * alpha))
        cmd.linear.x, cmd.angular.z = k * cmd.linear.x, k * cmd.angular.z
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = RouteFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
