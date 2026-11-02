"""ROS 2 node (CUSTOM) that counts robot collisions from the bridged Gazebo contact sensors.

Subscribes /contacts/chassis, /contacts/left_wheel, /contacts/right_wheel (ros_gz_interfaces/Contacts) and publishes `collisions` (std_msgs/String, JSON from
CollisionCounter.summary(): total, wall, debris, other, contact_time, events) at 2 Hz and whenever a collision is counted.
This is an EVALUATION node: nothing in localization, planning or driving reads it.
"""
import json

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import String

from .collisions import CollisionCounter, partner_model


class CollisionCounterNode(Node):
    def __init__(self):
        super().__init__('collision_counter')
        self.counter = CollisionCounter(gap=float(self.declare_parameter('gap', 1.0).value))
        self.pub = self.create_publisher(String, 'collisions', 10)
        for name in ('chassis', 'left_wheel', 'right_wheel'):         # bridged by robot_sim.launch.py
            self.create_subscription(Contacts, 'contacts/' + name, self.on_contacts, 50)
        self.create_timer(0.5, self.publish)

    def on_contacts(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        models = {partner_model(c.collision1.name, c.collision2.name) for c in m.contacts} - {None}
        new = self.counter.update(t, models)
        for t_ev, model in new:
            self.get_logger().info(f'COLLISION #{len(self.counter.events)}: {model} at t={t_ev:.1f}')
        if new:
            self.publish()

    def publish(self):
        self.pub.publish(String(data=json.dumps(self.counter.summary())))


def main():
    rclpy.init()
    node = CollisionCounterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
