"""ROS 2 wrapper for the simulated arm/gripper (Account 2). CUSTOM node; the logic is in manipulation_bridge.py and
manipulation_controller.py (pure Python, tested offline, also driven by the sandbox end-to-end mission test).

It answers Account 1's /mission/task (PICKUP / DISPOSE / STOP, docs/mission_interfaces.md) and reports
/manipulation/status (5 Hz and on every change) and /disposal/status. It never publishes cmd_vel or /route: the
Mission Manager drives the robot and stops it before asking for a pickup or a disposal.

Inputs:
  /mission/task                 String JSON, latched by the Mission Manager (subscribed TRANSIENT_LOCAL)
  /perception/debris            String JSON (object size and position; last_seen_base = relative to the robot)
  pose_topic                    ESTIMATED pose (map -> base): object and tray positions relative to the robot
  /joint_states                 arm + finger joint positions (Gazebo JointStatePublisher, SIMULATED)
  /contacts/left_finger, /contacts/right_finger   ros_gz_interfaces/Contacts (finger contact sensors, SIMULATED)
  depth_topic                   depth image: points inside the disposal tray (sensor-based disposal check)
  /sim/dynamic_poses            tf2_msgs/TFMessage of Gazebo's model poses (SIMULATED STATE, optional,
                                use_sim_pose:=true): extra evidence for lift and disposal, never for perception
Outputs:
  arm/<joint>/cmd               std_msgs/Float64 position targets (bridged to Gazebo JointPositionController)
  /manipulation/status, /disposal/status   String JSON
"""
import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64, String

from sewer_perception.camera_model import CameraModel
from sewer_perception.ros_utils import image_to_numpy, yaw_of

from .manipulation_bridge import ManipulationBridge

CMD_TOPICS = {'arm_turret_joint': 'arm/turret/cmd', 'arm_shoulder_joint': 'arm/shoulder/cmd',
              'arm_elbow_joint': 'arm/elbow/cmd', 'arm_wrist_joint': 'arm/wrist/cmd',
              'arm_wrist_yaw_joint': 'arm/wrist_yaw/cmd',
              'left_finger_joint': 'arm/left_finger/cmd', 'right_finger_joint': 'arm/right_finger/cmd'}


class ManipulationNode(Node):
    def __init__(self):
        super().__init__('manipulation')
        d = self.declare_parameter
        station_path = d('station_path', '').value
        self.use_sim_pose = bool(d('use_sim_pose', True).value)
        station = None
        if station_path:
            with open(station_path) as f:
                station = yaml.safe_load(f)
        self.b = ManipulationBridge(station, log=self.get_logger().info)
        self.dcam = CameraModel(320, 180)
        self.last_state = None
        self.cmd_pubs = {j: self.create_publisher(Float64, t, 10) for j, t in CMD_TOPICS.items()}
        self.status_pub = self.create_publisher(String, '/manipulation/status', 10)
        self.disposal_pub = self.create_publisher(String, '/disposal/status', 10)
        latched = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/mission/task', lambda m: self.b.on_task(m.data, self.now()), latched)
        self.create_subscription(String, '/perception/debris', lambda m: self.b.on_debris(m.data), 10)
        self.create_subscription(PoseWithCovarianceStamped, d('pose_topic', '/reliability/estimated_pose').value,
                                 self.on_pose, qos_profile_sensor_data)
        # wheel odometry: carries the camera's last sighting of an object forward when it is beside the robot
        self.create_subscription(Odometry, d('odom_topic', '/odom').value, self.on_odom, qos_profile_sensor_data)
        self.create_subscription(JointState, '/joint_states', lambda m: self.b.on_joints(m.name, m.position),
                                 qos_profile_sensor_data)
        self.create_subscription(Image, d('depth_topic', '/robot_depth').value, self.on_depth, qos_profile_sensor_data)
        try:
            from ros_gz_interfaces.msg import Contacts
            for side in ('left', 'right'):
                self.create_subscription(Contacts, f'/contacts/{side}_finger',
                                         lambda m, s=side: self.on_contacts(s, m), 50)
        except ImportError:
            self.get_logger().error('ros_gz_interfaces missing: no finger contacts, every grasp will fail verification')
        if self.use_sim_pose:
            from tf2_msgs.msg import TFMessage
            self.create_subscription(TFMessage, '/sim/dynamic_poses', self.on_sim_poses, 10)
        self.create_timer(0.05, self.tick)
        self.create_timer(0.2, self.publish_status)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_pose(self, m):
        p = m.pose.pose
        self.b.on_pose((p.position.x, p.position.y, yaw_of(p.orientation)))

    def on_odom(self, m):
        p = m.pose.pose
        self.b.on_odom(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                       (p.position.x, p.position.y, yaw_of(p.orientation)))

    def on_contacts(self, side, m):
        t = self.now()
        for c in m.contacts:
            self.b.on_contact(side, c.collision1.name, c.collision2.name, t)

    def on_sim_poses(self, m):
        for tr in m.transforms:
            v = tr.transform.translation
            self.b.on_sim_pose(tr.child_frame_id, (v.x, v.y, v.z))

    def on_depth(self, m):
        if self.b.pose is None:
            return
        depth = image_to_numpy(m).astype(float)
        if depth.shape != (self.dcam.height, self.dcam.width):
            self.dcam = CameraModel(depth.shape[1], depth.shape[0])
        self.b.tray_count = self.b.tray_points(self.dcam, depth, self.b.pose, self.now()) if self.b.wants_tray() else None

    def tick(self):
        cmd = self.b.tick(self.now(), self.use_sim_pose)
        if cmd is None:
            return
        for j, v in cmd.items():
            self.cmd_pubs[j].publish(Float64(data=float(v)))
        if self.b.ctl.state != self.last_state:
            self.last_state = self.b.ctl.state
            self.publish_status()

    def publish_status(self):
        t = self.now()
        self.status_pub.publish(String(data=self.b.status_json(t)))
        if self.b.disposal_task is not None:
            self.disposal_pub.publish(String(data=self.b.disposal_json(t)))


def main():
    rclpy.init()
    node = ManipulationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
