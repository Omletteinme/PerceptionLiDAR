"""Record a synthetic perception dataset from Gazebo (Account 2). CUSTOM node.

Every `period` seconds of sim time it saves, under out_dir:
  rgb/<k>.png          robot_cam frame (SIMULATED by Gazebo)
  depth/<k>.npy        robot_depth frame, float32 planar depth [m]
  scan/<k>.npz         the LiDAR scan nearest in time (ranges, angles, per-beam reliability if published)
  index.jsonl          one line per frame: stamp, ESTIMATED pose (what perception uses) and, in a separate field
                       'eval_only_true_pose', the ground-truth pose from /ground_truth/odom
Labels come later from config/inspection_truth.yaml (sandbox/label_dataset.py); the ground truth recorded here is
used for scoring only, never as an input to the detectors. Evaluate with
  python3 sandbox/run_inspection_eval.py --dataset ~/sewer_ws/logs/dataset
"""
import json
import os

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan

from .ros_utils import PoseHistory, image_to_numpy, stamp_sec, yaw_of


class DatasetRecorder(Node):
    def __init__(self):
        super().__init__('dataset_recorder')
        d = self.declare_parameter
        self.out = os.path.expanduser(d('out_dir', '~/sewer_ws/logs/dataset').value)
        self.period = float(d('period', 0.5).value)
        for sub in ('rgb', 'depth', 'scan'):
            os.makedirs(os.path.join(self.out, sub), exist_ok=True)
        self.index = open(os.path.join(self.out, 'index.jsonl'), 'a')
        self.est, self.true = PoseHistory(), PoseHistory()
        self.last_depth, self.last_scan, self.t_last, self.k = None, None, -1e9, 0
        self.create_subscription(PoseWithCovarianceStamped, d('pose_topic', '/reliability/estimated_pose').value,
                                 lambda m: self._pose(self.est, m.header.stamp, m.pose.pose), qos_profile_sensor_data)
        self.create_subscription(Odometry, '/ground_truth/odom',
                                 lambda m: self._pose(self.true, m.header.stamp, m.pose.pose), qos_profile_sensor_data)
        self.create_subscription(Image, d('depth_topic', '/robot_depth').value, self.on_depth, qos_profile_sensor_data)
        self.create_subscription(Image, d('rgb_topic', '/robot_cam/image').value, self.on_rgb, qos_profile_sensor_data)
        self.create_subscription(LaserScan, d('scan_topic', '/reliability/reliability_scan').value, self.on_scan,
                                 qos_profile_sensor_data)
        self.get_logger().info(f'recording to {self.out}')

    def _pose(self, hist, stamp, p):
        hist.add(stamp_sec(stamp), (p.position.x, p.position.y, yaw_of(p.orientation)))

    def on_scan(self, m):
        r = np.array(m.ranges, np.float32)
        self.last_scan = (stamp_sec(m.header.stamp), r,
                          (m.angle_min + np.arange(len(r)) * m.angle_increment).astype(np.float32),
                          np.array(m.intensities, np.float32) if len(m.intensities) == len(r) else np.zeros(0, np.float32))

    def on_depth(self, m):
        self.last_depth = (stamp_sec(m.header.stamp), image_to_numpy(m))

    def on_rgb(self, m):
        t = stamp_sec(m.header.stamp)
        if t - self.t_last < self.period or self.last_depth is None or abs(t - self.last_depth[0]) > 0.15:
            return
        est, true = self.est.at(t), self.true.at(t, 0.05)
        if est is None:
            return
        from PIL import Image as PILImage
        name = f'{self.k:05d}'
        PILImage.fromarray(image_to_numpy(m)).save(os.path.join(self.out, 'rgb', name + '.png'))
        np.save(os.path.join(self.out, 'depth', name + '.npy'), self.last_depth[1].astype(np.float32))
        if self.last_scan is not None and abs(t - self.last_scan[0]) < 0.2:
            np.savez(os.path.join(self.out, 'scan', name + '.npz'), ranges=self.last_scan[1], angles=self.last_scan[2],
                     reliability=self.last_scan[3])
        self.index.write(json.dumps(dict(frame=name, stamp=round(t, 3), depth_stamp=round(self.last_depth[0], 3),
                                         est_pose=[round(v, 4) for v in est],
                                         eval_only_true_pose=None if true is None else [round(v, 4) for v in true]))
                         + '\n')
        self.index.flush()
        self.t_last, self.k = t, self.k + 1


def main():
    rclpy.init()
    node = DatasetRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.index.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
