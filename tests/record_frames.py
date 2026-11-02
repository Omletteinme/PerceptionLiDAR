#!/usr/bin/env python3
"""Save camera frames from Gazebo (bridged as sensor_msgs/Image) as JPEGs, at most `fps` frames per second of SIM time.

Usage: record_frames.py <out_dir> <fps> <max_seconds> name=/topic [name=/topic ...]
Stops after max_seconds (wall clock) or 2 s after /mission_status reports GOAL_REACHED, whichever is first.
Frames are written to <out_dir>/<name>/00000.jpg, 00001.jpg, ...
"""
import os
import sys
import time

import numpy as np
import rclpy
from PIL import Image as PILImage
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


class Recorder(Node):
    def __init__(self, out, fps, topics):
        super().__init__('frame_recorder', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.out, self.period = out, 1.0 / fps
        self.counts = {n: 0 for n in topics}
        self.times = {}
        self.last = {}
        self.goal_at = None
        for name, topic in topics.items():
            os.makedirs(f'{out}/{name}', exist_ok=True)
            self.times[name] = open(f'{out}/{name}/times.csv', 'w')   # index,sim time of every saved frame
            self.create_subscription(Image, topic, lambda m, n=name: self.on_image(m, n), qos_profile_sensor_data)
        self.create_subscription(String, '/mission_status', self.on_status, 10)

    def on_status(self, m):
        if m.data == 'GOAL_REACHED' and self.goal_at is None:
            self.goal_at = time.monotonic()

    def on_image(self, m, name):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if t - self.last.get(name, -1e9) < self.period - 1e-3:
            return
        self.last[name] = t
        rgb = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, -1)[:, :, :3]
        PILImage.fromarray(rgb).save(f'{self.out}/{name}/{self.counts[name]:05d}.jpg', quality=92)
        self.times[name].write(f'{self.counts[name]},{t:.3f}\n')
        self.times[name].flush()
        self.counts[name] += 1


def main():
    out, fps, max_s = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
    topics = dict(a.split('=', 1) for a in sys.argv[4:])
    rclpy.init()
    rec = Recorder(out, fps, topics)
    t0 = time.monotonic()
    while time.monotonic() - t0 < max_s:
        rclpy.spin_once(rec, timeout_sec=0.1)
        if rec.goal_at is not None and time.monotonic() - rec.goal_at > 2.0:
            break
    print('frames saved:', rec.counts, flush=True)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
