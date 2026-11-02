#!/usr/bin/env python3
"""Save one frame of the world's top-down camera (/topdown_cam/image, bridged) to a PNG. Scene check only."""
import sys

import matplotlib
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

matplotlib.use('Agg')
import matplotlib.image as mpimg  # noqa: E402

rclpy.init()
n = Node('topdown_saver')
got = []
n.create_subscription(Image, '/topdown_cam/image', got.append, 1)
for _ in range(300):
    rclpy.spin_once(n, timeout_sec=0.1)
    if got:
        break
if not got:
    print('NO IMAGE')
    sys.exit(1)
m = got[0]
img = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, -1)[:, :, :3]
mpimg.imsave(sys.argv[1], img)
print(f'saved {sys.argv[1]} {m.width}x{m.height} {m.encoding} mean={img.mean():.1f}')
rclpy.shutdown()
