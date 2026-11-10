"""Small ROS-message helpers shared by the Account 2 nodes (no cv_bridge dependency). CUSTOM code."""
import bisect
import math

import numpy as np


def stamp_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def image_to_numpy(msg):
    """sensor_msgs/Image -> numpy. rgb8/bgr8/rgba8 -> uint8 [H,W,3] RGB; 32FC1 -> float32 [H,W]; 16UC1 (mm) -> metres."""
    h, w, step, enc = msg.height, msg.width, msg.step, msg.encoding.lower()
    buf = np.frombuffer(bytes(msg.data), np.uint8)
    if enc in ('rgb8', 'bgr8', 'rgba8', 'bgra8'):
        ch = 4 if enc.endswith('a8') else 3
        img = buf.reshape(h, step)[:, :w * ch].reshape(h, w, ch)[..., :3]
        return img[..., ::-1].copy() if enc.startswith('bgr') else img.copy()
    if enc == '32fc1':
        return buf.reshape(h, step)[:, :w * 4].copy().view('<f4' if not msg.is_bigendian else '>f4').reshape(h, w)
    if enc in ('16uc1', 'mono16'):
        d = buf.reshape(h, step)[:, :w * 2].copy().view('<u2').reshape(h, w).astype(np.float32) / 1000.0
        d[d == 0] = np.inf
        return d
    if enc == 'mono8':
        g = buf.reshape(h, step)[:, :w]
        return np.repeat(g[..., None], 3, -1).copy()
    raise ValueError(f'unsupported image encoding {msg.encoding}')


class PoseHistory:
    """Recent (t, x, y, yaw) estimates; lookup of the estimate closest to a sensor time stamp."""

    def __init__(self, maxlen=600):
        self.t, self.p, self.maxlen = [], [], maxlen

    def add(self, t, pose):
        if self.t and t < self.t[-1]:          # time went backwards (sim reset): start over
            self.t, self.p = [], []
        self.t.append(t)
        self.p.append(tuple(pose))
        if len(self.t) > self.maxlen:
            del self.t[:len(self.t) - self.maxlen]
            del self.p[:len(self.p) - self.maxlen]

    def at(self, t, tol=0.12):
        if not self.t:
            return None
        k = bisect.bisect_left(self.t, t)
        best = min((i for i in (k - 1, k) if 0 <= i < len(self.t)), key=lambda i: abs(self.t[i] - t))
        return self.p[best] if abs(self.t[best] - t) <= tol else None

    def latest(self):
        return (self.t[-1], self.p[-1]) if self.t else (None, None)
