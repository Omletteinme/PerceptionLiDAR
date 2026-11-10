"""Pinhole model of the robot's front cameras and the frame chain camera_link -> base_link -> map.

CUSTOM code (Account 2), pure NumPy, no ROS imports.

Gazebo conventions (gz-sim camera / depth_camera sensors, SIMULATED):
  - the sensor looks along +x of its frame (camera_link); image column u grows to -y, row v grows to -z
  - fx = fy = (width / 2) / tan(hfov / 2), principal point at the image centre
  - the depth image holds PLANAR depth along +x (not the length of the ray), in metres; no return = inf
So a pixel (u, v) with depth d is the point d * (1, -(u - cx) / fx, -(v - cy) / fy) in camera_link.

Frames: camera_link is fixed on base_link (sewer_robot.urdf.xacro: xyz 0.16 0 0.05, rpy 0 0.05 0); base_link is
BASE_Z = 0.06 m above the floor (wheel radius); the localizer gives the 2D pose (x, y, yaw) of base_footprint in map.
World/map z is height above the floor.
"""
import math
from dataclasses import dataclass, field

import numpy as np

BASE_Z = 0.06


def rot_rpy(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch),
                              math.cos(yaw), math.sin(yaw))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


@dataclass
class CameraModel:
    width: int = 320
    height: int = 180
    hfov: float = 1.5
    mount_xyz: tuple = (0.16, 0.0, 0.05)        # camera_link in base_link
    mount_rpy: tuple = (0.0, 0.05, 0.0)
    near: float = 0.10
    far: float = 5.0
    _rays: np.ndarray = field(default=None, repr=False)

    @property
    def fx(self):
        return (self.width / 2) / math.tan(self.hfov / 2)

    @property
    def cx(self):
        return self.width / 2

    @property
    def cy(self):
        return self.height / 2

    def scaled(self, width, height):
        """Same camera at another resolution (the RGB and depth images share camera_link and hfov)."""
        return CameraModel(width, height, self.hfov, self.mount_xyz, self.mount_rpy, self.near, self.far)

    def rays_cam(self):
        """[H, W, 3] ray directions in camera_link with unit x component (so t along a ray = planar depth)."""
        if self._rays is None:
            u = np.arange(self.width) + 0.5
            v = np.arange(self.height) + 0.5
            uu, vv = np.meshgrid(u, v)
            self._rays = np.stack([np.ones_like(uu), -(uu - self.cx) / self.fx, -(vv - self.cy) / self.fx], -1)
        return self._rays

    # -------------------------------------------------------------- frames
    def cam_to_base(self):
        """(R, t): p_base = R @ p_cam + t."""
        return rot_rpy(*self.mount_rpy), np.array(self.mount_xyz, float)

    def cam_to_world(self, pose):
        """(R, t) from camera_link to map/world (z = height above floor) for a 2D base pose (x, y, yaw)."""
        Rb, tb = self.cam_to_base()
        x, y, yaw = pose
        Rz = rot_rpy(0.0, 0.0, yaw)
        return Rz @ Rb, Rz @ tb + np.array([x, y, BASE_Z])

    def depth_to_points(self, depth, stride=1):
        """[N, 3] camera_link points of all finite depth pixels in [near, far], plus their (v, u) indices."""
        d = np.asarray(depth, float)[::stride, ::stride]
        rays = self.rays_cam()[::stride, ::stride]
        ok = np.isfinite(d) & (d >= self.near) & (d <= self.far)
        vv, uu = np.nonzero(ok)
        return rays[ok] * d[ok][:, None], vv * stride, uu * stride

    def to_world(self, p_cam, pose):
        R, t = self.cam_to_world(pose)
        return p_cam @ R.T + t

    def to_base(self, p_cam):
        R, t = self.cam_to_base()
        return p_cam @ R.T + t

    def project_world(self, p_world, pose):
        """Pixel (u, v) and planar depth of world points; depth <= 0 means behind the camera."""
        R, t = self.cam_to_world(pose)
        pc = (np.atleast_2d(p_world) - t) @ R
        d = pc[:, 0]
        with np.errstate(divide='ignore', invalid='ignore'):
            u = self.cx - self.fx * pc[:, 1] / d
            v = self.cy - self.fx * pc[:, 2] / d
        return u, v, d

    def world_rays(self, pose):
        """Camera origin and [H, W, 3] world-frame ray directions (unit x component in the camera frame)."""
        R, t = self.cam_to_world(pose)
        return t, self.rays_cam() @ R.T


def base_to_map(pts_base, pose):
    """2D base_link points -> map (x, y). pts_base [N, >=2]."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    p = np.atleast_2d(pts_base)
    return np.stack([x + c * p[:, 0] - s * p[:, 1], y + s * p[:, 0] + c * p[:, 1]], -1)


def map_to_base(pts_map, pose):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    p = np.atleast_2d(pts_map)
    dx, dy = p[:, 0] - x, p[:, 1] - y
    return np.stack([c * dx + s * dy, -s * dx + c * dy], -1)
