"""Object-level debris detection from the depth camera (and LiDAR evidence clusters for objects out of camera range).

CUSTOM code (Account 2), pure NumPy/SciPy, no ROS imports.

Why a depth camera: the 2D LiDAR scans at z = 0.13 m, so debris small enough to pick up (< 0.13 m tall) is
invisible to it. The existing EvidenceGrid (sewer_perception.blockage) is kept unchanged for blockages/route
planning; this module adds SEPARATE objects with dimensions.

Per depth frame:
  0. register the frame to the a-priori walls: the wall points (0.35-0.95 m high, above any debris) are aligned to
     the map's wall faces by a few damped Gauss-Newton steps on a signed distance field. This removes the few
     centimetres / degree of localization error that would otherwise smear walls into 'debris' (frame-to-map
     registration from sensor data; ground truth is never used)
  1. pixels -> 3D points in the map frame with the (registered) ESTIMATED pose
  2. keep points that stand above the floor (z > z_min, which rejects the water film and sludge patches),
     are clear of the a-priori walls, outside known fixtures (the disposal station) and outside the robot's own
     arm/held-object exclusion box
  3. bin them into a 2.5 cm floor grid, label connected cells -> one cluster per object
  4. reject clusters that do not reach down to the floor (inflow pipes, ladders on the walls) or are too small
The tracker (object_tracker.py) fuses clusters over frames into stable objects with dimensions.
"""
import math
from dataclasses import dataclass, field

import numpy as np
from scipy import ndimage

from .camera_model import CameraModel


@dataclass
class DebrisParams:
    z_min: float = 0.025           # floor clutter (water 0.012 m, sludge 0.018 m) stays below this
    z_max: float = 0.70
    wall_clear: float = 0.10       # ignore points within this distance (map distance field) of an a-priori wall
    max_range: float = 3.0         # horizontal distance from the camera [m]
    cell: float = 0.025            # clustering grid [m]
    min_points: int = 12           # per cluster per frame
    floor_touch: float = 0.08      # a cluster's lowest point must be below this (debris rests on the floor)
    stride: int = 1                # depth pixel subsampling
    register: bool = True          # align each frame to the a-priori walls before detecting
    reg_max_xy: float = 0.15       # largest accepted registration correction [m]
    reg_max_yaw: float = 0.07      # [rad] (~4 deg)
    ignore_zones: list = field(default_factory=list)   # [(x_min, x_max, y_min, y_max)] map frame, known fixtures


@dataclass
class Cluster:
    points: np.ndarray             # [N, 3] map frame
    centroid: tuple
    bottom: float
    top: float
    source: str = 'depth'
    rel_base: tuple = None         # centroid in base_link (x, y) at detection time (for the manipulator)
    cam_xy: tuple = None           # camera position in the map when it was seen (viewing direction)


def footprint(xy, d0=0.01):
    """Bounding rectangle of 2D points: (cx, cy, length, width, yaw) with length >= width and yaw the direction of the
    long side in (-pi/2, pi/2].

    A camera mostly sees two faces of an object (an L-shaped outline), for which the minimum-area rectangle is
    ambiguous (a square seen on two faces fits a 45-degree rectangle just as well). The orientation is therefore
    chosen with the closeness criterion of L-shape fitting (Zhang et al., 2017): the rectangle whose edges the points
    lie closest to. The extent is then the bounding box in that orientation.
    """
    xy = np.asarray(xy, float)
    if len(xy) < 3:
        c = xy.mean(0) if len(xy) else np.zeros(2)
        return float(c[0]), float(c[1]), 0.0, 0.0, 0.0
    best = None
    for a in np.deg2rad(np.arange(0, 90, 1.0)):
        ca, sa = math.cos(a), math.sin(a)
        u = xy[:, 0] * ca + xy[:, 1] * sa
        v = -xy[:, 0] * sa + xy[:, 1] * ca
        du = np.minimum(u - u.min(), u.max() - u)
        dv = np.minimum(v - v.min(), v.max() - v)
        score = float(np.sum(1.0 / np.maximum(np.minimum(du, dv), d0)))
        if best is None or score > best[0] + 1e-9:
            cu, cv = (u.max() + u.min()) / 2, (v.max() + v.min()) / 2
            best = (score, cu * ca - cv * sa, cu * sa + cv * ca, u.max() - u.min(), v.max() - v.min(), a)
    _, cx, cy, lu, lv, a = best
    if lv > lu:
        lu, lv, a = lv, lu, a + math.pi / 2
    a = math.atan2(math.sin(a), math.cos(a))
    if a <= -math.pi / 2:
        a += math.pi
    elif a > math.pi / 2:
        a -= math.pi
    return float(cx), float(cy), float(lu), float(lv), float(a)


def roundness(xy, fp):
    """Median distance of the points to the nearest side of the fitted rectangle, relative to its width: ~0 for a
    box (points lie on its faces), ~0.15 or more for a cylinder (its outline bulges away from the corners)."""
    cx, cy, L, W, yaw = fp
    if W <= 0 or len(xy) < 8:
        return 0.0
    ca, sa = math.cos(yaw), math.sin(yaw)
    d = np.asarray(xy, float) - (cx, cy)
    u = d[:, 0] * ca + d[:, 1] * sa
    v = -d[:, 0] * sa + d[:, 1] * ca
    dist = np.minimum(L / 2 - np.abs(u), W / 2 - np.abs(v))
    return float(np.median(np.maximum(dist, 0)) / W)


def signed_distance(prior):
    """Signed distance to the wall surface [m]: + in free space, - inside walls."""
    free = ~prior.solid
    return (ndimage.distance_transform_edt(free) - ndimage.distance_transform_edt(~free)) * prior.res


def _bilinear(field, m, x, y):
    fx = (x - m.x0) / m.res - 0.5
    fy = (y - m.y0) / m.res - 0.5
    i0 = np.clip(np.floor(fx).astype(int), 0, m.w - 2)
    j0 = np.clip(np.floor(fy).astype(int), 0, m.h - 2)
    ax = np.clip(fx - i0, 0, 1)
    ay = np.clip(fy - j0, 0, 1)
    f00, f10 = field[j0, i0], field[j0, i0 + 1]
    f01, f11 = field[j0 + 1, i0], field[j0 + 1, i0 + 1]
    v = (f00 * (1 - ax) + f10 * ax) * (1 - ay) + (f01 * (1 - ax) + f11 * ax) * ay
    gx = ((f10 - f00) * (1 - ay) + (f11 - f01) * ay) / m.res
    gy = ((f01 - f00) * (1 - ax) + (f11 - f10) * ax) / m.res
    return v, gx, gy


def register_to_walls(prior, sdf, pw, pose, iters=6, huber=0.04, damping=50.0):
    """Small 2D correction (dx, dy, dyaw) about the robot that puts wall points pw [N,3] onto the a-priori wall faces.

    Damped Gauss-Newton: directions the walls do not constrain (along a straight corridor) stay near zero.
    Returns (corrected pose, (dx, dy, dyaw), number of points used)."""
    x, y, th = pose
    wall = (pw[:, 2] > 0.35) & (pw[:, 2] < 0.95)
    P = pw[wall, :2]
    if len(P) < 150:
        return pose, (0.0, 0.0, 0.0), int(len(P))
    v0, _, _ = _bilinear(sdf, prior, P[:, 0], P[:, 1])
    P = P[np.abs(v0) < 0.20]                      # near a mapped wall face; pipes and ladders stick out further
    if len(P) < 150:
        return pose, (0.0, 0.0, 0.0), int(len(P))
    c = np.array([x, y])
    d = np.zeros(3)
    for _ in range(iters):
        ca, sa = np.cos(d[2]), np.sin(d[2])
        rel = P - c
        Q = c + d[:2] + np.stack([ca * rel[:, 0] - sa * rel[:, 1], sa * rel[:, 0] + ca * rel[:, 1]], -1)
        r, gx, gy = _bilinear(sdf, prior, Q[:, 0], Q[:, 1])
        qr = Q - c - d[:2]
        J = np.stack([gx, gy, -gx * qr[:, 1] + gy * qr[:, 0]], -1)
        w = np.where(np.abs(r) <= huber, 1.0, huber / np.maximum(np.abs(r), 1e-9))
        H = (J * w[:, None]).T @ J + damping * np.diag([1.0, 1.0, 4.0])
        step = -np.linalg.solve(H, (J * w[:, None]).T @ r)
        d += step
        if np.abs(step).max() < 1e-4:
            break
    # the correction rotates the points about the robot: the robot pose moves by d[:2] and turns by d[2]
    return (x + d[0], y + d[1], th + d[2]), (float(d[0]), float(d[1]), float(d[2])), int(len(P))


class DepthDebrisDetector:
    def __init__(self, prior, cam: CameraModel, params=None):
        self.map = prior
        self.cam = cam
        self.p = params or DebrisParams()
        self.sdf = signed_distance(prior)
        self.last_correction = (0.0, 0.0, 0.0)

    def register(self, depth, pose):
        """Pose corrected by aligning this depth frame's wall points to the map (or the input pose if it fails)."""
        p = self.p
        if not p.register:
            return pose
        pc, _, _ = self.cam.depth_to_points(depth, 2)
        if len(pc) == 0:
            return pose
        new, d, n = register_to_walls(self.map, self.sdf, self.cam.to_world(pc, pose), pose)
        if math.hypot(d[0], d[1]) > p.reg_max_xy or abs(d[2]) > p.reg_max_yaw:
            return pose
        self.last_correction = d
        return new

    def candidate_points(self, depth, pose, exclusion_base=None):
        """Map-frame points that may belong to debris, and the same points in base_link (x, y)."""
        p = self.p
        pc, _, _ = self.cam.depth_to_points(depth, p.stride)
        if len(pc) == 0:
            return np.zeros((0, 3)), np.zeros((0, 2))
        pw = self.cam.to_world(pc, pose)
        pb = self.cam.to_base(pc)
        R, t = self.cam.cam_to_world(pose)
        keep = (pw[:, 2] > p.z_min) & (pw[:, 2] < p.z_max)
        keep &= np.hypot(pw[:, 0] - t[0], pw[:, 1] - t[1]) < p.max_range
        keep &= self.map.dist(pw[:, 0], pw[:, 1]) > p.wall_clear
        for x0, x1, y0, y1 in p.ignore_zones:
            keep &= ~((pw[:, 0] >= x0) & (pw[:, 0] <= x1) & (pw[:, 1] >= y0) & (pw[:, 1] <= y1))
        if exclusion_base is not None:          # robot's own arm / held object: (x0, x1, y0, y1, z0, z1) in base_link
            x0, x1, y0, y1, z0, z1 = exclusion_base
            keep &= ~((pb[:, 0] >= x0) & (pb[:, 0] <= x1) & (pb[:, 1] >= y0) & (pb[:, 1] <= y1)
                      & (pb[:, 2] >= z0) & (pb[:, 2] <= z1))
        return pw[keep], pb[keep, :2]

    def detect(self, depth, pose, exclusion_base=None, registered=False):
        """Clusters in this frame. The frame is registered to the walls first unless `registered` (already done)."""
        p = self.p
        if not registered:
            pose = self.register(depth, pose)
        pw, pb = self.candidate_points(depth, pose, exclusion_base)
        if len(pw) < p.min_points:
            return []
        ij = np.floor(pw[:, :2] / p.cell).astype(int)
        lo = ij.min(0)
        ij -= lo
        grid = np.zeros(ij.max(0) + 1, bool)
        grid[ij[:, 0], ij[:, 1]] = True
        cam_xy = tuple(float(v) for v in self.cam.cam_to_world(pose)[1][:2])
        labels, n = ndimage.label(ndimage.binary_dilation(grid, np.ones((3, 3))), np.ones((3, 3)))
        lab = labels[ij[:, 0], ij[:, 1]]
        out = []
        for k in range(1, n + 1):
            sel = lab == k
            if sel.sum() < p.min_points:
                continue
            pts = pw[sel]
            bottom, top = float(np.percentile(pts[:, 2], 2)), float(np.percentile(pts[:, 2], 98))
            if bottom > p.floor_touch or pts[:, 2].max() > p.z_max - 0.03:
                continue
            c = pts.mean(0)
            rb = pb[sel].mean(0)
            out.append(Cluster(pts, (float(c[0]), float(c[1]), float(c[2])), bottom, top, 'depth',
                               (float(rb[0]), float(rb[1])), cam_xy))
        return out


def lidar_clusters(grid, min_cells=6, bridge=0.5):
    """Separate objects from the existing EvidenceGrid (unchanged): one Cluster per obstacle region.

    A 2D LiDAR only ever sees the faces of a pile that point at the robot; seen from both sides, one pile becomes two
    separate strips of evidence with never-observed space inside. Strips joined through corridor cells that no
    trusted ray has crossed (within `bridge` metres) are therefore one object.
    Height is unknown from a 2D scan; the object reaches at least the LiDAR plane (0.13 m), recorded as bottom 0 /
    top 0.13 (a lower bound) with source 'lidar'.
    """
    occ = grid.L >= grid.p.occ_thresh
    if not occ.any():
        return []
    m = grid.map
    r = max(1, int(round(bridge / m.res)))
    near = ndimage.binary_dilation(occ, np.ones((2 * r + 1, 2 * r + 1)))
    connector = near & ~grid.seen & ~m.solid
    labels, n = ndimage.label(occ | connector, np.ones((3, 3)))
    labels[~occ] = 0
    out = []
    for k in range(1, n + 1):
        j, i = np.nonzero(labels == k)
        if len(i) < min_cells:
            continue
        x = m.x0 + (i + 0.5) * m.res
        y = m.y0 + (j + 0.5) * m.res
        pts = np.stack([x, y, np.full_like(x, 0.13)], -1)
        out.append(Cluster(pts, (float(x.mean()), float(y.mean()), 0.13), 0.0, 0.13, 'lidar'))
    return out
