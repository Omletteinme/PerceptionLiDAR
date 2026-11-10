"""Offline 3D renderer for the inspection scene: synthetic depth + RGB frames (and per-pixel labels) with no Gazebo.

CUSTOM code (Account 2), pure NumPy. A ray caster over the primitives of sewer_gazebo/scripts/inspection_scene.py
(boxes with yaw, vertical cylinders with caps, the floor, cracks painted on the walls), seen through the same
CameraModel the perception nodes use. It is a stand-in for Gazebo's renderer so the perception algorithms can be
developed and evaluated offline; flat colours with simple shading and noise, no shadows. The per-pixel label image
is GROUND TRUTH for evaluation only.
"""
import math
import os
import sys

import numpy as np

import sim2d  # noqa: F401  (puts src/* packages on sys.path)

sys.path.insert(0, os.path.join(sim2d.GAZEBO_PKG, 'scripts'))
import inspection_scene as sc  # noqa: E402
from sewer_perception.camera_model import CameraModel  # noqa: E402

FLOOR_RGB = (0.34, 0.33, 0.31)
LIGHT = np.array([-0.5, 0.35, 0.8]) / np.linalg.norm([-0.5, 0.35, 0.8])


class Scene:
    def __init__(self, include_small_debris=True, removed=()):
        prims = [p for p in sc.primitives(include_small_debris) if p['name'] not in removed]
        self.solids = [p for p in prims if p['kind'] in ('box', 'cyl')]
        self.cracks = [p for p in prims if p['kind'] == 'crack']
        self.models = sorted({p['model'] for p in self.solids} | {'floor'})
        self.model_id = {m: k for k, m in enumerate(self.models)}
        self.names = [p['name'] for p in self.solids]
        c = np.array([p['c'] for p in self.solids])
        ext = np.array([max(p['size'][:2]) / 2 * 1.42 if p['kind'] == 'box' else p['r'] for p in self.solids])
        self._c, self._ext = c, ext

    def render(self, cam: CameraModel, pose, noise=0.0, rng=None):
        """depth [H,W] (planar, inf = nothing within far), rgb [H,W,3] float 0..1, label [H,W] solid index or -1 (floor)
        or -2 (nothing). label is EVALUATION ONLY."""
        o, d = cam.world_rays(pose)
        H, W = d.shape[:2]
        d = d.reshape(-1, 3)
        best = np.full(len(d), np.inf)
        lab = np.full(len(d), -2)
        nrm = np.zeros((len(d), 3))
        # floor
        with np.errstate(divide='ignore', invalid='ignore'):
            tf = np.where(d[:, 2] < -1e-9, -o[2] / d[:, 2], np.inf)
        hit = tf < best
        best[hit], lab[hit] = tf[hit], -1
        nrm[hit] = (0, 0, 1)
        # cull primitives far away or behind
        rel = self._c[:, :2] - o[:2]
        dist = np.hypot(rel[:, 0], rel[:, 1]) - self._ext
        fwd = np.array([math.cos(pose[2]), math.sin(pose[2])])
        keep = (dist < cam.far + 0.5) & ((rel @ fwd) > -self._ext - 0.3)
        for k in np.nonzero(keep)[0]:
            p = self.solids[k]
            t, n = (_hit_box if p['kind'] == 'box' else _hit_cyl)(p, o, d)
            h = t < best
            best[h], lab[h] = t[h], k
            nrm[h] = n[h]
        best[best > cam.far] = np.inf
        lab[~np.isfinite(best)] = -2
        # colours
        rgb = np.zeros((len(d), 3))
        rgb[lab == -1] = FLOOR_RGB
        for k in np.unique(lab[lab >= 0]):
            rgb[lab == k] = self.solids[k]['rgb']
        pts = o + d * np.where(np.isfinite(best), best, 0)[:, None]
        wall = np.isin(lab, [k for k, p in enumerate(self.solids) if p['model'] == 'walls'])
        for cr in self.cracks:
            P = np.array(cr['polyline'])
            lo, hi = P.min(0) - 0.05, P.max(0) + 0.05
            cand = wall & np.all((pts >= lo) & (pts <= hi), 1)
            if not cand.any():
                continue
            q = pts[cand]
            dmin = np.full(len(q), np.inf)
            for a, b in zip(P[:-1], P[1:]):
                ab = b - a
                s = np.clip(((q - a) @ ab) / (ab @ ab), 0, 1)
                dmin = np.minimum(dmin, np.linalg.norm(q - (a + s[:, None] * ab), axis=1))
            idx = np.nonzero(cand)[0][dmin < cr['line_w'] / 2]
            rgb[idx] = cr['rgb']
        shade = 0.6 + 0.4 * np.abs(nrm @ LIGHT)
        att = 1.0 / (1.0 + 0.03 * np.where(np.isfinite(best), best, 0) ** 2)
        rgb = rgb * (shade * att)[:, None]
        rgb[lab == -2] = (0.08, 0.10, 0.14)
        if noise > 0:
            rng = rng or np.random.default_rng(0)
            rgb = rgb + rng.normal(0, noise, rgb.shape)
        return (best.reshape(H, W), np.clip(rgb, 0, 1).reshape(H, W, 3), lab.reshape(H, W))


def _hit_box(p, o, d):
    c = np.array(p['c'], float)
    cy, sy = math.cos(p['yaw']), math.sin(p['yaw'])
    R = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ol = (o - c) @ R
    dl = d @ R
    half = np.array(p['size'], float) / 2
    with np.errstate(divide='ignore', invalid='ignore'):
        inv = 1.0 / dl
        t1 = (-half - ol) * inv
        t2 = (half - ol) * inv
    tmin = np.nanmax(np.minimum(t1, t2), axis=1)
    tmax = np.nanmin(np.maximum(t1, t2), axis=1)
    ok = (tmax >= tmin) & (tmax > 0) & (tmin > 1e-6)
    t = np.where(ok, tmin, np.inf)
    axis = np.nanargmax(np.minimum(t1, t2), axis=1)
    nl = np.zeros_like(d)
    nl[np.arange(len(d)), axis] = 1.0
    return t, nl @ R.T


def _hit_cyl(p, o, d):
    cx, cy, cz = p['c']
    r, h = p['r'], p['h']
    ox, oy = o[0] - cx, o[1] - cy
    a = d[:, 0] ** 2 + d[:, 1] ** 2
    b = 2 * (ox * d[:, 0] + oy * d[:, 1])
    c = ox * ox + oy * oy - r * r
    disc = b * b - 4 * a * c
    t = np.full(len(d), np.inf)
    n = np.zeros_like(d)
    with np.errstate(invalid='ignore', divide='ignore'):
        sq = np.sqrt(np.maximum(disc, 0))
        t0 = (-b - sq) / (2 * a)
        z = o[2] + t0 * d[:, 2]
        side = (disc >= 0) & (t0 > 1e-6) & (np.abs(z - cz) <= h / 2) & (c > 0)
        t[side] = t0[side]
        n[side, 0] = (ox + t0[side] * d[side, 0]) / r
        n[side, 1] = (oy + t0[side] * d[side, 1]) / r
        # caps
        for zc, nz in ((cz + h / 2, 1.0), (cz - h / 2, -1.0)):
            tc = (zc - o[2]) / d[:, 2]
            px, py = ox + tc * d[:, 0], oy + tc * d[:, 1]
            cap = (tc > 1e-6) & (px * px + py * py <= r * r) & (tc < t)
            t[cap] = tc[cap]
            n[cap] = (0, 0, nz)
    return t, n


def lidar_scan(true_map, pose, rng=None, sigma=sim2d.LIDAR_SIGMA):
    """One 360-beam scan in the LiDAR plane of the true world (same model as sim2d)."""
    x, y, th = pose
    z = true_map.raycast(x + sim2d.LIDAR_X * math.cos(th), y + sim2d.LIDAR_X * math.sin(th), th + sim2d.ANGLES,
                         sim2d.RMAX, sim2d.RMIN, step=min(0.02, true_map.res / 2))
    if rng is not None:
        fin = np.isfinite(z)
        z[fin] += rng.normal(0, sigma, fin.sum())
    z[z >= sim2d.RMAX] = np.inf
    return z
