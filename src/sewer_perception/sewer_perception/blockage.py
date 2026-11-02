"""Blockage detection from reliability-weighted LiDAR evidence.

CUSTOM code (ours), pure NumPy: no ROS imports, so the sandbox and the ROS node share it.

Idea. The robot has the a-priori map (walls only). A beam whose return is clearly SHORTER than the map
predicts, and whose endpoint is not just a wall seen at a grazing angle, is evidence of an unmapped
obstacle. Evidence is accumulated in a log-odds grid so that persistent debris builds up while random
spurious returns do not; rays that pass through a cell erase the evidence there. Only beams whose
reliability r_i (sewer_localization.reliability) is high enough count at all, so sensor trouble does not
create phantom blockages.

For every graph edge the evidence is cut into cross-sections along the corridor: the free gap of a
section is the widest free lateral interval between the evidence and the walls. An edge is blocked when
its smallest gap is below `min_gap` (robot width 0.30 m + 2 x 0.10 m clearance = 0.50 m, the Day 4-5
blockage definition). The pose used to place the evidence is the filter's ESTIMATE, never ground truth.
"""
import math
from dataclasses import dataclass

import numpy as np
from scipy import ndimage

from sewer_localization.map_utils import OccupancyMap, valid_returns


@dataclass
class BlockageParams:
    lidar_x: float = 0.05
    rmin: float = 0.12
    rmax: float = 8.0
    margin: float = 0.30        # unexpected if measured < map-predicted - margin
    r_min: float = 0.5          # only beams at least this reliable count
    wall_clear: float = 0.15    # ignore endpoints this close to a mapped wall (pose error, grazing beams)
    l_occ: float = 1.0          # log-odds added to an unexpected endpoint cell
    l_free: float = -0.2        # log-odds added to cells a trusted ray passes through
    l_min: float = -2.0
    l_max: float = 6.0
    occ_thresh: float = 2.0     # a cell counts as obstacle at or above this log-odds
    min_gap: float = 0.50       # blocked when the free gap is narrower than this
    cell_dilate: float = 0.075  # lateral tolerance added around each evidence cell [m]
    half_w: float = 0.5         # corridor half-width
    min_cluster: int = 6        # ignore evidence clusters smaller than this many cells (~0.3 m of surface):
                                # a lone cell is a stray return, not debris (found by the Day 9 ablation)


class EvidenceGrid:
    def __init__(self, prior: OccupancyMap, params=None):
        self.map = prior
        self.p = params or BlockageParams()
        self.dist_prior = prior.distance_field
        self.L = np.zeros((prior.h, prior.w), np.float32)        # unexpected-obstacle log-odds
        self.seen = np.zeros((prior.h, prior.w), bool)           # cells a trusted ray has crossed

    def reset(self):
        self.L[:] = 0.0
        self.seen[:] = False

    # ------------------------------------------------------------------ evidence
    def update(self, pose, ranges, angles, r=None):
        """pose = (x, y, yaw) estimate in the map frame; r = per-beam reliability (None: trust all beams)."""
        p, m = self.p, self.map
        x, y, th = pose
        ranges = np.asarray(ranges, float)
        angles = np.asarray(angles, float)
        valid = valid_returns(ranges, p.rmin, p.rmax)
        trusted = valid if r is None else valid & (np.asarray(r, float) >= p.r_min)
        if not trusted.any():
            return
        lx, ly = x + p.lidar_x * math.cos(th), y + p.lidar_x * math.sin(th)
        ang = th + angles
        zhat = m.raycast(lx, ly, ang, p.rmax, p.rmin)
        zpred = np.where(np.isfinite(zhat), zhat, p.rmax)
        z = np.where(trusted, ranges, 0.0)
        ei, ej, ein = m.cells(lx + z * np.cos(ang), ly + z * np.sin(ang))
        unexpected = trusted & (ranges < zpred - p.margin) & ein
        near_wall = np.zeros_like(unexpected)
        near_wall[unexpected] = self.dist_prior[ej[unexpected], ei[unexpected]] <= p.wall_clear
        occ = unexpected & ~near_wall
        # free-space evidence along trusted rays, stopping just before the hit
        steps = np.arange(p.rmin, p.rmax, m.res)
        fi, fj, fin = m.cells(lx + steps[None, :] * np.cos(ang)[:, None],
                              ly + steps[None, :] * np.sin(ang)[:, None])
        sel = trusted[:, None] & (steps[None, :] < (z - 0.1)[:, None]) & fin
        free_idx = np.unique(fj[sel] * m.w + fi[sel])
        occ_idx = np.unique(ej[occ] * m.w + ei[occ])
        free_idx = np.setdiff1d(free_idx, occ_idx, assume_unique=True)
        flat = self.L.reshape(-1)
        flat[free_idx] = np.maximum(flat[free_idx] + p.l_free, p.l_min)
        flat[occ_idx] = np.minimum(flat[occ_idx] + p.l_occ, p.l_max)
        self.seen.reshape(-1)[free_idx] = True

    # ------------------------------------------------------------------ per-edge analysis
    def obstacle_cells(self):
        """World coordinates of all cells currently classified as unmapped obstacle (small clusters dropped)."""
        occ = self.L >= self.p.occ_thresh
        if occ.any():
            labels, n = ndimage.label(occ, structure=np.ones((3, 3)))
            sizes = np.bincount(labels.ravel())
            big = sizes >= self.p.min_cluster
            big[0] = False
            occ = big[labels]
        j, i = np.nonzero(occ)
        m = self.map
        return m.x0 + (i + 0.5) * m.res, m.y0 + (j + 0.5) * m.res

    def edge_status(self, A, B):
        """Gap analysis of the corridor from node A to node B (coordinates: s along A->B, d lateral, left +).

        Returns dict(gap, blocked, s_min, s_max, offset, observed, cells):
          gap       smallest free lateral gap over all cross-sections [m] (full width if no evidence)
          offset    lateral centre of that free gap relative to the centre line [m] (where to drive)
          s_min/max extent of the evidence along the edge, or None
          observed  fraction of the corridor floor a trusted ray has crossed (how much we have looked)
        """
        p, m = self.p, self.map
        ax, ay = A
        bx, by = B
        length = math.hypot(bx - ax, by - ay)
        a = ((bx - ax) / length, (by - ay) / length)
        n = (-a[1], a[0])
        out = dict(gap=2 * p.half_w, blocked=False, s_min=None, s_max=None, offset=0.0, observed=0.0, cells=0)
        ss, dd = np.meshgrid(np.arange(0.0, length, 0.1), np.arange(-p.half_w, p.half_w + 1e-9, 0.1))
        ci, cj, cin = m.cells(ax + ss * a[0] + dd * n[0], ay + ss * a[1] + dd * n[1])
        out['observed'] = float(self.seen[cj[cin], ci[cin]].mean()) if cin.any() else 0.0
        cx, cy = self.obstacle_cells()
        if len(cx) == 0:
            return out
        s = (cx - ax) * a[0] + (cy - ay) * a[1]
        d = (cx - ax) * n[0] + (cy - ay) * n[1]
        keep = (s >= 0.0) & (s <= length) & (np.abs(d) <= p.half_w + 0.05)
        s, d = s[keep], d[keep]
        if len(s) == 0:
            return out
        bins = np.floor(s / 0.05).astype(int)
        worst, worst_iv = 2 * p.half_w, (-p.half_w, p.half_w)
        for b in np.unique(bins):
            ds_ = np.sort(d[bins == b])
            lo = np.clip(ds_ - p.cell_dilate, -p.half_w, p.half_w)
            hi = np.clip(ds_ + p.cell_dilate, -p.half_w, p.half_w)
            edge, best, best_iv = -p.half_w, 0.0, (0.0, 0.0)
            for l, h in zip(lo, hi):
                if l - edge > best:
                    best, best_iv = l - edge, (edge, l)
                edge = max(edge, h)
            if p.half_w - edge > best:
                best, best_iv = p.half_w - edge, (edge, p.half_w)
            if best < worst:
                worst, worst_iv = best, best_iv
        out.update(gap=float(worst), blocked=bool(worst < p.min_gap), s_min=float(s.min()), s_max=float(s.max()),
                   offset=float(0.5 * (worst_iv[0] + worst_iv[1])), cells=int(len(s)))
        return out
