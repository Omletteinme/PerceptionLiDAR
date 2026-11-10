"""Structural defect detection: holes, damaged joints and deformations from the LiDAR, cracks from the RGB camera.

CUSTOM code (Account 2), pure NumPy/SciPy, no ROS imports. Inputs are sensor data, the a-priori wall map and the
ESTIMATED pose; simulator ground truth is never used.

LiDAR (WallDefectDetector). The prior map says where each beam should meet a wall. For every beam that hits a
mapped wall face at a usable angle, the signed perpendicular deviation from the face is
    dev = (measured - expected range) * cos(incidence)     (+ = recess behind the face, - = bulge in front of it)
A pose error shifts or tilts a whole wall, so in every scan a straight line (offset + slope along the wall) is fitted
robustly to the deviations of all beams on the same wall line, and each beam's deviation is taken RELATIVE to it
(defects are short compared with the visible wall, so the robust fit ignores them). Deviations are accumulated per wall-face cell over many scans;
runs of consistently deviating cells along a wall become defects:
    recess  narrower than 0.08 m -> damaged_joint (open joint), wider -> hole
    bulge   at least 0.15 m long -> deformation
Beams at grazing incidence, beams that end on large unmapped obstacles (debris against the wall) and unreliable
beams (per-beam reliability < 0.5 when available) are not used.

Camera (CrackDetector). Cracks are thin dark lines on the wall. Pixels whose depth puts them on an a-priori wall
face above the dark wet band (0.32-0.95 m) are searched with a morphological black-hat filter (dark features
narrower than the structuring element), then connected components that are long and thin are kept. Their 3D length
on the wall comes from the depth image and the estimated pose.

Severity is a documented rule on the measured size (SEVERITY below); it is a reporting aid, not an engineering
assessment. Defect ranking stays separate from debris-removal priority.
"""
import math
from dataclasses import dataclass

import numpy as np
from scipy import ndimage

from sewer_localization.map_utils import valid_returns

# type -> (measured quantity, [minor/moderate boundary, moderate/severe boundary]) in metres
SEVERITY = {
    'crack': ('length', (0.20, 0.50)),
    'hole': ('width', (0.10, 0.18)),
    'damaged_joint': ('width', (0.03, 0.06)),
    'deformation': ('depth', (0.04, 0.06)),
}


def severity(dtype, size):
    key, (a, b) = SEVERITY[dtype]
    v = size[key]
    label = 'minor' if v < a else ('moderate' if v < b else 'severe')
    return label, round(float(min(1.0, v / (2 * b))), 3)


# ============================================================================================ LiDAR wall defects
@dataclass
class WallParams:
    lidar_x: float = 0.05
    rmin: float = 0.12
    rmax: float = 8.0
    use_range: float = 3.5          # only beams whose expected wall hit is this close
    step: float = 0.005             # ray-cast step for the expected range [m]
    min_cos: float = 0.35           # incidence: |cos| between beam and wall normal (0.35 = 70 deg off normal)
    max_dev: float = 0.22           # larger deviations are not wall defects (debris, open junction, ...)
    detrend_min: int = 8            # beams on one wall line needed for the line fit
    detrend_tol: float = 0.015      # residual [m] above which a beam is an outlier of the wall line
    perp_cos: float = 0.97          # beams within ~14 deg of the wall normal measure the depth
    r_min: float = 0.5              # per-beam reliability gate (when available)
    min_obs: int = 4
    recess_thr: float = 0.03
    bulge_thr: float = 0.02
    joint_max_w: float = 0.08
    bulge_min_w: float = 0.15
    big_neg_frac: float = 0.25      # a cell whose beams often end far in front of the wall is debris, not a bulge


class WallDefectDetector:
    def __init__(self, prior, params=None):
        self.map = prior
        self.p = params or WallParams()
        m = prior
        solid = m.solid
        pad = np.pad(solid, 1, constant_values=True)
        free_n = {(1, 0): ~pad[1:-1, 2:], (-1, 0): ~pad[1:-1, :-2], (0, 1): ~pad[2:, 1:-1], (0, -1): ~pad[:-2, 1:-1]}
        nx = np.zeros(solid.shape)
        ny = np.zeros(solid.shape)
        for (dx, dy), f in free_n.items():
            nx += dx * f
            ny += dy * f
        norm = np.hypot(nx, ny)
        self.face = solid & (norm > 0)
        with np.errstate(invalid='ignore', divide='ignore'):
            self.nx = np.where(self.face, nx / norm, 0.0)
            self.ny = np.where(self.face, ny / norm, 0.0)
        # wall corners (junction edges): the face direction is ambiguous there and small pose errors look like
        # recesses; cells within 0.1 m of a corner are not used for defects
        corner = self.face & (np.abs(self.nx) > 0.1) & (np.abs(self.ny) > 0.1)
        pad8 = np.pad(~solid, 1, constant_values=False)
        diag_free = pad8[:-2, :-2] | pad8[:-2, 2:] | pad8[2:, :-2] | pad8[2:, 2:]
        corner |= solid & ~self.face & diag_free                 # convex corner cells (free only diagonally)
        self.near_corner = ndimage.binary_dilation(corner, np.ones((5, 5))) & self.face
        shape = solid.shape
        self.sum_w = np.zeros(shape)
        self.sum_wd = np.zeros(shape)
        self.n = np.zeros(shape, np.int32)
        self.n_bigneg = np.zeros(shape, np.int32)
        self.sum_dp = np.zeros(shape)              # near-perpendicular beams only: used to measure the depth
        self.n_p = np.zeros(shape, np.int32)

    def reset(self):
        for a in (self.sum_w, self.sum_wd, self.n, self.n_bigneg, self.sum_dp, self.n_p):
            a[:] = 0

    def update(self, pose, ranges, angles, r=None):
        p, m = self.p, self.map
        x, y, th = pose
        ranges = np.asarray(ranges, float)
        angles = np.asarray(angles, float)
        ok = valid_returns(ranges, p.rmin, p.rmax)
        if r is not None:
            ok &= np.asarray(r, float) >= p.r_min
        lx, ly = x + p.lidar_x * math.cos(th), y + p.lidar_x * math.sin(th)
        ang = th + angles
        zhat = m.raycast(lx, ly, ang, p.use_range, p.rmin, p.step) - p.step / 2
        ok &= np.isfinite(zhat)
        if not ok.any():
            return
        zhat = np.where(np.isfinite(zhat), zhat, 0.0)
        ranges = np.where(np.isfinite(ranges), ranges, 0.0)
        ex, ey = lx + zhat * np.cos(ang), ly + zhat * np.sin(ang)
        ci, cj, cin = m.cells(ex, ey)
        ok &= cin
        ci, cj = np.where(ok, ci, 0), np.where(ok, cj, 0)
        ok &= self.face[cj, ci]
        nx, ny = self.nx[cj, ci], self.ny[cj, ci]
        cosi = -(np.cos(ang) * nx + np.sin(ang) * ny)            # beam points INTO the wall: against the normal
        ok &= cosi >= p.min_cos
        res = np.where(ok, ranges - zhat, 0.0)
        bigneg = ok & (res < -p.max_dev)
        dev = res * cosi
        use = ok & (np.abs(res) <= p.max_dev)
        # local detrending along each wall line (same normal, same offset)
        offs = np.round((ex * nx + ey * ny) / 0.05).astype(int)
        key = np.round(nx * 10).astype(int) * 1000000 + np.round(ny * 10).astype(int) * 10000 + offs
        along = -ey * nx + ex * ny
        base = np.full_like(dev, np.nan)
        idx = np.nonzero(use)[0]
        for k in np.unique(key[idx]):
            sel = idx[key[idx] == k]
            if len(sel) < p.detrend_min:
                continue
            a, dv = along[sel], dev[sel]
            keep = np.abs(dv - np.median(dv)) < 2 * p.detrend_tol     # start from the median (robust to defects)
            if keep.sum() < p.detrend_min:
                continue
            for _ in range(3):                 # robust straight-line fit: refit without the outliers (the defects)
                A = np.stack([a[keep], np.ones(keep.sum())], -1)
                coef = np.linalg.lstsq(A, dv[keep], rcond=None)[0]
                fit = coef[0] * a + coef[1]
                keep = np.abs(dv - fit) < p.detrend_tol
                if keep.sum() < p.detrend_min:
                    break
            else:
                base[sel] = fit
        use &= np.isfinite(base)
        d = dev - np.where(np.isfinite(base), base, 0.0)
        wgt = cosi ** 8                     # near-perpendicular beams measure depth best; oblique ones mostly confirm
        np.add.at(self.sum_w, (cj[use], ci[use]), wgt[use])
        np.add.at(self.sum_wd, (cj[use], ci[use]), (wgt * d)[use])
        np.add.at(self.n, (cj[use], ci[use]), 1)
        np.add.at(self.n_bigneg, (cj[bigneg], ci[bigneg]), 1)
        perp = use & (cosi >= p.perp_cos)
        np.add.at(self.sum_dp, (cj[perp], ci[perp]), d[perp])
        np.add.at(self.n_p, (cj[perp], ci[perp]), 1)

    def mean_dev(self):
        with np.errstate(invalid='ignore', divide='ignore'):
            return np.where(self.n > 0, self.sum_wd / np.maximum(self.sum_w, 1e-9), np.nan)

    def detect(self):
        """Current defect list (accumulated evidence): dicts with type, x, y, z, normal, size, n_obs, confidence."""
        p, m = self.p, self.map
        md = self.mean_dev()
        enough = (self.n >= p.min_obs) & ~self.near_corner
        contaminated = ndimage.binary_dilation(self.n_bigneg >= np.maximum(1, p.big_neg_frac * (self.n + self.n_bigneg)),
                                               np.ones((7, 7))) & (self.n_bigneg > 0).any()
        out = []
        for sign, thr in ((1, p.recess_thr), (-1, p.bulge_thr)):
            cand = enough & (sign * md > thr)
            if sign < 0:
                cand &= ~contaminated
            lab, n = ndimage.label(cand, np.ones((3, 3)))
            for k in range(1, n + 1):
                j, i = np.nonzero(lab == k)
                nx, ny = float(self.nx[j, i].mean()), float(self.ny[j, i].mean())
                cx = m.x0 + (i + 0.5) * m.res
                cy = m.y0 + (j + 0.5) * m.res
                along = -cy * nx + cx * ny
                width = float(along.max() - along.min() + m.res)
                vals = sign * md[j, i]
                np_ = self.n_p[j, i]
                if (np_ >= 2).any():                 # depth from beams that looked straight at the wall
                    depth = float((sign * self.sum_dp[j, i][np_ >= 2] / np_[np_ >= 2]).max())
                else:
                    depth = float(np.percentile(vals, 90))
                if sign > 0:
                    dtype = 'damaged_joint' if width <= p.joint_max_w else 'hole'
                else:
                    if width < p.bulge_min_w:
                        continue
                    dtype = 'deformation'
                # report the point on the face (cell centre moved half a cell towards the free side)
                fx = float(cx.mean() + nx * m.res / 2)
                fy = float(cy.mean() + ny * m.res / 2)
                nobs = int(self.n[j, i].sum())
                size = dict(width=round(width, 3), depth=round(depth, 3), length=round(width, 3))
                conf = float((1 - math.exp(-nobs / 30.0)) * min(1.0, depth / (3 * thr)))
                out.append(dict(type=dtype, x=fx, y=fy, z=0.13, normal=(round(nx, 3), round(ny, 3)), size=size,
                                n_obs=nobs, confidence=round(conf, 3), source='lidar'))
        return out


# ============================================================================================ camera cracks
@dataclass
class CrackParams:
    z_lo: float = 0.32
    z_hi: float = 0.95
    wall_band: float = 0.06         # pixel's 3D point within this of an a-priori wall (map distance <= band)
    max_range: float = 3.0
    se: int = 9                     # black-hat structuring element [px] (at 640 px width)
    bh_thr: float = 0.07            # black-hat response (0..1 grey levels)
    min_px: int = 15
    min_len_px: float = 18.0
    max_thick_px: float = 4.5
    min_elong: float = 3.0
    erode: int = 3


class CrackDetector:
    def __init__(self, prior, cam, params=None):
        self.map = prior
        self.cam = cam                  # model at the RGB resolution
        self.p = params or CrackParams()

    def wall_mask(self, depth_full, pose):
        """Pixels that lie on an a-priori wall face within the crack search band; also returns the world points."""
        p = self.p
        H, W = depth_full.shape
        rays = self.cam.rays_cam()
        d = np.where(np.isfinite(depth_full), depth_full, 0.0)
        pc = rays * d[..., None]
        R, t = self.cam.cam_to_world(pose)
        pw = pc.reshape(-1, 3) @ R.T + t
        dist = self.map.dist(pw[:, 0], pw[:, 1]).reshape(H, W)
        z = pw[:, 2].reshape(H, W)
        rng = np.hypot(pw[:, 0] - t[0], pw[:, 1] - t[1]).reshape(H, W)
        mask = np.isfinite(depth_full) & (dist <= p.wall_band) & (z > p.z_lo) & (z < p.z_hi) & (rng < p.max_range)
        mask = ndimage.binary_erosion(mask, np.ones((2 * p.erode + 1, 2 * p.erode + 1)))
        return mask, pw.reshape(H, W, 3)

    def detect(self, rgb, depth, pose):
        """rgb [H,W,3] (uint8 or float 0..1); depth at the same or half resolution (upsampled here)."""
        p = self.p
        img = np.asarray(rgb, float)
        if img.max() > 1.5:
            img = img / 255.0
        H, W = img.shape[:2]
        dep = np.asarray(depth, float)
        if dep.shape != (H, W):
            fy, fx = H // dep.shape[0], W // dep.shape[1]
            dep = np.repeat(np.repeat(dep, fy, 0), fx, 1)[:H, :W]
        mask, pw = self.wall_mask(dep, pose)
        if mask.sum() < 50:
            return []
        grey = img.mean(-1)
        se = max(3, int(round(p.se * W / 640)))
        bh = ndimage.grey_closing(grey, size=(se, se)) - grey
        cand = mask & (bh > p.bh_thr)
        lab, n = ndimage.label(cand, np.ones((3, 3)))
        out = []
        for k, sl in enumerate(ndimage.find_objects(lab), start=1):
            if sl is None:
                continue
            vv, uu = np.nonzero(lab[sl] == k)
            if len(vv) < p.min_px:
                continue
            vv = vv + sl[0].start
            uu = uu + sl[1].start
            q = np.stack([uu, vv], -1).astype(float)
            q -= q.mean(0)
            ev, evec = np.linalg.eigh(np.cov(q.T) + 1e-9 * np.eye(2))
            proj = q @ evec[:, 1]
            length_px = float(proj.max() - proj.min() + 1)
            thick = len(vv) / length_px
            elong = math.sqrt(ev[1] / max(ev[0], 1e-9))
            if length_px < p.min_len_px or thick > p.max_thick_px or elong < p.min_elong:
                continue
            P = pw[vv, uu]
            c3 = P.mean(0)
            Q = P - c3
            _, _, vt = np.linalg.svd(Q, full_matrices=False)
            s3 = Q @ vt[0]
            length = float(s3.max() - s3.min())
            dmean = float(np.nanmean(dep[vv, uu]))
            width = thick * dmean / self.cam.fx
            # wall normal from the nearest face cell of the map
            i, j, _ = self.map.cells(np.array([c3[0]]), np.array([c3[1]]))
            ends = [[round(float(v), 4) for v in c3 + vt[0] * s3.min()], [round(float(v), 4) for v in c3 + vt[0] * s3.max()]]
            out.append(dict(type='crack', x=float(c3[0]), y=float(c3[1]), z=float(c3[2]), ends=ends,
                            size=dict(length=round(length, 3), width=round(width, 4), depth=0.0),
                            n_obs=1, confidence=round(float(min(1.0, 0.4 + length_px / 150.0)), 3), source='camera',
                            bbox=[int(uu.min()), int(vv.min()), int(uu.max()), int(vv.max())]))
        return out


# ============================================================================================ defect tracking
class DefectTracker:
    """Stable IDs for defects. LiDAR results are cumulative, so a matched LiDAR defect is REPLACED by the newest
    estimate; camera cracks are fused (longest length seen, n_obs counted, best evidence kept)."""

    def __init__(self, gate=0.30):
        self.gate = gate
        self.items = []
        self._next = 1

    def update(self, dets, stamp, evidence=None):
        for d in dets:
            best, bd = None, self.gate
            for it in self.items:
                same_family = (it['type'] == d['type'] or
                               {it['type'], d['type']} <= {'hole', 'damaged_joint'})
                dist = math.dist((it['x'], it['y'], it['z']), (d['x'], d['y'], d['z']))
                if d['type'] == 'crack' and it['type'] == 'crack':
                    dist = math.dist((it['x'], it['y']), (d['x'], d['y'])) + 0.5 * abs(it['z'] - d['z'])
                    if it.get('ends') and d.get('ends'):       # or close to the crack seen so far (a longer piece)
                        dist = min(dist, min(math.dist(p, q) for p in it['ends'] + [[it['x'], it['y'], it['z']]]
                                             for q in d['ends'] + [[d['x'], d['y'], d['z']]]))
                if same_family and dist < bd:
                    best, bd = it, dist
            if best is None:
                best = dict(id=f'F{self._next}', first_seen=stamp, n_obs=0, evidence=None)
                self._next += 1
                self.items.append(best)
                fresh = True
            else:
                fresh = False
            if d['source'] == 'lidar' or fresh:
                best.update({k: v for k, v in d.items() if k not in ('n_obs', 'bbox')})
                best['n_obs'] = d['n_obs'] if d['source'] == 'lidar' else best['n_obs'] + 1
            else:
                k = best['n_obs']
                if best.get('ends') and d.get('ends'):
                    # frames see different parts of a crack: keep the two furthest-apart end points seen so far
                    pts = [np.array(p) for p in best['ends'] + d['ends']]
                    a, b = max(((a, b) for i, a in enumerate(pts) for b in pts[i + 1:]),
                               key=lambda ab: np.linalg.norm(ab[0] - ab[1]))
                    best['ends'] = [a.tolist(), b.tolist()]
                    c = (a + b) / 2
                    best['x'], best['y'], best['z'] = float(c[0]), float(c[1]), float(c[2])
                    length = float(np.linalg.norm(a - b))
                else:
                    for c in ('x', 'y', 'z'):
                        best[c] = (best[c] * k + d[c]) / (k + 1)
                    length = max(best['size']['length'], d['size']['length'])
                best['size'] = dict(best['size'], length=round(length, 3),
                                    width=max(best['size']['width'], d['size']['width']))
                best['confidence'] = max(best['confidence'], d['confidence'])
                best['n_obs'] = k + 1
            best['last_seen'] = stamp
            label, score = severity(best['type'], best['size'])
            best['severity'], best['severity_score'] = label, score
            if evidence is not None and d.get('bbox') is not None:
                area = (d['bbox'][2] - d['bbox'][0]) * (d['bbox'][3] - d['bbox'][1])
                if best['evidence'] is None or area >= best['evidence'].get('area', 0):
                    best['evidence'] = dict(evidence, bbox=d['bbox'], area=area)
            elif best['evidence'] is None and d['source'] == 'lidar':
                best['evidence'] = dict(source='lidar', frame_stamp=stamp, image=None, bbox=None)
        return self.items
