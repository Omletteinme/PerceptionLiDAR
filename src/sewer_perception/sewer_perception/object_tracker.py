"""Multi-frame debris tracker: stable IDs, accumulated shape, dimensions and confidence.

CUSTOM code (Account 2), pure NumPy, no ROS imports.

Each track keeps a 2 cm floor grid of every cell its clusters covered (with the highest point seen in it), so the
dimensions improve as the robot sees the object from more sides. Association is greedy nearest-centroid with an
overlap test. A confirmed track that should be visible (in the camera's field of view, in range, not occluded) but
is not detected for `miss_limit` frames becomes 'missing'; it is never silently re-created under a new ID while it
is still being tracked.

States: tentative -> confirmed -> missing; set externally by the manipulation status: held, removed.
"""
import math
from dataclasses import dataclass, field

import numpy as np

from .debris_detector import footprint, roundness

VOX = 0.02
MAX_PTS_PER_FRAME = 150
MAX_PTS = 2000


@dataclass
class Track:
    id: str
    first_seen: float
    last_seen: float
    cells: dict = field(default_factory=dict)        # (i, j) -> [frames_seen, z_top, from_depth]
    hits: int = 0
    misses: int = 0
    state: str = 'tentative'
    sources: set = field(default_factory=set)
    rel_base: tuple = None                           # (x, y, stamp) of the latest depth detection in base_link
    min_range: float = float('inf')                  # closest the camera has been to it [m] (a near view sees its top)
    view_az: list = None                             # directions (map yaw, object -> camera) it was seen from
    points: np.ndarray = None                        # reservoir of raw map-frame points (for the dimensions)
    _cache: dict = None

    def add(self, cluster, stamp):
        keys = np.unique(np.floor(cluster.points[:, :2] / VOX).astype(int), axis=0)
        ztop = {}
        for (i, j), z in zip(np.floor(cluster.points[:, :2] / VOX).astype(int), cluster.points[:, 2]):
            k = (int(i), int(j))
            ztop[k] = max(ztop.get(k, 0.0), float(z))
        depth = cluster.source == 'depth'
        for i, j in keys:
            k = (int(i), int(j))
            c = self.cells.setdefault(k, [0, 0.0, False])
            c[0] += 1
            c[1] = max(c[1], ztop[k]) if depth or not c[2] else c[1]
            c[2] = c[2] or depth
        self.hits += 1
        self.misses = 0
        self.last_seen = stamp
        self.sources.add(cluster.source)
        sub = cluster.points[np.random.default_rng(self.hits).permutation(len(cluster.points))[:MAX_PTS_PER_FRAME]]
        self.points = sub if self.points is None else np.concatenate([self.points, sub])[-MAX_PTS:]
        if depth and cluster.rel_base is not None:
            self.rel_base = (cluster.rel_base[0], cluster.rel_base[1], stamp)
            self.min_range = min(self.min_range, float(np.hypot(*cluster.rel_base)))
        if depth and cluster.cam_xy is not None:
            az = math.atan2(cluster.cam_xy[1] - cluster.centroid[1], cluster.cam_xy[0] - cluster.centroid[0])
            self.view_az = (self.view_az or []) + [az]
            if len(self.view_az) > 300:
                self.view_az = self.view_az[-300:]
        self._cache = None

    def view_spread(self):
        """Angle [rad] spanned by the directions it was seen from. Seen from one direction only, the extent along the
        line of sight is hidden (a plank seen end-on looks short)."""
        if not self.view_az:
            return 0.0
        a = np.sort(np.mod(self.view_az, 2 * math.pi))
        gaps = np.diff(np.concatenate([a, [a[0] + 2 * math.pi]]))
        return float(2 * math.pi - gaps.max())

    def centroid(self):
        k = np.array(list(self.cells), float)
        return (k.mean(0) + 0.5) * VOX

    def geometry(self):
        """dict(x, y, yaw, length, width, height, shape, height_is_lower_bound) from the accumulated cells."""
        if self._cache is not None:
            return self._cache
        keys = np.array(list(self.cells))
        cnt = np.array([v[0] for v in self.cells.values()])
        ztop = np.array([v[1] for v in self.cells.values()])
        has_depth = any(v[2] for v in self.cells.values())
        keep = cnt >= min(2, max(1, self.hits // 2))      # cells seen once in a long track are mostly pose smear
        if keep.sum() < 3:
            keep = np.ones(len(keys), bool)
        xy = (keys[keep] + 0.5) * VOX
        good = {tuple(k) for k in keys[keep]}
        pts = self.points[:, :2]
        pk = np.floor(pts / VOX).astype(int)
        pts = pts[np.array([tuple(k) in good for k in pk])] if len(pts) else pts
        src = pts if len(pts) >= 3 else xy
        fp = footprint(src)
        cx, cy, L, W, yaw = fp
        height = float(np.percentile(ztop[keep], 95)) if has_depth else 0.13
        shape = 'round' if (abs(L - W) < 0.25 * max(L, 1e-6) and roundness(src, fp) > 0.06) else 'box'
        self._cache = dict(x=cx, y=cy, yaw=yaw, length=L, width=W, height=height, shape=shape,
                           height_is_lower_bound=not has_depth)
        return self._cache

    def confidence(self):
        n_cells = len(self.cells)
        return float((1 - math.exp(-self.hits / 3.0)) * min(1.0, n_cells / 6.0))


class DebrisTracker:
    def __init__(self, gate=0.30, confirm_hits=3, miss_limit=6, touch=0.08):
        """touch: a cluster within this distance of a track's cells belongs to that track (pieces of one object seen
        from a new angle must not start new tracks)."""
        self.gate, self.confirm_hits, self.miss_limit, self.touch = gate, confirm_hits, miss_limit, touch
        self.tracks = []
        self.aliases = {}
        self._next = 1

    def _new(self, stamp):
        t = Track(f'D{self._next}', stamp, stamp)
        self._next += 1
        self.tracks.append(t)
        return t

    def active(self):
        return [t for t in self.tracks if t.state in ('tentative', 'confirmed', 'missing')]

    def update(self, clusters, stamp, visible_fn=None, source='depth'):
        """Associate one frame of clusters. visible_fn(track) -> bool says whether an unmatched track should have been
        seen in this frame (only then does it count as a miss). Returns the list of matched/created tracks."""
        cands = self.active()
        pairs, touching = [], {}
        r = int(round(self.touch / VOX))
        ring = [(di, dj) for di in range(-r, r + 1) for dj in range(-r, r + 1)]
        for ci, c in enumerate(clusters):
            cxy = np.array(c.centroid[:2])
            cset = {(int(i), int(j)) for i, j in np.floor(c.points[:, :2] / VOX).astype(int)}
            grown = {(i + di, j + dj) for i, j in cset for di, dj in ring}
            for ti, t in enumerate(cands):
                g = t.geometry()
                d = float(np.hypot(*(cxy - (g['x'], g['y']))))
                overlap = len(cset & set(t.cells)) / max(1, len(cset))
                touch = not grown.isdisjoint(t.cells)
                if touch:
                    touching.setdefault(ci, []).append((d, ti))
                if d <= max(self.gate, 0.5 * g['length'] + 0.1) or overlap > 0.3 or touch:
                    pairs.append((d - overlap - (0.5 if touch else 0.0), ci, ti))
        pairs.sort()
        used_c, used_t, out = set(), set(), []
        for _, ci, ti in pairs:
            if ci in used_c or ti in used_t:
                continue
            used_c.add(ci)
            used_t.add(ti)
            cands[ti].add(clusters[ci], stamp)
            out.append(cands[ti])
        for ci, c in enumerate(clusters):
            # another piece of an object already matched this frame (e.g. a second face of a pile): same object
            if ci not in used_c and ci in touching:
                t = cands[min(touching[ci])[1]]
                t.add(c, stamp)
                t.hits -= 1                                   # one frame, one hit
                used_c.add(ci)
        for ci, c in enumerate(clusters):
            if ci not in used_c:
                t = self._new(stamp)
                t.add(c, stamp)
                out.append(t)
        for t in out:
            if t.hits >= self.confirm_hits or t.state == 'missing':
                t.state = 'confirmed'
        if source == 'depth' and visible_fn is not None:
            for ti, t in enumerate(cands):
                if ti not in used_t and t.state == 'confirmed' and visible_fn(t):
                    t.misses += 1
                    if t.misses >= self.miss_limit:
                        t.state = 'missing'
        self._merge()
        return out

    def absorb(self, a, b):
        """Merge track b into track a (the older ID survives; b's ID stays an alias of a)."""
        for k, v in b.cells.items():
            c = a.cells.setdefault(k, [0, 0.0, False])
            c[0] += v[0]
            c[1] = max(c[1], v[1])
            c[2] = c[2] or v[2]
        a.hits += b.hits
        if b.points is not None:
            a.points = b.points if a.points is None else np.concatenate([a.points, b.points])[-MAX_PTS:]
        a.sources |= b.sources
        a.min_range = min(a.min_range, b.min_range)
        a.view_az = (a.view_az or []) + (b.view_az or []) or None
        a.first_seen = min(a.first_seen, b.first_seen)
        a.last_seen = max(a.last_seen, b.last_seen)
        a._cache = None
        b.state = 'merged'
        self.aliases[b.id] = a.id

    def _merge(self):
        """Two tracks that have grown into the same object (overlapping cells) become one; the older ID survives."""
        act = sorted(self.active(), key=lambda t: t.first_seen)
        for a in act:
            for b in act:
                if a is b or b.state in ('held', 'removed', 'merged') or a.state == 'merged':
                    continue
                inter = len(set(a.cells) & set(b.cells))
                if inter >= 0.5 * min(len(a.cells), len(b.cells)):
                    self.absorb(a, b)
        self.tracks = [t for t in self.tracks if t.state != 'merged']

    def merge_if(self, same_object):
        """Merge pairs of active tracks for which same_object(a, b) is true (older ID survives)."""
        act = sorted(self.active(), key=lambda t: t.first_seen)
        for i, a in enumerate(act):
            for b in act[i + 1:]:
                if a.state != 'merged' and b.state != 'merged' and same_object(a, b):
                    self.absorb(a, b)
        self.tracks = [t for t in self.tracks if t.state != 'merged']

    def get(self, tid):
        while tid in self.aliases:
            tid = self.aliases[tid]
        return next((t for t in self.tracks if t.id == tid), None)

    def set_state(self, tid, state):
        t = self.get(tid)
        if t is not None:
            t.state = state
        return t


def make_visible_fn(cam, pose, depth, margin=0.15, max_range=2.5):
    """Track is visible if its centre projects into the image within range and nothing nearer hides it."""
    H, W = depth.shape

    def visible(t):
        g = t.geometry()
        u, v, d = cam.project_world(np.array([[g['x'], g['y'], max(0.02, g['height'] / 2)]]), pose)
        u, v, d = float(u[0]), float(v[0]), float(d[0])
        if not (0.3 < d < max_range) or not (5 <= u < W - 5 and 5 <= v < H - 5):
            return False
        obs = depth[int(v), int(u)]
        return bool(np.isfinite(obs) and obs > d - margin - max(g['length'], g['width']) / 2)
    return visible
