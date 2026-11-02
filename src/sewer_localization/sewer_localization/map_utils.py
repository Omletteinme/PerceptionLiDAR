"""Occupancy map, likelihood field and ray casting.

CUSTOM code (ours). Pure NumPy/SciPy: no ROS imports, so the sandbox and the ROS nodes share it.
"""
import numpy as np
import yaml
from scipy import ndimage


def valid_returns(ranges, rmin, rmax, margin=0.05):
    """True for beams that are real range returns (finite, inside [rmin, rmax - margin))."""
    r = np.asarray(ranges, float)
    with np.errstate(invalid='ignore'):
        return np.isfinite(r) & (r >= rmin) & (r < rmax - margin)


class OccupancyMap:
    """Boolean 'solid' grid. Column i <-> x = x0 + i*res, row j <-> y = y0 + j*res (bottom-up).

    Anything that is not known-free (walls and the unknown outside) is solid.
    """

    def __init__(self, solid, res, x0, y0):
        self.solid = np.asarray(solid, bool)
        self.res, self.x0, self.y0 = float(res), float(x0), float(y0)
        self.h, self.w = self.solid.shape
        self._dist = None

    @classmethod
    def load(cls, base):
        """Load `<base>.pgm` + `<base>.yaml` as written by sewer_gazebo/scripts/gen_network.py."""
        with open(base + '.yaml') as f:
            meta = yaml.safe_load(f)
        with open(base + '.pgm', 'rb') as f:
            raw = f.read()
        parts = raw.split(b'\n', 3)  # P5 / W H / 255 / data
        w, h = (int(v) for v in parts[1].split())
        img = np.frombuffer(parts[3], np.uint8).reshape(h, w)
        return cls(np.flipud(img) < 250, meta['resolution'], meta['origin'][0], meta['origin'][1])

    def copy(self):
        return OccupancyMap(self.solid.copy(), self.res, self.x0, self.y0)

    def cells(self, x, y):
        i = np.floor((np.asarray(x, float) - self.x0) / self.res).astype(int)
        j = np.floor((np.asarray(y, float) - self.y0) / self.res).astype(int)
        inside = (i >= 0) & (i < self.w) & (j >= 0) & (j < self.h)
        return i, j, inside

    def is_solid(self, x, y):
        i, j, inside = self.cells(x, y)
        out = np.ones(i.shape, bool)  # outside the map counts as solid
        out[inside] = self.solid[j[inside], i[inside]]
        return out

    def _centres(self):
        return (self.x0 + (np.arange(self.w) + 0.5) * self.res,
                self.y0 + (np.arange(self.h) + 0.5) * self.res)

    def add_box(self, cx, cy, sx, sy):
        xs, ys = self._centres()
        self.solid |= (np.abs(xs[None, :] - cx) <= sx / 2) & (np.abs(ys[:, None] - cy) <= sy / 2)
        self._dist = None

    def add_circle(self, cx, cy, r):
        xs, ys = self._centres()
        self.solid |= (xs[None, :] - cx) ** 2 + (ys[:, None] - cy) ** 2 <= r * r
        self._dist = None

    @property
    def distance_field(self):
        """Distance [m] from each cell to the nearest solid cell (0 inside solid)."""
        if self._dist is None:
            self._dist = ndimage.distance_transform_edt(~self.solid) * self.res
        return self._dist

    def dist(self, x, y):
        """Likelihood-field lookup; points outside the map are 'in a wall' (0)."""
        i, j, inside = self.cells(x, y)
        out = np.zeros(i.shape)
        out[inside] = self.distance_field[j[inside], i[inside]]
        return out

    def raycast(self, px, py, angles, rmax, rmin=0.0, step=0.02):
        """Range along each world-frame angle from (px, py); inf where nothing is hit before rmax."""
        r = np.arange(rmin, rmax, step)
        angles = np.asarray(angles, float)
        hit = self.is_solid(px + r[None, :] * np.cos(angles)[:, None],
                            py + r[None, :] * np.sin(angles)[:, None])
        first = hit.argmax(axis=1)
        return np.where(hit.any(axis=1), r[first], np.inf)
