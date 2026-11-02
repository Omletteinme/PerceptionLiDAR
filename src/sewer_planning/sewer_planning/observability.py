"""How well the prior map lets the filter pin down the robot's position along a corridor.

CUSTOM code (ours), pure NumPy/SciPy. For a point on the centre line we ray-cast a 360-beam scan in the
prior map and sum the likelihood-field information of every hit: a beam that hits a wall with normal n
tells the filter about the position component along n, with weight kappa / sigma_hit^2 (the same
likelihood field the localizer uses; one sixth of the beams are used by the filter, hence /stride).
Parallel side walls inform only the lateral axis; junctions, bends and end walls also inform the
along-corridor axis. The result is a 2x2 information matrix per sample along every edge.
"""
import numpy as np
from scipy import ndimage

PROFILE_STEP = 0.25   # spacing of the information samples along an edge [m]


class NormalField:
    """Surface normals of the prior map from the gradient of the smoothed occupancy."""

    def __init__(self, omap, sigma_cells=1.5):
        f = ndimage.gaussian_filter(omap.solid.astype(float), sigma_cells)
        self.gy, self.gx = np.gradient(f)   # axis 0 = rows = y, axis 1 = columns = x
        self.map = omap

    def normals(self, x, y):
        i, j, inside = self.map.cells(x, y)
        gx, gy = np.zeros(i.shape), np.zeros(i.shape)
        gx[inside], gy[inside] = self.gx[j[inside], i[inside]], self.gy[j[inside], i[inside]]
        norm = np.hypot(gx, gy)
        ok = inside & (norm > 1e-6)
        safe = np.where(ok, norm, 1.0)
        return gx / safe, gy / safe, ok


def edge_info_profile(omap, normals, A, B, kappa=0.25, sigma_hit=0.15, stride=6, rmax=8.0, rmin=0.12):
    """Information matrices along the edge A->B at PROFILE_STEP spacing. Returns (s[K], I[K,2,2])."""
    A, B = np.asarray(A, float), np.asarray(B, float)
    length = float(np.linalg.norm(B - A))
    a = (B - A) / length
    s_samples = np.append(np.arange(0.0, length, PROFILE_STEP), length)
    angles = np.radians(np.arange(0.0, 360.0, 1.0))
    coeff = kappa / sigma_hit ** 2 / stride
    info = np.zeros((len(s_samples), 2, 2))
    for k, s in enumerate(s_samples):
        p = A + s * a
        r = omap.raycast(p[0], p[1], angles, rmax, rmin)
        hit = np.isfinite(r)
        if not hit.any():
            continue
        nx, ny, ok = normals.normals(p[0] + r[hit] * np.cos(angles[hit]), p[1] + r[hit] * np.sin(angles[hit]))
        nx, ny = nx[ok], ny[ok]
        info[k] = coeff * np.array([[np.sum(nx * nx), np.sum(nx * ny)], [np.sum(nx * ny), np.sum(ny * ny)]])
    return s_samples, info
