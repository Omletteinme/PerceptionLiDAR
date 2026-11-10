"""Inspection scene: small debris, structural defects and the disposal station, on top of sewer_network_v2.

CUSTOM data + geometry helpers (Account 2, perception/inspection). Pure Python/NumPy, no ROS.
ONE description is used by:
  - gen_inspection_world.py   -> worlds/sewer_inspection.sdf (SIMULATED by Gazebo) + config/inspection_truth.yaml
  - sandbox/render3d.py       -> synthetic depth/RGB frames and LiDAR scans for offline tests and evaluation
so the Gazebo world, the offline dataset and the evaluation labels agree by construction.

The a-priori map stays maps/sewer_map_v2 (walls only). Nothing below is in the map: debris and defects must be
detected. inspection_truth.yaml is GROUND TRUTH for EVALUATION ONLY, never an input to perception.

Design notes (see ACCOUNT2_README.md):
  - The LiDAR scans at z = 0.13 m. Small debris is lower than that, so only the depth camera sees it; the big
    blockages B1-B3 from v2 cross the scan plane and are seen by both.
  - Small debris lies 0.25-0.30 m off the corridor centre line so the existing route follower (which drives the
    centre line) does not drive into it while the new mission manager does not exist yet.
  - Holes and damaged joints are recesses cut into the wall, deformations are smooth bulges, cracks are thin dark
    lines painted on the upper wall (camera only).
"""
import math

import numpy as np

import gen_network as gn

NET = gn.V2
HALF_W = gn.HALF_W
WALL_T = gn.WALL_T

# --------------------------------------------------------------------------------------------- small debris
# kind 'box': size = (sx, sy, sz) before yaw; 'cyl': size = (2r, 2r, h). mass is the SIMULATED mass in Gazebo.
# graspable_designed is the scenario designer's intent (what the evaluation expects the feasibility filter to say).
SMALL_DEBRIS = [
    dict(id='S1', kind='box', x=2.5, y=0.28, yaw=0.3, size=(0.06, 0.06, 0.06), mass=0.08,
         rgb=(0.45, 0.33, 0.22), graspable_designed=True, note='small brick fragment'),
    dict(id='S2', kind='cyl', x=6.0, y=-0.30, yaw=0.0, size=(0.07, 0.07, 0.09), mass=0.10,
         rgb=(0.30, 0.38, 0.45), graspable_designed=True, note='can / pipe offcut'),
    dict(id='S3', kind='box', x=7.72, y=1.2, yaw=0.0, size=(0.16, 0.22, 0.07), mass=1.20,
         rgb=(0.40, 0.40, 0.38), graspable_designed=False, note='concrete slab: too wide for the gripper'),
    dict(id='S4', kind='box', x=7.75, y=5.0, yaw=math.pi / 2, size=(0.10, 0.05, 0.05), mass=0.10,
         rgb=(0.50, 0.30, 0.20), graspable_designed=True, note='wood offcut'),
    dict(id='S5', kind='box', x=10.0, y=6.27, yaw=0.0, size=(0.08, 0.08, 0.11), mass=0.25,
         rgb=(0.35, 0.30, 0.25), graspable_designed=True, note='rock'),
    dict(id='S6', kind='box', x=12.25, y=8.0, yaw=math.pi / 2, size=(0.30, 0.08, 0.06), mass=0.90,
         rgb=(0.42, 0.30, 0.18), graspable_designed=False, note='beam: estimated mass over the payload'),
]

# --------------------------------------------------------------------------------------------- defects
# wall: 'N','S','E','W' side of the corridor of `edge`; at: coordinate along the corridor (x for E-W corridors, y for N-S).
DEFECTS = [
    dict(id='H1', type='hole', edge=('A', 'J1'), wall='S', at=3.475, width=0.15, depth=0.10),
    dict(id='H2', type='hole', edge=('M', 'E'), wall='W', at=9.0, width=0.20, depth=0.10),
    dict(id='J1', type='damaged_joint', edge=('J2', 'M'), wall='N', at=10.025, width=0.05, depth=0.10),
    dict(id='J2', type='damaged_joint', edge=('J3', 'J4'), wall='W', at=2.025, width=0.05, depth=0.10),
    dict(id='DF1', type='deformation', edge=('J1', 'J2'), wall='E', at=4.2, width=0.50, depth=0.05),
    dict(id='DF2', type='deformation', edge=('E', 'F'), wall='S', at=14.0, width=0.60, depth=0.07),
    dict(id='C1', type='crack', edge=('A', 'J1'), wall='N', at=6.2, z=0.52, width=0.40, depth=0.0, angle=0.9,
         line_w=0.010),
    dict(id='C2', type='crack', edge=('J1', 'J2'), wall='W', at=1.0, z=0.50, width=0.25, depth=0.0, angle=-0.6,
         line_w=0.010),
    dict(id='C3', type='crack', edge=('J2', 'M'), wall='S', at=11.0, z=0.55, width=0.60, depth=0.0, angle=0.25,
         line_w=0.012),
]

# --------------------------------------------------------------------------------------------- disposal station
# Open tray against the entrance end wall at A (the robot starts at (0, 0) facing +x). Rims are 4 cm tall, below the
# LiDAR plane. The robot drops debris into the inner rectangle.
DISPOSAL = dict(x_min=-0.44, x_max=-0.24, y_min=-0.15, y_max=0.15, rim_h=0.04, rim_t=0.01,
                approach=dict(x=0.04, y=0.0, yaw=math.pi), drop_height=0.05)   # tray centre - 0.38 m (grasp distance)


def corridor(edge):
    (ax, ay), (_, by) = NET['nodes'][edge[0]], NET['nodes'][edge[1]]
    horiz = ay == by
    return horiz, (ay if horiz else ax)


def wall_face(d):
    """(point on the wall face at the defect centre (x, y), inward normal (nx, ny))."""
    horiz, c = corridor(d['edge'])
    w = d['wall']
    if horiz:
        assert w in 'NS', d
        y = c + HALF_W if w == 'N' else c - HALF_W
        return (d['at'], y), (0.0, -1.0 if w == 'N' else 1.0)
    assert w in 'EW', d
    x = c + HALF_W if w == 'E' else c - HALF_W
    return (x, d['at']), (-1.0 if w == 'E' else 1.0, 0.0)


def build():
    """(x0, y0, free, wall_true) where wall_true has the recesses (holes, damaged joints) cut out."""
    x0, y0, free, wall = gn.build_grid(NET)
    wall = wall.copy()
    h, w = wall.shape
    xs = x0 + (np.arange(w) + 0.5) * gn.RES
    ys = y0 + (np.arange(h) + 0.5) * gn.RES
    for d in DEFECTS:
        if d['type'] not in ('hole', 'damaged_joint'):
            continue
        (fx, fy), (nx, ny) = wall_face(d)
        # recess region: along the face +-width/2, from the face `depth` into the wall (against the inward normal)
        if nx == 0:
            along = np.abs(xs[None, :] - fx) < d['width'] / 2 - 1e-6
            into = (ys[:, None] - fy) * -ny
        else:
            along = np.abs(ys[:, None] - fy) < d['width'] / 2 - 1e-6
            into = (xs[None, :] - fx) * -nx
        region = along & (into > 0) & (into < d['depth'] + 1e-6)
        wall &= ~region
    return x0, y0, free, wall


def deformation_cylinder(d):
    """Vertical cylinder whose cap intersects the wall face in a chord of d['width'] bulging d['depth'] inward."""
    (fx, fy), (nx, ny) = wall_face(d)
    c, h = d['width'], d['depth']
    r = (c * c / 4 + h * h) / (2 * h)
    # the innermost point of the bulge is at face + h * n; the centre is r behind it
    return dict(cx=fx + nx * (h - r), cy=fy + ny * (h - r), r=r)


def crack_polyline(d, n_seg=6, seed=0):
    """3D zigzag polyline on the wall face (world frame), slightly proud of the face (1 mm)."""
    (fx, fy), (nx, ny) = wall_face(d)
    rng = np.random.default_rng(seed + sum(map(ord, d['id'])))
    tx, ty = -ny, nx                                  # along the face
    L, a = d['width'], d['angle']
    pts = []
    for k in range(n_seg + 1):
        f = k / n_seg - 0.5
        s = f * L * math.cos(a)
        z = d['z'] + f * L * math.sin(a)
        jit = 0.0 if k in (0, n_seg) else rng.uniform(-0.025, 0.025)
        s += jit * math.sin(a)
        z += jit * math.cos(a)
        pts.append((fx + tx * s + nx * 0.001, fy + ty * s + ny * 0.001, z))
    return pts


def primitives(include_small_debris=True):
    """Everything with geometry, for the offline renderer. Each item: dict(kind, name, model, ...).

    kind 'box' : c=(x,y,z) centre, size=(sx,sy,sz), yaw, rgb
    kind 'cyl' : c=(x,y,z) centre, r, h, rgb
    kind 'crack': polyline [(x,y,z)...], line_w   (paint on the wall, camera only)
    """
    x0, y0, free, wall = build()
    out = []
    for n, (i0, i1, j0, j1) in enumerate(gn.merge_rects(wall)):
        sx, sy = (i1 - i0) * gn.RES, (j1 - j0) * gn.RES
        cx, cy = x0 + (i0 + i1) / 2 * gn.RES, y0 + (j0 + j1) / 2 * gn.RES
        # two-tone wall like the Gazebo world: dark wet band below 0.30 m, light above
        out.append(dict(kind='box', name=f'wl{n}', model='walls', c=(cx, cy, 0.15), size=(sx, sy, 0.30), yaw=0.0,
                        rgb=(0.20, 0.22, 0.20)))
        out.append(dict(kind='box', name=f'wu{n}', model='walls', c=(cx, cy, 0.65), size=(sx, sy, 0.70), yaw=0.0,
                        rgb=(0.55, 0.52, 0.47)))
    for bname, b in NET['blockages'].items():
        for k, p in enumerate(b['pieces']):
            if p[0] == 'box':
                _, cx, cy, sx, sy, hh = p
                out.append(dict(kind='box', name=f'{bname}_{k}', model=f'debris_{bname}', c=(cx, cy, hh / 2),
                                size=(sx, sy, hh), yaw=0.0, rgb=(0.50, 0.20, 0.13)))
            else:
                _, cx, cy, r, hh = p
                out.append(dict(kind='cyl', name=f'{bname}_{k}', model=f'debris_{bname}', c=(cx, cy, hh / 2), r=r,
                                h=hh, rgb=(0.50, 0.20, 0.13)))
    if include_small_debris:
        for s in SMALL_DEBRIS:
            sx, sy, sz = s['size']
            if s['kind'] == 'box':
                out.append(dict(kind='box', name=s['id'], model=f"debris_{s['id']}", c=(s['x'], s['y'], sz / 2),
                                size=(sx, sy, sz), yaw=s['yaw'], rgb=s['rgb']))
            else:
                out.append(dict(kind='cyl', name=s['id'], model=f"debris_{s['id']}", c=(s['x'], s['y'], sz / 2),
                                r=sx / 2, h=sz, rgb=s['rgb']))
    for d in DEFECTS:
        if d['type'] == 'deformation':
            cyl = deformation_cylinder(d)
            out.append(dict(kind='cyl', name=d['id'], model=f"defect_{d['id']}", c=(cyl['cx'], cyl['cy'], 0.5),
                            r=cyl['r'], h=1.0, rgb=(0.50, 0.48, 0.43)))
        elif d['type'] == 'crack':
            out.append(dict(kind='crack', name=d['id'], model='decor', polyline=crack_polyline(d),
                            line_w=d['line_w'], rgb=(0.06, 0.05, 0.05)))
    for k, rim in enumerate(disposal_rims()):
        out.append(dict(kind='box', name=f'rim{k}', model='disposal_station', yaw=0.0, rgb=(0.15, 0.6, 0.25), **rim))
    # flat floor clutter like the Gazebo world (water channel and sludge): must NOT be detected as debris
    for a, b in NET['edges']:
        (ax, ay), (bx, by) = NET['nodes'][a], NET['nodes'][b]
        horiz = ay == by
        length = math.dist((ax, ay), (bx, by))
        out.append(dict(kind='box', name=f'water_{a}{b}', model='decor', c=((ax + bx) / 2, (ay + by) / 2, 0.008),
                        size=(length + 0.3, 0.3, 0.008) if horiz else (0.3, length + 0.3, 0.008), yaw=0.0,
                        rgb=(0.16, 0.21, 0.16)))
        for f, side, rad in ((0.3, 1, 0.28), (0.72, -1, 0.35)):
            px, py = ax + (bx - ax) * f, ay + (by - ay) * f
            px, py = (px, py + side * 0.2) if horiz else (px + side * 0.2, py)
            out.append(dict(kind='cyl', name=f'sludge_{a}{b}{f}', model='decor', c=(px, py, 0.015), r=rad, h=0.006,
                            rgb=(0.22, 0.15, 0.09)))
    return out


def disposal_rims():
    d = DISPOSAL
    t, hh = d['rim_t'], d['rim_h']
    cx, cy = (d['x_min'] + d['x_max']) / 2, (d['y_min'] + d['y_max']) / 2
    sx, sy = d['x_max'] - d['x_min'], d['y_max'] - d['y_min']
    return [dict(c=(cx, d['y_min'] - t / 2, hh / 2), size=(sx + 2 * t, t, hh)),
            dict(c=(cx, d['y_max'] + t / 2, hh / 2), size=(sx + 2 * t, t, hh)),
            dict(c=(d['x_min'] - t / 2, cy, hh / 2), size=(t, sy, hh)),
            dict(c=(d['x_max'] + t / 2, cy, hh / 2), size=(t, sy, hh))]


def true_lidar_map(prior, upsample=5):
    """2D world as the LiDAR plane (z = 0.13 m) sees it: prior map with recesses cut and bulges/blockages added.

    Built on a grid `upsample` times finer than the prior map (1 cm by default) so curved bulges keep their depth.
    Small debris and the disposal tray are lower than the scan plane, so they are not in it.
    """
    from sewer_localization.map_utils import OccupancyMap
    x0, y0, free, wall = build()
    assert abs(prior.x0 - x0) < 1e-6 and abs(prior.y0 - y0) < 1e-6 and prior.solid.shape == wall.shape
    _, _, _, wall_prior = gn.build_grid(NET)
    recess = wall_prior & ~wall
    k = int(upsample)
    up = lambda a: np.repeat(np.repeat(a, k, 0), k, 1)   # noqa: E731
    m = OccupancyMap(up(prior.solid) & ~up(recess), prior.res / k, prior.x0, prior.y0)
    for d in DEFECTS:
        if d['type'] == 'deformation':
            c = deformation_cylinder(d)
            m.add_circle(c['cx'], c['cy'], c['r'])
    for b in NET['blockages'].values():
        for p in b['pieces']:
            if p[0] == 'box':
                m.add_box(p[1], p[2], p[3], p[4])
            else:
                m.add_circle(p[1], p[2], p[3])
    return m


def truth_dict():
    """Evaluation labels (written to config/inspection_truth.yaml)."""
    defects = []
    for d in DEFECTS:
        (fx, fy), n = wall_face(d)
        z = d.get('z', 0.13 if d['type'] != 'crack' else d['z'])
        entry = dict(id=d['id'], type=d['type'], edge=list(d['edge']), wall=d['wall'], x=round(fx, 3), y=round(fy, 3),
                     z=round(z, 3), normal=list(n), width=d['width'], depth=d['depth'])
        if d['type'] == 'crack':
            entry['length'] = d['width']
        defects.append(entry)
    debris = [dict(id=s['id'], kind=s['kind'], x=s['x'], y=s['y'], yaw=round(s['yaw'], 4), size=list(s['size']),
                   mass=s['mass'], graspable_designed=s['graspable_designed'], note=s['note']) for s in SMALL_DEBRIS]
    large = [dict(id=k, pieces=[list(p) for p in b['pieces']], graspable_designed=False)
             for k, b in NET['blockages'].items()]
    return dict(comment='GROUND TRUTH, EVALUATION ONLY. Never read this from perception, manipulation or planning.',
                base_network='sewer_network_v2', prior_map='maps/sewer_map_v2', small_debris=debris,
                large_debris=large, defects=defects, disposal=DISPOSAL)
