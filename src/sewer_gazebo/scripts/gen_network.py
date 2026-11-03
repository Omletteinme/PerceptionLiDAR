#!/usr/bin/env python3
"""Generate the sewer networks from ONE description per network (the tables below).

Networks (NETWORKS): 'sewer_network' (the classic one) and 'sewer_network_v2' (classic plus a second blockage and a
second loop). The file names follow one rule, suffix = name minus 'sewer_network' ('' or '_v2'):
  worlds/<name>.sdf, worlds/<name>_clear.sdf   Gazebo world WITH / WITHOUT blockages (SIMULATED by Gazebo when loaded)
  maps/sewer_map<suffix>.pgm + .yaml           a-priori occupancy map (walls only; blockages are NOT in the map)
  config/network_graph<suffix>.yaml            topology the planner may use (nodes, edges, start, goal)
  config/blockages_truth<suffix>.yaml          where the blockages really are: EVALUATION ONLY, never read by
                                               localization, perception or planning
Every world is called "sewer_network" inside (the contact-sensor bridge in robot_sim.launch.py relies on that).

Corridors are box-section channels (flat floor, vertical walls) so a wheeled robot can drive and the
2D LiDAR plane sees the same geometry as in a round pipe. Walls are built from the same 5 cm grid as
the map, so the map and the Gazebo world agree by construction.

LOOKS (visual only, no collision shapes): two-tone walls, coping, grimy floor, a murky water channel, sludge patches, ladders
and manhole collars at the dead ends, inflow pipes, lamps, rubble on the debris. The GPU LiDAR ray-casts VISUAL geometry at
about z = 0.13 m, so every decoration goes through Decor.add(), which refuses any shape that reaches into the band
LIDAR_BAND above free corridor space (it would show up as a phantom obstacle).

Usage:  python3 gen_network.py --out <path to sewer_gazebo> [--network all|sewer_network|sewer_network_v2] [--preview-dir dir]
"""
import argparse
import math
import os

import numpy as np
import yaml

RES = 0.05      # grid / map resolution [m]
HALF_W = 0.5    # corridor half-width: corridors are 1.0 m wide
WALL_T = 0.2    # wall thickness [m]
WALL_H = 1.0    # wall height [m]
MARGIN = 1.0    # free margin outside the walls in the map [m]
LIDAR_BAND = (0.05, 0.25)    # z range [m] that must stay empty of decoration above free space

# Blockage definition: a blockage is debris that leaves a free gap narrower than
# MIN_PASSABLE_GAP = robot width (0.30 m incl. wheels) + 2 x 0.10 m clearance = 0.50 m.
# 'full' = no gap; 'partial' = gap wider than the threshold (passable but tight).
MIN_PASSABLE_GAP = 0.50

# Node centre-lines [m]. Corridor rectangles extend HALF_W past each node, which closes dead ends
# and makes bends/junctions meet cleanly.
CLASSIC = dict(
    nodes={
        'A': (0.0, 0.0),     # entry (dead end behind the robot start)
        'J1': (8.0, 0.0),    # 4-way junction
        'D': (8.0, -5.0),    # dead end (south spur)
        'J2': (8.0, 6.0),    # bend
        'M': (12.0, 6.0),    # T-junction
        'E': (12.0, 10.0),   # dead end (north spur)
        'J3': (16.0, 6.0),   # bend
        'J4': (16.0, 0.0),   # goal (bend)
    },
    edges=[('A', 'J1'), ('J1', 'J4'), ('J1', 'D'), ('J1', 'J2'),
           ('J2', 'M'), ('M', 'E'), ('M', 'J3'), ('J3', 'J4')],
    start='A', goal='J4',
    blockages={
        # Full blockage on the short direct route J1 -> J4 (forces the long loop J1-J2-M-J3-J4).
        'B1': dict(edge=['J1', 'J4'], kind='full', free_gap=0.0,
                   pieces=[('box', 12.0, -0.25, 0.5, 0.6, 0.5),
                           ('box', 12.1, 0.25, 0.6, 0.6, 0.4),
                           ('cyl', 11.8, 0.0, 0.25, 0.5)]),
        # Partial blockage on the loop's first leg J1 -> J2: 0.60 m free gap on the east side.
        'B2': dict(edge=['J1', 'J2'], kind='partial', free_gap=0.6,
                   pieces=[('box', 7.7, 3.0, 0.4, 0.6, 0.35),
                           ('cyl', 7.7, 3.55, 0.12, 0.3)]),
    })

# v2: the north spur M-E is closed into a second loop (E-F-J3) and a second full blockage B3 sits on M-J3, so the robot
# must detect two blockages and replan twice: A-J1-J2-M-E-F-J3-J4 once both are known. The outline of the map does not grow.
V2 = dict(
    nodes={**CLASSIC['nodes'], 'F': (16.0, 10.0)},
    edges=CLASSIC['edges'] + [('E', 'F'), ('F', 'J3')],
    start='A', goal='J4',
    blockages={**CLASSIC['blockages'],
               'B3': dict(edge=['M', 'J3'], kind='full', free_gap=0.0,
                          pieces=[('box', 14.0, 5.75, 0.5, 0.6, 0.5),
                                  ('box', 14.1, 6.25, 0.6, 0.6, 0.4),
                                  ('cyl', 13.8, 6.0, 0.25, 0.5)])})

NETWORKS = {'sewer_network': CLASSIC, 'sewer_network_v2': V2}


def suffix(name):
    return name[len('sewer_network'):]


def build_grid(net):
    nodes, edges = net['nodes'], net['edges']
    rects = []
    for a, b in edges:
        (ax, ay), (bx, by) = nodes[a], nodes[b]
        assert ax == bx or ay == by, f'edge {a}-{b} is not axis aligned'
        rects.append((min(ax, bx) - HALF_W, min(ay, by) - HALF_W,
                      max(ax, bx) + HALF_W, max(ay, by) + HALF_W))
    pad = WALL_T + MARGIN
    x0 = math.floor((min(r[0] for r in rects) - pad) / RES) * RES
    y0 = math.floor((min(r[1] for r in rects) - pad) / RES) * RES
    x1 = math.ceil((max(r[2] for r in rects) + pad) / RES) * RES
    y1 = math.ceil((max(r[3] for r in rects) + pad) / RES) * RES
    w, h = int(round((x1 - x0) / RES)), int(round((y1 - y0) / RES))
    free = np.zeros((h, w), bool)   # row j <-> y = y0 + j*RES (bottom-up)
    for r in rects:
        i0, i1 = int(round((r[0] - x0) / RES)), int(round((r[2] - x0) / RES))
        j0, j1 = int(round((r[1] - y0) / RES)), int(round((r[3] - y0) / RES))
        free[j0:j1, i0:i1] = True
    k = int(round(WALL_T / RES))
    padded = np.pad(free, k)
    dil = free.copy()
    for dy in range(-k, k + 1):
        for dx in range(-k, k + 1):
            dil |= padded[k + dy:k + dy + h, k + dx:k + dx + w]
    wall = dil & ~free
    return x0, y0, free, wall


def merge_rects(mask):
    """Greedy merge of True cells into rectangles (i0, i1, j0, j1), half-open cell ranges."""
    h, w = mask.shape
    out, active = [], {}
    for j in range(h + 1):
        runs = set()
        if j < h:
            i = 0
            while i < w:
                if mask[j, i]:
                    s = i
                    while i < w and mask[j, i]:
                        i += 1
                    runs.add((s, i))
                else:
                    i += 1
        for key in list(active):
            if key not in runs:
                out.append((key[0], key[1], active.pop(key), j))
        for key in runs:
            active.setdefault(key, j)
    return out


def pose_xml(cx, cy, cz, roll=0.0, pitch=0.0, yaw=0.0):
    return f'<pose>{cx:.3f} {cy:.3f} {cz:.3f} {roll:.4f} {pitch:.4f} {yaw:.4f}</pose>'


def box_xml(kind, name, cx, cy, cz, sx, sy, sz, rgba):
    geom = f'<geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>'
    return geom_xml(kind, name, cx, cy, cz, geom, rgba)


def geom_xml(kind, name, cx, cy, cz, geom, rgba, rpy=(0.0, 0.0, 0.0), transparency=0.0):
    pose = pose_xml(cx, cy, cz, *rpy)
    if kind == 'collision':
        return f'<collision name="{name}">{pose}{geom}</collision>'
    c = ' '.join(str(round(v, 3)) for v in rgba)
    tr = f'<transparency>{transparency}</transparency>' if transparency else ''
    return (f'<visual name="{name}">{pose}{geom}{tr}'
            f'<material><ambient>{c}</ambient><diffuse>{c}</diffuse></material></visual>')


class Decor:
    """Collects VISUAL-only decoration and enforces the LiDAR rule: nothing in LIDAR_BAND above free corridor space."""

    def __init__(self, x0, y0, free):
        self.x0, self.y0, self.free = x0, y0, free
        self.items, self.n = [], 0

    def blocks_lidar(self, cx, cy, cz, sx, sy, sz):
        """True if the (axis-aligned bounding) box overlaps the LiDAR band over any free cell."""
        if cz + sz / 2 <= LIDAR_BAND[0] or cz - sz / 2 >= LIDAR_BAND[1]:
            return False
        h, w = self.free.shape
        i0 = max(0, int(math.floor((cx - sx / 2 - self.x0) / RES)))
        i1 = min(w, int(math.ceil((cx + sx / 2 - self.x0) / RES)))
        j0 = max(0, int(math.floor((cy - sy / 2 - self.y0) / RES)))
        j1 = min(h, int(math.ceil((cy + sy / 2 - self.y0) / RES)))
        return bool(self.free[j0:j1, i0:i1].any())

    def _check(self, name, bbox):
        if self.blocks_lidar(*bbox):
            raise ValueError(f'decoration {name!r} {bbox} would intersect the LiDAR band above free space')

    def box(self, name, cx, cy, cz, sx, sy, sz, rgba, rpy=(0, 0, 0), transparency=0.0):
        self._check(name, (cx, cy, cz, sx, sy, sz))
        geom = f'<geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>'
        self.items.append(geom_xml('visual', f'd{self.n}_{name}', cx, cy, cz, geom, rgba, rpy, transparency))
        self.n += 1

    def cyl(self, name, cx, cy, cz, r, length, rgba, axis='z', transparency=0.0):
        """Cylinder with its axis along z, x or y (bounding box computed accordingly)."""
        rpy = {'z': (0, 0, 0), 'x': (0, math.pi / 2, 0), 'y': (math.pi / 2, 0, 0)}[axis]
        size = {'z': (2 * r, 2 * r, length), 'x': (length, 2 * r, 2 * r), 'y': (2 * r, length, 2 * r)}[axis]
        self._check(name, (cx, cy, cz) + size)
        geom = f'<geometry><cylinder><radius>{r:.3f}</radius><length>{length:.3f}</length></cylinder></geometry>'
        self.items.append(geom_xml('visual', f'd{self.n}_{name}', cx, cy, cz, geom, rgba, rpy, transparency))
        self.n += 1


def jitter(rng, rgb, amount=0.04):
    return tuple(float(np.clip(c + rng.uniform(-amount, amount), 0, 1)) for c in rgb) + (1.0,)


def decorate(net, x0, y0, free, wall):
    """All the decorative visuals of one network (see the module docstring for the list)."""
    d = Decor(x0, y0, free)
    rng = np.random.default_rng(7)
    nodes, edges = net['nodes'], net['edges']
    cell = lambda i, j: (x0 + (i + 0.5) * RES, y0 + (j + 0.5) * RES)   # noqa: E731
    # corridor floor: thin grimy slabs over the free cells
    for i0, i1, j0, j1 in merge_rects(free):
        d.box('floor', x0 + (i0 + i1) / 2 * RES, y0 + (j0 + j1) / 2 * RES, 0.002, (i1 - i0) * RES, (j1 - j0) * RES, 0.004,
              jitter(rng, (0.34, 0.33, 0.31), 0.02))
    # murky water channel down the middle of every edge, and a few sludge patches beside it
    for a, b in edges:
        (ax, ay), (bx, by) = nodes[a], nodes[b]
        horiz = ay == by
        length = math.dist((ax, ay), (bx, by))
        cx, cy = (ax + bx) / 2, (ay + by) / 2
        d.box('water', cx, cy, 0.008, length + 0.3 if horiz else 0.3, 0.3 if horiz else length + 0.3, 0.008,
              (0.16, 0.21, 0.16, 1.0), transparency=0.35)
        for f, side, rad in ((0.3, 1, 0.28), (0.72, -1, 0.35)):
            px, py = ax + (bx - ax) * f, ay + (by - ay) * f
            off = side * 0.2
            px, py = (px, py + off) if horiz else (px + off, py)
            d.cyl('sludge', px, py, 0.015, rad, 0.006, (0.22, 0.15, 0.09, 1.0))
    # dead ends: ladder on the end wall and a manhole collar above it
    deg = {n: 0 for n in nodes}
    for a, b in edges:
        deg[a] += 1
        deg[b] += 1
    for n, (x, y) in nodes.items():
        if deg[n] != 1:
            continue
        nb = next(b if a == n else a for a, b in edges if n in (a, b))
        ux, uy = nodes[nb][0] - x, nodes[nb][1] - y
        ln = math.hypot(ux, uy)
        ux, uy = ux / ln, uy / ln                       # unit vector from the dead end into the corridor
        fx, fy = x - ux * (HALF_W - 0.04), y - uy * (HALF_W - 0.04)     # just in front of the end wall
        px, py = -uy, ux                                # across the corridor
        steel = (0.55, 0.5, 0.42, 1.0)
        for s in (-0.2, 0.2):                           # rails
            d.box('rail', fx + px * s, fy + py * s, 0.65, 0.03 if px == 0 else 0.03, 0.03, 0.62, steel)
        for z in (0.40, 0.58, 0.76, 0.94):              # rungs
            d.box('rung', fx, fy, z, 0.03 if abs(px) < 0.5 else 0.42, 0.42 if abs(px) < 0.5 else 0.03, 0.03, steel)
        d.cyl('manhole', x - ux * (HALF_W + 0.1), y - uy * (HALF_W + 0.1), WALL_H + 0.07, 0.42, 0.14, (0.3, 0.29, 0.27, 1.0))
    # inflow pipes in the side walls, away from junctions
    for k, (a, b) in enumerate(edges):
        (ax, ay), (bx, by) = nodes[a], nodes[b]
        length = math.dist((ax, ay), (bx, by))
        if length < 4.0:
            continue
        horiz = ay == by
        f = 0.5 + 0.12 * (-1) ** k
        side = 1 if k % 2 == 0 else -1
        px, py = ax + (bx - ax) * f, ay + (by - ay) * f
        wx, wy = (px, py + side * HALF_W) if horiz else (px + side * HALF_W, py)
        i, j = int((wx + (0.05 if not horiz else 0) * side - x0) / RES), int((wy + (0.05 if horiz else 0) * side - y0) / RES)
        if not wall[j, i]:
            continue
        axis = 'y' if horiz else 'x'
        tipx, tipy = (wx, wy - side * 0.14) if horiz else (wx - side * 0.14, wy)
        d.cyl('pipe', wx, wy, 0.55, 0.12, 0.30, (0.28, 0.27, 0.26, 1.0), axis=axis)
        d.cyl('pipemouth', tipx, tipy, 0.55, 0.095, 0.03, (0.04, 0.04, 0.04, 1.0), axis=axis)
    # rubble on the debris (visual only, above the LiDAR band and inside the pieces' footprints)
    for b in net['blockages'].values():
        for p in b['pieces']:
            h = p[-1]
            if p[0] == 'box':
                _, cx, cy, sx, sy, _ = p
                for _ in range(3):
                    s = rng.uniform(0.08, 0.14)
                    d.box('rubble', cx + rng.uniform(-sx / 2 + s / 2, sx / 2 - s / 2), cy + rng.uniform(-sy / 2 + s / 2, sy / 2 - s / 2),
                          h + s / 2, s, s, s, jitter(rng, (0.42, 0.38, 0.34), 0.06))
            else:
                _, cx, cy, r, _ = p
                d.cyl('rubble', cx, cy, h + 0.03, r * 0.8, 0.06, jitter(rng, (0.4, 0.3, 0.22), 0.06))
    return d


def lamps_xml(net):
    nodes, edges = net['nodes'], net['edges']
    deg = {n: 0 for n in nodes}
    for a, b in edges:
        deg[a] += 1
        deg[b] += 1
    out = []
    for n, (x, y) in nodes.items():
        if deg[n] >= 2:
            out.append(f'    <light type="point" name="lamp_{n}"><cast_shadows>false</cast_shadows><pose>{x:.2f} {y:.2f} 2.4 0 0 0</pose>'
                       '<diffuse>1.0 0.82 0.55 1</diffuse><specular>0.1 0.1 0.1 1</specular>'
                       '<attenuation><range>9</range><constant>0.5</constant><linear>0.08</linear><quadratic>0.01</quadratic></attenuation></light>')
    return '\n'.join(out)


def world_xml(net, x0, y0, free, wall, with_blockages):
    rng = np.random.default_rng(3)
    walls = []
    for n, (i0, i1, j0, j1) in enumerate(merge_rects(wall)):
        sx, sy = (i1 - i0) * RES, (j1 - j0) * RES
        cx, cy = x0 + (i0 + i1) / 2 * RES, y0 + (j0 + j1) / 2 * RES
        walls.append('        ' + box_xml('collision', f'c{n}', cx, cy, WALL_H / 2, sx, sy, WALL_H, None))
        walls.append('        ' + box_xml('visual', f'vl{n}', cx, cy, 0.15, sx, sy, 0.30, (0.20, 0.22, 0.20, 1)))   # wet lower band
        walls.append('        ' + box_xml('visual', f'vu{n}', cx, cy, 0.65, sx, sy, 0.70, jitter(rng, (0.55, 0.52, 0.47))))
        walls.append('        ' + box_xml('visual', f'vc{n}', cx, cy, WALL_H + 0.02, sx + 0.04, sy + 0.04, 0.04, (0.66, 0.63, 0.58, 1)))
    debris = []
    if with_blockages:
        for bname, b in net['blockages'].items():
            parts = []
            for n, p in enumerate(b['pieces']):
                if p[0] == 'box':
                    _, cx, cy, sx, sy, h = p
                    geom = f'<geometry><box><size>{sx} {sy} {h}</size></box></geometry>'
                else:
                    _, cx, cy, r, h = p
                    geom = f'<geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>'
                parts.append('        ' + geom_xml('collision', f'c{n}', cx, cy, p[-1] / 2, geom, None))
                parts.append('        ' + geom_xml('visual', f'v{n}', cx, cy, p[-1] / 2, geom, jitter(rng, (0.5, 0.2, 0.13), 0.05)))
            debris.append(f'    <model name="debris_{bname}"><static>true</static><link name="link">\n'
                          + '\n'.join(parts) + '\n    </link></model>')
    dec = decorate({**net, 'blockages': net['blockages'] if with_blockages else {}}, x0, y0, free, wall)
    decor_xml = '    <model name="decor"><static>true</static><link name="link">\n' + '\n'.join('        ' + s for s in dec.items) \
        + '\n    </link></model>'
    return f'''<?xml version="1.0"?>
<!-- GENERATED by scripts/gen_network.py - do not edit by hand. -->
<sdf version="1.9">
  <world name="sewer_network">
    <physics name="1ms" type="dart"><max_step_size>0.001</max_step_size><real_time_factor>1.0</real_time_factor></physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <gravity>0 0 -9.8</gravity>
    <scene><ambient>0.30 0.30 0.33 1</ambient><background>0.08 0.10 0.14 1</background><shadows>true</shadows></scene>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows><pose>0 0 10 0 0 0</pose>
      <diffuse>0.75 0.73 0.68 1</diffuse><specular>0.15 0.15 0.15 1</specular><direction>-0.5 0.35 -0.8</direction>
    </light>
{lamps_xml(net)}
    <model name="floor"><static>true</static><link name="link">
      <collision name="c"><geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry></collision>
      <visual name="v"><geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry>
        <material><ambient>0.10 0.09 0.08 1</ambient><diffuse>0.10 0.09 0.08 1</diffuse></material></visual>
    </link></model>
    <model name="walls"><static>true</static><link name="link">
{chr(10).join(walls)}
    </link></model>
{decor_xml}
{chr(10).join(debris)}
    <!-- Top-down camera for scene checks only (renders only while something subscribes). -->
    <model name="topdown_cam"><static>true</static><pose>8.5 2.5 24 0 1.5708 1.5708</pose><link name="link">
      <sensor name="cam" type="camera"><always_on>true</always_on><update_rate>1</update_rate>
        <topic>topdown_cam/image</topic>
        <camera><horizontal_fov>0.9</horizontal_fov><image><width>1000</width><height>900</height></image>
          <clip><near>0.1</near><far>60</far></clip></camera></sensor>
    </link></model>
    <!-- Oblique overview camera for 3D renders: looks at the middle of the network from the south-west. -->
    <model name="oblique_cam"><static>true</static><pose>-6 -10 12 0 0.56 0.71</pose><link name="link">
      <sensor name="cam" type="camera"><always_on>true</always_on><update_rate>10</update_rate>
        <topic>oblique_cam/image</topic>
        <camera><horizontal_fov>1.1</horizontal_fov><image><width>960</width><height>540</height></image>
          <clip><near>0.1</near><far>80</far></clip></camera></sensor>
    </link></model>
  </world>
</sdf>
'''


def write_map(path, x0, y0, free, wall):
    h, w = free.shape
    img = np.full((h, w), 205, np.uint8)   # unknown
    img[free] = 254
    img[wall] = 0
    top_down = np.flipud(img)              # PGM rows run top (max y) to bottom
    with open(path + '.pgm', 'wb') as f:
        f.write(f'P5\n{w} {h}\n255\n'.encode())
        f.write(top_down.tobytes())
    with open(path + '.yaml', 'w') as f:
        yaml.safe_dump({'image': os.path.basename(path) + '.pgm', 'mode': 'trinary', 'resolution': RES,
                        'origin': [round(x0, 3), round(y0, 3), 0.0], 'negate': 0,
                        'occupied_thresh': 0.65, 'free_thresh': 0.196}, f, sort_keys=False)
    return img


def write_graph(net, cfg_dir, sfx):
    nodes, edges = net['nodes'], net['edges']
    deg = {n: 0 for n in nodes}
    for a, b in edges:
        deg[a] += 1
        deg[b] += 1
    kind = lambda d: 'dead_end' if d == 1 else ('bend' if d == 2 else 'junction')  # noqa: E731
    graph = {
        'comment': 'Topology known a priori. Blockages are NOT listed here; they must be detected.',
        'start': net['start'], 'goal': net['goal'], 'corridor_width': 2 * HALF_W,
        'nodes': {n: {'x': p[0], 'y': p[1], 'type': kind(deg[n])} for n, p in nodes.items()},
        'edges': [{'from': a, 'to': b,
                   'length': round(math.dist(nodes[a], nodes[b]), 3)} for a, b in edges],
    }
    with open(os.path.join(cfg_dir, f'network_graph{sfx}.yaml'), 'w') as f:
        yaml.safe_dump(graph, f, sort_keys=False)
    truth = {
        'comment': 'GROUND TRUTH, EVALUATION ONLY. Never read this from localization/perception/planning.',
        'min_passable_gap': MIN_PASSABLE_GAP,
        'blockages': {k: {'edge': v['edge'], 'kind': v['kind'], 'free_gap': v['free_gap'],
                          'blocks_route': v['free_gap'] < MIN_PASSABLE_GAP,
                          'pieces': [list(p) for p in v['pieces']]} for k, v in net['blockages'].items()},
    }
    with open(os.path.join(cfg_dir, f'blockages_truth{sfx}.yaml'), 'w') as f:
        yaml.safe_dump(truth, f, sort_keys=False)


def preview(net, path, x0, y0, img):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    nodes, edges = net['nodes'], net['edges']
    h, w = img.shape
    fig, ax = plt.subplots(figsize=(11, 10))
    ax.imshow(np.flipud(img), cmap='gray', origin='upper',
              extent=[x0, x0 + w * RES, y0, y0 + h * RES], vmin=0, vmax=255)
    for a, b in edges:
        ax.plot(*zip(nodes[a], nodes[b]), 'c--', lw=1)
    for n, (x, y) in nodes.items():
        ax.plot(x, y, 'bo')
        ax.annotate(n, (x, y), xytext=(4, 4), textcoords='offset points', color='blue', fontsize=12)
    for k, b in net['blockages'].items():
        for p in b['pieces']:
            if p[0] == 'box':
                _, cx, cy, sx, sy, _ = p
                ax.add_patch(plt.Rectangle((cx - sx / 2, cy - sy / 2), sx, sy, color='orangered'))
            else:
                _, cx, cy, r, _ = p
                ax.add_patch(plt.Circle((cx, cy), r, color='orangered'))
        p0 = b['pieces'][0]
        ax.annotate(k, (p0[1], p0[2]), xytext=(10, 10), textcoords='offset points', color='red', fontsize=12)
    ax.set_aspect('equal')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title('Sewer network: map (grey/black), graph (cyan), blockages (red)')
    fig.savefig(path, dpi=80, bbox_inches='tight')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True, help='sewer_gazebo package directory')
    ap.add_argument('--network', default='all', choices=['all'] + list(NETWORKS))
    ap.add_argument('--preview-dir', help='optional directory for PNG previews')
    a = ap.parse_args()
    for dname in ('worlds', 'maps', 'config'):
        os.makedirs(os.path.join(a.out, dname), exist_ok=True)
    for name, net in NETWORKS.items():
        if a.network not in ('all', name):
            continue
        sfx = suffix(name)
        x0, y0, free, wall = build_grid(net)
        for wname, with_b in ((name, True), (name + '_clear', False)):
            with open(os.path.join(a.out, 'worlds', wname + '.sdf'), 'w') as f:
                f.write(world_xml(net, x0, y0, free, wall, with_b))
        img = write_map(os.path.join(a.out, 'maps', f'sewer_map{sfx}'), x0, y0, free, wall)
        write_graph(net, os.path.join(a.out, 'config'), sfx)
        n = len(merge_rects(wall))
        print(f'{name}: grid {free.shape[1]}x{free.shape[0]} cells, origin ({x0:.2f},{y0:.2f}), {n} wall boxes')
        if a.preview_dir:
            os.makedirs(a.preview_dir, exist_ok=True)
            preview(net, os.path.join(a.preview_dir, name + '.png'), x0, y0, img)


if __name__ == '__main__':
    main()
