#!/usr/bin/env python3
"""Generate the sewer network from ONE description (the NODES/EDGES/BLOCKAGES tables below).

Outputs (into the sewer_gazebo package directory):
  worlds/sewer_network.sdf        Gazebo world WITH blockages (SIMULATED by Gazebo when loaded)
  worlds/sewer_network_clear.sdf  same network, no blockages
  maps/sewer_map.pgm + .yaml      a-priori occupancy map (walls only; blockages are NOT in the map)
  config/network_graph.yaml       topology the planner may use (nodes, edges, start, goal)
  config/blockages_truth.yaml     where the blockages really are: EVALUATION ONLY, never read by
                                  localization, perception or planning

Corridors are box-section channels (flat floor, vertical walls) so a wheeled robot can drive and the
2D LiDAR plane sees the same geometry as in a round pipe. Walls are built from the same 5 cm grid as
the map, so the map and the Gazebo world agree by construction.

Usage:  python3 gen_network.py --out <path to sewer_gazebo> [--preview preview.png]
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

# Node centre-lines [m]. Corridor rectangles extend HALF_W past each node, which closes dead ends
# and makes bends/junctions meet cleanly.
NODES = {
    'A': (0.0, 0.0),     # entry (dead end behind the robot start)
    'J1': (8.0, 0.0),    # 4-way junction
    'D': (8.0, -5.0),    # dead end (south spur)
    'J2': (8.0, 6.0),    # bend
    'M': (12.0, 6.0),    # T-junction
    'E': (12.0, 10.0),   # dead end (north spur)
    'J3': (16.0, 6.0),   # bend
    'J4': (16.0, 0.0),   # goal (bend)
}
EDGES = [('A', 'J1'), ('J1', 'J4'), ('J1', 'D'), ('J1', 'J2'),
         ('J2', 'M'), ('M', 'E'), ('M', 'J3'), ('J3', 'J4')]
START, GOAL = 'A', 'J4'

# Blockage definition: a blockage is debris that leaves a free gap narrower than
# MIN_PASSABLE_GAP = robot width (0.30 m incl. wheels) + 2 x 0.10 m clearance = 0.50 m.
# 'full' = no gap; 'partial' = gap wider than the threshold (passable but tight).
MIN_PASSABLE_GAP = 0.50
BLOCKAGES = {
    # Full blockage on the short direct route J1 -> J4 (forces the long loop J1-J2-M-J3-J4).
    'B1': dict(edge=['J1', 'J4'], kind='full', free_gap=0.0,
               pieces=[('box', 12.0, -0.25, 0.5, 0.6, 0.5),
                       ('box', 12.1, 0.25, 0.6, 0.6, 0.4),
                       ('cyl', 11.8, 0.0, 0.25, 0.5)]),
    # Partial blockage on the loop's first leg J1 -> J2: 0.60 m free gap on the east side.
    'B2': dict(edge=['J1', 'J2'], kind='partial', free_gap=0.6,
               pieces=[('box', 7.7, 3.0, 0.4, 0.6, 0.35),
                       ('cyl', 7.7, 3.55, 0.12, 0.3)]),
}


def build_grid():
    rects = []
    for a, b in EDGES:
        (ax, ay), (bx, by) = NODES[a], NODES[b]
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


def box_xml(kind, name, cx, cy, cz, sx, sy, sz, rgba):
    geom = f'<geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>'
    return geom_xml(kind, name, cx, cy, cz, geom, rgba)


def geom_xml(kind, name, cx, cy, cz, geom, rgba):
    pose = f'<pose>{cx:.3f} {cy:.3f} {cz:.3f} 0 0 0</pose>'
    if kind == 'collision':
        return f'<collision name="{name}">{pose}{geom}</collision>'
    c = ' '.join(str(v) for v in rgba)
    return (f'<visual name="{name}">{pose}{geom}'
            f'<material><ambient>{c}</ambient><diffuse>{c}</diffuse></material></visual>')


def world_xml(x0, y0, wall, with_blockages):
    walls = []
    for n, (i0, i1, j0, j1) in enumerate(merge_rects(wall)):
        sx, sy = (i1 - i0) * RES, (j1 - j0) * RES
        cx, cy = x0 + (i0 + i1) / 2 * RES, y0 + (j0 + j1) / 2 * RES
        walls.append('        ' + box_xml('collision', f'c{n}', cx, cy, WALL_H / 2, sx, sy, WALL_H, None))
        walls.append('        ' + box_xml('visual', f'v{n}', cx, cy, WALL_H / 2, sx, sy, WALL_H, (0.55, 0.5, 0.45, 1)))
    debris = []
    if with_blockages:
        for bname, b in BLOCKAGES.items():
            parts = []
            for n, p in enumerate(b['pieces']):
                if p[0] == 'box':
                    _, cx, cy, sx, sy, h = p
                    geom = f'<geometry><box><size>{sx} {sy} {h}</size></box></geometry>'
                else:
                    _, cx, cy, r, h = p
                    geom = f'<geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>'
                parts.append('        ' + geom_xml('collision', f'c{n}', cx, cy, p[-1] / 2, geom, None))
                parts.append('        ' + geom_xml('visual', f'v{n}', cx, cy, p[-1] / 2, geom, (0.6, 0.18, 0.12, 1)))
            debris.append(f'    <model name="debris_{bname}"><static>true</static><link name="link">\n'
                          + '\n'.join(parts) + '\n    </link></model>')
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
    <scene><ambient>0.35 0.35 0.38 1</ambient><background>0.08 0.10 0.14 1</background><shadows>true</shadows></scene>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows><pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.88 0.82 1</diffuse><specular>0.2 0.2 0.2 1</specular><direction>-0.5 0.35 -0.8</direction>
    </light>
    <model name="floor"><static>true</static><link name="link">
      <collision name="c"><geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry></collision>
      <visual name="v"><geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry>
        <material><ambient>0.15 0.17 0.2 1</ambient><diffuse>0.15 0.17 0.2 1</diffuse></material></visual>
    </link></model>
    <model name="walls"><static>true</static><link name="link">
{chr(10).join(walls)}
    </link></model>
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


def write_graph(cfg_dir):
    deg = {n: 0 for n in NODES}
    for a, b in EDGES:
        deg[a] += 1
        deg[b] += 1
    kind = lambda d: 'dead_end' if d == 1 else ('bend' if d == 2 else 'junction')  # noqa: E731
    graph = {
        'comment': 'Topology known a priori. Blockages are NOT listed here; they must be detected.',
        'start': START, 'goal': GOAL, 'corridor_width': 2 * HALF_W,
        'nodes': {n: {'x': p[0], 'y': p[1], 'type': kind(deg[n])} for n, p in NODES.items()},
        'edges': [{'from': a, 'to': b,
                   'length': round(math.dist(NODES[a], NODES[b]), 3)} for a, b in EDGES],
    }
    with open(os.path.join(cfg_dir, 'network_graph.yaml'), 'w') as f:
        yaml.safe_dump(graph, f, sort_keys=False)
    truth = {
        'comment': 'GROUND TRUTH, EVALUATION ONLY. Never read this from localization/perception/planning.',
        'min_passable_gap': MIN_PASSABLE_GAP,
        'blockages': {k: {'edge': v['edge'], 'kind': v['kind'], 'free_gap': v['free_gap'],
                          'blocks_route': v['free_gap'] < MIN_PASSABLE_GAP,
                          'pieces': [list(p) for p in v['pieces']]} for k, v in BLOCKAGES.items()},
    }
    with open(os.path.join(cfg_dir, 'blockages_truth.yaml'), 'w') as f:
        yaml.safe_dump(truth, f, sort_keys=False)


def preview(path, x0, y0, img):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    h, w = img.shape
    fig, ax = plt.subplots(figsize=(11, 10))
    ax.imshow(np.flipud(img), cmap='gray', origin='upper',
              extent=[x0, x0 + w * RES, y0, y0 + h * RES], vmin=0, vmax=255)
    for a, b in EDGES:
        ax.plot(*zip(NODES[a], NODES[b]), 'c--', lw=1)
    for n, (x, y) in NODES.items():
        ax.plot(x, y, 'bo')
        ax.annotate(n, (x, y), xytext=(4, 4), textcoords='offset points', color='blue', fontsize=12)
    for k, b in BLOCKAGES.items():
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
    ap.add_argument('--preview', help='optional PNG preview path')
    a = ap.parse_args()
    for d in ('worlds', 'maps', 'config'):
        os.makedirs(os.path.join(a.out, d), exist_ok=True)
    x0, y0, free, wall = build_grid()
    for name, with_b in (('sewer_network', True), ('sewer_network_clear', False)):
        with open(os.path.join(a.out, 'worlds', name + '.sdf'), 'w') as f:
            f.write(world_xml(x0, y0, wall, with_b))
    img = write_map(os.path.join(a.out, 'maps', 'sewer_map'), x0, y0, free, wall)
    write_graph(os.path.join(a.out, 'config'))
    n = len(merge_rects(wall))
    print(f'grid {free.shape[1]}x{free.shape[0]} cells, origin ({x0:.2f},{y0:.2f}), {n} wall boxes')
    if a.preview:
        preview(a.preview, x0, y0, img)


if __name__ == '__main__':
    main()
