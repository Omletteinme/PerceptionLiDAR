#!/usr/bin/env python3
"""Generate the inspection world (Account 2): sewer_network_v2 + small debris + structural defects + disposal station.

Writes (does NOT touch the existing worlds, maps or configs):
  worlds/sewer_inspection.sdf          Gazebo world (SIMULATED by Gazebo when loaded); world name stays "sewer_network"
                                       because robot_sim.launch.py's contact bridge relies on it
  config/inspection_truth.yaml         GROUND TRUTH, EVALUATION ONLY
  config/inspection_station.yaml       disposal station (a-priori configuration, used by the nodes)
The a-priori map for this world is the unchanged maps/sewer_map_v2 (debris and defects are not in it).

Usage:  python3 gen_inspection_world.py --out <path to sewer_gazebo>
"""
import argparse
import math
import os
import sys

import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gen_network as gn  # noqa: E402
import inspection_scene as sc  # noqa: E402


def inertia_xml(kind, mass, size):
    sx, sy, sz = size
    if kind == 'box':
        ixx, iyy, izz = mass * (sy * sy + sz * sz) / 12, mass * (sx * sx + sz * sz) / 12, mass * (sx * sx + sy * sy) / 12
    else:
        r = sx / 2
        ixx = iyy = mass * (3 * r * r + sz * sz) / 12
        izz = mass * r * r / 2
    return (f'<inertial><mass>{mass}</mass><inertia><ixx>{ixx:.3e}</ixx><iyy>{iyy:.3e}</iyy><izz>{izz:.3e}</izz>'
            '<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>')


def geom(kind, size):
    sx, sy, sz = size
    if kind == 'box':
        return f'<geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>'
    return f'<geometry><cylinder><radius>{sx / 2:.4f}</radius><length>{sz:.3f}</length></cylinder></geometry>'


def material(rgb):
    c = ' '.join(f'{v:.3f}' for v in rgb) + ' 1'
    return f'<material><ambient>{c}</ambient><diffuse>{c}</diffuse></material>'


def small_debris_xml():
    """Dynamic (movable) debris models: the gripper must be able to pick them up for real (no teleporting)."""
    out = []
    for s in sc.SMALL_DEBRIS:
        sz = s['size'][2]
        g = geom(s['kind'], s['size'])
        friction = ('<surface><friction><ode><mu>1.2</mu><mu2>1.2</mu2></ode></friction>'
                    '<contact><ode><kp>1e5</kp><kd>10</kd><min_depth>0.001</min_depth></ode></contact></surface>')
        out.append(f'''    <model name="debris_{s['id']}"><pose>{s['x']:.3f} {s['y']:.3f} {sz / 2 + 0.001:.4f} 0 0 {s['yaw']:.4f}</pose>
      <link name="link">{inertia_xml(s['kind'], s['mass'], s['size'])}
        <collision name="collision">{g}{friction}</collision>
        <visual name="visual">{g}{material(s['rgb'])}</visual>
      </link></model>''')
    return '\n'.join(out)


def deformation_xml():
    out = []
    for d in sc.DEFECTS:
        if d['type'] != 'deformation':
            continue
        c = sc.deformation_cylinder(d)
        g = f'<geometry><cylinder><radius>{c["r"]:.4f}</radius><length>1.0</length></cylinder></geometry>'
        out.append(f'''    <model name="defect_{d['id']}"><static>true</static><pose>{c['cx']:.4f} {c['cy']:.4f} 0.5 0 0 0</pose>
      <link name="link"><collision name="c">{g}</collision>
        <visual name="v">{g}{material((0.50, 0.48, 0.43))}</visual></link></model>''')
    return '\n'.join(out)


def crack_xml():
    """Cracks: thin dark VISUAL boxes on the upper wall (z > 0.30, well above the LiDAR plane). Camera only."""
    vis = []
    for d in sc.DEFECTS:
        if d['type'] != 'crack':
            continue
        (_, _), (nx, ny) = sc.wall_face(d)
        pts = sc.crack_polyline(d)
        for k in range(len(pts) - 1):
            (x0, y0, z0), (x1, y1, z1) = pts[k], pts[k + 1]
            cx, cy, cz = (x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2
            seg = math.dist((x0, y0, z0), (x1, y1, z1)) + d['line_w']
            if nx == 0:     # face normal along y: segment lies in the x-z plane
                size, rpy = (seg, 0.002, d['line_w']), (0.0, -math.atan2(z1 - z0, x1 - x0), 0.0)
            else:           # face normal along x: segment lies in the y-z plane
                size, rpy = (0.002, seg, d['line_w']), (math.atan2(z1 - z0, y1 - y0), 0.0, 0.0)
            vis.append(f'<visual name="{d["id"]}_{k}">{gn.pose_xml(cx, cy, cz, *rpy)}'
                       f'<geometry><box><size>{size[0]:.4f} {size[1]:.4f} {size[2]:.4f}</size></box></geometry>'
                       f'{material((0.06, 0.05, 0.05))}</visual>')
    return ('    <model name="defect_cracks"><static>true</static><link name="link">\n        '
            + '\n        '.join(vis) + '\n    </link></model>')


def disposal_xml():
    d = sc.DISPOSAL
    parts = []
    for k, rim in enumerate(sc.disposal_rims()):
        g = f'<geometry><box><size>{rim["size"][0]:.3f} {rim["size"][1]:.3f} {rim["size"][2]:.3f}</size></box></geometry>'
        p = gn.pose_xml(*rim['c'])
        parts.append(f'<collision name="rim{k}">{p}{g}</collision><visual name="vrim{k}">{p}{g}{material((0.15, 0.6, 0.25))}</visual>')
    cx, cy = (d['x_min'] + d['x_max']) / 2, (d['y_min'] + d['y_max']) / 2
    sx, sy = d['x_max'] - d['x_min'], d['y_max'] - d['y_min']
    parts.append(f'<visual name="pad">{gn.pose_xml(cx, cy, 0.003)}<geometry><box><size>{sx:.3f} {sy:.3f} 0.004</size>'
                 f'</box></geometry>{material((0.1, 0.45, 0.2))}</visual>')
    return ('    <model name="disposal_station"><static>true</static><link name="link">\n        '
            + '\n        '.join(parts) + '\n    </link></model>')


def station_config():
    """A-PRIORI mission configuration (like the network graph, not ground truth): where the disposal tray is, where
    the robot stops to drop into it, and the zone perception ignores (the tray is infrastructure, not debris)."""
    d = sc.DISPOSAL
    m = 0.06
    return dict(comment='Disposal station: a-priori configuration (Account 1 navigates to approach; Account 2 drops/verifies).',
                tray=dict(x_min=d['x_min'], x_max=d['x_max'], y_min=d['y_min'], y_max=d['y_max'], rim_h=d['rim_h']),
                approach=dict(x=d['approach']['x'], y=d['approach']['y'], yaw=round(d['approach']['yaw'], 4)),
                ignore_zone=dict(x_min=round(d['x_min'] - d['rim_t'] - m, 3), x_max=round(d['x_max'] + d['rim_t'] + m, 3),
                                 y_min=round(d['y_min'] - d['rim_t'] - m, 3), y_max=round(d['y_max'] + d['rim_t'] + m, 3)))


def world_sdf():
    x0, y0, free, wall = sc.build()
    base = gn.world_xml(sc.NET, x0, y0, free, wall, True)
    extra = '\n'.join(['    <!-- ===== Account 2 inspection scene (GENERATED by gen_inspection_world.py) ===== -->',
                       small_debris_xml(), deformation_xml(), crack_xml(), disposal_xml()])
    base = base.replace('<!-- GENERATED by scripts/gen_network.py - do not edit by hand. -->',
                        '<!-- GENERATED by scripts/gen_inspection_world.py (on top of gen_network.py) - do not edit. -->')
    i = base.rindex('  </world>')
    return base[:i] + extra + '\n' + base[i:]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True, help='sewer_gazebo package directory')
    a = ap.parse_args()
    os.makedirs(os.path.join(a.out, 'worlds'), exist_ok=True)
    os.makedirs(os.path.join(a.out, 'config'), exist_ok=True)
    with open(os.path.join(a.out, 'worlds', 'sewer_inspection.sdf'), 'w') as f:
        f.write(world_sdf())
    with open(os.path.join(a.out, 'config', 'inspection_truth.yaml'), 'w') as f:
        yaml.safe_dump(sc.truth_dict(), f, sort_keys=False)
    with open(os.path.join(a.out, 'config', 'inspection_station.yaml'), 'w') as f:
        yaml.safe_dump(station_config(), f, sort_keys=False)
    print(f'sewer_inspection: {len(sc.SMALL_DEBRIS)} small debris, {len(sc.DEFECTS)} defects, disposal station at A')


if __name__ == '__main__':
    main()
