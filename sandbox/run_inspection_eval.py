#!/usr/bin/env python3
"""Offline evaluation of the Account 2 perception pipeline on the synthetic inspection dataset (no ROS, no Gazebo).

The robot drives a coverage route through the inspection world (sewer_network_v2 + small debris + defects). Along
the way the sandbox renders depth/RGB frames (render3d.py) and LiDAR scans of the true world, and the SAME detector
classes the ROS node uses process them with the pose ESTIMATE:
  clean   estimate = true pose
  drift   estimate = true pose + slowly varying error (about 3 cm, 1 deg; like a working particle filter)
  rough   estimate = true pose + larger error (about 6 cm, 2.5 deg) with jitter
Outputs go to sandbox/results/: inspection_eval_<scenario>.json (scored by inspection_scoring.py) and a PNG map.

  python3 run_inspection_eval.py [--scenario clean|drift|rough|all] [--seed 0]
  python3 run_inspection_eval.py --dataset ~/sewer_ws/logs/dataset      # frames recorded in Gazebo
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np

import render3d
import sim2d
from inspection_scoring import load_truth, score_debris, score_defects

sys.path.insert(0, os.path.join(sim2d.GAZEBO_PKG, 'scripts'))
import inspection_scene as sc  # noqa: E402
from sewer_localization.map_utils import OccupancyMap  # noqa: E402
from sewer_perception import interfaces as itf  # noqa: E402
from sewer_perception.camera_model import CameraModel  # noqa: E402
from sewer_perception.inspection_core import InspectionCore  # noqa: E402
import yaml  # noqa: E402

RESULTS = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'results')
# Coverage route through every reachable corridor of v2 (B1 and B3 block J1-J4 and M-J3; B2 is passed on the east side)
ROUTE = [(4, 0), (8, 0), (8, -4.5), (8, 0), (8.0, 2.3), (8.22, 2.7), (8.22, 3.9), (8, 4.3), (8, 6), (12, 6),
         (12, 10), (16, 10), (16, 6), (16, 0)]
ERR = {'clean': (0.0, 0.0, 0.0), 'drift': (0.03, math.radians(1.0), 0.005), 'rough': (0.06, math.radians(2.5), 0.015)}


def pose_errors(n, scenario, rng):
    """[n, 3] estimate errors: a slowly varying (low-pass filtered) error with the scenario's size plus jitter."""
    sxy, syaw, jit = ERR[scenario]
    if sxy == 0:
        return np.zeros((n, 3))
    white = rng.normal(0, 1, (n, 3))
    a = 0.98
    lp = np.zeros((n, 3))
    for k in range(1, n):
        lp[k] = a * lp[k - 1] + math.sqrt(1 - a * a) * white[k]
    e = lp * np.array([sxy, sxy, syaw])
    return e + rng.normal(0, 1, (n, 3)) * np.array([jit, jit, jit * 0.3])


def load_world():
    G = sim2d.GAZEBO_PKG
    prior = OccupancyMap.load(os.path.join(G, 'maps', 'sewer_map_v2'))
    with open(os.path.join(G, 'config', 'network_graph_v2.yaml')) as f:
        g = yaml.safe_load(f)
    nodes = {k: (v['x'], v['y']) for k, v in g['nodes'].items()}
    edges = [(e['from'], e['to']) for e in g['edges']]
    with open(os.path.join(G, 'config', 'inspection_station.yaml')) as f:
        station = yaml.safe_load(f)
    return prior, nodes, edges, station


def finish(core, t, observed, extra):
    """Final messages from the core, checked against the documented schema and scored against the truth."""
    dmsg, fmsg = core.messages(t)
    itf.validate_debris(dmsg)
    itf.validate_defects(fmsg)
    truth = load_truth()
    res = dict(extra, debris=score_debris(dmsg, truth, observed), defects=score_defects(fmsg, truth))
    return res, dmsg, fmsg


def run(scenario, seed=0, depth_every=5, rgb_every=2, scan_every=2, verbose=True):
    """Offline synthetic run. The perception pipeline is InspectionCore, the same class the ROS node uses."""
    rng = np.random.default_rng(seed)
    prior, nodes, edges, station = load_world()
    true_map = sc.true_lidar_map(prior)
    scene = render3d.Scene()
    dcam, rcam = CameraModel(320, 180), CameraModel(640, 360)
    core = InspectionCore(prior, nodes, edges, station,
                          save_evidence=lambda rgb, t, boxes, tag, circles: f'{tag}_frame_{t:.1f}')
    poses = sim2d.trajectory(tuple(ROUTE))[0]
    err = pose_errors(len(poses), scenario, rng)
    observed = set()
    model_of = {k: p['model'] for k, p in enumerate(scene.solids)}
    t0 = time.time()
    n_depth = 0
    for k, true_pose in enumerate(poses):
        t = k * sim2d.DT
        est = tuple(true_pose + err[k])
        if k % scan_every == 0:
            core.on_scan(t, est, render3d.lidar_scan(true_map, true_pose, rng), sim2d.ANGLES)
        if k % depth_every == 0:
            depth, _, lab = scene.render(dcam, true_pose)
            depth = depth + rng.normal(0, 0.003, depth.shape)                 # small depth noise
            vis, cnt = np.unique(lab[lab >= 0], return_counts=True)
            for v, c in zip(vis, cnt):                                         # truth objects seen (eval only)
                if c >= 30 and model_of[v].startswith('debris_'):
                    observed.add(model_of[v][len('debris_'):])
            core.on_depth(t, est, depth)
            if n_depth % rgb_every == 0:
                _, rgb, _ = scene.render(rcam, true_pose, noise=0.02, rng=rng)
                core.on_rgb(t, est, (rgb * 255).astype(np.uint8))
            n_depth += 1
        if k % 50 == 0:
            core.messages(t)                    # as the node's publish timer does (LiDAR clusters, wall defects)
            if verbose and k % 200 == 0:
                print(f'  [{scenario}] step {k}/{len(poses)}  tracks {len(core.tracker.tracks)}  '
                      f'{time.time() - t0:.0f}s', flush=True)
    t = len(poses) * sim2d.DT
    extra = dict(scenario=scenario, seed=seed, frames=n_depth, scans=len(poses) // scan_every,
                 runtime_s=round(time.time() - t0, 1),
                 pose_error_rms=dict(xy=round(float(np.sqrt((err[:, :2] ** 2).sum(1).mean())), 3),
                                     yaw_deg=round(float(np.degrees(np.sqrt((err[:, 2] ** 2).mean()))), 2)))
    res, dmsg, fmsg = finish(core, t, observed, extra)
    return res, dmsg, fmsg, poses


def run_dataset(path, verbose=True):
    """Evaluate on a dataset RECORDED IN GAZEBO by dataset_recorder_node (rgb/, depth/, scan/, index.jsonl).
    Perception gets the recorded ESTIMATED pose; the recorded true pose is used only to decide which truth objects
    were in view (labels from label_dataset.py)."""
    import json
    from PIL import Image
    from label_dataset import label_frames
    prior, nodes, edges, station = load_world()
    core = InspectionCore(prior, nodes, edges, station,
                          save_evidence=lambda rgb, t, boxes, tag, circles: f'{tag}_frame_{t:.1f}')
    with open(os.path.join(path, 'index.jsonl')) as f:
        rows = [json.loads(line) for line in f if line.strip()]
    labels = label_frames(path, rows)
    observed = {o['id'] for lab in labels for o in lab['debris']}
    t0 = time.time()
    for k, row in enumerate(rows):
        est, t = tuple(row['est_pose']), row['stamp']
        scan = os.path.join(path, 'scan', row['frame'] + '.npz')
        if os.path.exists(scan):
            z = np.load(scan)
            core.on_scan(t, est, z['ranges'], z['angles'], z['reliability'] if z['reliability'].size else None)
        core.on_depth(t, est, np.load(os.path.join(path, 'depth', row['frame'] + '.npy')))
        core.on_rgb(t, est, np.asarray(Image.open(os.path.join(path, 'rgb', row['frame'] + '.png')).convert('RGB')))
        if k % 10 == 0:
            core.messages(t)
    t = rows[-1]['stamp'] if rows else 0.0
    res, dmsg, fmsg = finish(core, t, observed, dict(dataset=path, frames=len(rows),
                                                      runtime_s=round(time.time() - t0, 1)))
    return res, dmsg, fmsg, np.array([r['eval_only_true_pose'] or r['est_pose'] for r in rows])


def plot(path, dmsg, fmsg, poses, truth):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    G = sim2d.GAZEBO_PKG
    prior = OccupancyMap.load(os.path.join(G, 'maps', 'sewer_map_v2'))
    fig, ax = plt.subplots(figsize=(13, 10))
    ax.imshow(~prior.solid, cmap='gray', origin='lower', vmin=-1, vmax=1.3,
              extent=[prior.x0, prior.x0 + prior.w * prior.res, prior.y0, prior.y0 + prior.h * prior.res])
    ax.plot(poses[:, 0], poses[:, 1], '-', color='0.6', lw=0.8, label='route')
    for s in truth['small_debris']:
        ax.plot(s['x'], s['y'], 'o', mfc='none', mec='k', ms=11)
        ax.annotate(s['id'], (s['x'], s['y']), xytext=(6, -10), textcoords='offset points', fontsize=8)
    for d in truth['defects']:
        ax.plot(d['x'], d['y'], 'D', mfc='none', mec='m', ms=10)
        ax.annotate(d['id'], (d['x'], d['y']), xytext=(6, 4), textcoords='offset points', fontsize=8, color='m')
    for o in dmsg['objects']:
        c = 'g' if o['graspable'] else 'r'
        ax.plot(o['x'], o['y'], 's', color=c, ms=6)
        ax.annotate(o['id'], (o['x'], o['y']), xytext=(-14, 6), textcoords='offset points', fontsize=7, color=c)
    colours = {'crack': 'gold', 'hole': 'tab:blue', 'damaged_joint': 'purple', 'deformation': 'tab:orange'}
    for d in fmsg['defects']:
        ax.plot(d['x'], d['y'], '^', color=colours[d['type']], ms=8)
    ax.set_xlim(-1, 17.5)
    ax.set_ylim(-5.8, 11)
    ax.set_aspect('equal')
    ax.set_title('Inspection eval: truth (black circles = debris, magenta diamonds = defects) vs detections\n'
                 'squares: debris (green graspable, red not); triangles: defects (gold crack, blue hole, purple joint, orange deformation)',
                 fontsize=9)
    fig.savefig(path, dpi=80, bbox_inches='tight')
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--scenario', default='all', choices=['all'] + list(ERR))
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--dataset', default='', help='evaluate a dataset recorded in Gazebo (dataset_recorder_node)')
    a = ap.parse_args()
    os.makedirs(RESULTS, exist_ok=True)
    truth = load_truth()
    jobs = [('gazebo_dataset', None)] if a.dataset else [(s, s) for s in (list(ERR) if a.scenario == 'all' else [a.scenario])]
    for s, scen in jobs:
        res, dmsg, fmsg, poses = run_dataset(a.dataset) if a.dataset else run(scen, a.seed)
        base = os.path.join(RESULTS, f'inspection_eval_{s}')
        with open(base + '.json', 'w') as f:
            json.dump(res, f, indent=1)
        with open(base + '_messages.json', 'w') as f:
            json.dump(dict(debris=dmsg, defects=fmsg), f)
        plot(base + '.png', dmsg, fmsg, poses, truth)
        d, f_ = res['debris'], res['defects']
        print(f"[{s}] debris P {d['precision']} R {d['recall']} (observed {d.get('recall_observed')}) "
              f"feas-acc {d['feasibility_accuracy']} dims {d['mean_abs_dim_err']} FP {d['false_positives']} miss {d['misses']} "
              f"largest-feasible {d.get('largest_feasible')}")
        print(f"[{s}] defects P {f_['precision']} R {f_['recall']} type-acc {f_['type_accuracy']} "
              f"sev-acc {f_['severity_accuracy']} loc {f_['mean_loc_err']} FP {f_['false_positives']} miss {f_['misses']}")


if __name__ == '__main__':
    main()
