#!/usr/bin/env python3
"""Label a dataset recorded in Gazebo (dataset_recorder_node) from the inspection world's truth (EVALUATION ONLY).

For every frame it projects the truth objects and defects (config/inspection_truth.yaml) into the camera with the
recorded TRUE pose ('eval_only_true_pose') and keeps those inside the image, within range and not hidden (the
recorded depth at the object's centre is not much nearer than the object). Output: <dataset>/labels.jsonl, one line
per frame: {frame, debris: [{id, bbox:[u0,v0,u1,v1], distance}], defects: [{id, type, u, v, distance}]}.
These labels are what makes the recorded frames a labelled dataset; perception never reads them.

  python3 label_dataset.py ~/sewer_ws/logs/dataset
"""
import json
import math
import os
import sys

import numpy as np

import sim2d  # noqa: F401,E402  (puts src/* on sys.path)
from inspection_scoring import load_truth
from sewer_perception.camera_model import CameraModel


def _corners(o):
    sx, sy, sz = o['size']
    c, s = math.cos(o['yaw']), math.sin(o['yaw'])
    pts = []
    for u in (-sx / 2, sx / 2):
        for v in (-sy / 2, sy / 2):
            for z in (0.0, sz):
                pts.append((o['x'] + c * u - s * v, o['y'] + s * u + c * v, z))
    return np.array(pts)


def label_frames(path, rows, max_range=3.0, truth=None):
    truth = truth or load_truth()
    rgb_cam = CameraModel(640, 360)
    out = []
    for row in rows:
        tp = row.get('eval_only_true_pose')
        lab = dict(frame=row['frame'], debris=[], defects=[])
        if tp is None:
            out.append(lab)
            continue
        depth_file = os.path.join(path, 'depth', row['frame'] + '.npy')
        depth = np.load(depth_file) if os.path.exists(depth_file) else None
        dcam = CameraModel(depth.shape[1], depth.shape[0]) if depth is not None else None
        for o in truth['small_debris']:
            u, v, d = rgb_cam.project_world(_corners(o), tp)
            if (d <= 0.2).any():
                continue
            cu, cv, cd = rgb_cam.project_world(np.array([[o['x'], o['y'], o['size'][2] / 2]]), tp)
            if not (0 <= cu[0] < 640 and 0 <= cv[0] < 360 and cd[0] < max_range):
                continue
            if depth is not None:
                du, dv, _ = dcam.project_world(np.array([[o['x'], o['y'], o['size'][2] / 2]]), tp)
                obs = depth[int(np.clip(dv[0], 0, dcam.height - 1)), int(np.clip(du[0], 0, dcam.width - 1))]
                if np.isfinite(obs) and obs < cd[0] - 0.15:
                    continue                                    # something nearer hides it
            lab['debris'].append(dict(id=o['id'], distance=round(float(cd[0]), 3),
                                      bbox=[int(max(0, u.min())), int(max(0, v.min())), int(min(639, u.max())),
                                            int(min(359, v.max()))]))
        for dfc in truth['defects']:
            z = dfc['z'] if dfc['type'] == 'crack' else 0.25
            u, v, d = rgb_cam.project_world(np.array([[dfc['x'], dfc['y'], z]]), tp)
            if 0.3 < d[0] < max_range and 0 <= u[0] < 640 and 0 <= v[0] < 360:
                lab['defects'].append(dict(id=dfc['id'], type=dfc['type'], u=int(u[0]), v=int(v[0]),
                                           distance=round(float(d[0]), 3)))
        out.append(lab)
    return out


def main():
    path = os.path.expanduser(sys.argv[1] if len(sys.argv) > 1 else '~/sewer_ws/logs/dataset')
    with open(os.path.join(path, 'index.jsonl')) as f:
        rows = [json.loads(line) for line in f if line.strip()]
    labels = label_frames(path, rows)
    with open(os.path.join(path, 'labels.jsonl'), 'w') as f:
        for lab in labels:
            f.write(json.dumps(lab) + '\n')
    n_d = sum(len(lab['debris']) for lab in labels)
    n_f = sum(len(lab['defects']) for lab in labels)
    seen = sorted({o['id'] for lab in labels for o in lab['debris']} | {o['id'] for lab in labels for o in lab['defects']})
    print(f'{len(labels)} frames labelled: {n_d} debris and {n_f} defect instances; items seen: {seen}')


if __name__ == '__main__':
    main()
