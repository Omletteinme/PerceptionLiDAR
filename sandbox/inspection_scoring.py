#!/usr/bin/env python3
"""Score Account 2 perception output against the inspection world's ground truth (EVALUATION ONLY).

Works on the JSON messages themselves, so the same scorer is used for
  - the offline pipeline (run_inspection_eval.py), and
  - a live Gazebo run: save the last messages with
        ros2 topic echo --once --full-length /perception/debris  std_msgs/msg/String > debris.yaml
        ros2 topic echo --once --full-length /perception/defects std_msgs/msg/String > defects.yaml
    then:  python3 inspection_scoring.py debris.yaml defects.yaml [--out result.json]
Truth: sewer_gazebo/config/inspection_truth.yaml (written by gen_inspection_world.py). It is read HERE only.

Matching: a detection matches the nearest unmatched truth item within the gate (debris 0.25 m from the centre, or
inside the footprint + 0.3 m for the large piles; defects 0.30 m along the wall, same type family). Unmatched
detections are false positives; unmatched truth items are misses.
"""
import argparse
import json
import math
import os
import sys

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
TRUTH = os.path.join(HERE, '..', 'src', 'sewer_gazebo', 'config', 'inspection_truth.yaml')
sys.path.insert(0, os.path.join(HERE, '..', 'src', 'sewer_perception'))
from sewer_perception.defect_detector import severity  # noqa: E402

FAMILY = {'hole': 'recess', 'damaged_joint': 'recess', 'deformation': 'bulge', 'crack': 'crack'}


def load_truth(path=TRUTH):
    with open(path) as f:
        return yaml.safe_load(f)


def truth_objects(truth):
    out = []
    for s in truth['small_debris']:
        sx, sy, sz = s['size']
        L, W = max(sx, sy), min(sx, sy)
        out.append(dict(id=s['id'], x=s['x'], y=s['y'], length=L, width=W, height=sz, small=True,
                        graspable=s['graspable_designed'], volume=sx * sy * sz * (math.pi / 4 if s['kind'] == 'cyl' else 1)))
    for b in truth['large_debris']:
        pts = []
        for p in b['pieces']:
            if p[0] == 'box':
                _, cx, cy, sx, sy, _ = p
                pts += [(cx - sx / 2, cy - sy / 2), (cx + sx / 2, cy + sy / 2)]
            else:
                _, cx, cy, r, _ = p
                pts += [(cx - r, cy - r), (cx + r, cy + r)]
        pts = np.array(pts)
        lo, hi = pts.min(0), pts.max(0)
        out.append(dict(id=b['id'], x=float((lo[0] + hi[0]) / 2), y=float((lo[1] + hi[1]) / 2), small=False,
                        graspable=False, box=(float(lo[0]), float(hi[0]), float(lo[1]), float(hi[1]))))
    return out


def _match(dets, truths, dist_fn):
    pairs = sorted((dist_fn(d, t), i, j) for i, d in enumerate(dets) for j, t in enumerate(truths))
    used_d, used_t, m = set(), set(), {}
    for dist, i, j in pairs:
        if math.isinf(dist) or i in used_d or j in used_t:
            continue
        used_d.add(i)
        used_t.add(j)
        m[i] = (j, dist)
    return m


def score_debris(msg, truth, observed=None):
    """observed: optional set of truth ids that were actually in the camera's view (recall over seen objects)."""
    tr = truth_objects(truth)
    dets = [o for o in msg['objects'] if o.get('state', 'confirmed') in ('confirmed', 'missing')]

    def dist(d, t):
        dx, dy = d['x'] - t['x'], d['y'] - t['y']
        if t['small']:
            r = math.hypot(dx, dy)
            return r if r <= 0.25 else math.inf
        x0, x1, y0, y1 = t['box']
        out = math.hypot(max(x0 - d['x'], 0, d['x'] - x1), max(y0 - d['y'], 0, d['y'] - y1))
        return out if out <= 0.3 else math.inf
    m = _match(dets, tr, dist)
    rows, feas_ok, feas_n = [], 0, 0
    for i, (j, dd) in m.items():
        d, t = dets[i], tr[j]
        row = dict(det=d['id'], truth=t['id'], pos_err=round(dd, 3), graspable=d['graspable'],
                   graspable_truth=t['graspable'], reasons=d.get('feasibility', {}).get('reasons', []))
        if t['small']:
            L, W, H = d['size']
            row.update(dim_err=dict(length=round(L - t['length'], 3), width=round(W - t['width'], 3),
                                    height=round(H - t['height'], 3)))
        feas_n += 1
        feas_ok += int(row['graspable'] == row['graspable_truth'])
        rows.append(row)
    fps = [dets[i]['id'] for i in range(len(dets)) if i not in m]
    misses = [tr[j]['id'] for j in range(len(tr)) if j not in {v[0] for v in m.values()}]
    small = [r for r in rows if 'dim_err' in r]
    out = dict(n_detections=len(dets), n_truth=len(tr), matched=len(rows),
               precision=round(len(rows) / max(1, len(dets)), 3), recall=round(len(rows) / max(1, len(tr)), 3),
               false_positives=fps, misses=misses, matches=rows,
               feasibility_accuracy=round(feas_ok / max(1, feas_n), 3),
               mean_abs_dim_err=None if not small else {k: round(float(np.mean([abs(r['dim_err'][k]) for r in small])), 3)
                                                        for k in ('length', 'width', 'height')},
               mean_pos_err_small=None if not small else round(float(np.mean([r['pos_err'] for r in small])), 3))
    if observed is not None:
        seen = [t['id'] for t in tr if t['id'] in observed]
        out['recall_observed'] = round(sum(1 for r in rows if r['truth'] in observed) / max(1, len(seen)), 3)
        out['observed'] = seen
    # largest-feasible check (the selection itself is Account 1's; this checks the data it will rank on)
    by_truth = {r['truth']: r for r in rows}
    feasible_truth = [t for t in tr if t['small'] and t['graspable']]
    if feasible_truth:
        exp = max(feasible_truth, key=lambda t: t['volume'])['id']
        feas_det = [o for o in dets if o['graspable']]
        got = None
        if feas_det:
            big = max(feas_det, key=lambda o: o['size'][0] * o['size'][1] * o['size'][2])
            got = next((r['truth'] for r in rows if r['det'] == big['id']), big['id'])
        out['largest_feasible'] = dict(expected=exp, from_detections=got, correct=got == exp,
                                       expected_was_detected=exp in by_truth)
    return out


def score_defects(msg, truth):
    tr = truth['defects']
    dets = msg['defects']

    def dist(d, t):
        if FAMILY[d['type']] != FAMILY[t['type']]:
            return math.inf
        r = math.hypot(d['x'] - t['x'], d['y'] - t['y'])
        return r if r <= 0.30 else math.inf
    m = _match(dets, tr, dist)
    per = {}
    for typ in ('crack', 'hole', 'damaged_joint', 'deformation'):
        nd = sum(1 for d in dets if d['type'] == typ)
        nt = sum(1 for t in tr if t['type'] == typ)
        tp = sum(1 for i, (j, _) in m.items() if dets[i]['type'] == typ and tr[j]['type'] == typ)
        per[typ] = dict(detections=nd, truth=nt, correct_type=tp,
                        precision=round(tp / nd, 3) if nd else None, recall=round(tp / nt, 3) if nt else None)
    rows = []
    for i, (j, dd) in m.items():
        d, t = dets[i], tr[j]
        tsize = dict(length=t.get('length', t['width']), width=t['width'], depth=t['depth'])
        tsev = severity(t['type'], tsize)[0]
        rows.append(dict(det=d['id'], truth=t['id'], type=d['type'], type_truth=t['type'], loc_err=round(dd, 3),
                         severity=d['severity_label'], severity_truth=tsev, size=d['size'],
                         has_evidence_image=bool(d['evidence'])))
    fps = [dict(id=dets[i]['id'], type=dets[i]['type'], x=dets[i]['x'], y=dets[i]['y'])
           for i in range(len(dets)) if i not in m]
    misses = [tr[j]['id'] for j in range(len(tr)) if j not in {v[0] for v in m.values()}]
    return dict(n_detections=len(dets), n_truth=len(tr), matched=len(rows),
                precision=round(len(rows) / max(1, len(dets)), 3), recall=round(len(rows) / max(1, len(tr)), 3),
                type_accuracy=round(sum(r['type'] == r['type_truth'] for r in rows) / max(1, len(rows)), 3),
                severity_accuracy=round(sum(r['severity'] == r['severity_truth'] for r in rows) / max(1, len(rows)), 3),
                mean_loc_err=round(float(np.mean([r['loc_err'] for r in rows])), 3) if rows else None,
                per_class=per, matches=rows, false_positives=fps, misses=misses)


def _load_msg(path):
    """A JSON file, or the YAML that `ros2 topic echo --once` writes for a std_msgs/String (data: '...')."""
    with open(path) as f:
        txt = f.read()
    try:
        return json.loads(txt)
    except ValueError:
        y = yaml.safe_load(txt.split('\n---')[0])
        return json.loads(y['data'])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('debris')
    ap.add_argument('defects')
    ap.add_argument('--truth', default=TRUTH)
    ap.add_argument('--out')
    a = ap.parse_args()
    truth = load_truth(a.truth)
    res = dict(debris=score_debris(_load_msg(a.debris), truth), defects=score_defects(_load_msg(a.defects), truth))
    txt = json.dumps(res, indent=1)
    print(txt)
    if a.out:
        with open(a.out, 'w') as f:
            f.write(txt)


if __name__ == '__main__':
    main()
