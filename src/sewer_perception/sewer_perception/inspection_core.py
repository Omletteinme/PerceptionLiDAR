"""Inspection pipeline without ROS (Account 2): debris objects + structural defects -> documented JSON messages.

CUSTOM code, pure NumPy. inspection_node.py (ROS), sandbox/run_inspection_eval.py (offline evaluation) and
sandbox/mission_sim_account2.py (offline end-to-end mission with Account 1's Mission Manager) all use this class, so
they run the same logic. Inputs are sensor data with the ESTIMATED pose; ground truth never enters.

  on_scan(t, pose, ranges, angles, r)   2D LiDAR (+ per-beam reliability): wall defects, LiDAR object clusters
  on_depth(t, pose, depth)              depth camera: frame-to-wall registration, debris clusters -> tracker
  on_rgb(t, pose, rgb)                  RGB camera: cracks, evidence frames
  on_manipulation(msg) / on_disposal(msg)   arm status: exclude the arm from perception, mark held / removed objects
  messages(stamp)                       -> (/perception/debris dict, /perception/defects dict)
"""
import math
from collections import deque

import numpy as np

from . import interfaces as itf
from .blockage import EvidenceGrid
from .camera_model import CameraModel, map_to_base
from .debris_detector import DebrisParams, DepthDebrisDetector, lidar_clusters
from .defect_detector import CrackDetector, DefectTracker, WallDefectDetector
from .feasibility import GripperLimits, assess, corridor_of, obstruction
from .object_tracker import DebrisTracker, make_visible_fn

# base_link box around the lowered arm and gripper (in front of the camera; reach 0.65 m): ignored by the depth detector
ARM_EXCLUSION = (0.10, 0.80, -0.60, 0.60, -0.10, 0.70)
ARM_BUSY = ('approaching', 'grasping', 'lifting', 'releasing')
PUBLISHED_STATES = ('confirmed', 'missing')        # held / removed objects are the Mission Manager's records now


class InspectionCore:
    def __init__(self, prior, nodes, edges, station=None, save_evidence=None, evidence_frames=40,
                 limits=None, crack_every=1):
        """save_evidence(rgb, t, boxes, tag, circles) -> image reference string (or None): how evidence frames are
        stored (PNG files in the ROS node, a frame name offline)."""
        self.prior, self.nodes, self.edges = prior, nodes, edges
        zones = []
        if station:
            z = station['ignore_zone']
            zones.append((z['x_min'], z['x_max'], z['y_min'], z['y_max']))
        self.dcam = CameraModel(320, 180)
        self.debris = DepthDebrisDetector(prior, self.dcam, DebrisParams(ignore_zones=zones))
        self.tracker = DebrisTracker()
        self.grid = EvidenceGrid(prior)              # existing class (unchanged), own instance: LiDAR object clusters
        self.walls = WallDefectDetector(prior)
        self.cracks = None
        self.defects = DefectTracker()
        self.limits = limits or GripperLimits()
        self.save_evidence = save_evidence or (lambda *a, **k: None)
        self.frames = deque(maxlen=evidence_frames)  # recent (t, pose, rgb) for LiDAR-defect evidence
        self.last_depth = None                        # (t, depth, registration correction)
        self.manip = {}
        self.crack_every, self.n_rgb = crack_every, 0

    # ------------------------------------------------------------------ arm status
    def arm_state(self):
        return self.manip.get('detail_state', str(self.manip.get('state', '')).lower())

    def exclusion(self):
        return ARM_EXCLUSION if self.arm_state() in ARM_BUSY else None

    def on_manipulation(self, s):
        self.manip = s
        oid = s.get('object_id')
        st = self.arm_state()
        if oid and st in ('lifting', 'holding') and s.get('verification', {}).get('grasp_verified'):
            self.tracker.set_state(oid, 'held')
        elif oid and st == 'failed':
            t = self.tracker.get(oid)
            if t is not None and t.state == 'held':
                t.state = 'missing'                   # dropped somewhere: let perception find it again

    def on_disposal(self, s):
        if s.get('verified') and s.get('object_id'):
            self.tracker.set_state(s['object_id'], 'removed')

    # ------------------------------------------------------------------ sensors
    def on_scan(self, t, pose, ranges, angles, r=None):
        if self.manip.get('arm_in_scan_plane'):
            return                    # the arm is in the scan: no wall/obstacle evidence until it is folded again
        self.grid.update(pose, ranges, angles, r)
        self.walls.update(pose, ranges, angles, r)

    def on_depth(self, t, pose, depth):
        depth = np.asarray(depth, float)
        if depth.shape != (self.dcam.height, self.dcam.width):
            self.dcam = CameraModel(depth.shape[1], depth.shape[0])
            self.debris.cam = self.dcam
        pose = self.debris.register(depth, pose)    # align this frame to the a-priori walls
        self.last_depth = (t, depth, self.debris.last_correction)
        clusters = self.debris.detect(depth, pose, self.exclusion(), registered=True)
        self.tracker.update(clusters, t, make_visible_fn(self.dcam, pose, depth))
        # for the manipulator: where the object is relative to the robot right now. The fused footprint centre (all
        # views) in the frame of this wall-registered pose, not the centroid of this one partial view, which only
        # sees the near faces and is biased towards the camera by up to half the object (found by the end-to-end test)
        for tr in self.tracker.tracks:
            if tr.rel_base is not None and tr.rel_base[2] == t and tr.points is not None:
                g = tr.geometry()
                bx, by = map_to_base([(g['x'], g['y'])], pose)[0]
                tr.rel_base = (float(bx), float(by), t)
        return pose

    def on_rgb(self, t, pose, rgb):
        self.n_rgb += 1
        if self.n_rgb % self.crack_every or self.last_depth is None:
            return
        if abs(t - self.last_depth[0]) > 0.15 or self.arm_state() in ARM_BUSY:
            return
        dx, dy, dth = self.last_depth[2]             # same registration correction as the depth frame
        pose = (pose[0] + dx, pose[1] + dy, pose[2] + dth)
        if self.cracks is None or self.cracks.cam.width != rgb.shape[1]:
            self.cracks = CrackDetector(self.prior, CameraModel(rgb.shape[1], rgb.shape[0]))
        dets = self.cracks.detect(rgb, self.last_depth[1], pose)
        ev = None
        if dets:
            ref = self.save_evidence(rgb, t, [d['bbox'] for d in dets], 'crack', ())
            ev = dict(source='camera', image=ref, frame_stamp=t)
        for d in dets:
            d['edge'] = self.edge_of(d['x'], d['y'])
        self.defects.update(dets, t, ev)
        self.frames.append((t, pose, rgb))
        self.lidar_defect_evidence()

    # ------------------------------------------------------------------ helpers
    def edge_of(self, x, y):
        c = corridor_of(x, y, self.nodes, self.edges, half_w=0.75)
        return None if c is None else c[0]

    def lidar_defect_evidence(self):
        """Attach a camera frame to LiDAR-detected defects that have none yet. The LiDAR confirms a wall defect when
        the robot is beside it, when the camera has usually already passed it, so recent frames are searched too."""
        if self.cracks is None:
            return
        cam = self.cracks.cam
        todo = [it for it in self.defects.items
                if it.get('source') == 'lidar' and not (it.get('evidence') or {}).get('image')]
        for t, pose, rgb in reversed(self.frames):
            if not todo:
                break
            marks, hit = [], []
            for it in todo:
                u, v, dd = cam.project_world(np.array([[it['x'], it['y'], 0.25]]), pose)
                if 0.5 < dd[0] < 2.5 and 20 <= u[0] < cam.width - 20 and 20 <= v[0] < cam.height - 20:
                    marks.append((float(u[0]), float(v[0]), f"{it['id']} {it['type']}"))
                    hit.append(it)
            if hit:
                ref = self.save_evidence(rgb, t, [], 'lidar_defect', marks)
                for it in hit:
                    it['evidence'] = dict(source='lidar', image=ref, frame_stamp=t, bbox=None)
                todo = [it for it in todo if it not in hit]

    def same_pile(self, a, b, bridge=0.6):
        """Two tall tracks are faces of ONE pile if they are close and nothing between them has ever been observed
        (no LiDAR ray crossed it: it is inside the pile). The camera sees a pile's back face from the far side as a
        separate cluster otherwise."""
        ga, gb = a.geometry(), b.geometry()
        if ga['height'] < 0.13 or gb['height'] < 0.13:
            return False
        pa = (np.array(list(a.cells), float) + 0.5) * 0.02
        pb = (np.array(list(b.cells), float) + 0.5) * 0.02
        if len(pa) > 400:
            pa = pa[:: len(pa) // 400 + 1]
        if len(pb) > 400:
            pb = pb[:: len(pb) // 400 + 1]
        d = np.hypot(pa[:, None, 0] - pb[None, :, 0], pa[:, None, 1] - pb[None, :, 1])
        i, j = np.unravel_index(np.argmin(d), d.shape)
        if d[i, j] > bridge:
            return False
        m, grid = self.prior, self.grid
        n = max(2, int(d[i, j] / (m.res / 2)))
        xs = np.linspace(pa[i, 0], pb[j, 0], n)
        ys = np.linspace(pa[i, 1], pb[j, 1], n)
        ci, cj, inside = m.cells(xs, ys)
        if not inside.all():
            return False
        seen_free = grid.seen[cj, ci] & (grid.L[cj, ci] < grid.p.occ_thresh) & ~m.solid[cj, ci]
        return not seen_free[1:-1].any()

    # ------------------------------------------------------------------ outputs
    def messages(self, stamp):
        wd = self.walls.detect()
        for d in wd:
            d['edge'] = self.edge_of(d['x'], d['y'])
        self.defects.update(wd, stamp)
        self.lidar_defect_evidence()
        if not self.manip.get('arm_in_scan_plane'):
            self.tracker.update(lidar_clusters(self.grid), stamp, source='lidar')
        self.tracker.merge_if(self.same_pile)
        objs = []
        for tr in self.tracker.tracks:
            if tr.state not in PUBLISHED_STATES:
                continue
            g = tr.geometry()
            obj = dict(g, confidence=tr.confidence(), n_frames=tr.hits, min_range=tr.min_range,
                       view_spread=tr.view_spread() if tr.view_az else math.pi)
            feas = assess(obj, self.prior, self.limits)
            if g['height_is_lower_bound'] and feas['graspable']:
                feas = dict(feas, graspable=False, reasons=['height_unknown_lidar_only'])
            if tr.state != 'confirmed' and feas['graspable']:
                feas = dict(feas, graspable=False, reasons=[f'state_{tr.state}'])
            objs.append(itf.debris_object(tr, feas, obstruction(obj, self.nodes, self.edges), self.limits.max_width))
        return (itf.debris_msg(objs, stamp),
                itf.defects_msg([itf.defect_item(x) for x in self.defects.items], stamp))
