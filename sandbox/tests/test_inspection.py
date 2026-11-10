"""Account 2: inspection scene, camera model, debris detection/tracking, feasibility, defects, message contracts.

No ROS, no Gazebo: frames come from the offline renderer (render3d.py), scans from the true LiDAR world.
"""
import math
import os
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

import render3d
import sim2d
from sewer_localization.map_utils import OccupancyMap
from sewer_perception import interfaces as itf
from sewer_perception.blockage import EvidenceGrid
from sewer_perception.camera_model import CameraModel, base_to_map, map_to_base
from sewer_perception.debris_detector import DebrisParams, DepthDebrisDetector, footprint, lidar_clusters
from sewer_perception.defect_detector import CrackDetector, DefectTracker, WallDefectDetector, severity
from sewer_perception.feasibility import assess, obstruction
from sewer_perception.object_tracker import DebrisTracker, make_visible_fn

sc = render3d.sc
G = sim2d.GAZEBO_PKG


@pytest.fixture(scope='module')
def prior():
    return OccupancyMap.load(os.path.join(G, 'maps', 'sewer_map_v2'))


@pytest.fixture(scope='module')
def scene():
    return render3d.Scene()


@pytest.fixture(scope='module')
def graph():
    g = yaml.safe_load(open(os.path.join(G, 'config', 'network_graph_v2.yaml')))
    return {k: (v['x'], v['y']) for k, v in g['nodes'].items()}, [(e['from'], e['to']) for e in g['edges']]


# ------------------------------------------------------------------------------------------ scene
def test_generated_world_and_configs_are_up_to_date(tmp_path):
    import gen_inspection_world as gw
    out = tmp_path / 'gz'
    os.makedirs(out / 'worlds')
    os.makedirs(out / 'config')
    import sys
    argv = sys.argv
    sys.argv = ['x', '--out', str(out)]
    try:
        gw.main()
    finally:
        sys.argv = argv
    for rel in ('worlds/sewer_inspection.sdf', 'config/inspection_truth.yaml', 'config/inspection_station.yaml'):
        assert open(out / rel).read() == open(os.path.join(G, rel)).read(), f'{rel} is stale: rerun gen_inspection_world.py'
    w = ET.parse(os.path.join(G, 'worlds', 'sewer_inspection.sdf')).getroot().find('world')
    assert w.get('name') == 'sewer_network'                       # contact bridge relies on this name
    models = {m.get('name') for m in w.findall('model')}
    assert {f"debris_{s['id']}" for s in sc.SMALL_DEBRIS} <= models and 'disposal_station' in models
    for m in w.findall('model'):
        if m.get('name').startswith('debris_S'):
            assert m.find('static') is None                         # movable: the gripper must really lift them


def test_scene_rules(prior):
    for s in sc.SMALL_DEBRIS:
        assert s['size'][2] < 0.13                                  # below the LiDAR plane: camera-only objects
        assert abs(s['y'] if abs(s['y']) < 1 else 0) >= 0 and prior.dist(np.array([s['x']]), np.array([s['y']]))[0] > 0.1
    for d in sc.DEFECTS:
        if d['type'] == 'crack':
            assert d['z'] - d['width'] / 2 * abs(math.sin(d['angle'])) > 0.30   # above the wet band and LiDAR
    true = sc.true_lidar_map(prior)
    h1 = next(d for d in sc.DEFECTS if d['id'] == 'H1')
    (fx, fy), (nx, ny) = sc.wall_face(h1)
    assert not true.is_solid(np.array([fx]), np.array([fy - ny * 0.05]))[0]      # recess cut into the wall
    assert prior.is_solid(np.array([fx]), np.array([fy - ny * 0.05]))[0]         # but the prior map is unchanged
    df = next(d for d in sc.DEFECTS if d['id'] == 'DF1')
    (fx, fy), (nx, ny) = sc.wall_face(df)
    assert true.is_solid(np.array([fx + nx * 0.04]), np.array([fy + ny * 0.04]))[0]   # bulge in front of the face


# ------------------------------------------------------------------------------------------ camera
def test_camera_model_round_trip_and_floor_geometry():
    cam = CameraModel(320, 180)
    pose = (1.0, 2.0, 0.7)
    rng = np.random.default_rng(0)
    pw = np.stack([1 + rng.uniform(0.5, 3, 50) * math.cos(0.7), 2 + rng.uniform(0.5, 3, 50) * math.sin(0.7),
                   rng.uniform(0, 0.5, 50)], -1)
    u, v, d = cam.project_world(pw, pose)
    R, t = cam.cam_to_world(pose)
    back = (cam.rays_cam()[0, 0] * 0)                                # shape check only
    assert back.shape == (3,)
    ray = np.stack([np.ones_like(u), -(u - cam.cx) / cam.fx, -(v - cam.cy) / cam.fx], -1) * d[:, None]
    assert np.allclose(ray @ R.T + t, pw, atol=1e-9)
    assert np.allclose(map_to_base(base_to_map([[0.3, -0.2]], pose), pose), [[0.3, -0.2]])


def test_renderer_and_camera_agree_on_depth(scene):
    cam = CameraModel(320, 180)
    depth, _, lab = scene.render(cam, (0.0, 0.0, 0.0))
    pc, vv, uu = cam.depth_to_points(depth)
    pw = cam.to_world(pc, (0.0, 0.0, 0.0))
    floor = lab[vv, uu] == -1
    assert np.abs(pw[floor, 2]).max() < 1e-6                         # floor pixels land on z = 0


def test_footprint_of_a_rotated_rectangle():
    a = 0.4
    u, v = np.meshgrid(np.linspace(-0.1, 0.1, 30), np.linspace(-0.03, 0.03, 10))
    xy = np.stack([u.ravel() * math.cos(a) - v.ravel() * math.sin(a) + 2,
                   u.ravel() * math.sin(a) + v.ravel() * math.cos(a) - 1], -1)
    cx, cy, L, W, yaw = footprint(xy)
    assert abs(cx - 2) < 1e-3 and abs(cy + 1) < 1e-3 and abs(L - 0.2) < 0.005 and abs(W - 0.06) < 0.005
    assert abs(yaw - a) < math.radians(2.5)


# ------------------------------------------------------------------------------------------ debris
@pytest.fixture(scope='module')
def corridor_run(prior, scene):
    """Drive A -> 7 m along A-J1 (debris S1, S2 and two sludge patches)."""
    cam = CameraModel(320, 180)
    det = DepthDebrisDetector(prior, cam, DebrisParams(ignore_zones=[(-0.51, -0.17, -0.22, 0.22)]))
    trk = DebrisTracker()
    for k, x in enumerate(np.arange(0.0, 7.0, 0.3)):
        pose = (x, 0.0, 0.0)
        depth, _, _ = scene.render(cam, pose)
        trk.update(det.detect(depth, pose), k * 0.75, make_visible_fn(cam, pose, depth))
    return trk


def test_separate_small_debris_found_with_dimensions(corridor_run):
    conf = [t for t in corridor_run.tracks if t.state == 'confirmed']
    assert len(conf) == 2                                             # S1 and S2 only: no sludge, water or tray
    truth = {s['id']: s for s in sc.SMALL_DEBRIS}
    for t, sid in zip(sorted(conf, key=lambda t: t.geometry()['x']), ('S1', 'S2')):
        g, s = t.geometry(), truth[sid]
        assert math.hypot(g['x'] - s['x'], g['y'] - s['y']) < 0.03
        assert abs(g['height'] - s['size'][2]) < 0.01
        assert abs(g['length'] - max(s['size'][:2])) < 0.02
        assert t.confidence() > 0.8


def test_tracker_keeps_ids_and_marks_unseen_objects_missing(prior, scene):
    cam = CameraModel(320, 180)
    det = DepthDebrisDetector(prior, cam)
    trk = DebrisTracker(miss_limit=3)
    for k, x in enumerate((1.6, 1.7, 1.8, 1.9)):
        d, _, _ = scene.render(cam, (x, 0.0, 0.0))
        trk.update(det.detect(d, (x, 0.0, 0.0)), k, make_visible_fn(cam, (x, 0.0, 0.0), d))
    assert [t.id for t in trk.tracks if t.state == 'confirmed'] == ['D1']
    gone = render3d.Scene(removed=('S1',))                             # the object disappears (picked by someone)
    for k in range(4):
        d, _, _ = gone.render(cam, (1.9, 0.0, 0.0))
        trk.update(det.detect(d, (1.9, 0.0, 0.0)), 10 + k, make_visible_fn(cam, (1.9, 0.0, 0.0), d))
    assert trk.get('D1').state == 'missing'
    trk.set_state('D1', 'held')
    d, _, _ = scene.render(cam, (1.9, 0.0, 0.0))
    trk.update(det.detect(d, (1.9, 0.0, 0.0)), 20, None)
    assert trk.get('D1').state == 'held' and trk.get('D2') is not None      # a held object is not re-associated


def test_feasibility_rules_match_the_scenario_design(prior, graph):
    nodes, edges = graph
    for s in sc.SMALL_DEBRIS:
        sx, sy, sz = s['size']
        L, W = max(sx, sy), min(sx, sy)
        yaw = s['yaw'] + (math.pi / 2 if sy > sx else 0.0)
        o = dict(x=s['x'], y=s['y'], yaw=yaw, length=L, width=W, height=sz,
                 shape='round' if s['kind'] == 'cyl' else 'box', confidence=0.9, n_frames=10)
        f = assess(o, prior)
        assert f['graspable'] == s['graspable_designed'], (s['id'], f)
        if f['graspable']:
            p = f['approach_poses'][0]
            assert abs(math.hypot(p['x'] - s['x'], p['y'] - s['y']) - 0.42) < 1e-3
            assert prior.dist(np.array([p['x']]), np.array([p['y']]))[0] >= 0.18
    big = dict(x=12.0, y=0.0, yaw=0.0, length=0.6, width=1.0, height=0.5, confidence=1, n_frames=10)
    assert obstruction(big, nodes, edges)['severity'] == 'full'
    s1 = dict(x=2.5, y=0.28, yaw=0.3, length=0.06, width=0.06, height=0.06)
    assert obstruction(s1, nodes, edges)['severity'] == 'minor' and obstruction(s1, nodes, edges)['edge'] == 'A-J1'
    low = dict(s1, confidence=0.2, n_frames=1, shape='box')
    assert 'low_confidence' in assess(low, prior)['reasons']


def test_lidar_clusters_join_the_two_faces_of_one_pile(prior):
    grid = EvidenceGrid(prior)
    for x in (12.0, 12.4):                                            # two faces of one pile, interior never seen
        i, j, _ = prior.cells(np.full(15, x), np.linspace(-0.35, 0.35, 15))
        grid.L[j, i] = 5.0
    assert len(lidar_clusters(grid)) == 1
    xs, ys = np.meshgrid(np.arange(11.0, 13.5, 0.025), np.arange(-0.5, 0.5, 0.025))
    ii, jj, _ = prior.cells(xs.ravel(), ys.ravel())
    grid.seen[jj, ii] = True                                          # the robot drove between them: gap observed
    assert len(lidar_clusters(grid)) == 2


# ------------------------------------------------------------------------------------------ defects
def test_severity_rule():
    assert severity('crack', dict(length=0.1))[0] == 'minor'
    assert severity('crack', dict(length=0.3))[0] == 'moderate'
    assert severity('hole', dict(width=0.2))[0] == 'severe'
    assert severity('deformation', dict(depth=0.05))[0] == 'moderate'


@pytest.mark.parametrize('bias', [(0.0, 0.0, 0.0), (0.03, -0.02, math.radians(1.0))])
def test_wall_defects_along_the_first_corridor(prior, bias):
    true = sc.true_lidar_map(prior)
    wd = WallDefectDetector(prior)
    rng = np.random.default_rng(3)
    for x in np.arange(0.5, 7.5, 0.04):
        pose = np.array([x, 0.0, 0.0])
        wd.update(tuple(pose + bias), render3d.lidar_scan(true, pose, rng), sim2d.ANGLES)
    found = wd.detect()
    assert len(found) == 1, found                                     # H1 and nothing else in A-J1
    h = found[0]
    assert h['type'] == 'hole' and abs(h['x'] - 3.475) < 0.06 and abs(h['y'] + 0.5) < 0.06
    assert abs(h['size']['width'] - 0.15) <= 0.05 and abs(h['size']['depth'] - 0.10) < 0.03


def test_crack_detector_and_fusion(prior, scene):
    rc, dc = CameraModel(640, 360), CameraModel(320, 180)
    cd = CrackDetector(prior, rc)
    rng = np.random.default_rng(0)
    pose = (4.8, 0.0, 0.0)
    _, rgb, _ = scene.render(rc, pose, noise=0.02, rng=rng)
    dep, _, _ = scene.render(dc, pose)
    det = cd.detect((rgb * 255).astype(np.uint8), dep, pose)
    assert len(det) == 1 and abs(det[0]['size']['length'] - 0.40) < 0.05
    assert abs(det[0]['x'] - 6.2) < 0.1 and abs(det[0]['y'] - 0.5) < 0.03
    for p in ((1.0, 0.0, 0.0), (3.0, 0.0, 0.0), (12.0, 7.0, math.pi / 2)):       # no cracks there
        _, rgb, _ = scene.render(rc, p, noise=0.02, rng=rng)
        dep, _, _ = scene.render(dc, p)
        assert cd.detect(rgb, dep, p) == []
    tr = DefectTracker()
    a = dict(det[0], ends=[[6.1, 0.5, 0.4], [6.2, 0.5, 0.5]], x=6.15, z=0.45, size=dict(length=0.14, width=0.005, depth=0))
    b = dict(det[0], ends=[[6.2, 0.5, 0.5], [6.3, 0.5, 0.6]], x=6.25, z=0.55, size=dict(length=0.14, width=0.005, depth=0))
    tr.update([a], 1.0, dict(source='camera', image='a.png', frame_stamp=1.0))
    tr.update([b], 2.0, dict(source='camera', image='b.png', frame_stamp=2.0))
    assert len(tr.items) == 1 and tr.items[0]['id'] == 'F1'
    assert abs(tr.items[0]['size']['length'] - math.dist([6.1, 0.5, 0.4], [6.3, 0.5, 0.6])) < 1e-3


# ------------------------------------------------------------------------------------------ contracts
def test_messages_validate_and_bad_ones_are_rejected(corridor_run, prior, graph):
    nodes, edges = graph
    objs = []
    for t in corridor_run.tracks:
        if t.state == 'confirmed':
            g = t.geometry()
            o = dict(g, confidence=t.confidence(), n_frames=t.hits)
            objs.append(itf.debris_object(t, assess(o, prior), obstruction(o, nodes, edges)))
    m = itf.debris_msg(objs, 12.5)
    assert itf.validate_debris(m) and all(o['graspable'] for o in objs)
    bad = itf.debris_msg([dict(objs[0], feasibility=dict(objs[0]['feasibility'], reasons=['too_wide']))], 1.0)
    with pytest.raises(itf.SchemaError):
        itf.validate_debris(bad)
    with pytest.raises(itf.SchemaError):
        itf.validate_disposal(itf.disposal_msg(1.0, 'T', 'D1', True, False, True))
    assert itf.validate_manipulation(itf.manipulation_msg('holding', 1.0, 'T', 'D1', dict(grasp_verified=True)))


def test_messages_pass_account1_parser(corridor_run, prior, graph):
    """Every Account 2 message goes through Account 1's own parser (sewer_planning.mission_interfaces) without a
    single error, and every Mission Manager task is understood (docs/mission_interfaces.md)."""
    mi = pytest.importorskip('sewer_planning.mission_interfaces')
    nodes, edges = graph
    objs = []
    for t in corridor_run.tracks:
        if t.state == 'confirmed':
            g = t.geometry()
            o = dict(g, confidence=t.confidence(), n_frames=t.hits)
            objs.append(itf.debris_object(t, assess(o, prior), obstruction(o, nodes, edges)))
    parsed, errors = mi.parse_debris(itf.dumps(itf.debris_msg(objs, 12.5)))
    assert errors == [] and [d.id for d in parsed] == [o['id'] for o in objs]
    assert all(d.graspable is True and d.size[2] > 0.04 and 0 < d.confidence <= 1 for d in parsed)
    tr = DefectTracker()
    tr.update([dict(type='crack', x=6.2, y=0.5, z=0.5, size=dict(length=0.4, width=0.008, depth=0.0), n_obs=1,
                    confidence=0.8, source='camera', bbox=[1, 2, 3, 4], ends=[[6.0, 0.5, 0.4], [6.4, 0.5, 0.6]]),
               dict(type='hole', x=3.475, y=-0.5, z=0.13, normal=(0, 1), size=dict(width=0.15, depth=0.1, length=0.15),
                    n_obs=40, confidence=0.9, source='lidar')], 5.0, dict(source='camera', image='/x/crack.png', frame_stamp=5.0))
    dparsed, derr = mi.parse_defects(itf.dumps(itf.defects_msg([itf.defect_item(x) for x in tr.items], 5.0)))
    assert derr == [] and {d.type for d in dparsed} == {'crack', 'hole'}
    assert all(0 <= d.severity <= 1 for d in dparsed) and dparsed[0].evidence == '/x/crack.png'
    for st in ('idle', 'approaching', 'grasping', 'lifting', 'holding', 'releasing', 'released', 'failed'):
        v = dict(grasp_verified=True, lift_verified=st == 'holding', release_verified=st == 'released')
        ms = mi.parse_manipulation(itf.dumps(itf.manipulation_msg(st, 3.0, 'T4', 'D1', v, 'r' if st == 'failed' else '')))
        assert ms.task_id == 'T4' and ms.object_id == 'D1'
        assert ms.verified == (st in ('holding', 'released'))
    lifting = mi.parse_manipulation(itf.dumps(itf.manipulation_msg('lifting', 3.0, 'T4', 'D1', dict(grasp_verified=True))))
    assert lifting.state == 'GRASPING' and not lifting.verified        # HOLDING only once the lift is verified
    ds = mi.parse_disposal(itf.dumps(itf.disposal_msg(4.0, 'T5', 'D1', True, True, True)))
    assert ds.verified and ds.released and ds.station_reached and ds.task_id == 'T5'
    for typ, mine in (('PICKUP', 'pickup'), ('DISPOSE', 'release'), ('STOP', 'stop')):
        task = mi.MissionTask(task_id='T9', type=typ, stamp=1.0, object_id='D1',
                              target=dict(x=2.5, y=0.28, yaw=0.0, frame_id='map'), action='x')
        assert itf.parse_task(task.to_json())['type'] == mine
    assert itf.parse_task(mi.MissionTask(task_id='T1', type='EXPLORE', stamp=1.0).to_json()) is None
    with pytest.raises(itf.SchemaError):
        itf.parse_task(mi.MissionTask(task_id='T1', type='PICKUP', stamp=1.0).to_json())       # no object id
    with pytest.raises(itf.SchemaError):
        itf.parse_task('{"task_id": "T1", "type": "PICKUP", "object_id": "D1", "target": {"frame_id": "odom"}}')


def test_object_seen_end_on_is_not_graspable_yet(prior, scene):
    """Approaching plank S6 (0.30 m long, along the corridor) end-on, the camera cannot see its length; it must not be
    offered for pickup until it has been seen from more than one direction (found by the end-to-end mission test)."""
    cam = CameraModel(320, 180)
    det, trk = DepthDebrisDetector(prior, cam), DebrisTracker()
    for k, y in enumerate(np.arange(6.3, 7.25, 0.15)):
        pose = (12.0, y, math.pi / 2)
        d, _, _ = scene.render(cam, pose)
        trk.update(det.detect(d, pose), k, make_visible_fn(cam, pose, d))
    t = next(t for t in trk.tracks if math.hypot(*(t.centroid() - (12.25, 8.0))) < 0.3)
    g = t.geometry()
    f = assess(dict(g, confidence=t.confidence(), n_frames=t.hits, min_range=t.min_range,
                    view_spread=t.view_spread()), prior)
    assert t.view_spread() < math.radians(20) and not f['graspable']
    assert any(r.startswith('seen_from_one_direction_only') for r in f['reasons'])
