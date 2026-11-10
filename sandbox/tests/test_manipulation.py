"""Account 2: arm kinematics, URDF consistency, LiDAR self-filter and the pick/carry/release controller (no ROS)."""
import math
import os
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from fake_arm_world import FakeArmWorld
from sewer_manipulation.arm_kinematics import (FINGERS, JOINTS, ArmGeometry, crosses_scan_plane, fk, grasp_height,
                                               ik, scan_self_mask)

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
GEO = ArmGeometry()


def test_ik_fk_round_trip_over_the_workspace():
    rng = np.random.default_rng(0)
    n_ok = 0
    for _ in range(300):
        t = (rng.uniform(0.25, 0.48), rng.uniform(-0.10, 0.10), rng.uniform(-0.04, 0.20))
        q = ik(GEO, t)
        if q is None:
            continue
        n_ok += 1
        assert np.allclose(fk(GEO, q)['tip'], t, atol=1e-6)
        assert abs(fk(GEO, q)['pitch']) < 1e-9                      # gripper stays horizontal
    assert n_ok > 200


def test_working_poses_are_reachable_and_stow_stays_out_of_the_scan_plane():
    stow = ik(GEO, GEO.stow)
    assert stow is not None and not crosses_scan_plane(GEO, stow)
    assert not crosses_scan_plane(GEO, stow, 0.04, dict(height=0.15, length=0.10, width=0.08))   # carrying
    # every graspable height, anywhere the Mission Manager stops: 0.25 m before the near face, +-0.25 m, and up to
    # 0.3 m beside the centre line; the gripper is lowered onto the object from 0.12 m above and lifted 0.12 m
    for h in (0.03, 0.06, 0.11, 0.15):
        for x in (0.25, 0.35, 0.45, 0.55, 0.60):
            for y in ((-0.3, 0.0, 0.3) if x <= 0.55 else (-0.2, 0.0, 0.2)):
                z = grasp_height(GEO, h) - GEO.base_z
                for dz in (0.0, 0.12):
                    assert ik(GEO, (x, y, z + dz)) is not None, (x, y, h, dz)
    q = ik(GEO, (0.38, 0.0, grasp_height(GEO, 0.06) - GEO.base_z))
    assert crosses_scan_plane(GEO, q)                                  # lowered arm is in the scan: needs the filter
    p = fk(GEO, stow)
    # folded over the robot: inside its turning circle (0.30 m) and above everything but the beacon mast
    assert all(math.hypot(p[k][0], p[k][1]) <= 0.30 and p[k][2] > 0.15 for k in ('elbow', 'wrist', 'tip'))


def _urdf_tip(q, finger=0.0):
    xacro = pytest.importorskip('xacro')
    path = os.path.join(ROOT, 'src', 'sewer_description', 'urdf', 'sewer_robot_inspection.urdf.xacro')
    root = ET.fromstring(xacro.process_file(path).toxml())
    joints = {j.get('name'): j for j in root.findall('joint')}
    by_child = {j.find('child').get('link'): j for j in joints.values()}
    chain, link = [], 'left_finger_link'
    while link in by_child and link != 'base_link':
        j = by_child[link]
        chain.append(j)
        link = j.find('parent').get('link')
    chain.reverse()
    T = np.eye(4)
    vals = dict(zip(JOINTS, q), left_finger_joint=finger)
    from sewer_perception.camera_model import rot_rpy
    for j in chain:
        o = j.find('origin')
        xyz = [float(v) for v in (o.get('xyz') or '0 0 0').split()]
        rpy = [float(v) for v in (o.get('rpy') or '0 0 0').split()]
        A = np.eye(4)
        A[:3, :3], A[:3, 3] = rot_rpy(*rpy), xyz
        T = T @ A
        if j.get('type') in ('revolute', 'prismatic'):
            ax = np.array([float(v) for v in j.find('axis').get('xyz').split()])
            v = vals[j.get('name')]
            M = np.eye(4)
            if j.get('type') == 'revolute':
                K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
                M[:3, :3] = np.eye(3) + math.sin(v) * K + (1 - math.cos(v)) * K @ K
            else:
                M[:3, 3] = ax * v
            T = T @ M
    return T[:3, 3]


def test_urdf_arm_matches_the_kinematics_model():
    rng = np.random.default_rng(1)
    for _ in range(20):
        q = [rng.uniform(lo, hi) for lo, hi in GEO.limits]
        assert np.allclose(_urdf_tip(q), fk(GEO, q)['tip'], atol=1e-6)


def test_self_filter_masks_the_lowered_arm_and_nothing_when_folded():
    ang = np.linspace(-math.pi, math.pi, 360)
    q = ik(GEO, (0.38, 0.0, -0.03))
    r = np.full(360, 2.0)
    r[np.abs(ang) < 0.15] = 0.24                     # beams that end on the forearm ~0.23 m ahead
    m = scan_self_mask(GEO, q, 0.03, ang, r)
    assert m[np.abs(ang) < 0.15].all() and not m[np.abs(ang) > 0.6].any()
    assert not scan_self_mask(GEO, ik(GEO, GEO.stow), 0.05, ang, np.full(360, 0.2)).any()


# ------------------------------------------------------------------------------------------ controller
def pickup(w, obj_id='D1'):
    w.step(5)
    w.ctl.request({'type': 'stow', 'task_id': 'S'}, w.t)
    w.run_until(('idle',), 20)
    ok = w.ctl.request({'type': 'pickup', 'task_id': 'T1', 'object_id': obj_id}, w.t, w.obj_in_base())
    w.run_until(('holding', 'failed'), 60)
    return ok


def test_pickup_carry_release_and_disposal_are_verified():
    w = FakeArmWorld()
    pickup(w)
    v = w.ctl.verification
    assert w.ctl.state == 'holding', (w.ctl.reason, v)
    assert v['grasp_verified'] and v['lift_verified'] and v['object_model'] == 'debris_S1'
    assert v['object_lift_m'] >= 0.08 and abs(v['finger_gap'] - 0.06) < 0.005
    assert w.states[:5] == ['idle', 'approaching', 'grasping', 'lifting', 'holding']
    assert not w.ctl.status()['arm_in_scan_plane']                   # carrying in the folded pose
    w.move_robot((0.04, 0.0, math.pi))                               # the Mission Manager drove to the station
    w.step(5)
    w.ctl.request({'type': 'release', 'task_id': 'T2'}, w.t, station_reached=True, drop=w.tray_in_base())
    w.run_until(('released', 'failed'), 60)
    d = w.ctl.disposal
    assert w.ctl.state == 'released' and d['object_released'] and d['disposal_verified'], d
    assert set(d['evidence']) == {'camera_tray_points', 'sim_pose_in_tray'}
    assert TRAY_contains(w.obj)


def TRAY_contains(o):
    return -0.44 < o['x'] < -0.24 and -0.15 < o['y'] < 0.15


def test_mission_manager_stand_off_and_station_tolerance():
    """Where Account 1's Mission Manager actually stops: on the corridor centre line, 0.25 m before the object's
    near face, up to 0.25 m short; the object 0.28 m beside the centre line. At the station it stops anywhere within
    0.25 m of the entrance node facing the end wall; the arm aims at the tray from wherever it is."""
    w = FakeArmWorld(robot=(2.5 - 0.03 - 0.25 - 0.22, 0.0, 0.0))
    pickup(w)
    assert w.ctl.state == 'holding', (w.ctl.reason, w.ctl.verification)
    w.move_robot((0.18, 0.12, math.pi - 0.15))
    w.step(5)
    w.ctl.request({'type': 'release', 'task_id': 'T2'}, w.t, station_reached=True, drop=w.tray_in_base())
    w.run_until(('released', 'failed'), 60)
    assert w.ctl.disposal['disposal_verified'] and TRAY_contains(w.obj), (w.ctl.disposal, w.obj)


def test_stop_freezes_the_arm_and_keeps_the_object():
    w = FakeArmWorld()
    pickup(w)
    w.ctl.request({'type': 'release', 'task_id': 'T2'}, w.t, station_reached=True, drop=(0.42, 0.0))
    w.step(10)                                                        # moving towards the drop point
    w.ctl.request({'type': 'stop', 'task_id': 'T3'}, w.t)
    q0 = dict(w.q)
    w.step(20)
    assert w.ctl.state == 'holding' and w.ctl.reason == 'stopped' and not w.ctl.steps
    assert all(abs(w.q[j] - q0[j]) < 0.08 for j in JOINTS) and w.held_since is not None


def test_drop_point_out_of_reach_keeps_holding():
    w = FakeArmWorld()
    pickup(w)
    assert not w.ctl.request({'type': 'release', 'task_id': 'T2'}, w.t, station_reached=False, drop=(1.2, 0.0))
    assert w.ctl.state == 'holding' and w.ctl.disposal['reason'].startswith('failed')


def test_release_away_from_the_station_is_not_counted_as_disposal():
    w = FakeArmWorld()
    pickup(w)
    w.ctl.request({'type': 'release', 'task_id': 'T2'}, w.t, station_reached=False, drop=(0.42, 0.0))
    w.run_until(('released', 'failed'), 60)
    assert w.ctl.disposal['object_released'] and not w.ctl.disposal['disposal_verified']
    assert w.ctl.disposal['reason'].startswith('failed: released away')


@pytest.mark.parametrize('kw, reason', [(dict(present=False), 'no_finger_contact'),
                                        (dict(one_sided=True), 'one_sided_contact')])
def test_failed_grasps_are_reported_and_the_arm_folds_away(kw, reason):
    w = FakeArmWorld(**kw)
    pickup(w)
    assert w.ctl.state == 'failed' and w.ctl.reason == reason
    assert not w.ctl.verification.get('grasp_verified')
    w.run_until(('failed',), 30)                                     # retreat finishes, state stays failed
    assert w.ctl.state == 'failed' and not w.ctl.status()['arm_in_scan_plane']


def test_flickering_contacts_are_not_a_slip():
    """Gazebo publishes contact messages only on steps with contact; with flicker they arrive in bursts."""
    w = FakeArmWorld(flicker=8)                                      # a message every 0.4 s
    pickup(w)
    assert w.ctl.state == 'holding', (w.ctl.reason, w.ctl.verification)
    w.step(100)
    assert w.ctl.state == 'holding'


def test_a_slipping_object_is_detected():
    w = FakeArmWorld(slip_after=4.0)
    pickup(w)
    w.step(200)
    assert w.ctl.state == 'failed' and w.ctl.reason in ('object_slipped', 'dropped')


def test_impossible_requests_are_refused_without_moving():
    w = FakeArmWorld(size=(0.16, 0.16, 0.06))                          # 16 cm whichever way the gripper turns
    w.step(3)
    q0 = dict(w.q)
    assert not w.ctl.request({'type': 'pickup', 'task_id': 'T', 'object_id': 'D'}, w.t, w.obj_in_base())
    assert w.ctl.reason.startswith('too_wide_for_gripper') and not w.ctl.steps
    w.step(5)
    assert all(abs(w.q[j] - q0[j]) < 1e-9 for j in JOINTS)
    far = FakeArmWorld(robot=(1.5, 0.28, 0.0))                        # object 1 m ahead: out of reach
    far.step(3)
    assert not far.ctl.request({'type': 'pickup', 'task_id': 'T', 'object_id': 'D'}, far.t, far.obj_in_base())
    assert far.ctl.reason.startswith('unreachable')


def test_wrist_twist_grasps_objects_lying_across_or_at_an_angle():
    """The Mission Manager stops facing along the corridor; the wrist yaw turns the fingers to the object's narrow side
    (a 10 x 5 cm object lying across the corridor is 10 cm 'wide' for a gripper that cannot twist)."""
    for yaw in (math.pi / 2, 0.8, -1.2):
        w = FakeArmWorld(robot=(2.0, 0.0, 0.0), obj=(2.45, 0.2), size=(0.10, 0.05, 0.05))
        w.obj_yaw = yaw
        pickup(w)
        assert w.ctl.state == 'holding', (yaw, w.ctl.reason, w.ctl.verification)
        assert abs(w.ctl.verification['finger_gap'] - 0.05) < 0.005


def test_a_second_pickup_while_holding_is_refused_and_the_object_kept():
    w = FakeArmWorld()
    pickup(w)
    assert not w.ctl.request({'type': 'pickup', 'task_id': 'T3', 'object_id': 'D9'}, w.t, w.obj_in_base())
    w.step(20)
    assert w.ctl.state == 'holding'


def test_finger_constants_match_the_feasibility_limits():
    from sewer_perception.feasibility import GripperLimits
    lim = GripperLimits()
    assert 2 * GEO.finger_max - 2 * 0.01 >= lim.max_width          # 1 cm spare each side at the widest object
    assert abs(lim.grasp_x - GEO.grasp_x) < 1e-9
    assert set(FINGERS) == {'left_finger_joint', 'right_finger_joint'}


def test_bridge_disposal_status_keeps_its_own_object():
    """A finished disposal must keep naming the disposed object when the next PICKUP starts (perception would
    otherwise mark the NEW target as removed; found by the end-to-end mission test)."""
    import json as _json
    from sewer_manipulation.manipulation_bridge import ManipulationBridge
    b = ManipulationBridge({'tray': dict(x_min=-0.44, x_max=-0.24, y_min=-0.15, y_max=0.15)}, log=lambda s: None)
    b.on_pose((0.05, 0.0, math.pi))
    b.on_debris(_json.dumps(dict(frame_id='map', stamp=1.0, objects=[
        dict(id='D2', x=2.5, y=0.28, yaw=0.0, size=[0.06, 0.06, 0.06], stamp=1.0)])))
    b.ctl.object_id, b.ctl.state = 'D1', 'holding'
    b.on_task(_json.dumps(dict(task_id='T5', type='DISPOSE', stamp=1.0, object_id='D1', target={})), 1.0)
    b.ctl.disposal.update(object_released=True, disposal_verified=True)
    b.on_pose((2.1, 0.28, 0.0))
    b.on_task(_json.dumps(dict(task_id='T6', type='PICKUP', stamp=2.0, object_id='D2',
                               target=dict(x=2.5, y=0.28, frame_id='map'))), 2.0)
    d = _json.loads(b.disposal_json(2.5))
    assert d['object_id'] == 'D1' and d['task_id'] == 'T5' and d['verified']
    assert _json.loads(b.status_json(2.5))['object_id'] == 'D2'


def test_object_beside_the_robot_is_picked():
    """Perception may only mark an object graspable once the robot is nearly level with it (seen from two directions),
    so the Mission Manager can already be past its stand-off point: object beside the robot, 0.28 m to the side
    (found by the end-to-end mission test). The arm swings round the turret axis to reach it."""
    for robot, obj in (((2.47, 0.0, 0.0), (2.5, 0.28)), ((5.95, 0.0, 0.0), (6.0, -0.30))):
        w = FakeArmWorld(robot=robot, obj=obj)
        pickup(w)
        assert w.ctl.state == 'holding', (robot, w.ctl.reason, w.ctl.verification)


def test_bridge_waits_for_the_base_to_stop_before_grasping():
    """The Mission Manager sends PICKUP within 0.25 m of the stand-off point, possibly still rolling: the bridge plans
    the grasp only once the estimated pose has been still, from where the robot then is."""
    import json as _json
    from sewer_manipulation.manipulation_bridge import ManipulationBridge
    b = ManipulationBridge(None, log=lambda s: None)
    for n in JOINTS + FINGERS:
        b.q[n] = 0.0
    b.on_debris(_json.dumps(dict(frame_id='map', stamp=0.0, objects=[
        dict(id='D1', x=2.5, y=0.28, yaw=0.0, size=[0.06, 0.06, 0.06], stamp=0.0)])))
    t, x = 0.0, 1.9
    b.on_pose((x, 0.0, 0.0))
    b.tick(t)
    b.on_task(_json.dumps(dict(task_id='T1', type='PICKUP', stamp=t, object_id='D1',
                               target=dict(x=2.5, y=0.28, frame_id='map'))), t)
    while x < 2.2:                                                     # still rolling 0.3 m
        t, x = t + 0.05, x + 0.015
        b.on_pose((x, 0.0, 0.0))
        b.tick(t)
        assert b.pending is not None and b.ctl.reason == 'waiting for the robot to stop'
    for _ in range(20):
        t += 0.05
        b.tick(t)
    assert b.pending is None and b.ctl.state == 'approaching', b.ctl.reason
    assert abs(b.ctl.obj['x'] - (2.5 - x)) < 0.01                       # planned from where the robot stopped


def test_urdf_joint_limits_match_the_kinematics_model():
    xacro = pytest.importorskip('xacro')
    path = os.path.join(ROOT, 'src', 'sewer_description', 'urdf', 'sewer_robot_inspection.urdf.xacro')
    joints = {j.get('name'): j for j in ET.fromstring(xacro.process_file(path).toxml()).findall('joint')}
    for name, (lo, hi) in zip(JOINTS, GEO.limits):
        lim = joints[name].find('limit')
        assert abs(float(lim.get('lower')) - lo) < 1e-9 and abs(float(lim.get('upper')) - hi) < 1e-9, name


def test_last_sighting_is_carried_forward_with_odometry():
    """Object beside the robot, out of the camera's view; the localization estimate has drifted 6 cm since the camera
    last saw the object. The bridge aims from where the camera saw it relative to the robot, moved with wheel
    odometry, not from the current estimate (found by the end-to-end mission
    test: the fingers closed beside the object)."""
    import json as _json
    from sewer_manipulation.manipulation_bridge import ManipulationBridge
    b = ManipulationBridge(None, log=lambda s: None)
    # true robot: drove from x=7.0 to x=7.45 along +x; object at (7.50, 0.28) seen at t=10 when the robot was at 7.0
    b.on_odom(10.0, (1.0, 2.0, 0.3))                                   # odometry frame is arbitrary
    c, s = math.cos(0.3), math.sin(0.3)
    b.on_odom(15.0, (1.0 + 0.45 * c, 2.0 + 0.45 * s, 0.3))
    b.on_pose((7.51, 0.0, 0.0))                                        # estimate drifted: 6 cm ahead of the truth
    b.on_debris(_json.dumps(dict(frame_id='map', stamp=10.0, objects=[
        dict(id='D1', x=7.50, y=0.28, yaw=0.0, size=[0.10, 0.05, 0.05], stamp=10.0,
             last_seen_base=dict(x=0.50, y=0.28, stamp=10.0))])))
    o = b.object_in_base('D1', 15.0)
    assert b.target_source == 'camera+odometry'
    assert abs(o['x'] - 0.05) < 1e-6 and abs(o['y'] - 0.28) < 1e-6     # truth, not the estimate's -0.01
    b.odom = []                                                        # no odometry: falls back to the map pose
    assert abs(b.object_in_base('D1', 15.0)['x'] + 0.01) < 1e-6 and b.target_source == 'map+localization'


def test_tray_is_located_from_its_rims_despite_localization_error():
    """Disposal is checked against the tray the depth camera SEES, not where the pose estimate puts it (found by the
    disturbed end-to-end mission: 20 cm localization error, the object landed beside the tray, points there were
    counted as 'in the tray'). The drop point comes from the located tray too."""
    import yaml
    import render3d
    import sim2d
    from sewer_perception.camera_model import CameraModel, map_to_base
    from sewer_manipulation.manipulation_bridge import ManipulationBridge
    with open(os.path.join(sim2d.GAZEBO_PKG, 'config', 'inspection_station.yaml')) as f:
        st = yaml.safe_load(f)
    b = ManipulationBridge(st, log=lambda s: None)
    cam = CameraModel(320, 180)
    true = (0.06, 0.03, math.pi - 0.05)
    want = map_to_base([(-0.34, 0.0)], true)[0]
    empty, _, _ = render3d.Scene(include_small_debris=False).render(cam, true)
    for err in ((0.0, 0.0, 0.0), (0.2, 0.0, 0.0), (-0.1, 0.15, 0.04), (0.25, -0.2, -0.05)):
        est = tuple(a + e for a, e in zip(true, err))
        n = b.tray_points(cam, empty, est, 1.0)
        assert n is not None and n < 10, (err, n)                       # empty tray reads empty
        assert math.hypot(b.tray_seen[1] - want[0], b.tray_seen[2] - want[1]) < 0.01, (err, b.tray_seen)
        assert b.drop_point(1.5) == b.tray_seen[1:]
    assert b.drop_point(5.0) != b.tray_seen[1:]                         # stale: back to the station config
    # far from the station, facing away: no rims in view, so no camera evidence at all
    far = (2.0, 0.0, 0.0)
    assert b.tray_points(cam, render3d.Scene(include_small_debris=False).render(cam, far)[0], far, 2.0) is None


def test_disposal_is_never_verified_when_the_simulator_says_outside_the_tray():
    w = FakeArmWorld()
    pickup(w)
    w.move_robot((0.04, 0.0, math.pi))
    w.step(5)
    drop = w.tray_in_base()
    w.ctl.request({'type': 'release', 'task_id': 'T2'}, w.t, station_reached=True, drop=(drop[0], drop[1] + 0.25))
    w.run_until(('released', 'failed'), 60)
    w.step(60)
    d = w.ctl.disposal
    assert d['object_released'] and not d['disposal_verified'] and 'outside the tray' in d['reason'], d
