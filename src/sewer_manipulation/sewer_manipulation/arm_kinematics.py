"""Kinematics of the simulated inspection arm (sewer_robot_inspection.urdf.xacro). CUSTOM code (Account 2), pure NumPy.

Design (chosen after checking the robot geometry and the Mission Manager's behaviour, see ACCOUNT2_README.md):
  - a short mast on top of the LiDAR housing carries a turret (yaw) joint at base_link (0.05, 0, 0.12); the 2D LiDAR
    scans at base_link z = 0.07, so the folded arm never crosses the scan plane while the robot drives
  - shoulder, elbow and wrist pitch joints (planar arm, upper arm 0.32 m, forearm 0.30 m); the wrist pitch keeps the
    gripper horizontal
  - a wrist YAW joint turns the gripper about the vertical, so the fingers can close across the object's narrow side
    whatever direction the robot faces (the Mission Manager always stops facing ALONG the corridor; without this
    joint, objects lying at an angle or across the corridor were too wide across the fingers: found by the end-to-end
    test sandbox/mission_sim_account2.py)
  - two prismatic fingers close sideways (inner opening 0..0.11 m), finger centre 0.06 m beyond the wrist
  - 5 arm joints + 2 finger joints: picks objects off the floor 0.25-0.60 m ahead, up to 0.3 m to the side, and
    beside the robot (turret +-1.9 rad), which covers the Mission Manager's stand-off (0.25 m before the object's near
    face, arrival tolerance 0.25 m) and the case where it is already level with the object; folds back over the robot
    for carrying in a 1 m corridor

Angles: URDF pitch joints rotate about +y, so a POSITIVE joint angle points the link DOWN. Internally elevations
e (positive up) are used: q_pitch = -e. The wrist yaw joint rotates about the gripper's own z axis (vertical when the
gripper is horizontal); heading = turret + wrist yaw.
"""
import math
from dataclasses import dataclass

import numpy as np

JOINTS = ('arm_turret_joint', 'arm_shoulder_joint', 'arm_elbow_joint', 'arm_wrist_joint', 'arm_wrist_yaw_joint')
FINGERS = ('left_finger_joint', 'right_finger_joint')


@dataclass
class ArmGeometry:
    turret_xyz: tuple = (0.05, 0.0, 0.12)      # turret axis in base_link
    shoulder_dz: float = 0.01                  # shoulder above the turret joint
    l1: float = 0.32
    l2: float = 0.30
    g: float = 0.06                            # wrist axis -> finger centre
    finger_max: float = 0.055                  # each finger's travel; inner opening = 2 q
    finger_len: float = 0.07
    finger_h: float = 0.04
    finger_t: float = 0.012
    base_z: float = 0.06                       # base_link above the floor
    lidar_z: float = 0.07                      # LiDAR scan plane in base_link
    limits: tuple = ((-1.9, 1.9), (-2.7, 1.6), (-2.8, 2.8), (-2.8, 2.8), (-1.7, 1.7))
    grasp_x: float = 0.42                      # nominal object distance ahead of base_link at grasp time
    stow: tuple = (0.14, 0.0, 0.25)            # finger centre (base_link) when folded / carrying

    @property
    def shoulder(self):
        x, y, z = self.turret_xyz
        return np.array([x, y, z + self.shoulder_dz])


def _gripper_axes(psi, theta, qy):
    """Gripper forward (finger direction) and side (finger closing direction) unit vectors in base_link:
    Rz(psi) Ry(theta) Rz(qy) applied to x and y, theta = sum of the pitch joint angles."""
    ct, st, cy, sy, cp, sp = math.cos(theta), math.sin(theta), math.cos(qy), math.sin(qy), math.cos(psi), math.sin(psi)
    f = np.array([ct * cy, sy, -st * cy])
    s = np.array([-ct * sy, cy, st * sy])
    R = np.array([[cp, -sp, 0.0], [sp, cp, 0.0], [0.0, 0.0, 1.0]])
    return R @ f, R @ s


def fk(geo: ArmGeometry, q):
    """Joint angles (turret, shoulder, elbow, wrist pitch[, wrist yaw]) -> dict of base_link points: shoulder, elbow,
    wrist, tip (finger centre), gripper pitch (rad, 0 = horizontal), heading (yaw of the finger direction), and the
    gripper's forward / side unit vectors."""
    psi, qs, qe, qw = q[:4]
    qy = q[4] if len(q) > 4 else 0.0
    e1, e2, e3 = -qs, -qe, -qw
    sh = geo.shoulder
    def pt(r, z):                                   # noqa: E306
        return sh + np.array([r * math.cos(psi), r * math.sin(psi), z])
    er, ez = geo.l1 * math.cos(e1), geo.l1 * math.sin(e1)
    wr, wz = er + geo.l2 * math.cos(e1 + e2), ez + geo.l2 * math.sin(e1 + e2)
    phi = e1 + e2 + e3
    fwd, side = _gripper_axes(psi, qs + qe + qw, qy)
    wrist = pt(wr, wz)
    return dict(shoulder=sh.copy(), elbow=pt(er, ez), wrist=wrist, tip=wrist + geo.g * fwd, pitch=phi, yaw=psi,
                heading=psi + qy, fwd=fwd, side=side)


def radial_heading(geo: ArmGeometry, target):
    sh = geo.shoulder
    return math.atan2(target[1] - sh[1], target[0] - sh[0])


def ik(geo: ArmGeometry, target, heading=None):
    """Finger-centre target (base_link x, y, z) with a HORIZONTAL gripper whose fingers point along `heading`
    (base_link yaw; None = straight out from the turret) -> joint angles, or None if unreachable or outside the joint
    limits. Elbow-up solution (the elbow stays above the line shoulder-wrist)."""
    if heading is None:
        heading = radial_heading(geo, target)
    sh = geo.shoulder
    wx, wy = target[0] - geo.g * math.cos(heading), target[1] - geo.g * math.sin(heading)
    dx, dy, wz = wx - sh[0], wy - sh[1], target[2] - sh[2]
    psi = math.atan2(dy, dx)
    wr = math.hypot(dx, dy)
    D = (wr * wr + wz * wz - geo.l1 ** 2 - geo.l2 ** 2) / (2 * geo.l1 * geo.l2)
    if abs(D) > 1.0:
        return None
    e2 = -math.acos(D)
    e1 = math.atan2(wz, wr) - math.atan2(geo.l2 * math.sin(e2), geo.l1 + geo.l2 * math.cos(e2))
    e3 = -e1 - e2
    qy = math.atan2(math.sin(heading - psi), math.cos(heading - psi))
    q = (psi, -e1, -e2, -e3, qy)
    for v, (lo, hi) in zip(q, geo.limits):
        if not (lo - 1e-9 <= v <= hi + 1e-9):
            return None
    return q


def grasp_height(geo: ArmGeometry, obj_height):
    """Finger-centre height above the floor for an object of this height (fingers clear the floor by 5 mm)."""
    return max(obj_height / 2, geo.finger_h / 2 + 0.005)


def segments(geo: ArmGeometry, q, finger_q=0.0):
    """Arm geometry as base_link line segments with a half-thickness, for the LiDAR self-filter."""
    p = fk(geo, q)
    out = [(p['shoulder'], p['elbow'], 0.015), (p['elbow'], p['wrist'], 0.015)]
    fwd, side = p['fwd'], p['side']
    a = p['wrist'] + fwd * (geo.g - geo.finger_len / 2)
    b = p['wrist'] + fwd * (geo.g + geo.finger_len / 2)
    off = finger_q + geo.finger_t / 2
    out.append((p['wrist'], a, 0.03))                                     # palm
    out.append((a + side * off, b + side * off, max(geo.finger_h / 2, geo.finger_t)))
    out.append((a - side * off, b - side * off, max(geo.finger_h / 2, geo.finger_t)))
    return out


def crosses_scan_plane(geo: ArmGeometry, q, finger_q=0.0, held=None):
    """True if any part of the arm (or the held object, dict(height) hanging between the fingers) reaches the LiDAR
    scan plane further than the LiDAR's minimum range (0.12 m)."""
    lid = np.array([0.05, 0.0, geo.lidar_z])
    for a, b, t in segments(geo, q, finger_q):
        zlo, zhi = min(a[2], b[2]) - t, max(a[2], b[2]) + t
        if zlo <= geo.lidar_z <= zhi and max(np.linalg.norm((a - lid)[:2]), np.linalg.norm((b - lid)[:2])) > 0.12:
            return True
    if held is not None:
        tip = fk(geo, q)['tip']
        if tip[2] - held['height'] / 2 <= geo.lidar_z <= tip[2] + held['height'] / 2:
            return True
    return False


def scan_self_mask(geo: ArmGeometry, q, finger_q, angles, ranges, held=None, lidar_xy=(0.05, 0.0), margin=0.03):
    """Boolean mask of LiDAR beams (angles in the LiDAR/base frame) that end on the robot's own arm or held object.

    Every arm segment that crosses the scan plane occupies a small region of the plane; beams whose direction falls in
    that region's angular sector and whose range does not exceed its far side (+ margin) are the robot seeing itself.
    """
    angles = np.asarray(angles, float)
    ranges = np.asarray(ranges, float)
    mask = np.zeros(len(angles), bool)
    blobs = []
    for a, b, t in segments(geo, q, finger_q):
        zlo, zhi = min(a[2], b[2]) - t, max(a[2], b[2]) + t
        if not (zlo <= geo.lidar_z <= zhi):
            continue
        if abs(b[2] - a[2]) > 1e-6:          # crossing point of the segment axis with the plane (clipped)
            s = np.clip((geo.lidar_z - a[2]) / (b[2] - a[2]), 0, 1)
            pts = [a + s * (b - a)]
            if t > 0.02:                      # thick parts lie in the plane over a stretch
                pts += [a, b]
        else:
            pts = [a, b]
        for pnt in pts:
            blobs.append((pnt[0], pnt[1], t + margin))
    if held is not None:
        tip = fk(geo, q)['tip']
        if tip[2] - held['height'] / 2 <= geo.lidar_z <= tip[2] + held['height'] / 2:
            blobs.append((tip[0], tip[1], max(held['length'], held['width']) / 2 + margin))
    for bx, by, rad in blobs:
        dx, dy = bx - lidar_xy[0], by - lidar_xy[1]
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            continue
        half = math.asin(min(1.0, rad / dist))
        dang = np.abs(np.arctan2(np.sin(angles - math.atan2(dy, dx)), np.cos(angles - math.atan2(dy, dx))))
        mask |= (dang <= half) & ~(ranges > dist + rad)       # inf/nan ranges compare False -> masked too
    return mask
