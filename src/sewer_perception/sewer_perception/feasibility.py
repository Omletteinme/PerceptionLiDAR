"""Removal feasibility of a debris object for the simulated arm/gripper, and its obstruction of the corridor.

CUSTOM code (Account 2), pure Python/NumPy. The limits come from the arm design in
sewer_description/urdf/sewer_robot_inspection.urdf.xacro (see ACCOUNT2_README.md, "Arm design"):
  - two fingers, inner opening 0..0.11 m; grasp width must leave 1 cm each side          -> width <= 0.09 m
  - finger pads 0.04 m tall, centred at max(h/2, 0.025) above the floor                   -> 0.03 <= h <= 0.15 m
  - payload 0.5 kg; mass is NOT observable, it is ESTIMATED from the measured volume with an assumed density of
    600 kg/m^3 (wood/rubble-like; documented assumption, a real system would weigh or test-lift)
  - arm reach: object centre 0.25-0.60 m ahead of base_link, up to 0.3 m to the side, or beside the robot (turret +-1.9 rad)
  - carried object must stay inside the robot's swept circle in a 1 m corridor                -> length <= 0.32 m
  - the size must come from a near view (camera within 1.2 m): seen from far, only the front face is visible
  - and from more than one direction (viewing directions spanning >= 20 deg): end-on, an object's length is hidden
A target is 'graspable' only when every check passes; otherwise the reasons are listed. The Mission Manager
(Account 1) decides whether and when to pick anything up.
"""
import math
from dataclasses import dataclass

import numpy as np

MIN_PASSABLE_GAP = 0.50      # same blockage definition as gen_network.py / blockage.py


@dataclass
class GripperLimits:
    max_width: float = 0.09
    min_width: float = 0.015
    min_height: float = 0.03
    max_height: float = 0.15
    payload: float = 0.5
    density: float = 600.0
    max_carry_length: float = 0.32
    grasp_x: float = 0.42              # nominal object centre ahead of base_link at grasp time (arm reach 0.25-0.60 m)
    approach_clear: float = 0.18       # robot centre at the approach pose must be this far from a mapped wall
    side_clear: float = 0.03           # free space beside the object for a finger
    min_confidence: float = 0.5
    min_frames: int = 3
    near_view: float = 1.2             # graspable only once seen from this close: from far away the low camera sees
                                       # only an object's front face and under-estimates its depth
    min_view_spread: float = 0.35      # [rad] ~20 deg: seen from one direction only, the extent along the line of
                                       # sight is hidden (a plank seen end-on looks short and light)


def estimated_mass(length, width, height, shape, density):
    vol = length * width * height
    if shape == 'round':
        vol *= math.pi / 4
    return vol * density


def approach_poses(obj, prior, lim: GripperLimits):
    """Map-frame poses (x, y, yaw) where the robot should stop so the object sits at the grasp point with its narrow
    side across the fingers. Only poses clear of the a-priori walls are returned, best first (along a corridor)."""
    x, y, yaw = obj['x'], obj['y'], obj['yaw']
    L, W = obj['length'], obj['width']
    dirs = []
    if W <= lim.max_width:
        dirs += [yaw, yaw + math.pi]              # approach along the long axis: fingers squeeze the width
    if L <= lim.max_width:
        dirs += [yaw + math.pi / 2, yaw - math.pi / 2]
    out = []
    for a in dirs:
        a = math.atan2(math.sin(a), math.cos(a))
        px, py = x - lim.grasp_x * math.cos(a), y - lim.grasp_x * math.sin(a)
        clear = float(prior.dist(np.array([px]), np.array([py]))[0])
        if clear >= lim.approach_clear:
            axis_err = min(abs(math.sin(a)), abs(math.cos(a)))        # 0 when along x or y (the corridor axes)
            out.append((axis_err, -clear, dict(x=round(px, 3), y=round(py, 3), yaw=round(a, 3))))
    out.sort(key=lambda t: t[:2])
    return [t[2] for t in out]


def assess(obj, prior, lim=None):
    """obj: dict(x, y, yaw, length, width, height, shape, confidence, n_frames). Returns the feasibility dict."""
    lim = lim or GripperLimits()
    reasons = []
    if obj['confidence'] < lim.min_confidence or obj['n_frames'] < lim.min_frames:
        reasons.append('low_confidence')
    if obj.get('min_range', 0.0) > lim.near_view:
        reasons.append(f'not_seen_up_close (closest view {obj["min_range"]:.1f} m)')
    if obj.get('view_spread', math.pi) < lim.min_view_spread:
        reasons.append(f'seen_from_one_direction_only ({math.degrees(obj["view_spread"]):.0f} deg)')
    W, L, H = obj['width'], obj['length'], obj['height']
    if W > lim.max_width:
        reasons.append(f'too_wide ({W:.3f} > {lim.max_width} m)')
    if W < lim.min_width:
        reasons.append('too_thin')
    if H > lim.max_height:
        reasons.append(f'too_tall ({H:.3f} > {lim.max_height} m)')
    if H < lim.min_height:
        reasons.append('too_flat')
    if L > lim.max_carry_length:
        reasons.append(f'too_long_to_carry ({L:.3f} > {lim.max_carry_length} m)')
    mass = estimated_mass(L, W, H, obj.get('shape', 'box'), lim.density)
    if mass > lim.payload:
        reasons.append(f'too_heavy_estimated ({mass:.2f} > {lim.payload} kg)')
    wall_gap = float(prior.dist(np.array([obj['x']]), np.array([obj['y']]))[0]) - max(L, W) / 2
    if wall_gap < lim.side_clear:
        reasons.append('against_wall')
    poses = approach_poses(obj, prior, lim) if W <= lim.max_width else []
    if W <= lim.max_width and not poses:
        reasons.append('no_safe_approach_pose')
    return dict(graspable=not reasons, reasons=reasons, grasp_width=round(W, 3), est_mass_kg=round(mass, 3),
                approach_poses=poses)


def corridor_of(x, y, nodes, edges, half_w=0.5):
    """The graph edge whose corridor contains (x, y): (name 'A-B', s along A->B, d lateral, unit a, unit n)."""
    best = None
    for a, b in edges:
        (ax, ay), (bx, by) = nodes[a], nodes[b]
        length = math.hypot(bx - ax, by - ay)
        ux, uy = (bx - ax) / length, (by - ay) / length
        s = (x - ax) * ux + (y - ay) * uy
        d = -(x - ax) * uy + (y - ay) * ux
        out_s = max(0.0, -s, s - length)
        score = out_s + max(0.0, abs(d) - half_w)
        if best is None or score < best[0]:
            best = (score, f'{a}-{b}', s, d, (ux, uy), (-uy, ux))
    if best is None or best[0] > 0.3:
        return None
    return best[1:]


def obstruction(obj, nodes, edges, half_w=0.5):
    """How much of its corridor the object closes: lateral occupancy, remaining free gap and a severity label."""
    c = corridor_of(obj['x'], obj['y'], nodes, edges, half_w)
    if c is None:
        return dict(edge=None, free_gap=None, lateral_occupancy=None, severity='unknown')
    name, s, d, a, n = c
    ca, sa = math.cos(obj['yaw']), math.sin(obj['yaw'])
    L, W = obj['length'] / 2, obj['width'] / 2
    corners = [(obj['x'] + ca * u - sa * v, obj['y'] + sa * u + ca * v) for u in (-L, L) for v in (-W, W)]
    ax, ay = nodes[name.split('-')[0]]
    lat = [-(px - ax) * a[1] + (py - ay) * a[0] for px, py in corners]
    lo, hi = max(min(lat), -half_w), min(max(lat), half_w)
    occ = max(0.0, hi - lo)
    gap = max(lo + half_w, half_w - hi) if occ > 0 else 2 * half_w
    sev = 'full' if gap < MIN_PASSABLE_GAP else ('partial' if occ >= 0.1 * 2 * half_w else 'minor')
    return dict(edge=name, free_gap=round(gap, 3), lateral_occupancy=round(occ / (2 * half_w), 3), severity=sev)
