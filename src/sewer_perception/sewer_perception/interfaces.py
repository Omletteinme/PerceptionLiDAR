"""Account 2 side of the message contracts in docs/mission_interfaces.md (JSON on std_msgs/String).

CUSTOM code (Account 2), pure Python. The schemas are the ones Account 1 proposed and parses in
sewer_planning/mission_interfaces.py; this module BUILDS them (perception and manipulation outputs) and parses
/mission/task. The required fields are exactly the documented ones; Account 2 adds a few extra fields (marked
'extra' below) with details the Mission Manager may ignore: the reasons an object is not graspable, approach poses,
the severity label, the verification evidence. The compatibility test (sandbox/tests/test_inspection.py) runs every
message through Account 1's own parser.

  /perception/debris      Account 2 -> 1
  /perception/defects     Account 2 -> 1
  /manipulation/status    Account 2 -> 1   state IDLE / APPROACHING / GRASPING / HOLDING / RELEASING / RELEASED / FAILED
  /disposal/status        Account 2 -> 1
  /mission/task           Account 1 -> 2   type PICKUP / DISPOSE / STOP (others are not for the manipulator)

Distinct states, as the build plan requires: detected (in /perception/debris), graspable (field), held (HOLDING +
verified), released (RELEASED, disposal 'released'), disposal verified (disposal 'verified'). Nothing here commands
motion.
"""
import json

MAP_FRAME = 'map'
MANIP_STATES = ('IDLE', 'APPROACHING', 'GRASPING', 'HOLDING', 'RELEASING', 'RELEASED', 'FAILED')
TASK_TYPES = ('NONE', 'EXPLORE', 'NAVIGATE', 'PICKUP', 'RETURN', 'DISPOSE', 'STOP')
ARM_TASKS = {'PICKUP': 'pickup', 'DISPOSE': 'release', 'STOP': 'stop'}
DEFECT_TYPES = ('crack', 'hole', 'deformation', 'damaged_joint')
# controller state (manipulation_controller.py) -> documented state. 'lifting' is still part of the grasp: the
# documented HOLDING (with verified) is only reported once the lift has been verified.
STATE_MAP = {'idle': 'IDLE', 'approaching': 'APPROACHING', 'grasping': 'GRASPING', 'lifting': 'GRASPING',
             'holding': 'HOLDING', 'releasing': 'RELEASING', 'released': 'RELEASED', 'failed': 'FAILED'}


class SchemaError(ValueError):
    pass


def _r(v, n=3):
    return None if v is None else round(float(v), n)


# ------------------------------------------------------------------------------------------ perception
def category(g):
    """Size class from the measured footprint/height (documented rule, not a learned classifier)."""
    m = max(g['length'], g['width'])
    if m <= 0.12 and g['height'] <= 0.15:
        return 'small'
    if m <= 0.35:
        return 'medium'
    return 'large'


def obstruction_value(obstr):
    """Documented 0..1 obstruction: 1 = closes the corridor (free gap below the 0.50 m passable width),
    otherwise the fraction of the corridor width the object occupies."""
    if obstr.get('severity') == 'full':
        return 1.0
    occ = obstr.get('lateral_occupancy')
    return 0.0 if occ is None else float(min(max(occ, 0.0), 1.0))


def grasp_score(feas, width, sigma, max_width):
    """0..1: how comfortably the object fits the gripper (width margin); 0 when not graspable."""
    if not feas['graspable']:
        return 0.0
    return round(max(0.0, min(1.0, 1.0 - (width + sigma) / max_width)) * 0.8 + 0.2, 3)


def debris_object(track, feas, obstr, max_width=0.09, evidence=''):
    """One /perception/debris entry from an object_tracker.Track, its feasibility and obstruction dicts."""
    g = track.geometry()
    rb = track.rel_base
    lower = bool(g['height_is_lower_bound'])
    size_sigma = [0.015, 0.015, 0.10 if lower else 0.01]      # measured accuracy (sandbox evaluation, drift case)
    return dict(
        # documented fields
        id=track.id, x=_r(g['x']), y=_r(g['y']), z=_r(g['height'] / 2), yaw=_r(g['yaw']),
        size=[_r(g['length']), _r(g['width']), _r(g['height'])], size_sigma=size_sigma, pos_sigma=0.04,
        category=category(g), confidence=_r(min(1.0, track.confidence())),
        obstruction=_r(obstruction_value(obstr)), graspable=bool(feas['graspable']),
        grasp_score=grasp_score(feas, g['width'], size_sigma[1], max_width), stamp=_r(track.last_seen),
        evidence=evidence,
        # extra (Account 2 details)
        state=track.state, shape=g['shape'], height_is_lower_bound=lower,
        feasibility=dict(reasons=feas['reasons'], grasp_width=feas['grasp_width'], est_mass_kg=feas['est_mass_kg'],
                         approach_poses=feas['approach_poses']),
        obstruction_detail=obstr, n_frames=int(track.hits), first_seen=_r(track.first_seen), sources=sorted(track.sources),
        last_seen_base=None if rb is None else dict(x=_r(rb[0]), y=_r(rb[1]), stamp=_r(rb[2])))


def debris_msg(objects, stamp):
    return dict(frame_id=MAP_FRAME, stamp=_r(stamp), objects=objects)


def defect_item(d):
    """One /perception/defects entry from a defect_detector.DefectTracker item."""
    ev = d.get('evidence') or {}
    size = d['size']
    extent = size['length'] if d['type'] == 'crack' else size['width']
    return dict(
        # documented fields
        id=d['id'], type=d['type'], x=_r(d['x']), y=_r(d['y']), severity=_r(d['severity_score']),
        confidence=_r(min(1.0, d['confidence'])), extent=_r(extent), stamp=_r(d['last_seen']),
        evidence=ev.get('image') or '',
        # extra
        z=_r(d['z']), severity_label=d['severity'], size={k: _r(v, 4) for k, v in size.items()},
        wall_normal=list(d.get('normal') or (0.0, 0.0)), n_obs=int(d['n_obs']), first_seen=_r(d['first_seen']),
        edge=d.get('edge'), evidence_detail=dict(source=ev.get('source', d.get('source')),
                                                 frame_stamp=_r(ev.get('frame_stamp')), bbox=ev.get('bbox')))


def defects_msg(items, stamp):
    return dict(frame_id=MAP_FRAME, stamp=_r(stamp), defects=items)


# ------------------------------------------------------------------------------------------ manipulation
def manipulation_msg(ctl_state, stamp, task_id=None, object_id=None, verification=None, reason='',
                     arm_in_scan_plane=False):
    """ctl_state: manipulation_controller state (lower case). `verified` is true only from physical evidence:
    HOLDING -> grasp AND lift verified (contacts on both fingers, finger gap, lift); RELEASED -> release verified."""
    v = verification or {}
    state = STATE_MAP[ctl_state]
    if state == 'HOLDING':
        verified = bool(v.get('grasp_verified') and v.get('lift_verified'))
    elif state == 'RELEASED':
        verified = bool(v.get('release_verified'))
    else:
        verified = False
    return dict(state=state, stamp=_r(stamp), object_id=object_id or '', verified=verified, task_id=task_id or '',
                reason=reason or '',
                # extra
                detail_state=ctl_state, verification=v, arm_in_scan_plane=bool(arm_in_scan_plane))


def disposal_msg(stamp, task_id=None, object_id=None, station_reached=False, released=False, verified=False,
                 reason='', evidence=()):
    return dict(stamp=_r(stamp), object_id=object_id or '', task_id=task_id or '',
                station_reached=bool(station_reached), released=bool(released), verified=bool(verified),
                reason=reason or '', evidence=list(evidence))


def dumps(msg):
    return json.dumps(msg, separators=(',', ':'))


# ------------------------------------------------------------------------------------------ checks
def _need(d, keys, where):
    miss = [k for k in keys if k not in d]
    if miss:
        raise SchemaError(f'{where}: missing {miss}')


def validate_debris(m):
    _need(m, ('frame_id', 'stamp', 'objects'), 'debris')
    if m['frame_id'] != MAP_FRAME:
        raise SchemaError('debris: frame_id must be map')
    for o in m['objects']:
        _need(o, ('id', 'x', 'y', 'size', 'confidence', 'graspable', 'stamp'), f"debris {o.get('id')}")
        if len(o['size']) != 3 or min(o['size']) < 0 or not 0 <= o['confidence'] <= 1:
            raise SchemaError(f"debris {o['id']}: bad size/confidence")
        if o['graspable'] and o.get('feasibility', {}).get('reasons'):
            raise SchemaError(f"debris {o['id']}: graspable with rejection reasons")
    return True


def validate_defects(m):
    _need(m, ('frame_id', 'stamp', 'defects'), 'defects')
    if m['frame_id'] != MAP_FRAME:
        raise SchemaError('defects: frame_id must be map')
    for d in m['defects']:
        _need(d, ('id', 'type', 'x', 'y', 'severity', 'confidence', 'stamp'), f"defect {d.get('id')}")
        if d['type'] not in DEFECT_TYPES or not 0 <= d['severity'] <= 1:
            raise SchemaError(f"defect {d['id']}: bad type/severity")
    return True


def validate_manipulation(m):
    _need(m, ('state', 'stamp', 'object_id', 'verified', 'task_id', 'reason'), 'manipulation')
    if m['state'] not in MANIP_STATES:
        raise SchemaError(f"manipulation: bad state {m['state']}")
    return True


def validate_disposal(m):
    _need(m, ('stamp', 'object_id', 'station_reached', 'released', 'verified', 'reason'), 'disposal')
    if m['verified'] and not m['released']:
        raise SchemaError('disposal: verified without release')
    return True


def parse_task(text):
    """/mission/task (Account 1's MissionTask JSON) -> dict, or None when the task is not for the manipulator."""
    m = json.loads(text)
    _need(m, ('task_id', 'type'), 'task')
    typ = str(m['type']).upper()
    if typ not in TASK_TYPES:
        raise SchemaError(f'task: unknown type {m["type"]!r}')
    if typ not in ARM_TASKS:
        return None
    if typ == 'PICKUP' and not m.get('object_id'):
        raise SchemaError('task: PICKUP needs object_id')
    tgt = m.get('target') or {}
    if tgt and tgt.get('frame_id', MAP_FRAME) != MAP_FRAME:
        raise SchemaError('task: target must be in the map frame')
    return dict(task_id=str(m['task_id']), type=ARM_TASKS[typ], mission_type=typ, object_id=m.get('object_id') or '',
                stamp=m.get('stamp'), target=tgt, action=m.get('action', ''))
