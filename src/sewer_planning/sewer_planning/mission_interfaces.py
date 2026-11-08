"""Message contracts between the Mission Manager (Account 1) and perception / manipulation (Account 2).

CUSTOM code (ours), pure Python, no ROS imports. PROPOSED schemas, see docs/mission_interfaces.md: they follow the
repository convention of JSON in std_msgs/String (like edge_gaps, route_info, localization_health) so that no new
message package is needed before the lead freezes the interfaces. This module is the single place that parses and
validates them; the Mission Manager only ever sees the dataclasses below.

Rules enforced here (build plan, section 4):
  - every detection carries frame_id and stamp; only frame_id == 'map' is accepted (others are rejected, never
    compared with map coordinates);
  - objects have stable string ids;
  - estimates carry confidence / uncertainty;
  - detected / graspable / holding / released / disposal-verified are separate fields and states.
"""
import json
from dataclasses import asdict, dataclass, field

MAP_FRAME = 'map'
DEFECT_TYPES = ('crack', 'hole', 'deformation', 'damaged_joint')
MANIP_STATES = ('IDLE', 'APPROACHING', 'GRASPING', 'HOLDING', 'RELEASING', 'RELEASED', 'FAILED')
TASK_TYPES = ('NONE', 'EXPLORE', 'NAVIGATE', 'PICKUP', 'RETURN', 'DISPOSE', 'STOP')


class SchemaError(ValueError):
    pass


def _num(d, k, default=None):
    v = d.get(k, default)
    if v is None:
        if default is None:
            raise SchemaError(f'missing field {k!r}')
        return default
    try:
        return float(v)
    except (TypeError, ValueError):
        raise SchemaError(f'field {k!r} is not a number: {v!r}') from None


def _frame(d):
    f = d.get('frame_id')
    if f is None:
        raise SchemaError('missing frame_id')
    if f != MAP_FRAME:
        raise SchemaError(f'frame_id {f!r} is not {MAP_FRAME!r}: transform detections into the map frame first')
    return f


# ---------------------------------------------------------------------- perception -> mission (Account 2 -> 1)
@dataclass
class Debris:
    """One debris object (/perception/debris). Map frame. Dimensions are ESTIMATES with uncertainty."""
    id: str
    x: float
    y: float
    size: tuple                    # (length, width, height) [m]
    confidence: float              # 0..1, detection confidence
    stamp: float                   # time of the latest observation [s, sim time]
    z: float = 0.0
    yaw: float = 0.0
    size_sigma: tuple = (0.0, 0.0, 0.0)
    pos_sigma: float = 0.0
    category: str = 'unknown'
    obstruction: float = 0.0       # 0 = no obstruction ... 1 = closes the corridor (perception's estimate)
    graspable: object = None       # True / False from perception's own gripper check, None = not assessed
    grasp_score: object = None     # optional 0..1 grasp quality from perception
    evidence: str = ''

    @property
    def max_dim(self):
        return max(self.size[0], self.size[1])


@dataclass
class Defect:
    """One structural defect (/perception/defects). A reporting item, never a task."""
    id: str
    type: str
    x: float
    y: float
    severity: float                # 0..1
    confidence: float
    stamp: float
    extent: float = 0.0            # [m] along the wall
    evidence: str = ''             # image/frame reference


def parse_debris(text):
    """JSON {frame_id, stamp, objects: [...]} -> (list[Debris], list of per-object error strings)."""
    msg = json.loads(text)
    _frame(msg)
    stamp = _num(msg, 'stamp')
    out, errors = [], []
    for o in msg.get('objects', []):
        try:
            if 'frame_id' in o:
                _frame(o)
            size = tuple(float(v) for v in o['size'])
            if len(size) != 3 or min(size) < 0:
                raise SchemaError('size must be [length, width, height] >= 0')
            conf = _num(o, 'confidence')
            if not 0.0 <= conf <= 1.0:
                raise SchemaError('confidence outside [0, 1]')
            g = o.get('graspable')
            out.append(Debris(id=str(o['id']), x=_num(o, 'x'), y=_num(o, 'y'), size=size, confidence=conf,
                              stamp=_num(o, 'stamp', stamp), z=_num(o, 'z', 0.0), yaw=_num(o, 'yaw', 0.0),
                              size_sigma=tuple(float(v) for v in o.get('size_sigma', (0, 0, 0))),
                              pos_sigma=_num(o, 'pos_sigma', 0.0), category=str(o.get('category', 'unknown')),
                              obstruction=min(max(_num(o, 'obstruction', 0.0), 0.0), 1.0),
                              graspable=None if g is None else bool(g),
                              grasp_score=None if o.get('grasp_score') is None else _num(o, 'grasp_score'),
                              evidence=str(o.get('evidence', ''))))
        except (KeyError, SchemaError, TypeError, ValueError) as e:
            errors.append(f'debris {o.get("id", "?")}: {e}')
    return out, errors


def parse_defects(text):
    msg = json.loads(text)
    _frame(msg)
    stamp = _num(msg, 'stamp')
    out, errors = [], []
    for o in msg.get('defects', []):
        try:
            if 'frame_id' in o:
                _frame(o)
            t = str(o['type'])
            if t not in DEFECT_TYPES:
                raise SchemaError(f'unknown defect type {t!r} (expected one of {DEFECT_TYPES})')
            out.append(Defect(id=str(o['id']), type=t, x=_num(o, 'x'), y=_num(o, 'y'),
                              severity=min(max(_num(o, 'severity'), 0.0), 1.0), confidence=_num(o, 'confidence'),
                              stamp=_num(o, 'stamp', stamp), extent=_num(o, 'extent', 0.0),
                              evidence=str(o.get('evidence', ''))))
        except (KeyError, SchemaError, TypeError, ValueError) as e:
            errors.append(f'defect {o.get("id", "?")}: {e}')
    return out, errors


# ---------------------------------------------------------------------- manipulation -> mission (Account 2 -> 1)
@dataclass
class ManipulationStatus:
    """/manipulation/status. `verified` must come from simulated contact / attachment / lift evidence, never from a
    command acknowledgement."""
    state: str
    stamp: float
    object_id: str = ''
    verified: bool = False
    reason: str = ''
    task_id: str = ''


@dataclass
class DisposalStatus:
    """/disposal/status."""
    stamp: float
    object_id: str = ''
    station_reached: bool = False
    released: bool = False
    verified: bool = False         # object is no longer held AND lies inside the disposal zone
    reason: str = ''
    task_id: str = ''


def parse_manipulation(text):
    d = json.loads(text)
    s = str(d.get('state', ''))
    if s not in MANIP_STATES:
        raise SchemaError(f'unknown manipulation state {s!r}')
    return ManipulationStatus(state=s, stamp=_num(d, 'stamp'), object_id=str(d.get('object_id', '')),
                              verified=bool(d.get('verified', False)), reason=str(d.get('reason', '')),
                              task_id=str(d.get('task_id', '')))


def parse_disposal(text):
    d = json.loads(text)
    return DisposalStatus(stamp=_num(d, 'stamp'), object_id=str(d.get('object_id', '')),
                          station_reached=bool(d.get('station_reached', False)),
                          released=bool(d.get('released', False)), verified=bool(d.get('verified', False)),
                          reason=str(d.get('reason', '')), task_id=str(d.get('task_id', '')))


# ---------------------------------------------------------------------- mission -> everyone (Account 1 -> 2)
@dataclass
class MissionTask:
    """/mission/task. The only way the mission asks manipulation to act. task_id is unique per request (retries get
    a new id) so a status can be matched to the request it answers."""
    task_id: str
    type: str
    stamp: float
    object_id: str = ''
    target: dict = field(default_factory=dict)       # {x, y, yaw, frame_id}: object pose or station pose
    action: str = ''                                 # e.g. 'grasp_and_lift', 'release_in_zone', 'stop'

    def to_json(self):
        if self.type not in TASK_TYPES:
            raise SchemaError(f'unknown task type {self.type!r}')
        return json.dumps(asdict(self))


def debris_to_dict(d):
    return asdict(d)
