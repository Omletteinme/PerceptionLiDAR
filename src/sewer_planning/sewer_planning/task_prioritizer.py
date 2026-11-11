"""Debris task prioritization: the largest FEASIBLE object, not the largest object at any cost.

CUSTOM code (ours), pure NumPy. Two stages (build plan, section 6):

1. Hard feasibility filters, each with a recorded rejection reason:
     stale       not observed for `stale_after` seconds
     confidence  detection confidence below `min_confidence`
     ungraspable perception says not graspable, or the estimated size (plus its uncertainty) exceeds the gripper limits
     attempts    pickup already failed `max_attempts` times
     unreachable no route to any stand-off point in front of the object
     no_return   no route from the stand-off point back to the disposal station
     carry       the carried object would not fit through the narrowest corridor gap on the way back
2. Ranking of the survivors with the tunable heuristic
     priority = w_size * size + w_obstruction * obstruction + w_access * accessibility + w_grasp * graspability
   every term normalised to [0, 1]:
     size          estimated volume / volume of the largest FEASIBLE candidate currently known
     obstruction   perception's obstruction estimate (0 none ... 1 closes the corridor)
     accessibility exp(-route cost / access_scale)            (route cost from the uncertainty-aware RouteModel)
     graspability  perception's grasp score if given, else the width margin inside the gripper, times confidence
   Default weights 0.4 / 0.3 / 0.2 / 0.1: size first (the brief), then how much the object obstructs the flow, then how
   cheap it is to reach. This is a proposed baseline heuristic, not a proven optimal policy.

Gripper limits: Account 2's real arm (docs/account2_interface_review.md): 0.11 m opening, 0.32 m length, 0.15 m height.
"""
import math
from dataclasses import dataclass

import numpy as np

from .point_route import approach_points, plan_to_point


@dataclass
class GripperLimits:
    max_width: float = 0.11      # finger opening [m] (the object's smaller horizontal dimension must fit)
    max_length: float = 0.32     # longest object carried folded over the robot [m]
    max_height: float = 0.15     # [m]
    robot_width: float = 0.30    # chassis + wheels [m]


@dataclass
class PriorityParams:
    w_size: float = 0.4
    w_obstruction: float = 0.3
    w_access: float = 0.2
    w_grasp: float = 0.1
    min_confidence: float = 0.5
    stale_after: float = 30.0     # [s]
    max_attempts: int = 2
    access_scale: float = 20.0    # [m of route cost]
    standoff: float = 0.25        # stop this far before the object's near face [m] (the follower stops within 0.2 m
                                  # of its goal, so the object ends up 0.25-0.45 m ahead: inside a 0.65 m arm reach)
    carry_clearance: float = 0.05 # per side, when carrying through a narrow gap [m]


class TaskPrioritizer:
    def __init__(self, model, limits=None, params=None):
        self.model = model
        self.lim = limits or GripperLimits()
        self.p = params or PriorityParams()

    def _size_check(self, d):
        """None if the size fits the gripper (estimate + 1 sigma), else the reason."""
        lim, sz, sg = self.lim, d.size, d.size_sigma
        horiz = sorted([sz[0] + sg[0], sz[1] + sg[1]])
        if horiz[0] > lim.max_width:
            return f'too wide for the gripper ({horiz[0]:.2f} m > {lim.max_width:.2f} m)'
        if horiz[1] > lim.max_length:
            return f'too long to carry ({horiz[1]:.2f} m > {lim.max_length:.2f} m)'
        if sz[2] + sg[2] > lim.max_height:
            return f'too tall ({sz[2] + sg[2]:.2f} m > {lim.max_height:.2f} m)'
        return None

    def carry_width(self, d):
        """Free gap the robot needs while carrying debris d: chassis or object (held across the front), plus clearance."""
        return max(self.lim.robot_width, sorted(d.size[:2])[0]) + 2 * self.p.carry_clearance

    def _min_gap(self, legs, status):
        gaps = [status[k]['gap'] for k in {self.model.g.key(e) for e, _, _ in legs}
                if k in status and not status[k].get('blocked') and status[k].get('s_min') is not None]
        return min(gaps) if gaps else math.inf

    def evaluate(self, debris, loc, status, R, P0, now, home, attempts=None, done=()):
        """Rank the debris. Returns dict(ranked=[...], rejected=[...]), ranked best first.

        ranked item: dict(id, score, terms{size, obstruction, accessibility, graspability}, approach{edge, s, yaw, xy},
                          legs, route_cost, debris)
        rejected item: dict(id, reason, detail)
        """
        p, lim, g = self.p, self.lim, self.model.g
        attempts = attempts or {}
        ranked, rejected = [], []
        vmax = lim.max_width * lim.max_length * lim.max_height
        for d in debris:
            if d.id in done:
                continue
            rej = None
            if now - d.stamp > p.stale_after:
                rej = ('stale', f'last seen {now - d.stamp:.0f} s ago')
            elif d.confidence < p.min_confidence:
                rej = ('confidence', f'{d.confidence:.2f} < {p.min_confidence:.2f}')
            elif d.graspable is False:
                rej = ('ungraspable', 'perception reports it cannot be grasped')
            elif self._size_check(d):
                rej = ('ungraspable', self._size_check(d))
            elif attempts.get(d.id, 0) >= p.max_attempts:
                rej = ('attempts', f'pickup failed {attempts[d.id]} times')
            if rej:
                rejected.append(dict(id=d.id, reason=rej[0], detail=rej[1]))
                continue
            best = None
            for e, s_a, s_obj, yaw in approach_points(g, d.x, d.y, 0.5 * d.max_dim, p.standoff):
                r = plan_to_point(self.model, loc, status, R, P0, e, s_a)
                if r and (best is None or r['cost'] < best[0]['cost']):
                    best = (r, e, s_a, yaw)
            if best is None:
                rejected.append(dict(id=d.id, reason='unreachable', detail='no route to a stand-off point'))
                continue
            r, e, s_a, yaw = best
            back = self.model.plan(('edge', e, s_a, 0.0), status, R, self.model.initial_covariance(), home)
            if not back:
                rejected.append(dict(id=d.id, reason='no_return', detail=f'no route from it back to {home}'))
                continue
            need = self.carry_width(d)
            back = self.model.plan(('edge', e, s_a, 0.0), carry_status(status, need), R,
                                   self.model.initial_covariance(), home)
            if not back:
                gap = self._min_gap(self.model.plan(('edge', e, s_a, 0.0), status, R, self.model.initial_covariance(),
                                                    home)[0]['legs'], status)
                rejected.append(dict(id=d.id, reason='carry', detail=f'needs {need:.2f} m of free gap, every route '
                                                                     f'back is narrower (best {gap:.2f} m)'))
                continue
            w = sorted(d.size[:2])[0]
            grasp = d.grasp_score if d.grasp_score is not None else max(0.0, 1.0 - w / lim.max_width)
            xy = g.point(e, s_a)
            ranked.append(dict(id=d.id, volume=d.size[0] * d.size[1] * d.size[2],
                               terms=dict(obstruction=d.obstruction, accessibility=math.exp(-r['cost'] / p.access_scale),
                                          graspability=min(1.0, max(0.0, grasp)) * d.confidence),
                               approach=dict(edge=g.key(e), s=round(s_a, 3), yaw=round(yaw, 3),
                                             xy=[round(float(xy[0]), 3), round(float(xy[1]), 3)]),
                               legs=r['legs'], route_cost=r['cost'], debris=d,
                               edge=e, s=s_a, return_cost=back[0]['total']))
        # size is relative to the largest FEASIBLE candidate (absolute volumes of graspable objects are all tiny next
        # to the gripper limit, which would let distance decide everything)
        vbest = max([c['volume'] for c in ranked], default=0.0) or vmax
        for c in ranked:
            t = c['terms']
            t['size'] = c['volume'] / vbest
            c['score'] = (p.w_size * t['size'] + p.w_obstruction * t['obstruction'] +
                          p.w_access * t['accessibility'] + p.w_grasp * t['graspability'])
            c['terms'] = {k: round(v, 3) for k, v in t.items()}
        ranked.sort(key=lambda c: -c['score'])
        return dict(ranked=ranked, rejected=rejected)


def carry_status(status, width):
    """Edge status as seen by a robot that needs `width` metres of free gap (e.g. while carrying an object).

    Corridors whose detected free gap is narrower are reported as blocked (keeping the obstruction's extent), so the
    existing RouteModel routes around them exactly as it routes around full blockages.
    """
    out = {}
    for k, st in (status or {}).items():
        if not st.get('blocked') and st.get('s_min') is not None and st.get('gap', 1e9) < width:
            st = dict(st, blocked=True, carry_blocked=True)
        out[k] = st
    return out


def approach_xy(graph, item):
    return np.asarray(graph.point(item['edge'], item['s']), float)
