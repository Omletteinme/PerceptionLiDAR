"""Routes to an arbitrary point on a corridor (not only to graph nodes), e.g. a stand-off point in front of debris.

CUSTOM code (ours), pure NumPy. Built on the existing RouteModel (route_cost.py): the route to one of the corridor's end
nodes is RouteModel.plan (uncertainty-aware cost, blocked edges removed); the last leg along the corridor is costed with
RouteModel.leg and must not cross a detected blockage.
"""
import math

import numpy as np

from .graph import Graph


def leg_crosses_blockage(status, edge, s0, s1, tol=0.05):
    st = status.get(Graph.key(edge))
    if not st or not st.get('blocked') or st.get('s_min') is None:
        return False
    lo, hi = min(s0, s1), max(s0, s1)
    return hi > st['s_min'] - tol and lo < st['s_max'] + tol


def plan_to_point(model, loc, status, R, P0, edge, s_t):
    """Cheapest way from the robot (loc = Graph.locate result) to s_t on `edge`.

    Returns dict(legs, cost, entry) or None if the point cannot be reached without crossing a blockage.
    """
    g, w_unc, best = model.g, model.p.w_unc, None
    if loc[0] == 'edge' and loc[1] == edge:
        if not leg_crosses_blockage(status, edge, loc[2], s_t):
            _, l, u, nw, _ = model.leg(edge, loc[2], s_t, P0, R, status)
            best = dict(legs=[(edge, loc[2], s_t)], cost=l + w_unc * u + nw, entry=None)
    L = g.length(edge)
    for entry, s_in in ((edge[0], 0.0), (edge[1], L)):
        if leg_crosses_blockage(status, edge, s_in, s_t):
            continue
        plans = model.plan(loc, status, R, P0, entry)
        if not plans:
            continue
        p = plans[0]
        _, l, u, nw, _ = model.leg(edge, s_in, s_t, (p['sigma_end'] ** 2) * np.eye(2), R, status)
        cost = p['total'] + l + w_unc * u + nw
        if best is None or cost < best['cost']:
            best = dict(legs=p['legs'] + [(edge, s_in, s_t)], cost=cost, entry=entry)
    return best


def approach_points(graph, x, y, half_len, standoff, max_offset=0.7):
    """Stand-off points from which the robot can face an object at (x, y): [(edge, s_approach, s_object, yaw)].

    One per side of the object on its corridor (or one per incident corridor if the object lies at a node). The robot
    stops `standoff` metres before the object's near face, on the centre line, facing it. Objects farther than
    `max_offset` from every corridor centre line give no points.
    """
    loc = graph.locate(x, y)
    out = []
    if loc[0] == 'node':
        n = loc[1]
        for v in graph.neighbours(n):
            e, fwd = graph.edge_between(n, v)
            L = graph.length(e)
            s_obj = 0.0 if fwd else L
            s_a = s_obj + (half_len + standoff) * (1 if fwd else -1)
            if 0.0 <= s_a <= L:
                d = graph.direction(e) * (-1 if fwd else 1)
                out.append((e, s_a, s_obj, math.atan2(d[1], d[0])))
        return out
    _, e, s_obj, d = loc
    if abs(d) > max_offset:
        return out
    L, a = graph.length(e), graph.direction(e)
    s_lo, s_hi = s_obj - half_len - standoff, s_obj + half_len + standoff
    if s_lo >= -0.05:
        out.append((e, max(s_lo, 0.0), s_obj, math.atan2(a[1], a[0])))
    if s_hi <= L + 0.05:
        out.append((e, min(s_hi, L), s_obj, math.atan2(-a[1], -a[0])))
    return out
