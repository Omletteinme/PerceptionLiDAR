"""Uncertainty-aware route planning on the sewer graph.

CUSTOM code (ours), pure NumPy. This is how localization uncertainty enters the route cost.

Cost of a route = length
                + w_unc * integral of predicted position uncertainty sigma(s) ds
                + narrow-gap penalty for partly obstructed corridors.

sigma(s) comes from propagating a 2x2 position covariance P along the route, exactly like a Kalman
filter would:
  predict: P += q_along * u u^T + q_lat * n n^T per metre (odometry drift, mostly along the direction of travel)
  update : every `update_spacing` metres P <- (P^-1 + R * I(s))^-1, where I(s) is the information the PRIOR
           MAP gives at that spot (observability.py) and R in (0, 1] is the live scan reliability.
So a long featureless stretch lets uncertainty grow, a junction or bend pulls it back, and when the
sensor becomes unreliable (R falls) every stretch gets more uncertain, which raises its cost.
The starting covariance comes from the localizer (with a floor, because particle clouds are
overconfident). Blocked edges (free gap < min_gap) are removed. Routes are enumerated exhaustively: the
network is tiny and the cost depends on the whole route, not on single edges.
"""
import math
from dataclasses import dataclass

import numpy as np

from .observability import PROFILE_STEP, NormalField, edge_info_profile


@dataclass
class CostParams:
    q_along: float = 0.005        # m^2 of along-track variance per metre (about 1.5% odometry drift)
    q_lat: float = 0.0005         # m^2 per metre laterally (walls keep this small)
    update_spacing: float = 0.5   # metres between statistically independent scan updates
    ds: float = 0.1               # integration step [m]
    w_unc: float = 2.0            # cost [m] per m^2 of integrated sigma
    p0_floor: float = 0.05        # floor on the starting sigma [m]
    min_gap: float = 0.50         # corridors narrower than this are blocked
    narrow_gap: float = 0.80      # corridors narrower than this get a penalty
    w_narrow: float = 5.0         # cost [m] per metre of gap shortfall below narrow_gap
    shift_gap: float = 0.90       # drive off-centre when an obstruction narrows the corridor below this
    r_floor: float = 0.1          # lower bound on the reliability used for scaling


def sigma_eff(P):
    return math.sqrt(max(float(np.linalg.eigvalsh(P)[-1]), 0.0))


class RouteModel:
    def __init__(self, graph, omap, params=None):
        self.g = graph
        self.p = params or CostParams()
        nf = NormalField(omap)
        self.profiles = {e: edge_info_profile(omap, nf, graph.nodes[e[0]], graph.nodes[e[1]]) for e in graph.edges}

    # ------------------------------------------------------------------ cost of one leg
    def _info(self, edge, s):
        s_arr, info = self.profiles[edge]
        return info[min(max(int(round(s / PROFILE_STEP)), 0), len(s_arr) - 1)]

    def leg(self, edge, s0, s1, P, R, status):
        """Drive along `edge` from s0 to s1. Returns (P, length, integral of sigma, narrow-gap penalty, sigma trace)."""
        p, g = self.p, self.g
        dist = abs(s1 - s0)
        if dist < 1e-9:
            return P, 0.0, 0.0, 0.0, []
        sign = 1.0 if s1 > s0 else -1.0
        u = g.direction(edge) * sign
        n = np.array([-u[1], u[0]])
        drift = p.q_along * np.outer(u, u) + p.q_lat * np.outer(n, n)
        steps = max(1, int(math.ceil(dist / p.ds)))
        h = dist / steps
        Rs = min(max(R, p.r_floor), 1.0)
        U, since, trace = 0.0, 0.0, []
        for k in range(steps):
            s = s0 + sign * (k + 1) * h
            P = P + drift * h
            since += h
            if since >= p.update_spacing:
                P = np.linalg.inv(np.linalg.inv(P + 1e-12 * np.eye(2)) + Rs * self._info(edge, s))
                since = 0.0
            sg = sigma_eff(P)
            U += sg * h
            trace.append((s, sg))
        narrow = 0.0
        st = status.get(g.key(edge))
        if st and not st['blocked'] and st['gap'] < p.narrow_gap and st['s_min'] is not None:
            lo, hi = min(s0, s1), max(s0, s1)
            if hi >= st['s_min'] - 0.5 and lo <= st['s_max'] + 0.5:
                narrow = p.w_narrow * (p.narrow_gap - st['gap'])
        return P, dist, U, narrow, trace

    # ------------------------------------------------------------------ search
    def _blocked(self, edge, status):
        st = status.get(self.g.key(edge))
        return bool(st and st['blocked'])

    def first_options(self, loc, status):
        """Where the robot can go first: [(node reached, legs [(edge, s0, s1)], edge to exclude later)]."""
        if loc[0] == 'node':
            return [(loc[1], [], None)]
        _, edge, s, _ = loc
        L = self.g.length(edge)
        out = []
        st = status.get(self.g.key(edge))
        blocked = bool(st and st['blocked'])
        ahead_b = not blocked or (st['s_max'] is not None and s > st['s_max'] + 0.3)   # toward B allowed
        ahead_a = not blocked or (st['s_min'] is not None and s < st['s_min'] - 0.3)   # toward A allowed
        if ahead_b:
            out.append((edge[1], [(edge, s, L)], edge))
        if ahead_a:
            out.append((edge[0], [(edge, s, 0.0)], edge))
        return out

    def simple_paths(self, node, goal, status, excluded):
        def rec(u, seen):
            if u == goal:
                yield [u]
                return
            for v in self.g.neighbours(u):
                e, _ = self.g.edge_between(u, v)
                if v in seen or e == excluded or self._blocked(e, status):
                    continue
                for tail in rec(v, seen | {v}):
                    yield [u] + tail
        yield from rec(node, {node})

    def plan(self, loc, status, R, P0, goal):
        """All feasible routes from the robot's location to `goal`, cheapest first.

        Each candidate: dict(nodes, legs, length, unc, narrow, total, sigma_end, trace).
        """
        p = self.p
        cands = []
        for node, legs0, excluded in self.first_options(loc, status):
            P, length, U, narrow, trace = P0, 0.0, 0.0, 0.0, []
            for edge, s0, s1 in legs0:
                P, l, u, nw, tr = self.leg(edge, s0, s1, P, R, status)
                length, U, narrow, trace = length + l, U + u, narrow + nw, trace + tr
            for path in self.simple_paths(node, goal, status, excluded):
                Pp, lp, up, nwp, trp = P, length, U, narrow, list(trace)
                legs = list(legs0)
                for a, b in zip(path[:-1], path[1:]):
                    edge, fwd = self.g.edge_between(a, b)
                    s0, s1 = (0.0, self.g.length(edge)) if fwd else (self.g.length(edge), 0.0)
                    Pp, l, u, nw, tr = self.leg(edge, s0, s1, Pp, R, status)
                    lp, up, nwp, trp = lp + l, up + u, nwp + nw, trp + tr
                    legs.append((edge, s0, s1))
                cands.append(dict(nodes=path, legs=legs, length=lp, unc=up, narrow=nwp,
                                  total=lp + p.w_unc * up + nwp, sigma_end=sigma_eff(Pp), trace=trp))
        cands.sort(key=lambda c: c['total'])
        return cands

    def initial_covariance(self, cov2=None):
        """Starting covariance: the localizer's, plus a floor (particle clouds are overconfident)."""
        base = np.zeros((2, 2)) if cov2 is None else np.asarray(cov2, float)
        return base + (self.p.p0_floor ** 2) * np.eye(2)

    # ------------------------------------------------------------------ geometry of a route
    def obstruction_points(self, edge, s0, s1, st):
        """Off-centre waypoints that steer through the free gap beside an obstruction on this leg."""
        p, g = self.p, self.g
        if not st or st['blocked'] or st['gap'] >= p.shift_gap or st['s_min'] is None:
            return []
        off = st['offset']
        seq = [(st['s_min'] - 0.7, 0.0), (st['s_min'] - 0.25, off), (st['s_max'] + 0.25, off), (st['s_max'] + 0.7, 0.0)]
        if s1 < s0:
            seq = seq[::-1]
        lo, hi = min(s0, s1), max(s0, s1)
        return [g.point(edge, s, d) for s, d in seq if lo + 0.05 < s < hi - 0.05]

    def waypoints(self, robot_xy, candidate, status):
        """Polyline from the robot through the route's nodes, with off-centre points around obstructions."""
        g = self.g
        pts = [np.asarray(robot_xy, float)]
        for edge, s0, s1 in candidate['legs']:
            pts += self.obstruction_points(edge, s0, s1, status.get(g.key(edge)))
            pts.append(g.point(edge, s1))
        return pts
