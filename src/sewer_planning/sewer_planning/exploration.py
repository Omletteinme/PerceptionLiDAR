"""Frontier exploration of the sewer network: where to go next so that every accessible corridor gets inspected.

CUSTOM code (ours), pure NumPy, no ROS imports (shared by the sandbox and exploration_node).

The map representation is the existing one: a known corridor graph (topology from the graph yaml) with blockages that
are NOT known in advance and come from the blockage detector at run time. Exploration therefore works on the graph,
not on a grid: a FRONTIER is an accessible stretch of corridor (CoverageTracker.sections) that still has unexplored
bins. For each frontier the robot can either drive along the stretch directly (when it is already on it) or reach one
of its open ends first and then traverse it to the far end; a dead-end stub in front of detected debris is traversed up
to `stop_short` metres before the debris.

Route costs come from the existing uncertainty-aware RouteModel (route_cost.py): length + w_unc * integrated predicted
position uncertainty + narrow-gap penalty, with the same blocked-edge handling. The cheapest frontier is chosen
(nearest-frontier rule), with hysteresis so the target does not flip between nearly equal options. When nothing is
left, the robot returns to the entrance (graph start node) and the exploration ends COMPLETE, or INCOMPLETE if some
accessible stretch could not be covered (the reason is reported, coverage stays below 100 %).

Motion is NOT commanded here: the result is a waypoint polyline that the existing route follower drives (pure pursuit,
health-aware speed). Localization health enters through `trusted` (no coverage is marked on a doubtful pose) and the
follower's own slow-down/stop response.
"""
import math
from dataclasses import dataclass

import numpy as np

from .coverage_tracker import CoverageTracker
from .graph import Graph

INIT, EXPLORING, RETURNING, COMPLETE, INCOMPLETE = 'INIT', 'EXPLORING', 'RETURNING', 'COMPLETE', 'INCOMPLETE'


@dataclass
class ExploreParams:
    stop_short: float = 0.6       # stop this far before detected debris at the end of a dead-end stub [m]
    arrive_tol: float = 0.3       # a target counts as reached within this distance [m]
    hysteresis: float = 1.0       # keep the current target unless another is cheaper by this much [m of cost]
    patience: int = 4             # explorer steps spent at a reached target before its leftover bins are skipped
    return_home: bool = True      # return to the entrance when exploration is done
    max_entries: int = 3          # frontier entry nodes costed with the full RouteModel per step (nearest by length)


class Explorer:
    def __init__(self, graph, model, tracker=None, params=None, home=None):
        self.g, self.model = graph, model
        self.tracker = tracker or CoverageTracker(graph)
        self.p = params or ExploreParams()
        self.home = home or graph.start
        self.state, self.reason = INIT, ''
        self.current = None           # key (edge key, target s) of the current target
        self.dwell = {}               # target key -> explorer steps spent at it after arriving
        self.status = {}

    # ------------------------------------------------------------------ candidate targets
    def _stub_targets(self, e, lo, hi, open_lo, open_hi):
        """Points to drive to at the two ends of an accessible stretch (dead stub ends stop short of the debris)."""
        L, m = self.g.length(e), self.tracker.p.block_margin
        t_lo = lo if (open_lo or lo <= 1e-6) else min(hi, lo - m + self.p.stop_short)
        t_hi = hi if (open_hi or hi >= L - 1e-6) else max(lo, hi + m - self.p.stop_short)
        return t_lo, t_hi

    def _has_frontier(self, e, a, b, lo, hi):
        """Unexplored accessible bins between s=a and s=b (the far end is extended by the sensing radius)."""
        r = self.tracker.p.sense_radius
        lo_, hi_ = (a, b + r) if b >= a else (b - r, a)
        return len(self.tracker.frontier_bins(e, max(lo, lo_), min(hi, hi_))) > 0

    def _path_lengths(self, loc):
        """Plain shortest driving distance from the robot to every node, avoiding blocked edges (Dijkstra).

        Only a prefilter: RouteModel.plan enumerates every simple route with the uncertainty integral, which is too slow
        to run for every frontier entry at 2 Hz, so the full cost is computed for the `max_entries` nearest entries.
        """
        dist = {}
        for node, legs, _ in self.model.first_options(loc, self.status):
            d0 = sum(abs(s1 - s0) for _, s0, s1 in legs)
            dist[node] = min(dist.get(node, math.inf), d0)
        todo = dict(dist)
        while todo:
            u = min(todo, key=todo.get)
            du = todo.pop(u)
            for v in self.g.neighbours(u):
                e, _ = self.g.edge_between(u, v)
                st = self.status.get(Graph.key(e))
                if st and st.get('blocked'):
                    continue
                dv = du + self.g.length(e)
                if dv < dist.get(v, math.inf):
                    dist[v] = todo[v] = dv
        return dist

    def candidates(self, loc, R, P0):
        """Every way to reach and cover a frontier, cheapest first.

        Each: dict(key, edge, s, legs, cost, entry). `legs` are RouteModel legs [(edge, s0, s1)].
        """
        out, status, p_unc = [], self.status, self.model.p.w_unc
        entries = []                                      # (prefilter length, edge, entry node, s_in, target s)
        for e, lo, hi, open_lo, open_hi in self.tracker.sections():
            if len(self.tracker.frontier_bins(e, lo, hi)) == 0:
                continue
            t_lo, t_hi = self._stub_targets(e, lo, hi, open_lo, open_hi)
            on_edge = loc[0] == 'edge' and loc[1] == e and lo - 0.3 <= loc[2] <= hi + 0.3
            if on_edge:
                s_r = loc[2]
                for t in (t_lo, t_hi):
                    if self._has_frontier(e, s_r, t, lo, hi):
                        _, l, u, nw, _ = self.model.leg(e, s_r, t, P0, R, status)
                        out.append(dict(key=(Graph.key(e), round(t, 2)), edge=e, s=t, legs=[(e, s_r, t)],
                                        cost=l + p_unc * u + nw, entry=None))
                continue
            L = self.g.length(e)
            for entry, s_in, t, ok in ((e[0], 0.0, t_hi, open_lo), (e[1], L, t_lo, open_hi)):
                if ok:
                    entries.append((e, entry, s_in, t))
        if entries:
            dist = self._path_lengths(loc)
            entries = [(dist[n] + abs(t - s_in), e, n, s_in, t) for e, n, s_in, t in entries if n in dist]
            entries.sort(key=lambda c: c[0])
            keep = sorted({n for _, _, n, _, _ in entries}, key=lambda n: dist[n])[:self.p.max_entries]
            plans = {n: self.model.plan(loc, status, R, P0, n) for n in keep}
            for _, e, n, s_in, t in entries:
                if not plans.get(n):
                    continue
                best = plans[n][0]
                Pu = (best['sigma_end'] ** 2) * np.eye(2)
                _, l, u, nw, _ = self.model.leg(e, s_in, t, Pu, R, status)
                out.append(dict(key=(Graph.key(e), round(t, 2)), edge=e, s=t, legs=best['legs'] + [(e, s_in, t)],
                                cost=best['total'] + l + p_unc * u + nw, entry=n))
        out.sort(key=lambda c: c['cost'])
        return out

    # ------------------------------------------------------------------ one decision step
    def step(self, xy, cov2=None, R=0.9, status=None, trusted=True):
        """Update coverage from the estimated position and choose what to do next.

        xy: estimated (x, y) in the map frame; cov2: its 2x2 covariance; R: smoothed scan reliability; status: per-edge
        blockage status from the detector; trusted: False while the localization health is CRITICAL.
        Returns dict(state, reason, target, waypoints, frontiers, candidates).
        """
        if status is not None:
            self.status = status
        self.tracker.set_status(self.status)
        cov = np.zeros((2, 2)) if cov2 is None else np.asarray(cov2, float)
        sigma = math.sqrt(max(float(np.linalg.eigvalsh(cov)[-1]), 0.0))
        self.tracker.update_pose(xy[0], xy[1], sigma, trusted)
        x, y = float(xy[0]), float(xy[1])
        loc = self.g.locate(x, y)
        P0 = self.model.initial_covariance(cov)

        cands = self.candidates(loc, R, P0)
        frontiers = sum(1 for e, lo, hi, *_ in self.tracker.sections() if len(self.tracker.frontier_bins(e, lo, hi)))
        out = dict(frontiers=frontiers, candidates=[dict(target=f'{c["key"][0]}@{c["key"][1]}', cost=round(c['cost'], 2),
                                                         entry=c['entry']) for c in cands[:4]])

        # a target reached without covering its bins (should not happen; guards against an endless loop)
        for c in cands:
            tgt = self.g.point(c['edge'], c['s'])
            if math.hypot(tgt[0] - x, tgt[1] - y) < self.p.arrive_tol:
                self.dwell[c['key']] = self.dwell.get(c['key'], 0) + 1
                if self.dwell[c['key']] > self.p.patience:
                    r = 1.5 * max(self.tracker.p.sense_radius, self.tracker.p.bin_len)
                    self.tracker.skip(c['edge'], c['s'] - r, c['s'] + r)
                    self.reason = f'skipped bins near {c["key"][0]}@{c["key"][1]} (reached, not covered)'
        if any(self.dwell.get(c['key'], 0) > self.p.patience for c in cands):
            self.tracker.set_status(self.status)
            cands = self.candidates(loc, R, P0)

        if cands:
            chosen = cands[0]
            if self.current is not None:
                for c in cands:
                    if c['key'] == self.current and c['cost'] <= chosen['cost'] + self.p.hysteresis:
                        chosen = c
                        break
            self.current = chosen['key']
            self.state, self.reason = EXPLORING, ''
            tgt = self.g.point(chosen['edge'], chosen['s'])
            out.update(state=self.state, reason='', waypoints=self._waypoints((x, y), chosen['legs'], tgt),
                       target=dict(edge=chosen['key'][0], s=chosen['key'][1], xy=[round(float(tgt[0]), 3),
                                   round(float(tgt[1]), 3)], entry=chosen['entry'], cost=round(chosen['cost'], 2)))
            return out

        # nothing left to explore (or nothing reachable): return to the entrance
        self.current = None
        summ = self.tracker.summary()
        why = '' if summ['complete'] else (self.reason or f'{summ["remaining_m"] + summ["skipped_m"]:.1f} m accessible '
                                                                       'corridor could not be covered')
        home = np.array(self.g.nodes[self.home])
        done = COMPLETE if summ['complete'] else INCOMPLETE
        if not self.p.return_home or math.hypot(home[0] - x, home[1] - y) < self.p.arrive_tol:
            self.state, self.reason = done, why
            out.update(state=self.state, reason=why, waypoints=None, target=None)
            return out
        plans = self.model.plan(loc, self.status, R, P0, self.home)
        if not plans:
            self.state, self.reason = done, (why + '; ' if why else '') + 'no route back to the entrance'
            out.update(state=self.state, reason=self.reason, waypoints=None, target=None)
            return out
        self.state, self.reason = RETURNING, why
        out.update(state=self.state, reason=why, waypoints=self._waypoints((x, y), plans[0]['legs'], home),
                   target=dict(edge=None, s=None, xy=[float(home[0]), float(home[1])], entry=self.home,
                               cost=round(plans[0]['total'], 2)))
        return out

    def _waypoints(self, xy, legs, tgt):
        pts = self.model.waypoints(xy, dict(legs=legs), self.status)
        if not legs:
            pts.append(np.asarray(tgt, float))
        return pts
