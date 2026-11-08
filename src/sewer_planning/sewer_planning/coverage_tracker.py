"""Coverage of the sewer network: which parts of which corridors the robot has inspected.

CUSTOM code (ours), pure NumPy, no ROS imports (shared by the sandbox and exploration_node).

Every corridor (graph edge) is cut into bins of `bin_len` metres along its centre line. A bin is EXPLORED once the
robot's ESTIMATED position has been within `sense_radius` of the bin centre while the localization was trusted (the
caller passes trusted=False while the health state is CRITICAL, so a doubtful pose never marks coverage). Ground truth
is never used here.

Accessibility comes from the blockage detector's per-edge status (`edge_gaps`, see sewer_perception.blockage):
  - nodes reachable from the entrance through edges that are not blocked are REACHABLE;
  - a blocked edge splits into two dead-end stubs: [0, s_min - block_margin) is reachable from its first node, and
    (s_max + block_margin, L] from its second node; the bins in between are BLOCKED (cannot be visited);
  - bins on edges whose ends cannot be reached are UNREACHABLE.
Coverage = explored accessible length / accessible length. It is a claim about the known topology only (the graph
yaml); the evaluator checks it offline against the world truth. Completion is never inferred from visiting the start
and end nodes: every accessible bin has to be visited.
"""
import math
from collections import deque
from dataclasses import dataclass

import numpy as np

UNEXPLORED, EXPLORED, BLOCKED, UNREACHABLE, SKIPPED = 'unexplored', 'explored', 'blocked', 'unreachable', 'skipped'


@dataclass
class CoverageParams:
    bin_len: float = 0.5        # length of one coverage bin [m]
    sense_radius: float = 0.6   # a bin counts as inspected when the robot passes within this distance of its centre [m]
    half_width: float = 0.5     # corridor half-width: the robot must be inside the corridor to cover it [m]
    block_margin: float = 0.3   # bins this close to detected debris are treated as blocked [m]
    sigma_ok: float = 0.25      # a bin marked with pose sigma above this counts as low-confidence coverage [m]


class CoverageTracker:
    def __init__(self, graph, params=None, start=None):
        self.g = graph
        self.p = params or CoverageParams()
        self.start = start or graph.start
        self.centres, self.explored, self.sigma_at, self.skipped = {}, {}, {}, {}
        for e in graph.edges:
            L = graph.length(e)
            n = max(1, int(math.ceil(L / self.p.bin_len - 1e-9)))
            self.centres[e] = (np.arange(n) + 0.5) * (L / n)
            self.explored[e] = np.zeros(n, bool)
            self.sigma_at[e] = np.full(n, np.inf)
            self.skipped[e] = np.zeros(n, bool)
        self.status = {}
        self._access = None

    # ------------------------------------------------------------------ inputs
    def set_status(self, status):
        """Per-edge blockage status from the detector: {'A-B': {blocked, s_min, s_max, ...}}."""
        self.status = dict(status or {})
        self._access = None

    def update_pose(self, x, y, sigma=0.0, trusted=True):
        """Mark the bins near the estimated position (x, y) as explored. Returns the number of newly explored bins."""
        if not trusted:
            return 0
        p, new = self.p, 0
        q = np.array([x, y], float)
        for e in self.g.edges:
            A = np.array(self.g.nodes[e[0]])
            a, nrm, L = self.g.direction(e), self.g.normal(e), self.g.length(e)
            s, d = float((q - A) @ a), float((q - A) @ nrm)
            if abs(d) > p.half_width or s < -p.half_width or s > L + p.half_width:
                continue
            near = np.hypot(self.centres[e] - s, d) <= p.sense_radius
            fresh = near & ~self.explored[e]
            new += int(fresh.sum())
            self.explored[e] |= near
            self.sigma_at[e][near] = np.minimum(self.sigma_at[e][near], sigma)
        return new

    def skip(self, edge, s_lo, s_hi):
        """Give up on the unexplored bins of `edge` in [s_lo, s_hi] (target reached repeatedly without covering them).

        They stay out of the explored count (coverage stays below 100 %) and are reported as SKIPPED.
        """
        c = self.centres[edge]
        self.skipped[edge] |= (c >= min(s_lo, s_hi) - 1e-9) & (c <= max(s_lo, s_hi) + 1e-9) & ~self.explored[edge]

    # ------------------------------------------------------------------ accessibility
    def _blocked(self, e):
        st = self.status.get(self.g.key(e))
        return bool(st and st.get('blocked'))

    def reachable_nodes(self):
        seen, todo = {self.start}, deque([self.start])
        while todo:
            u = todo.popleft()
            for v in self.g.neighbours(u):
                e, _ = self.g.edge_between(u, v)
                if v not in seen and not self._blocked(e):
                    seen.add(v)
                    todo.append(v)
        return seen

    def sections(self):
        """Accessible stretches of every edge: [(edge, s_lo, s_hi, open_lo, open_hi)].

        open_lo/open_hi: whether the robot can enter the stretch through the edge's first/second node. An unblocked edge
        between reachable nodes is one stretch open at both ends; a blocked edge gives up to two dead-end stubs.
        """
        if self._access is None:
            self._compute_access()
        return self._access[1]

    def bin_states(self, edge):
        if self._access is None:
            self._compute_access()
        acc, blk = self._access[0][edge]
        out = np.full(len(self.centres[edge]), UNREACHABLE, dtype=object)
        out[blk] = BLOCKED
        out[acc] = UNEXPLORED
        out[acc & self.skipped[edge]] = SKIPPED
        out[acc & self.explored[edge]] = EXPLORED
        return out

    def _compute_access(self):
        reach = self.reachable_nodes()
        per_edge, secs = {}, []
        for e in self.g.edges:
            c, L = self.centres[e], self.g.length(e)
            ra, rb = e[0] in reach, e[1] in reach
            acc, blk = np.zeros(len(c), bool), np.zeros(len(c), bool)
            st = self.status.get(self.g.key(e))
            if self._blocked(e) and st.get('s_min') is not None:
                lo = st['s_min'] - self.p.block_margin
                hi = st['s_max'] + self.p.block_margin
                blk = (c >= lo) & (c <= hi)
                if ra:
                    acc |= c < lo
                    if lo > 0:
                        secs.append((e, 0.0, max(0.0, lo), True, False))
                if rb:
                    acc |= c > hi
                    if hi < L:
                        secs.append((e, min(L, hi), L, False, True))
            elif self._blocked(e):                      # blocked but no extent known: treat the whole edge as blocked
                blk[:] = True
            elif ra or rb:
                acc[:] = True
                secs.append((e, 0.0, L, ra, rb))
            per_edge[e] = (acc, blk)
        self._access = (per_edge, secs)

    # ------------------------------------------------------------------ outputs
    def frontier_bins(self, edge, s_lo, s_hi):
        """Centres of the accessible, unexplored, not-skipped bins of `edge` inside [s_lo, s_hi]."""
        states = self.bin_states(edge)
        c = self.centres[edge]
        m = (states == UNEXPLORED) & (c >= s_lo - 1e-9) & (c <= s_hi + 1e-9)
        return c[m]

    def summary(self):
        """Coverage report (JSON-serialisable)."""
        bl = lambda e: self.g.length(e) / len(self.centres[e])   # noqa: E731  (actual bin length of edge e)
        tot = dict(explored=0.0, accessible=0.0, blocked=0.0, unreachable=0.0, skipped=0.0, low_conf=0.0)
        edges = {}
        for e in self.g.edges:
            st = self.bin_states(e)
            b = bl(e)
            n = {k: int((st == k).sum()) for k in (EXPLORED, UNEXPLORED, BLOCKED, UNREACHABLE, SKIPPED)}
            acc = n[EXPLORED] + n[UNEXPLORED] + n[SKIPPED]
            low = int(((st == EXPLORED) & (self.sigma_at[e] > self.p.sigma_ok)).sum())
            tot['explored'] += n[EXPLORED] * b
            tot['accessible'] += acc * b
            tot['blocked'] += n[BLOCKED] * b
            tot['unreachable'] += n[UNREACHABLE] * b
            tot['skipped'] += n[SKIPPED] * b
            tot['low_conf'] += low * b
            if acc == 0:
                state = BLOCKED if n[BLOCKED] else UNREACHABLE
            elif n[UNEXPLORED] == 0 and n[SKIPPED] == 0:
                state = 'complete'
            elif n[EXPLORED] == 0:
                state = UNEXPLORED
            else:
                state = 'partial'
            edges[self.g.key(e)] = dict(length=round(self.g.length(e), 2), state=state,
                                        explored=round(n[EXPLORED] * b, 2), accessible=round(acc * b, 2),
                                        blocked=round(n[BLOCKED] * b, 2), skipped=round(n[SKIPPED] * b, 2))
        total = sum(self.g.length(e) for e in self.g.edges)
        cov = tot['explored'] / tot['accessible'] if tot['accessible'] > 0 else 0.0
        conf = 1.0 - tot['low_conf'] / tot['explored'] if tot['explored'] > 0 else 0.0
        remaining = tot['accessible'] - tot['explored'] - tot['skipped']
        return dict(coverage=round(cov, 4), confidence=round(conf, 4),
                    explored_m=round(tot['explored'], 2), accessible_m=round(tot['accessible'], 2),
                    remaining_m=round(max(remaining, 0.0), 2), blocked_m=round(tot['blocked'], 2),
                    unreachable_m=round(tot['unreachable'], 2), skipped_m=round(tot['skipped'], 2),
                    total_known_m=round(total, 2),
                    complete=bool(tot['accessible'] > 0 and tot['accessible'] - tot['explored'] < 1e-6),
                    segments=dict(total=len(edges),
                                  complete=sum(v['state'] == 'complete' for v in edges.values()),
                                  partial=sum(v['state'] == 'partial' for v in edges.values()),
                                  unexplored=sum(v['state'] == UNEXPLORED for v in edges.values()),
                                  blocked=sum(v['state'] == BLOCKED for v in edges.values()),
                                  obstructed=sum(v['blocked'] > 0 for v in edges.values()),
                                  unreachable=sum(v['state'] == UNREACHABLE for v in edges.values())),
                    edges=edges)
