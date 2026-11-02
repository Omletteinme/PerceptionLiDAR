"""The sewer network as a graph (topology known a priori; blockages are not part of it).

CUSTOM code (ours), pure NumPy. Edge coordinates: s along node A -> node B of the edge as written in
network_graph.yaml, d lateral (left of A->B is positive). Edge key: 'A-B'.
"""
import math

import numpy as np
import yaml


class Graph:
    def __init__(self, nodes, edges, start, goal, width):
        self.nodes = {k: (float(v[0]), float(v[1])) for k, v in nodes.items()}
        self.edges = [tuple(e) for e in edges]
        self.start, self.goal, self.width = start, goal, width
        self._geom = {}
        for e in self.edges:
            A, B = np.array(self.nodes[e[0]]), np.array(self.nodes[e[1]])
            length = float(np.linalg.norm(B - A))
            a = (B - A) / length
            self._geom[e] = (A, B, length, a, np.array([-a[1], a[0]]))

    @classmethod
    def load(cls, path):
        with open(path) as f:
            g = yaml.safe_load(f)
        return cls({k: (v['x'], v['y']) for k, v in g['nodes'].items()},
                   [(e['from'], e['to']) for e in g['edges']], g['start'], g['goal'], g['corridor_width'])

    @staticmethod
    def key(edge):
        return f'{edge[0]}-{edge[1]}'

    def length(self, edge):
        return self._geom[edge][2]

    def direction(self, edge):
        return self._geom[edge][3]

    def normal(self, edge):
        return self._geom[edge][4]

    def point(self, edge, s, d=0.0):
        A, _, _, a, n = self._geom[edge]
        return A + s * a + d * n

    def edge_between(self, u, v):
        """(edge as stored, travelling A->B?) for the edge joining nodes u and v."""
        for e in self.edges:
            if e == (u, v):
                return e, True
            if e == (v, u):
                return e, False
        raise KeyError((u, v))

    def neighbours(self, u):
        return [e[1] if e[0] == u else e[0] for e in self.edges if u in e]

    def locate(self, x, y, node_snap=0.35):
        """('node', name) if within node_snap of a node, else ('edge', edge, s, d) on the nearest corridor."""
        p = np.array([x, y])
        for name, xy in self.nodes.items():
            if math.hypot(x - xy[0], y - xy[1]) < node_snap:
                return ('node', name)
        # nearest corridor by TRUE distance to its centre-line segment (not by lateral offset alone: beside a
        # junction several corridors share the same offset, and picking the wrong one sends the robot back to
        # the node it just left; found in the Day 9 live runs)
        best, best_dist = None, 0.7
        for e in self.edges:
            A, _, length, a, n = self._geom[e]
            s, d = float((p - A) @ a), float((p - A) @ n)
            s_clip = min(max(s, 0.0), length)
            dist = math.hypot(s - s_clip, d)
            if dist < best_dist:
                best, best_dist = ('edge', e, s_clip, d), dist
        if best is not None:
            return best
        name = min(self.nodes, key=lambda k: math.hypot(x - self.nodes[k][0], y - self.nodes[k][1]))
        return ('node', name)
