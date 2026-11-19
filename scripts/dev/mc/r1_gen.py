#!/usr/bin/env python3
"""RFP R1 (autonomous path planning, PILS): the random maps - N cases, each a C-Track-sized field with random
obstacles, a random start and goal, and the global optimum (the reference of R1-2). The planner itself runs
on the on-board computer (r1_bench.py), r1_eval.py judges.

usage: r1_gen.py OUT_JSON [--n 300] [--seed 20261105]

Random map rules (fixed before the test, docs/montecarlo-plan-2026-11-05.md section 2):
  field        the C-Track field, X -5..95 m, Y -12..38 m (100 x 50 m; the planner's 400 x 200 px map at 0.25 m)
  obstacles    5-20 (uniform), each one of
                 box   50 %: 2-10 m x 2-10 m footprint, 3-6 m high, any yaw (tent, container, building)
                 wall  25 %: 4-15 m long, 0.3 m thick, 3 m high, any yaw (fence, plywood wall)
                 pole  25 %: 0.3-1.0 m diameter, 3-6 m high (pole, tree trunk)
               all higher than the flight (1.7 m) + clearance, so the problem is the 2D one the reference solves;
               centres >= 3 m inside the field; footprints do not overlap (0.5 m apart)
  start, goal  inside the field (3 m margin), >= 1.0 m outside every obstacle grown by the vehicle radius 1.2 m,
               >= 40 m apart, the straight line between them blocked by at least one grown obstacle (no trivial
               case), and a path exists (else the map is drawn again)
  reference    the global optimum: the shortest path among the same obstacles grown by the same radius (1.2 m,
               corner arcs circumscribed - scripts/dev/ctrack_pp.py's inflated_polygon), a visibility graph
"""
import argparse
import heapq
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from ctrack_pp import inflated_polygon, box_corners  # noqa: E402

FIELD = ((-5.0, 95.0), (-12.0, 38.0))
RADIUS, ALT = 1.2, 1.7


def ccw(poly):
    a = sum(poly[i][0] * poly[(i + 1) % len(poly)][1] - poly[(i + 1) % len(poly)][0] * poly[i][1]
            for i in range(len(poly)))
    return poly if a > 0 else poly[::-1]


class Polys:
    """convex polygons as half-plane sets; exact strict-interior tests for points and segments (Cyrus-Beck)"""

    def __init__(self, polys):
        self.polys = [np.asarray(ccw(p), float) for p in polys]
        self.N, self.C = [], []                         # outward normals, offsets: inside <=> N p < C
        for p in self.polys:
            e = np.roll(p, -1, axis=0) - p
            n = np.column_stack([e[:, 1], -e[:, 0]])
            n /= np.linalg.norm(n, axis=1, keepdims=True)
            self.N.append(n)
            self.C.append(np.einsum('ij,ij->i', n, p))

    def point_inside(self, x, y, eps=1e-3):
        q = np.array([x, y])
        return any(np.all(n @ q < c - eps) for n, c in zip(self.N, self.C))

    def visible_from(self, a, B, eps=1e-6):
        """bool per row of B: the segment a -> B[j] does not cross the interior of any polygon"""
        free = np.ones(len(B), bool)
        d = B - a                                                   # (J, 2)
        for n, c in zip(self.N, self.C):
            num = c - n @ a                                         # (K,)  >0: a on the inner side of edge k
            den = d @ n.T                                           # (J, K)
            with np.errstate(divide='ignore', invalid='ignore'):
                t = num[None, :] / den
            enter = np.where(den < 0, t, -np.inf)                   # crossing into the half-plane
            leave = np.where(den > 0, t, np.inf)
            parallel_out = (np.abs(den) < 1e-12) & (num[None, :] <= 0)
            t0 = np.maximum(enter.max(axis=1), 0.0)
            t1 = np.minimum(leave.min(axis=1), 1.0)
            hit = (t1 - t0 > 1e-6) & ~parallel_out.any(axis=1)
            free &= ~hit
        return free


def shortest(P, a, b):
    """visibility-graph shortest path a -> b among the polygons (Dijkstra, visibility on demand)"""
    nodes = [a, b] + [tuple(v) for p in P.polys for v in p]
    nodes = [v for v in nodes if not P.point_inside(*v)]
    if a not in nodes or b not in nodes:
        return None, math.inf
    X = np.array(nodes)
    ia, ib = nodes.index(a), nodes.index(b)
    dist = np.full(len(X), np.inf)
    dist[ia] = 0.0
    prev = np.full(len(X), -1)
    done = np.zeros(len(X), bool)
    heap = [(0.0, ia)]
    while heap:
        d, i = heapq.heappop(heap)
        if done[i]:
            continue
        done[i] = True
        if i == ib:
            break
        cand = np.nonzero(~done)[0]
        nd = d + np.hypot(*(X[cand] - X[i]).T)
        better = nd < dist[cand]
        if not better.any():
            continue
        cand, nd = cand[better], nd[better]
        vis = P.visible_from(X[i], X[cand])
        for j, v in zip(cand[vis], nd[vis]):
            dist[j], prev[j] = v, i
            heapq.heappush(heap, (v, int(j)))
    if not np.isfinite(dist[ib]):
        return None, math.inf
    path, i = [nodes[ib]], ib
    while i != ia:
        i = prev[i]
        path.append(nodes[i])
    return path[::-1], float(dist[ib])


def footprint(o):
    if o['shape'] == 'cylinder':
        r = o['size'][0] / 2.0
        return [(o['pos'][0] + r * math.cos(2 * math.pi * k / 16), o['pos'][1] + r * math.sin(2 * math.pi * k / 16))
                for k in range(16)]
    return box_corners(o)


def overlap(o1, o2, gap=0.5):
    """footprints closer than gap (grown polygons intersect)"""
    p1 = Polys([inflated_polygon(o1, gap / 2.0, arc=2)])
    q = np.array(inflated_polygon(o2, gap / 2.0, arc=2))
    if any(p1.point_inside(*v, eps=0.0) for v in q):
        return True
    p2 = Polys([q.tolist()])
    return any(p2.point_inside(*v, eps=0.0) for v in p1.polys[0])


def draw_obstacles(rng):
    (x0, x1), (y0, y1) = FIELD
    n = int(rng.integers(5, 21))
    obs = []
    tries = 0
    while len(obs) < n and tries < 2000:
        tries += 1
        kind = rng.choice(['box', 'wall', 'pole'], p=[0.5, 0.25, 0.25])
        pos = [float(rng.uniform(x0 + 3, x1 - 3)), float(rng.uniform(y0 + 3, y1 - 3))]
        yaw = float(rng.uniform(0, 180))
        if kind == 'box':
            o = {'shape': 'box', 'size': [float(rng.uniform(2, 10)), float(rng.uniform(2, 10)), float(rng.uniform(3, 6))]}
        elif kind == 'wall':
            o = {'shape': 'box', 'size': [0.3, float(rng.uniform(4, 15)), 3.0]}
        else:
            dd = float(rng.uniform(0.3, 1.0))
            o = {'shape': 'cylinder', 'size': [dd, dd, float(rng.uniform(3, 6))]}
        o.update(name=f'{kind.capitalize()}_{len(obs) + 1}', kind=str(kind), pos=pos, yaw_deg=yaw)
        if any(overlap(o, p) for p in obs):
            continue
        obs.append(o)
    return obs


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('out')
    ap.add_argument('--n', type=int, default=300)
    ap.add_argument('--seed', type=int, default=20261105)
    a = ap.parse_args()
    rng = np.random.default_rng(a.seed)
    (x0, x1), (y0, y1) = FIELD
    cases, redraws = [], 0
    while len(cases) < a.n:
        obs = draw_obstacles(rng)
        P = Polys([inflated_polygon(o, RADIUS) for o in obs])
        clear = Polys([inflated_polygon(o, RADIUS + 1.0) for o in obs])
        ok = False
        for _ in range(200):
            s = (float(rng.uniform(x0 + 3, x1 - 3)), float(rng.uniform(y0 + 3, y1 - 3)))
            g = (float(rng.uniform(x0 + 3, x1 - 3)), float(rng.uniform(y0 + 3, y1 - 3)))
            if math.dist(s, g) < 40.0 or clear.point_inside(*s) or clear.point_inside(*g):
                continue
            if P.visible_from(np.array(s), np.array([g]))[0]:
                continue                                            # straight line free: trivial
            ok = True
            break
        if not ok:
            redraws += 1
            continue
        path, L = shortest(P, s, g)
        if path is None:
            redraws += 1
            continue
        cases.append({'case': len(cases), 'obstacles': obs, 'start': list(s), 'goal': list(g),
                      'straight_m': round(math.dist(s, g), 2), 'reference_m': round(L, 3),
                      'reference_path': [[round(x, 3), round(y, 3)] for x, y in path]})
    meta = {'rules': __doc__.split('Random map rules')[1].strip(), 'seed': a.seed, 'field': FIELD, 'radius_m': RADIUS,
            'alt_m': ALT, 'n': len(cases), 'redraws': redraws}
    json.dump({'meta': meta, 'cases': cases}, open(a.out, 'w'), indent=1)
    n_obs = [len(c['obstacles']) for c in cases]
    ref = [c['reference_m'] / c['straight_m'] for c in cases]
    print(f'{len(cases)} cases -> {a.out} ({redraws} maps drawn again); obstacles {min(n_obs)}-{max(n_obs)}, '
          f'start-goal {min(c["straight_m"] for c in cases):.0f}-{max(c["straight_m"] for c in cases):.0f} m, '
          f'optimum / straight line {min(ref):.3f}-{max(ref):.3f}')


if __name__ == '__main__':
    main()
