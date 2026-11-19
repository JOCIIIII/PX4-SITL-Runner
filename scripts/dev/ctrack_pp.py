#!/usr/bin/env python3
"""Hybrid PP (hybrid-learning-path-planning 0.3.2: U-Net + Watershed + CUDA visibility graph + Dijkstra) on the
C-Track field, for the final demonstration scenarios A (path planning around the pre-known tents) and D (the
integrated flight, planned over Start -> WP1..WP4 -> Goal).

The height map comes straight from the layout (flat ground + the obstacles whose names match --known, default the
two tents = the "pre-known" obstacles), so no V-World map is needed. The flight height is fixed (one layer at
--alt): the tents are higher than the flight, so the plan goes around them, as at the 10-02 field test (PP runs
19/20 there were Z_MIN = Z_MAX too). Each leg is planned with the planner's full pipeline (HybridPlanner.plan), so
its total_ms is the on-board planning time the RFP item asks about (<= 1.0 s); the length is compared with the
shortest path among the same obstacles inflated by the same radius (a visibility graph), the "global optimum" of
the 120 % item.

Runs in the ROS2 image with the GPU and the libraries scripts/dev/ctrack_pp.sh mounts (PYTHONPATH=/pp_libs):
    scripts/dev/ctrack_pp.sh OUT_DIR START,WP1            # names from the layout or X,Y
    scripts/dev/ctrack_pp.sh OUT_DIR START,WP2,WP3,WP5,WP6,GOAL --alt 1.7
Outputs in OUT_DIR: pp_path.csv (layout X,Y,h - CTRACK_PATH for scripts/dev/ctrack_super.sh CTRACK_APP=mission),
pp_result.json (per leg: success, planning ms, length, reference length, ratio), pp_route.png, pp_config.json,
heightmap.npy.
"""
import argparse
import heapq
import json
import math
import os
import re
import sys
import time

import numpy as np


def _ctrack_dir():
    for d in (os.environ.get('CTRACK_DIR', ''), '~/Documents/A4VAI-SITL/A4VAI-Unreal',
              '~/Desktop/00_Project/00_A4VAI/00_Development/00_Simulator/02_InhouseSim/00_Unreal/00_Map'):
        if d and os.path.isdir(os.path.expanduser(d)):
            return os.path.expanduser(d)
    return os.path.expanduser('~/Documents/A4VAI-SITL/A4VAI-Unreal')


# ------------------------------------------------------------------ geometry
def box_corners(o, inflate=0.0):
    sx, sy = o['size'][0] / 2.0 + inflate, o['size'][1] / 2.0 + inflate
    c, s = math.cos(math.radians(o.get('yaw_deg', 0.0))), math.sin(math.radians(o.get('yaw_deg', 0.0)))
    return [(o['pos'][0] + c * x - s * y, o['pos'][1] + s * x + c * y) for x, y in ((-sx, -sy), (sx, -sy), (sx, sy), (-sx, sy))]


def inflated_polygon(o, r, arc=6):
    """Convex polygon containing the obstacle grown by r (corner arcs circumscribed)."""
    k = 1.0 / math.cos(math.pi / (4 * arc))          # circumscribe each arc step
    if o['shape'] == 'cylinder':
        R, n = (o['size'][0] / 2.0 + r) * (1.0 / math.cos(math.pi / 32)), 32
        return [(o['pos'][0] + R * math.cos(2 * math.pi * i / n), o['pos'][1] + R * math.sin(2 * math.pi * i / n)) for i in range(n)]
    sx, sy = o['size'][0] / 2.0, o['size'][1] / 2.0
    c, s = math.cos(math.radians(o.get('yaw_deg', 0.0))), math.sin(math.radians(o.get('yaw_deg', 0.0)))
    pts = []
    for (cx, cy), a0 in (((sx, -sy), -90), ((sx, sy), 0), ((-sx, sy), 90), ((-sx, -sy), 180)):
        for i in range(arc + 1):
            a = math.radians(a0 + 90.0 * i / arc)
            x, y = cx + r * k * math.cos(a), cy + r * k * math.sin(a)
            pts.append((o['pos'][0] + c * x - s * y, o['pos'][1] + s * x + c * y))
    return pts


def inside(poly, x, y):
    """strictly inside a convex counter-clockwise polygon (with a 1 mm tolerance)"""
    n = len(poly)
    for i in range(n):
        ax, ay = poly[i]; bx, by = poly[(i + 1) % n]
        if (bx - ax) * (y - ay) - (by - ay) * (x - ax) <= 1e-3 * math.hypot(bx - ax, by - ay):
            return False
    return True


def segment_free(polys, a, b, step=0.05):
    L = math.hypot(b[0] - a[0], b[1] - a[1])
    n = max(int(L / step), 1)
    for p in polys:
        xs = [p_[0] for p_ in p]; ys = [p_[1] for p_ in p]
        if max(a[0], b[0]) < min(xs) or min(a[0], b[0]) > max(xs) or max(a[1], b[1]) < min(ys) or min(a[1], b[1]) > max(ys):
            continue
        for i in range(1, n):
            t = i / n
            if inside(p, a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])):
                return False
    return True


def shortest_path(polys, a, b):
    """visibility-graph shortest path among convex polygons (2D)"""
    nodes = [a, b] + [v for p in polys for v in p]
    nodes = [v for v in nodes if not any(inside(p, *v) for p in polys)] if polys else nodes
    if a not in nodes or b not in nodes:
        return None, math.inf
    ia, ib = nodes.index(a), nodes.index(b)
    dist, prev, seen = {ia: 0.0}, {}, set()
    heap = [(0.0, ia)]
    while heap:
        d, i = heapq.heappop(heap)
        if i in seen:
            continue
        seen.add(i)
        if i == ib:
            break
        for j in range(len(nodes)):
            if j in seen:
                continue
            nd = d + math.hypot(nodes[j][0] - nodes[i][0], nodes[j][1] - nodes[i][1])
            if nd < dist.get(j, math.inf) and segment_free(polys, nodes[i], nodes[j]):
                dist[j], prev[j] = nd, i
                heapq.heappush(heap, (nd, j))
    if ib not in dist:
        return None, math.inf
    path, i = [nodes[ib]], ib
    while i != ia:
        i = prev[i]; path.append(nodes[i])
    return path[::-1], dist[ib]


# ------------------------------------------------------------------ main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('out')
    ap.add_argument('points', help='comma-separated layout waypoint names or X:Y pairs, e.g. START,WP1 or 0:0,60:0')
    ap.add_argument('--layout', default=os.path.join(_ctrack_dir(), 'layout', 'ctrack_layout.json'))
    ap.add_argument('--alt', type=float, default=1.7, help='flight height above the ground [m] (= the test map runs\' path height)')
    ap.add_argument('--res', type=float, default=0.25, help='height map resolution [m/px]')
    ap.add_argument('--radius', type=float, default=1.2, help='vehicle horizontal radius [m] (X8 prop tips 0.77 + margin; 10-02 PP_RADIUS 1.2)')
    ap.add_argument('--vclear', type=float, default=0.5, help='vertical clearance under the vehicle centre [m]')
    ap.add_argument('--known', default=r'^Tent_', help='regex of the pre-known obstacle names put in the height map')
    ap.add_argument('--sigma', type=float, default=0.8)
    ap.add_argument('--rho-soft', type=float, default=0.99)
    ap.add_argument('--rho-caution', type=float, default=0.4)
    ap.add_argument('--marker-q', type=float, default=0.7)
    ap.add_argument('--no-fence', action='store_true', help='do not wall the field edges off')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    lay = json.load(open(a.layout))
    wps = lay['waypoints']
    pts = []
    for tok in a.points.split(','):
        tok = tok.strip()
        pts.append((tok, tuple(wps[tok]) if tok in wps else tuple(float(v) for v in tok.split(':'))))
    known = [o for o in lay['obstacles'] if re.search(a.known, o['name']) and o.get('z', 0.0) <= a.alt]
    (fx0, fx1), (fy0, fy1) = lay['field']['x_range'], lay['field']['y_range']
    m = 2.0
    x0, y0 = fx0 - m, fy0 - m
    W, H = int(math.ceil((fx1 - fx0 + 2 * m) / a.res)), int(math.ceil((fy1 - fy0 + 2 * m) / a.res))

    # height map: rows Y, cols X
    hm = np.zeros((H, W), np.float32)
    yy, xx = np.mgrid[0:H, 0:W]
    X, Y = x0 + (xx + 0.5) * a.res, y0 + (yy + 0.5) * a.res
    for o in known:
        top = o.get('z', 0.0) + o['size'][2]
        if o['shape'] == 'cylinder':
            mask = np.hypot(X - o['pos'][0], Y - o['pos'][1]) <= o['size'][0] / 2.0
        else:
            c, s = math.cos(math.radians(o.get('yaw_deg', 0.0))), math.sin(math.radians(o.get('yaw_deg', 0.0)))
            dx, dy = X - o['pos'][0], Y - o['pos'][1]
            lx, ly = c * dx + s * dy, -s * dx + c * dy
            mask = (np.abs(lx) <= o['size'][0] / 2.0) & (np.abs(ly) <= o['size'][1] / 2.0)
        hm[mask] = np.maximum(hm[mask], top)
    if not a.no_fence:
        hm[(X < fx0) | (X > fx1) | (Y < fy0) | (Y > fy1)] = 50.0
    np.save(os.path.join(a.out, 'heightmap.npy'), hm)

    import hybrid_learning_path_planning as hpp
    from hybrid_learning_path_planning import HybridPlanner
    model = os.path.join(os.path.dirname(hpp.__file__), 'assets', 'waypoint_unet_v031_256.onnx')
    cfg = {
        'version': 'hybrid_learning_pp_v032',
        'model': {'backend': 'onnxruntime', 'path': model, 'input_size': 256, 'providers': ['CPUExecutionProvider']},
        'map': {'resolution_m_per_pixel': a.res},
        'grid': {'resolution_xy_m': [a.res, a.res]},
        'vehicle': {'horizontal_radius_m': a.radius, 'vertical_clearance_m': a.vclear, 'max_flight_path_angle_deg': 90.0},
        'slicing': {'z_min_m': a.alt, 'z_max_m': a.alt, 'num_layers': 1},
        'fields': {'sigma_m': a.sigma, 'rho_soft': a.rho_soft, 'rho_caution': a.rho_caution},
        'candidate_generation': {'method': 'watershed', 'watershed': {
            'marker_sigma_px': 1.2, 'marker_quantile': a.marker_q, 'region_heat_quantile': 0.75,
            'connectivity': 8, 'safety_max_nodes_per_layer': 192}},
        'graph_3d': {'edge_candidate_policy': 'all_pairs', 'pair_batch_size': 65536,
                     'backend': 'cuda_visibility_matrix', 'collision_epsilon_m': 0.001, 'max_soft_risk': 0.75},
    }
    cfg_path = os.path.join(a.out, 'pp_config.json')
    json.dump(cfg, open(cfg_path, 'w'), indent=1)
    planner = HybridPlanner.from_config(cfg_path)

    to_px = lambda x, y: ((x - x0) / a.res - 0.5, (y - y0) / a.res - 0.5)
    to_xy = lambda c, r: (x0 + (c + 0.5) * a.res, y0 + (r + 0.5) * a.res)
    polys = [inflated_polygon(o, a.radius) for o in known]
    # warm-up (CUDA context, numba JIT, ONNX session) as the on-board node does at start-up: the timed plans below
    # are what a mission request costs, the first-call time is reported separately
    (_, w0), (_, w1) = pts[0], pts[1]
    t0 = time.perf_counter()
    planner.plan(hm, (*to_px(*w0), a.alt), (*to_px(*w1), a.alt))
    warmup_ms = round((time.perf_counter() - t0) * 1000.0, 1)
    print(f'[ctrack_pp] warm-up plan {warmup_ms} ms (first call)', flush=True)
    legs, merged = [], []
    for (na, pa), (nb, pb) in zip(pts, pts[1:]):
        ca, ra = to_px(*pa); cb, rb = to_px(*pb)
        t0 = time.perf_counter()
        res = planner.plan(hm, (ca, ra, a.alt), (cb, rb, a.alt))
        wall_ms = (time.perf_counter() - t0) * 1000.0
        leg = {'from': na, 'to': nb, 'success': bool(res.success), 'reason': res.failure_reason,
               'planning_ms': round(float(res.runtime_ms.get('total_ms', wall_ms)), 1), 'wall_ms': round(wall_ms, 1),
               'timings_ms': {k: round(float(v), 2) for k, v in res.runtime_ms.items()}}
        if res.success:
            p = [to_xy(c, r) for c, r, _ in np.asarray(res.waypoints)]
            p[0], p[-1] = tuple(pa), tuple(pb)
            leg['waypoints'] = [[round(x, 3), round(y, 3)] for x, y in p]
            leg['length_m'] = round(sum(math.hypot(q[0] - o[0], q[1] - o[1]) for o, q in zip(p, p[1:])), 2)
            merged += p if not merged else p[1:]
        ref, ref_len = shortest_path(polys, tuple(pa), tuple(pb))
        leg['reference_m'] = round(ref_len, 2) if math.isfinite(ref_len) else None
        leg['reference_path'] = [[round(x, 3), round(y, 3)] for x, y in ref] if ref else None
        if res.success and leg['reference_m']:
            leg['ratio'] = round(leg['length_m'] / leg['reference_m'], 3)
        legs.append(leg)
        print(f"[ctrack_pp] {na} -> {nb}: {'ok' if res.success else 'FAILED ' + str(res.failure_reason)}, "
              f"{leg['planning_ms']} ms, {leg.get('length_m', '-')} m (reference {leg['reference_m']} m, "
              f"ratio {leg.get('ratio', '-')}), {len(leg.get('waypoints', []))} waypoints", flush=True)
    ok = all(l['success'] for l in legs)
    total = sum(l.get('length_m', 0.0) for l in legs)
    ref_total = sum(l['reference_m'] or 0.0 for l in legs)
    summary = {'layout': a.layout, 'points': [[n, list(p)] for n, p in pts], 'alt_m': a.alt, 'res_m': a.res,
               'radius_m': a.radius, 'known_obstacles': [o['name'] for o in known], 'success': ok,
               'length_m': round(total, 2), 'reference_m': round(ref_total, 2),
               'ratio': round(total / ref_total, 3) if ok and ref_total else None,
               'max_planning_ms': max(l['planning_ms'] for l in legs), 'legs': legs,
               'hybrid_version': hpp.__version__, 'warmup_ms': warmup_ms}
    json.dump(summary, open(os.path.join(a.out, 'pp_result.json'), 'w'), indent=1, ensure_ascii=False)
    if ok:
        with open(os.path.join(a.out, 'pp_path.csv'), 'w') as f:
            f.write('X,Y,h\n')
            for x, y in merged:
                f.write(f'{x:.3f},{y:.3f},{a.alt:.2f}\n')
    try:
        import cv2
        s = 6                                                    # px per metre in the figure
        img = np.full((int((fy1 - fy0 + 2 * m) * s), int((fx1 - fx0 + 2 * m) * s), 3), 245, np.uint8)
        P = lambda x, y: (int((x - x0) * s), int((y - y0) * s))
        for o in lay['obstacles']:
            col = (40, 120, 230) if o in known else (150, 150, 150)
            if o['shape'] == 'cylinder':
                cv2.circle(img, P(*o['pos']), max(int(o['size'][0] / 2 * s), 2), col, -1)
            else:
                cv2.fillPoly(img, [np.array([P(*c) for c in box_corners(o)], np.int32)], col)
        for p in polys:
            cv2.polylines(img, [np.array([P(*c) for c in p], np.int32)], True, (90, 160, 250), 1)
        for l in legs:
            if l.get('reference_path'):
                cv2.polylines(img, [np.array([P(*c) for c in l['reference_path']], np.int32)], False, (120, 200, 120), 1)
        if merged:
            cv2.polylines(img, [np.array([P(*c) for c in merged], np.int32)], False, (200, 60, 20), 2)
            for c in merged:
                cv2.circle(img, P(*c), 3, (200, 60, 20), -1)
        for n, p in pts:
            cv2.circle(img, P(*p), 5, (0, 0, 0), 2)
            cv2.putText(img, n, (P(*p)[0] + 6, P(*p)[1] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1)
        cv2.putText(img, f"Hybrid PP {total:.1f} m vs reference {ref_total:.1f} m (x{total / ref_total if ref_total else 0:.2f}), "
                    f"max plan {summary['max_planning_ms']:.0f} ms", (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        cv2.imwrite(os.path.join(a.out, 'pp_route.png'), img)
    except Exception as e:                                    # the figure is optional
        print(f'[ctrack_pp] figure skipped: {e}')
    print(f"[ctrack_pp] {'OK' if ok else 'FAILED'}: {total:.1f} m vs reference {ref_total:.1f} m"
          f"{' (ratio %.3f)' % (total / ref_total) if ok and ref_total else ''}, max planning {summary['max_planning_ms']} ms -> {a.out}")
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
