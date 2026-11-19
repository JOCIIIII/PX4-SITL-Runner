#!/usr/bin/env python3
"""Path-following error of a test map run against its planned path (final demonstration scenarios A, B, D; RFP item:
path error <= 1.0 m, also in the fan wind).

The track is the simulator's pose (airsim_odom.csv, the recorder's) when there is one, else PX4's estimate (as
scripts/dev/scenario_clearance.py). Only the part flown along the path counts: from the first sample within 1.5 m of it
at > 1 m/s to the last sample before the path's end (--end-radius), so the take-off climb and the landing do not.

usage: scenario_path_metrics.py RUN_DIR PATH_CSV [--zone X0,X1,Y0,Y1] [--end-radius 3] [--json OUT]
  PATH_CSV: layout X,Y[,h] rows (scenario_pp.py's pp_path.csv, scenario_demo.sh's course files)
  --zone: report the error inside this layout box separately (the fan wind zone B: 69,89,3,23)
"""
import argparse
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from scenario_clearance import load_track  # noqa: E402


def read_path(p):
    pts = []
    for line in open(p):
        line = line.split('#')[0].strip()
        try:
            v = [float(c) for c in line.split(',') if c.strip()]
        except ValueError:
            continue
        if len(v) >= 2:
            pts.append(v[:2])
    return np.array(pts, float)


def dist_to_polyline(P, X, Y):
    """distance of each (X, Y) to the polyline P and the arc length of the closest point"""
    best = np.full(len(X), np.inf)
    s_best = np.zeros(len(X))
    s0 = 0.0
    for a, b in zip(P, P[1:]):
        ab = b - a
        L = float(np.hypot(*ab))
        if L < 1e-9:
            continue
        u = np.clip(((X - a[0]) * ab[0] + (Y - a[1]) * ab[1]) / (L * L), 0.0, 1.0)
        d = np.hypot(a[0] + u * ab[0] - X, a[1] + u * ab[1] - Y)
        m = d < best
        best[m], s_best[m] = d[m], s0 + u[m] * L
        s0 += L
    return best, s_best, s0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('run_dir')
    ap.add_argument('path_csv')
    ap.add_argument('--zone', default=None)
    ap.add_argument('--end-radius', type=float, default=3.0)
    ap.add_argument('--json', default=None)
    a = ap.parse_args()
    run = open(os.path.join(a.run_dir, 'scenario_run.txt')).read().split()
    frm = run[0]
    lay_path = os.path.join(a.run_dir, 'layout_used.json')
    if os.path.exists(lay_path):
        wps = json.load(open(lay_path))['waypoints']
    else:
        from scenario_clearance import DEF_LAYOUT
        wps = json.load(open(DEF_LAYOUT))['waypoints']
    fx, fy = wps[frm] if frm in wps else map(float, frm.split(','))
    rest = {'x500': 0.23}.get(run[8] if len(run) > 8 else 'x8', 0.44)
    t, pn, pe, pd, armed, v = load_track(a.run_dir)
    X, Y, H = fx + pe, fy - pn, rest - pd
    src = 'PX4 estimate'
    truth = os.path.join(a.run_dir, 'airsim_odom.csv')
    if os.path.exists(truth):
        g = np.genfromtxt(truth, delimiter=',', names=True)
        X, Y, H = (np.interp(t, g['t'], g[k]) for k in ('x', 'y', 'z'))
        Y = -Y
        src = 'simulator pose'
    P = read_path(a.path_csv)
    d, s, L = dist_to_polyline(P, X, Y)
    fly = H > rest + 0.3
    on = fly & (d < 1.5) & (v > 1.0)
    if not on.any():
        print(f'{os.path.basename(a.run_dir)}: never flew along the path ({src})')
        sys.exit(1)
    i0 = int(np.argmax(on))
    # the end: within end-radius of the path's last point after half of the path (closed courses end where
    # they start, so the start itself must not count)
    d_end = np.hypot(X - P[-1][0], Y - P[-1][1])
    left = np.nonzero((np.arange(len(t)) > i0) & (d_end > a.end_radius + 1.0))[0]
    first_out = int(left[0]) if len(left) else len(t)
    near_end = fly & (d_end < a.end_radius) & (s > 0.5 * L) & (np.arange(len(t)) > first_out)
    i1 = int(np.argmax(near_end)) if near_end.any() else int(np.nonzero(fly)[0][-1])
    sl = slice(i0, i1 + 1)
    e, sp = d[sl], v[sl]
    res = {'run': os.path.basename(a.run_dir.rstrip('/')), 'source': src, 'path_length_m': round(L, 1),
           'covered_m': round(float(s[sl].max() - s[sl].min()), 1), 'reached_end': bool(near_end.any()),
           'time_s': round(float(t[i1] - t[i0]), 1),
           'xte_mean_m': round(float(e.mean()), 3), 'xte_p95_m': round(float(np.percentile(e, 95)), 3),
           'xte_max_m': round(float(e.max()), 3), 'within_1m_pct': round(100.0 * float((e <= 1.0).mean()), 1),
           'speed_mean': round(float(sp.mean()), 2), 'speed_max': round(float(sp.max()), 2),
           'height_mean_m': round(float(H[sl].mean()), 2), 'height_min_m': round(float(H[sl].min()), 2),
           'height_max_m': round(float(H[sl].max()), 2)}
    if a.zone:
        x0, x1, y0, y1 = (float(c) for c in a.zone.split(','))
        z = (X[sl] >= x0) & (X[sl] <= x1) & (Y[sl] >= y0) & (Y[sl] <= y1)
        if z.any():
            res.update({'zone_time_s': round(float(z.sum()) * float(np.median(np.diff(t))), 1),
                        'zone_xte_mean_m': round(float(e[z].mean()), 3), 'zone_xte_max_m': round(float(e[z].max()), 3),
                        'zone_within_1m_pct': round(100.0 * float((e[z] <= 1.0).mean()), 1),
                        'zone_speed_mean': round(float(sp[z].mean()), 2)})
    print(f"{res['run']}: path {res['path_length_m']} m, covered {res['covered_m']} m in {res['time_s']} s "
          f"({'reached the end' if res['reached_end'] else 'did NOT reach the end'}; {src})")
    print(f"  cross-track error mean {res['xte_mean_m']} / p95 {res['xte_p95_m']} / max {res['xte_max_m']} m, "
          f"{res['within_1m_pct']} % within 1 m; speed mean {res['speed_mean']} max {res['speed_max']} m/s; "
          f"height {res['height_min_m']}..{res['height_max_m']} m")
    if 'zone_xte_max_m' in res:
        print(f"  in the wind zone ({res['zone_time_s']} s): error mean {res['zone_xte_mean_m']} max "
              f"{res['zone_xte_max_m']} m, {res['zone_within_1m_pct']} % within 1 m, speed {res['zone_speed_mean']} m/s")
    if a.json:
        json.dump(res, open(a.json, 'w'), indent=1)


if __name__ == '__main__':
    main()
