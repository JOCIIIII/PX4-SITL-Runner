#!/usr/bin/env python3
"""How well the lidar saw the poles in a C-Track run: per lidar scan of the avoidance node's dump
(noclimb_ca_dump.npz), the returns within --r of each pole's axis, against the distance to the pole, whether it was
ahead, and the vehicle's pitch. Answers "is a thin (20 mm) pole seen, and does it vanish when the nose comes up?".

usage: ctrack_pole_hits.py RUN_DIR [--layout LAYOUT] [--r 0.15]
  the dump's points and pose are PX4 local ENU from the take-off point FROM (ctrack_run.txt): X = fx + x, Y = fy - y
"""
import argparse
import json
import math
import os
import re

import numpy as np


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('run_dir')
    ap.add_argument('--layout', default=None)
    ap.add_argument('--r', type=float, default=0.15, help='a return within this of the pole axis is a pole hit [m]')
    ap.add_argument('--ahead-deg', type=float, default=30.0)
    a = ap.parse_args()
    run = open(os.path.join(a.run_dir, 'ctrack_run.txt')).read().split()
    if a.layout is None:
        from ctrack_clearance import DEF_LAYOUT
        a.layout = DEF_LAYOUT
    lay = json.load(open(a.layout))
    wps = lay['waypoints']
    frm = run[0]
    fx, fy = wps[frm] if frm in wps else map(float, frm.split(','))
    poles = [(o['name'], o['pos'][0] - fx, fy - o['pos'][1], o['size'][0], o['size'][2]) for o in lay['obstacles']
             if re.match(r'^Pole_', o['name'])]
    d = np.load(os.path.join(a.run_dir, 'noclimb_ca_dump.npz'))
    st, sn, xyz = d['scan_t'], d['scan_n'], d['scan_xyz']
    off = np.concatenate([[0], np.cumsum(sn)])
    t, pos, vel, R = d['t'], d['pos'], d['vel'], d['R']
    rows = []
    for i in range(len(st)):
        k = int(np.argmin(np.abs(t - st[i])))
        if abs(t[k] - st[i]) > 0.2:
            continue
        p = xyz[off[i]:off[i + 1]]
        Rk = R[k]
        pitch = math.degrees(math.asin(float(np.clip(Rk[2, 0], -1, 1))))    # body x up = nose up (FLU -> ENU)
        v = vel[k][:2]
        hd = math.atan2(v[1], v[0]) if np.hypot(*v) > 0.5 else math.atan2(Rk[1, 0], Rk[0, 0])
        for name, px, py, dia, h in poles:
            dx, dy = px - pos[k][0], py - pos[k][1]
            dist = math.hypot(dx, dy)
            if dist > 15.0:
                continue
            brg = abs(math.degrees(math.remainder(math.atan2(dy, dx) - hd, 2 * math.pi)))
            near = np.hypot(p[:, 0] - px, p[:, 1] - py) < a.r + dia / 2 if len(p) else np.zeros(0, bool)
            rows.append((name, dist, brg, pitch, int(near.sum())))
    if not rows:
        print('no scans near poles'); return
    name, dist, brg, pitch, hits = (np.array(c) for c in zip(*rows))
    ahead = brg < a.ahead_deg
    print(f'{os.path.basename(a.run_dir.rstrip("/"))}: {len(set(st))} scans, pole diameter {sorted(set(p[3] for p in poles))} m; '
          f'scans with a pole within 15 m ahead (+-{a.ahead_deg:.0f} deg): {int(ahead.sum())}')
    print('  distance  scans  hits/scan  0-hit scans | pitch>+8 deg: scans 0-hit')
    for lo, hi in ((0, 3), (3, 5), (5, 7), (7, 10), (10, 15)):
        m = ahead & (dist >= lo) & (dist < hi)
        if not m.any():
            continue
        up = m & (pitch > 8)
        print(f'  {lo:>2}-{hi:<2} m  {m.sum():5d}  {hits[m].mean():8.2f}   {100 * (hits[m] == 0).mean():6.0f} %   |'
              f'  {up.sum():5d} {100 * (hits[up] == 0).mean() if up.any() else float("nan"):5.0f} %')
    for n in sorted(set(name)):
        m = ahead & (name == n)
        if not m.any():
            continue
        seen = m & (hits > 0)
        first = dist[seen].max() if seen.any() else float('nan')
        print(f'  {n}: first seen ahead at {first:4.1f} m, closest {dist[name == n].min():4.1f} m '
              f'(centre to axis), scans ahead with 0 hits {100 * (hits[m] == 0).mean():3.0f} %')


if __name__ == '__main__':
    import sys
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    main()
