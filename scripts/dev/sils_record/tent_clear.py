#!/usr/bin/env python3
"""Closest approach of a ctrack run to the spawned canopy tent (tent17.json): legs (cylinders) and roof slabs, X8 slab
(-0.44/+0.09 m around the centre, prop radius 0.77 m). Uses ctrack_clearance.load_track + airsim_odom.csv when present."""
import sys, json, math, os  # noqa: E401
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from ctrack_clearance import load_track, box_dist
run = sys.argv[1]
T = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'tent17.json')))
R, BELOW, ABOVE, REST = 0.77, 0.44, 0.09, 0.44
frm = open(os.path.join(run, 'ctrack_run.txt')).read().split()[0]
fx, fy = map(float, frm.split(','))
t, pn, pe, pd, armed, v = load_track(run)
X, Y, H = fx + pe, fy - pn, REST - pd
g = os.path.join(run, 'airsim_odom.csv')
if os.path.exists(g):
    a = np.genfromtxt(g, delimiter=',', names=True)
    X, Y, H = (np.interp(t, a['t'], a[k]) for k in ('x', 'y', 'z')); Y = -Y
fly = H > REST + 0.3
lo, hi = H - BELOW, H + ABOVE
worst = []
for name, x, y, zc, (sx, sy, sz) in T['parts']:
    z0, z1 = zc - sz / 2, zc + sz / 2
    if name.startswith('Tent17_leg'):
        dh = np.maximum(np.hypot(X - x, Y - y) - sx / 2, 0)
    else:
        dh = box_dist(X, Y, x, y, sx, sy, 0.0)
    dz = np.maximum(np.maximum(z0 - hi, lo - z1), 0)          # vertical gap between the vehicle slab and the part
    # contact when the prop disc (radius R, at the centre height band) overlaps the part
    m = np.where(fly, np.where(dz > 0, np.hypot(np.maximum(dh - R, 0), dz) * np.sign(1) , dh - R), np.inf)
    i = int(np.argmin(m))
    worst.append((m[i], name, dh[i], dz[i], X[i], Y[i], H[i], v[i]))
worst.sort()
for w in worst[:5]:
    print(f'  {w[1]:14s} margin {w[0]:+.2f} m (horiz {w[2]:.2f}, vert gap {w[3]:.2f}) at ({w[4]:.1f},{w[5]:.1f}) h {w[6]:.2f} v {w[7]:.1f}')
cx, cy = T['centre']
near = fly & (np.abs(X - cx) < 4)
if near.any():
    print(f'  passing the tent (|X-{cx}|<4): Y {Y[near].min():.1f}..{Y[near].max():.1f}, h {H[near].min():.2f}..{H[near].max():.2f}, '
          f'speed {v[near].min():.1f}..{v[near].max():.1f}; under the roof: {bool((near & (np.abs(Y-cy) < 3) & (np.abs(X-cx) < 1.5)).any())}')
print(f'  end ({X[-1]:.1f},{Y[-1]:.1f}), max speed {v[fly].max():.1f}')
