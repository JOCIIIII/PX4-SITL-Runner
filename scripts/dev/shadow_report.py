#!/usr/bin/env python3
"""Shadow test report (scripts/dev/onboard_real.sh shadow): for each run of the policy (one per engagement /
/shadow/reset) - where it was carried, the nearest return it saw, how far from the obstacle it first
steered off the line to its goal and to which side, and its speed command; plus a top-down picture of the
returns at the carried height (cloud_map.npz), the track and the policy's waypoints.
usage: shadow_report.py RUN_DIR [--png out.png] [--since UNIX_T]"""
import argparse
import os

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument('run')
ap.add_argument('--png', default=None)
ap.add_argument('--since', type=float, default=0.0)
ap.add_argument('--steer-deg', type=float, default=15.0, help='waypoint off the goal line by this much = steering')
a = ap.parse_args()
D = a.run.rstrip('/') + '/'

d = np.genfromtxt(D + 'noclimb_ca.csv', delimiter=',', names=True, dtype=None, encoding=None)
f = lambda c: np.array([float(x) if x not in ('', None) else np.nan for x in d[c]])   # noqa: E731
t, mode = f('t'), np.array(d['mode']).astype(str)
x, y, z, vx, vy = f('x'), f('y'), f('z'), f('vx'), f('vy')
wx, wy, dmin, spd, rem = f('wp_cmd_x'), f('wp_cmd_y'), f('d_img_min'), f('speed_cmd'), f('remaining')
ref = f('ref_bearing')
keep = (t >= a.since) & (mode == 'CA') & np.isfinite(wx)
# a new run starts where 'remaining' jumps back up (reset -> new goal ahead)
starts = np.flatnonzero(keep & np.r_[True, np.diff(rem) > 5.0])
print(f'{D}: {keep.sum()} policy steps, {len(starts)} run(s)')
segs = []
for i, s in enumerate(starts):
    e = starts[i + 1] if i + 1 < len(starts) else len(t)
    k = np.flatnonzero(keep[s:e]) + s
    if len(k) < 10:
        continue
    goal_dir = ref[k[0]]                                   # the line to the goal at engagement (ENU rad)
    wb = np.arctan2(wy[k] - y[k], wx[k] - x[k])            # where the policy sends it
    off = np.degrees((wb - goal_dir + np.pi) % (2 * np.pi) - np.pi)
    walked = np.hypot(x[k] - x[k[0]], y[k] - y[k[0]])
    v = np.hypot(vx[k], vy[k])
    j = np.flatnonzero((np.abs(off) > a.steer_deg) & (walked > 0.5))
    print(f'run {len(segs) + 1}: {t[k[-1]] - t[k[0]]:.0f} s, walked {walked.max():.1f} m at {np.median(v[v > 0.2]) if (v > 0.2).any() else 0:.2f} m/s')
    print(f'  nearest return seen: {np.nanmin(dmin[k]):.2f} m')
    if j.size:
        side = 'left' if off[j[0]] > 0 else 'right'
        print(f'  first steered {abs(off[j[0]]):.0f} deg {side} after {walked[j[0]]:.1f} m, nearest return then {dmin[k][j[0]]:.2f} m')
    else:
        print(f'  never steered more than {a.steer_deg:.0f} deg off the goal line')
    print(f'  speed command: median {np.nanmedian(spd[k]):.2f}, min {np.nanmin(spd[k]):.2f} m/s')
    segs.append((k, off))

if a.png and segs:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(9, 9))
    if os.path.exists(D + 'cloud_map.npz'):
        c = np.load(D + 'cloud_map.npz')['xyz']
        zc = np.nanmedian(z[keep])
        band = (c[:, 2] > zc - 0.6) & (c[:, 2] < zc + 1.0)
        ax.scatter(c[band, 0], c[band, 1], s=1, c='0.4', label='returns at the carried height (-0.6..+1.0 m)')
    for n, (k, off) in enumerate(segs):
        ax.plot(x[k], y[k], '-', lw=2, label=f'run {n + 1} track')
        for q in k[::10]:
            ax.annotate('', xy=(wx[q], wy[q]), xytext=(x[q], y[q]),
                        arrowprops=dict(arrowstyle='->', color='tab:red', lw=0.8))
    ax.plot([], [], color='tab:red', label='policy waypoint (every 0.5 s)')
    span = np.r_[x[keep], y[keep]]
    m = 6.0
    ax.set_xlim(np.nanmin(x[keep]) - m, np.nanmax(x[keep]) + m)
    ax.set_ylim(np.nanmin(y[keep]) - m, np.nanmax(y[keep]) + m)
    ax.set_aspect('equal')
    ax.set_xlabel('east (m)')
    ax.set_ylabel('north (m)')
    ax.grid(alpha=0.3)
    ax.legend(loc='upper left', fontsize=8)
    ax.set_title(os.path.basename(D.rstrip('/')))
    fig.savefig(a.png, dpi=110, bbox_inches='tight')
    print(f'picture: {a.png}')
