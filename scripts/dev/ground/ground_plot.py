#!/usr/bin/env python3
"""Map of one ground_run.sh mission: the waypoints PathFollowing got, the estimated and the simulator's true track,
the vehicle footprint every few seconds; for a layout run also the obstacles and the planner's inflation.
North up (x = east, y = north). One plot per image, no title (for slides).
  ground_plot.py RUN_DIR [--layout LAYOUT --vehicle ugv|usv --radius R] [--out PNG] [--every S]"""
import argparse
import csv
import glob
import json
import math
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np

for f in (os.path.expanduser('~/.local/share/fonts/NanumSquareR.ttf'),
          os.path.expanduser('~/.local/share/fonts/NanumSquareL.ttf')):
    if os.path.exists(f):
        font_manager.fontManager.addfont(f)
        plt.rcParams['font.family'] = font_manager.FontProperties(fname=f).get_name()
        break
plt.rcParams['axes.unicode_minus'] = False
FOOT = {'ugv': (0.50, 0.45), 'usv': (4.9, 2.5)}


def truth_local(run, F, P):
    """simulator truth in the estimator's local frame at the driver's rows (as ground_metrics.py)"""
    from pyulog import ULog
    u = ULog(glob.glob(os.path.join(run, 'px4_rootfs', 'log', '*', '*.ulg'))[0])
    d = {x.name: x.data for x in u.data_list if x.multi_id == 0}
    lp, gt, ag = d['vehicle_local_position'], d['vehicle_local_position_groundtruth'], d['vehicle_attitude_groundtruth']
    us = np.array([float(r['px4_us']) for r in F])
    # the driver's own clock instead of the per-row estimator timestamps: a single bad timestamp in the stream
    # mapped one row 35 m away (i_usv_pp_g2)
    tt = np.array([float(r['t']) for r in F])
    us = float(np.median(us - tt * 1e6)) + tt * 1e6
    v = np.array([float(r['speed_fwd']) for r in F])
    k = np.flatnonzero(v > 0.5 * max(float(v.max()), 0.1))[::25][:60]
    offs = [us[i] - float(lp['timestamp'][int(np.argmin(np.abs(lp['x'] - P[i, 0]) + np.abs(lp['y'] - P[i, 1])))]) for i in k]
    us = us - float(np.median(offs))
    m0 = (lp['timestamp'] > us[0] - 1.5e6) & (lp['timestamp'] < us[0] + 0.2e6)
    ox = float(np.mean(lp['x'][m0] - np.interp(lp['timestamp'][m0], gt['timestamp'], gt['x'])))
    oy = float(np.mean(lp['y'][m0] - np.interp(lp['timestamp'][m0], gt['timestamp'], gt['y'])))
    w, x, y, z = (np.interp(us, ag['timestamp'], ag[f'q[{i}]']) for i in range(4))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.column_stack([np.interp(us, gt['timestamp'], gt['x']) + ox, np.interp(us, gt['timestamp'], gt['y']) + oy]), yaw


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run')
    ap.add_argument('--layout', default='')
    ap.add_argument('--vehicle', default='ugv')
    ap.add_argument('--radius', type=float, default=0.0, help='planner inflation radius to draw [m]')
    ap.add_argument('--every', type=float, default=0.0, help='footprint every S seconds (0: auto)')
    ap.add_argument('--out', default='')
    a = ap.parse_args()
    F = [r for r in csv.DictReader(open(os.path.join(a.run, 'out', 'ground_driver.csv'))) if r['state'] == 'FOLLOW']
    W = np.array([[float(v) for v in l.split(',')[:2]] for l in open(os.path.join(a.run, 'out', 'wp_local.csv')) if l.strip()])
    P = np.array([[float(r['x']), float(r['y'])] for r in F])
    T, yaw = truth_local(a.run, F, P)
    off = np.zeros(2)
    lay = None
    if a.layout:
        lay = json.load(open(a.layout))
        off = np.array(lay['waypoints']['START'], dtype=float)
    W, P, T = W + off, P + off, T + off
    fig, ax = plt.subplots(figsize=(7, 7))
    if lay:
        for o in lay['obstacles']:
            if o['shape'] == 'cylinder':
                ax.add_patch(plt.Circle((o['pos'][1], o['pos'][0]), o['size'][0] / 2, color='#d9822b', zorder=2))
                if a.radius > 0:
                    ax.add_patch(plt.Circle((o['pos'][1], o['pos'][0]), o['size'][0] / 2 + a.radius, fill=False,
                                            ls='--', lw=0.6, color='#d9822b', zorder=2))
            else:
                c, s = math.cos(math.radians(o.get('yaw_deg', 0))), math.sin(math.radians(o.get('yaw_deg', 0)))
                for grow, kw in ((0.0, dict(color='#d9822b', zorder=2)), (a.radius, dict(fill=False, ls='--', lw=0.6, color='#d9822b', zorder=2))):
                    if grow == 0.0 or a.radius > 0:
                        hx, hy = o['size'][0] / 2 + grow, o['size'][1] / 2 + grow
                        pts = [(o['pos'][0] + c * x - s * y, o['pos'][1] + s * x + c * y) for x, y in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy))]
                        ax.add_patch(plt.Polygon([(q[1], q[0]) for q in pts], closed=True, **kw))
    ax.plot(W[:, 1], W[:, 0], '-', color='#2b6cb0', lw=1.2, label='경로 (경로추종 입력)', zorder=3)
    ax.plot(T[:, 1], T[:, 0], '-', color='#c53030', lw=1.4, label='실제 궤적 (시뮬레이터 참값)', zorder=4)
    ax.plot(P[:, 1], P[:, 0], ':', color='#555555', lw=1.0, label='추정 궤적 (PX4 EKF)', zorder=4)
    L, Wd = FOOT[a.vehicle]
    t = np.array([float(r['t']) for r in F])
    every = a.every or max(2.0, (t[-1] - t[0]) / 12.0)
    nxt = t[0]
    for i in range(len(t)):
        if t[i] >= nxt:
            c, s = math.cos(yaw[i]), math.sin(yaw[i])
            pts = [(T[i, 0] + c * x - s * y, T[i, 1] + s * x + c * y) for x, y in ((-L / 2, -Wd / 2), (L / 2, -Wd / 2), (L / 2, Wd / 2), (-L / 2, Wd / 2))]
            ax.add_patch(plt.Polygon([(q[1], q[0]) for q in pts], closed=True, fill=False, lw=0.8, color='#c53030', zorder=5))
            nxt += every
    ax.plot(W[0, 1], W[0, 0], 'ko', ms=5, zorder=6)
    ax.plot(W[-1, 1], W[-1, 0], 'k*', ms=10, zorder=6)
    ax.set_aspect('equal')
    ax.set_xlabel('동쪽 [m]')
    ax.set_ylabel('북쪽 [m]')
    if lay:
        (x0, x1), (y0, y1) = lay['field']['x_range'], lay['field']['y_range']
        m = 0.03 * max(x1 - x0, y1 - y0)
        ax.set_xlim(y0 - m, y1 + m)
        ax.set_ylim(x0 - m, x1 + m)
    ax.grid(alpha=0.3)
    ax.legend(loc='best', fontsize=8)
    out = a.out or os.path.join(a.run, 'out', 'map.png')
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    print(out)


if __name__ == '__main__':
    main()
