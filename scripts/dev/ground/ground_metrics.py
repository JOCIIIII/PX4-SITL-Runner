#!/usr/bin/env python3
"""Result of one ground_run.sh mission (runs/NAME): path error of the estimate and of the simulator's true
position against the waypoint polyline, the share of the run flown by MPPI, speed, completion.
    ground_metrics.py RUN_DIR [RUN_DIR ...] [--json]
Path error: horizontal distance to the polyline (out/wp_local.csv), searched around the progress along the
path (so a second lap or a crossing is not taken for the first), from the start of FOLLOW to the first pass of
the end point. The true position comes from PX4's ulog (vehicle_local_position_groundtruth) shifted into the
estimator's frame by the offset at rest before the start."""
import csv
import glob
import json
import math
import os
import sys

import numpy as np


def path_error(P, W, back=2.0, ahead=6.0):
    seg = W[1:] - W[:-1]
    L = np.hypot(seg[:, 0], seg[:, 1])
    cum = np.concatenate([[0.0], np.cumsum(L)])
    e = np.zeros(len(P))
    s_at = np.zeros(len(P))
    s = 0.0
    for k, p in enumerate(P):
        i0 = max(int(np.searchsorted(cum, s - back) - 1), 0)
        i1 = min(int(np.searchsorted(cum, s + ahead)), len(seg))
        best = (1e9, s)
        for i in range(i0, max(i1, i0 + 1)):
            if L[i] < 1e-9:
                continue
            t = min(max(float(np.dot(p - W[i], seg[i])) / (L[i] * L[i]), 0.0), 1.0)
            d = float(np.hypot(*(p - (W[i] + t * seg[i]))))
            if d < best[0]:
                best = (d, cum[i] + t * L[i])
        e[k], s = best
        s_at[k] = s
    return e, s_at, float(cum[-1])


def one(run):
    out = {'run': os.path.basename(run.rstrip('/'))}
    res = os.path.join(run, 'out', 'ground_driver_result.txt')
    out['result'] = open(res).read().strip() if os.path.exists(res) else 'no_result'
    rows = list(csv.DictReader(open(os.path.join(run, 'out', 'ground_driver.csv'))))
    F = [r for r in rows if r['state'] == 'FOLLOW']
    if not F:
        return out
    W = np.array([[float(v) for v in l.split(',')[:2]] for l in open(os.path.join(run, 'out', 'wp_local.csv')) if l.strip()])
    t = np.array([float(r['t']) for r in F])
    P = np.array([[float(r['x']), float(r['y'])] for r in F])
    v = np.array([float(r['speed_fwd']) for r in F])
    guid = np.array([int(r['guid_used']) for r in F])
    cmd_ok = np.array([float(r['speed_cmd']) > 0 or int(r['guid_used']) > 0 for r in F])
    e, s, length = path_error(P, W, back=max(2.0, 0.05 * length_of(W)), ahead=max(6.0, 0.15 * length_of(W)))
    end = np.flatnonzero(s >= length - 1e-3)
    n_end = int(end[0]) if len(end) else len(F) - 1
    m = np.arange(len(F)) <= n_end
    mm = m & (guid >= 1)
    out.update(path_len_m=round(length, 1), follow_s=round(float(t[-1] - t[0]), 1), reached_end=bool(len(end)),
               e_p95=round(float(np.percentile(e[m], 95)), 3), e_max=round(float(e[m].max()), 3),
               e_mean=round(float(e[m].mean()), 3),
               mppi_pct=round(100.0 * float(mm.sum()) / max(int(m.sum()), 1), 1),
               e_p95_mppi=round(float(np.percentile(e[mm], 95)), 3) if mm.sum() > 10 else None,
               e_max_mppi=round(float(e[mm].max()), 3) if mm.sum() > 10 else None,
               speed_mean=round(float(v[m].mean()), 2), speed_max=round(float(v[m].max()), 2),
               end_err_m=round(float(np.hypot(*(P[-1] - W[-1]))), 2), start_offset_m=round(float(e[0]), 2))
    try:
        from pyulog import ULog
        u = ULog(glob.glob(os.path.join(run, 'px4_rootfs', 'log', '*', '*.ulg'))[0])
        d = {x.name: x.data for x in u.data_list if x.multi_id == 0}
        lp, gt = d['vehicle_local_position'], d['vehicle_local_position_groundtruth']
        us = np.array([float(r['px4_us']) for r in F])
        # the driver's own clock instead of the per-row estimator timestamps: a single bad timestamp in the stream
        # mapped one row 35 m away (i_usv_pp_g2)
        tt = np.array([float(r['t']) for r in F])
        us = float(np.median(us - tt * 1e6)) + tt * 1e6
        # the driver's timestamps came over uXRCE-DDS (time-synchronised to the agent's clock), the ulog's
        # are PX4 boot time: map them through the estimate itself, matching positions while moving
        k = np.flatnonzero(v > 0.5 * max(float(v.max()), 0.1))[::25][:60]
        offs = []
        for i in k:
            j = int(np.argmin(np.abs(lp['x'] - P[i, 0]) + np.abs(lp['y'] - P[i, 1])))
            offs.append(us[i] - float(lp['timestamp'][j]))
        us = us - float(np.median(offs))
        # offset of the two local frames: the second before FOLLOW starts (vehicle at rest)
        m0 = (lp['timestamp'] > us[0] - 1.5e6) & (lp['timestamp'] < us[0] + 0.2e6)
        ox = float(np.mean(lp['x'][m0] - np.interp(lp['timestamp'][m0], gt['timestamp'], gt['x'])))
        oy = float(np.mean(lp['y'][m0] - np.interp(lp['timestamp'][m0], gt['timestamp'], gt['y'])))
        T = np.column_stack([np.interp(us, gt['timestamp'], gt['x']) + ox, np.interp(us, gt['timestamp'], gt['y']) + oy])
        et, _, _ = path_error(T, W, back=max(2.0, 0.05 * length), ahead=max(6.0, 0.15 * length))
        out.update(e_true_p95=round(float(np.percentile(et[m], 95)), 3), e_true_max=round(float(et[m].max()), 3),
                   est_err_max=round(float(np.hypot(*(P - T).T)[m].max()), 3),
                   end_err_true_m=round(float(np.hypot(*(T[-1] - W[-1]))), 2))
    except Exception as ex:                      # no pyulog / no log: the estimate-based numbers stand alone
        out['truth'] = f'unavailable ({type(ex).__name__})'
    return out


def length_of(W):
    return float(np.sum(np.hypot(*(W[1:] - W[:-1]).T)))


def main():
    runs = [a for a in sys.argv[1:] if not a.startswith('--')]
    res = [one(r) for r in runs]
    if '--json' in sys.argv:
        print(json.dumps(res, indent=1))
        return
    for r in res:
        print(' | '.join(f'{k} {v}' for k, v in r.items()))


if __name__ == '__main__':
    main()
