#!/usr/bin/env python3
"""How well PathFollowing (MPPI) held a waypoint path in wind: one line per run dir.

Path = the waypoint CSV the run flew (pf_test PF_TEST_WP_CSV; x north, y east, z up, relative to
home). Over the path phase (from "Waypoints sent" to "Path following COMPLETE" in
path_following_test.log): cross-track error to the polyline, height error, ground speed per leg
(heading of the leg vs the wind: head / tail / cross), tilt and thrust at the limit (px4_sitl.ulg).

usage: pf_wind_metrics.py RUN_DIR [RUN_DIR ...] [--wp CSV] [--wind-dir-deg 0] [--speed V]
       --wind-dir-deg: direction the wind blows TOWARD, compass deg (Gazebo windDirectionMean 0 1 0 = 0, north)
"""
import argparse
import math
import os
import re

import numpy as np


def load_csv(path):
    return np.genfromtxt(path, delimiter=',', names=True)


def path_times(run):
    t_sent = t_done = None
    log = os.path.join(run, 'path_following_test.log')
    if os.path.exists(log):
        for line in open(log, errors='ignore'):
            # search, not match: the status printer's clear-screen codes (ESC[2J ESC[H) can sit in front of a line
            m = re.search(r'\[\w+\] \[(\d+\.\d+)\]', line)
            if not m:
                continue
            if 'Waypoints sent' in line and t_sent is None:
                t_sent = float(m.group(1))
            if 'Path following COMPLETE' in line and t_done is None:
                t_done = float(m.group(1))
    return t_sent, t_done


def seg_dist(p, a, b):
    ab = b - a
    t = np.clip(((p - a) @ ab) / max(ab @ ab, 1e-9), 0.0, 1.0)
    q = a + t[:, None] * ab
    return np.hypot(*(p - q).T), t


def ulog_stats(run, t0, t1, t_ros_ref):
    f = os.path.join(run, 'px4_sitl.ulg')
    if not os.path.exists(f):
        return {}
    try:
        from pyulog import ULog
        u = ULog(f, ['vehicle_attitude', 'vehicle_thrust_setpoint', 'vehicle_local_position'])
    except Exception:
        return {}
    def topic(name):
        for d in u.data_list:
            if d.name == name and d.multi_id == 0:
                return d.data
        return None
    lp = topic('vehicle_local_position')
    att = topic('vehicle_attitude')
    th = topic('vehicle_thrust_setpoint')
    if lp is None or att is None:
        return {}
    # align ulog (boot) time to ROS time by the x/y track: ROS t of px4.csv vs ulog local position
    off = t_ros_ref
    out = {}
    ta = att['timestamp'] * 1e-6 + off
    m = (ta >= t0) & (ta <= t1)
    q0, q1, q2, q3 = (att[f'q[{i}]'][m] for i in range(4))
    tilt = np.degrees(np.arccos(np.clip(1 - 2 * (q1 * q1 + q2 * q2), -1, 1)))
    if tilt.size:
        out['tilt_max'] = float(tilt.max())
        out['tilt_p95'] = float(np.percentile(tilt, 95))
        out['tilt_ge34_pct'] = float(100 * np.mean(tilt >= 34.0))
    if th is not None:
        tt = th['timestamp'] * 1e-6 + off
        mt = (tt >= t0) & (tt <= t1)
        mag = np.sqrt(sum(th[f'xyz[{i}]'][mt] ** 2 for i in range(3)))
        if mag.size:
            out['thrust_max'] = float(mag.max())
            out['thrust_sat_pct'] = float(100 * np.mean(mag >= 0.99))
    return out


def ulog_offset(run, px):
    """ROS time - ulog time, from the first motion in both tracks (px4.csv is ROS-stamped)."""
    f = os.path.join(run, 'px4_sitl.ulg')
    try:
        from pyulog import ULog
        u = ULog(f, ['vehicle_local_position'])
        lp = [d for d in u.data_list if d.name == 'vehicle_local_position' and d.multi_id == 0][0].data
    except Exception:
        return None
    zu = lp['z']; tu = lp['timestamp'] * 1e-6
    zr = px['z']; tr = px['t']
    ku = np.argmax(zu < -1.0); kr = np.argmax(zr < -1.0)            # crossing 1 m up on take-off
    if zu[ku] >= -1.0 or zr[kr] >= -1.0:
        return None
    return float(tr[kr] - tu[ku])


def analyze(run, wp_csv, wind_dir_deg, speed):
    px = load_csv(os.path.join(run, 'px4.csv'))               # t, ts_us, x (N), y (E), z (D), vx, vy, vz
    wp = np.genfromtxt(wp_csv, delimiter=',', names=True)
    W = np.column_stack([wp['x'], wp['y']]); z_ref = -float(wp['z'][0])
    t_sent, t_done = path_times(run)
    res = {'run': os.path.basename(run.rstrip('/')), 'completed': t_done is not None}
    t0 = t_sent if t_sent else px['t'][0]
    t1 = t_done if t_done else px['t'][-1]
    m = (px['t'] >= t0) & (px['t'] <= t1)
    P = np.column_stack([px['x'][m], px['y'][m]])
    if len(P) < 10:
        res['note'] = 'no path phase'
        return res
    dists = []
    for i in range(len(W) - 1):
        d, _ = seg_dist(P, W[i], W[i + 1])
        dists.append(d)
    D = np.min(np.vstack(dists), axis=0)
    leg = np.argmin(np.vstack(dists), axis=0)
    res.update(path_time_s=float(t1 - t0), xte_mean=float(D.mean()), xte_p95=float(np.percentile(D, 95)), xte_max=float(D.max()))
    zerr = np.abs(px['z'][m] - z_ref)
    res.update(alt_err_mean=float(zerr.mean()), alt_err_max=float(zerr.max()))
    v = np.hypot(px['vx'][m], px['vy'][m])
    wind = np.array([math.cos(math.radians(wind_dir_deg)), math.sin(math.radians(wind_dir_deg))])   # (N, E)
    kinds, xte_kinds = {}, {}
    core_all = np.zeros(len(P), dtype=bool)
    for i in range(len(W) - 1):
        u = (W[i + 1] - W[i]) / max(np.linalg.norm(W[i + 1] - W[i]), 1e-9)
        c = float(u @ wind)
        kind = 'tail' if c > 0.7 else ('head' if c < -0.7 else 'cross')
        sel = leg == i
        # steady part of the leg: away from both ends by 20 % of its length
        _, tt = seg_dist(P[sel], W[i], W[i + 1]) if sel.any() else (None, np.zeros(0))
        core = (tt > 0.2) & (tt < 0.8)
        if core.any():
            kinds.setdefault(kind, []).append(float(v[sel][core].mean()))
            xte_kinds.setdefault(kind, []).append(D[sel][core])
            core_all[np.flatnonzero(sel)[core]] = True
    for k in ('head', 'tail', 'cross'):
        if k in kinds:
            res[f'v_{k}'] = float(np.mean(kinds[k]))
            res[f'xte_core_{k}'] = float(np.concatenate(xte_kinds[k]).mean())
    # cross-track error on the straight parts only (the corner overshoot - xte_max - is mostly turn
    # radius at speed, not wind)
    if core_all.any():
        res.update(xte_core_mean=float(D[core_all].mean()), xte_core_max=float(D[core_all].max()))
    off = ulog_offset(run, px)
    if off is not None:
        res.update(ulog_stats(run, t0, t1, off))
    if speed:
        res['speed_cmd'] = speed
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('runs', nargs='+')
    ap.add_argument('--wp', default=os.path.expanduser('~/Documents/A4VAI-SITL/ROS2/test_runs/wp_wind_square.csv'))
    ap.add_argument('--wind-dir-deg', type=float, default=0.0)
    ap.add_argument('--speed', type=float, default=None)
    a = ap.parse_args()
    for r in a.runs:
        res = analyze(r, a.wp, a.wind_dir_deg, a.speed)
        print(' '.join(f'{k}={v:.2f}' if isinstance(v, float) else f'{k}={v}' for k, v in res.items()))


if __name__ == '__main__':
    main()
