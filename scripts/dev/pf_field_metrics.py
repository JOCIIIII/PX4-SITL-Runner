#!/usr/bin/env python3
"""How well a PathFollowing-only flight (scripts/dev/pf_field.sh -> onboard_real.sh mission) followed its path, from
the vehicle's own logs - no flight_recorder needed:
  flight_driver.csv   the driver's per-tick log (PX4 local position / velocity / attitude, phase); onboard_real.sh
                      stop copies it into the run folder (SITL: <workspace>/logs/flight_driver_<time>.csv, --driver)
  flight_driver.log   "armed at N .. E .." = the take-off point the mission's rows are relative to
  the waypoints       wp.csv in the run folder, else <workspace>/missions/NAME.csv (onboard_real.sh mission keeps
                      the mission there), or --wp
During FOLLOW: the distance to the path polyline split into the tracking error before the end (mean / p95 / max)
and the overshoot past the last waypoint (the switch to landing: ~1 / 1.6 / 1.9 m at 3 / 5 / 7 m/s in SITL - it
sets the plain maximum of lines, squares and spirals); height error, speed (mean, median over the middle 60 %, max),
tilt (p95, max, share of the time within 2 deg of --tilt-limit) and PX4 position resets / dead reckoning.

usage (on the Jetson, after onboard_real.sh stop NAME):
  python3 scripts/dev/pf_field_metrics.py ~/Documents/A4VAI-SITL/ROS2/test_runs/NAME [--wp CSV] [--driver CSV]
"""
import argparse
import csv
import glob
import math
import os
import re
import sys

import numpy as np


def seg_dist(P, a, b):
    d = b - a
    L2 = float(d @ d)
    t = np.clip(((P - a) @ d) / L2, 0.0, 1.0) if L2 > 0 else np.zeros(len(P))
    return np.linalg.norm(P - (a + t[:, None] * d), axis=1)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('run_dir')
    ap.add_argument('--wp', default=None)
    ap.add_argument('--driver', default=None)
    ap.add_argument('--tilt-limit', type=float, default=30.0, help='the profile tilt limit [deg] (x8_5s: 30)')
    a = ap.parse_args()
    run = os.path.abspath(os.path.expanduser(a.run_dir))
    name = os.path.basename(run.rstrip('/'))

    drv = a.driver
    if drv is None:
        cands = [os.path.join(run, 'flight_driver.csv')] + sorted(glob.glob(os.path.join(run, 'flight_driver_*.csv')))
        drv = next((c for c in cands if os.path.exists(c)), None)
    if drv is None:
        sys.exit(f'{name}: no flight_driver.csv (onboard_real.sh stop copies it; or --driver)')
    wp = a.wp
    if wp is None:
        for c in (os.path.join(run, 'wp.csv'), os.path.join(os.path.dirname(os.path.dirname(run)), 'missions', name + '.csv')):
            if os.path.exists(c):
                wp = c
                break
    if wp is None:
        sys.exit(f'{name}: no waypoints (wp.csv here or missions/{name}.csv; or --wp)')

    rows = list(csv.DictReader(open(drv)))
    if not rows:
        sys.exit(f'{name}: {drv} is empty')
    f = lambda r, k: float(r[k]) if r.get(k, '') not in ('', None) else float('nan')   # noqa: E731

    # the take-off point: the driver's own "armed at" line, else the first armed row
    home = None
    events = []
    log = os.path.join(run, 'flight_driver.log')
    if os.path.exists(log):
        for line in open(log, errors='ignore'):
            m = re.search(r'armed at N ([-+\d.]+) E ([-+\d.]+)', line)
            if m and home is None:
                home = (float(m.group(1)), float(m.group(2)))
            if re.search(r'ABORT|pilot|yield|FOLLOW -> |-> FOLLOW', line):
                events.append(re.sub(r'^.*\[flight_driver\]: ', '', line.strip())[:150])
    if home is None:
        armed = [r for r in rows if r.get('armed') in ('1', 'true', 'True')]
        if not armed:
            sys.exit(f'{name}: never armed')
        home = (f(armed[0], 'x'), f(armed[0], 'y'))

    W = np.genfromtxt(wp, delimiter=',', names=True, comments='#')
    W = np.atleast_1d(W)
    path = np.column_stack([W['x'] + home[0], W['y'] + home[1]])
    z_wp = float(np.mean(W['z']))
    if len(path) == 1:
        path = np.vstack([path, path])

    fol = [r for r in rows if r['phase'] == 'FOLLOW']
    if len(fol) < 10:
        sys.exit(f'{name}: no FOLLOW phase in {os.path.basename(drv)} ({len(fol)} rows); events: {events}')
    t = np.array([f(r, 't') for r in fol])
    P = np.array([[f(r, 'x'), f(r, 'y')] for r in fol])
    v = np.array([math.hypot(f(r, 'vx'), f(r, 'vy')) for r in fol])
    h = np.array([f(r, 'height') for r in fol])
    q = np.array([[f(r, 'q0'), f(r, 'q1'), f(r, 'q2'), f(r, 'q3')] for r in fol])
    tilt = np.degrees(np.arccos(np.clip(1.0 - 2.0 * (q[:, 1] ** 2 + q[:, 2] ** 2), -1.0, 1.0)))
    D = np.min(np.vstack([seg_dist(P, path[i], path[i + 1]) for i in range(len(path) - 1)]), axis=0)
    n = len(v)
    mid = v[int(0.2 * n):int(0.8 * n)] if n > 10 else v
    dur = t[-1] - t[0]
    plen = float(np.sum(np.hypot(*np.diff(path, axis=0).T)))

    print(f'{name}: {len(path)} waypoints, path {plen:.1f} m, take-off point N {home[0]:+.2f} E {home[1]:+.2f}; '
          f'FOLLOW {dur:.1f} s ({os.path.basename(drv)}, {os.path.basename(wp)})')
    for e in events:
        print(f'  {e}')
    # past the end: the samples after the last one still behind the last waypoint (along the last segment)
    u = path[-1] - path[-2]
    u = u / max(np.linalg.norm(u), 1e-9)
    s_end = (P - path[-1]) @ u
    behind = np.nonzero(s_end <= 0.0)[0]
    k = int(behind[-1]) + 1 if len(behind) else 0
    T = D[:k] if k > 0 else D
    over = float(max(0.0, s_end[k:].max())) if k < len(P) else 0.0
    print(f'  tracking error before the end: mean {T.mean():.2f} / p95 {np.percentile(T, 95):.2f} / max {T.max():.2f} m, '
          f'{100.0 * np.mean(T <= 1.0):.0f} % within 1 m; overshoot past the end {over:.2f} m')
    print(f'  (all of FOLLOW: max {D.max():.2f} m)')
    print(f'  height above the take-off point: target {z_wp:.1f} m, error mean {np.nanmean(np.abs(h - z_wp)):.2f} / '
          f'max {np.nanmax(np.abs(h - z_wp)):.2f} m')
    print(f'  speed: mean {v.mean():.2f}, middle 60 % median {np.median(mid):.2f}, max {v.max():.2f} m/s')
    near = 100.0 * np.mean(tilt >= a.tilt_limit - 2.0)
    print(f'  tilt: p95 {np.percentile(tilt, 95):.1f}, max {tilt.max():.1f} deg; {near:.0f} % of the time within 2 deg '
          f'of the {a.tilt_limit:g} deg limit')
    xr = {r.get('xy_resets') for r in fol}
    zr = {r.get('z_resets') for r in fol}
    dr = any(r.get('dead_reckoning') in ('1', 'true', 'True') for r in fol)
    if len(xr) > 1 or len(zr) > 1 or dr:
        print(f'  PX4 estimate: xy resets {sorted(xr)}, z resets {sorted(zr)}, dead reckoning {dr}')


if __name__ == '__main__':
    main()
