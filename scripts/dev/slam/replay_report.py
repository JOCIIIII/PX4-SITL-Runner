#!/usr/bin/env python3
"""What the bridge did on a replay (scripts/dev/slam/replay_bridge.sh OUT_DIR) or a real run (~/slam_ws/runs/NAME):
holes in the bridge's /Odometry intake, its state changes (WAIT / REST / RUN, re-alignments = reset_counter), how
many samples it sent / held / coasted, and how far its output sat from PX4's own position.

    python3 scripts/dev/slam/replay_report.py DIR [--hole 1.0]
"""
import argparse
import csv
import io
import json
import os
import re

import numpy as np


def load_csv(path):
    raw = open(path, 'rb').read().replace(b'\0', b'').decode('utf-8', 'ignore')
    return [r for r in csv.DictReader(io.StringIO(raw)) if r.get('t_rx')]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('dir')
    ap.add_argument('--hole', type=float, default=1.0)
    a = ap.parse_args()
    rows = load_csv(os.path.join(a.dir, 'bridge.csv'))
    f = lambda k: np.array([float(r[k]) if r.get(k) not in (None, '', 'nan') else np.nan for r in rows])
    t, ts, sent = f('t_rx'), f('t_sample'), f('sent')
    coast = f('coast') if 'coast' in rows[0] else np.zeros(len(rows))
    x, y, z = f('x'), f('y'), f('z')
    px, py, pz, xyv = f('px4_x'), f('px4_y'), f('px4_z'), f('px4_xy_valid')
    dt = np.diff(t)
    holes = np.where(dt > a.hole)[0]
    print(f'{a.dir}: {len(rows)} rows over {t[-1]-t[0]:.0f} s; sent {int(np.nansum(sent))}, not sent {int(np.sum(sent == 0))}, '
          f'coasted {int(np.nansum(coast))}')
    print(f'  intake holes > {a.hole} s: {len(holes)}  ' + ' '.join(f'{t[i+1]-t[0]:.0f}s:{dt[i]:.2f}s' for i in holes[:12]))
    real = coast == 0
    d = np.hypot(x - px, y - py)
    ok = real & (xyv == 1) & np.isfinite(d)
    if ok.any():
        print(f'  |bridge - PX4| horizontal (PX4 valid): median {np.nanmedian(d[ok]):.2f} m, p95 {np.nanpercentile(d[ok], 95):.2f} m, '
              f'max {np.nanmax(d[ok]):.2f} m; vertical max {np.nanmax(np.abs(z - pz)[ok]):.2f} m')
    # state changes from status.log (ros2 topic echo) or bridge.log
    st = os.path.join(a.dir, 'status.log')
    if os.path.exists(st):
        states, resets, last, lastr = [], [], None, None
        for line in open(st, errors='ignore'):
            m = re.search(r"data: '(\{.*\})'", line)
            if not m:
                continue
            try:
                s = json.loads(m.group(1))
            except Exception:
                continue
            if s.get('state') != last:
                states.append(s.get('state')); last = s.get('state')
            if s.get('reset_counter') != lastr:
                resets.append(s.get('reset_counter')); lastr = s.get('reset_counter')
        print(f'  state sequence: {" > ".join(str(s) for s in states)}; reset_counter values: {resets}')
    bl = os.path.join(a.dir, 'bridge.log')
    if os.path.exists(bl):
        txt = open(bl, errors='ignore').read()
        for key in ('restarted / lost', 'jumped', 'aligned (', 'coasting', 'resumed', 'unhealthy', 'unusable'):
            n = txt.count(key)
            if n:
                print(f'  log "{key}": {n}')


if __name__ == '__main__':
    main()
