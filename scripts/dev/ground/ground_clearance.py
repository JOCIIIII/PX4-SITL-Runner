#!/usr/bin/env python3
"""Clearance of a planned-path run (ground_run.sh with a layout world) against the layout's obstacles, from the
simulator's true pose in PX4's log: the vehicle's footprint rectangle (perimeter sampled every 5 cm) at every
true pose of the run, signed distance to every box / cylinder (negative = overlap = contact).
  ground_clearance.py RUN_DIR LAYOUT ugv|usv [--json]
Footprints: UGV 0.50 x 0.45 m (r1_rover chassis + wheels), USV 4.9 x 2.5 m (boat hull, floats 2.06 m apart).
The truth local frame starts at the spawn point = the layout's START (ground_world.py)."""
import glob
import json
import math
import os
import sys

import numpy as np

FOOT = {'ugv': (0.50, 0.45), 'usv': (4.9, 2.5)}


def sd_box(px, py, o):
    c, s = math.cos(math.radians(o.get('yaw_deg', 0.0))), math.sin(math.radians(o.get('yaw_deg', 0.0)))
    dx, dy = px - o['pos'][0], py - o['pos'][1]
    lx, ly = np.abs(c * dx + s * dy) - o['size'][0] / 2.0, np.abs(-s * dx + c * dy) - o['size'][1] / 2.0
    out = np.hypot(np.maximum(lx, 0.0), np.maximum(ly, 0.0))
    return out + np.minimum(np.maximum(lx, ly), 0.0)


def sd_obstacle(px, py, o):
    if o['shape'] == 'cylinder':
        return np.hypot(px - o['pos'][0], py - o['pos'][1]) - o['size'][0] / 2.0
    return sd_box(px, py, o)


def main():
    run, layout, veh = sys.argv[1], sys.argv[2], sys.argv[3]
    lay = json.load(open(layout))
    from pyulog import ULog
    u = ULog(glob.glob(os.path.join(run, 'px4_rootfs', 'log', '*', '*.ulg'))[0])
    d = {x.name: x.data for x in u.data_list if x.multi_id == 0}
    cm = d['vehicle_control_mode']
    armed = cm['flag_armed'] > 0
    t_arm, t_dis = cm['timestamp'][np.argmax(armed)], cm['timestamp'][len(armed) - 1 - np.argmax(armed[::-1])]
    gt, ag = d['vehicle_local_position_groundtruth'], d['vehicle_attitude_groundtruth']
    m = (gt['timestamp'] >= t_arm) & (gt['timestamp'] <= t_dis)
    t = gt['timestamp'][m]
    sx, sy = lay['waypoints']['START']
    X, Y = sx + gt['x'][m], sy + gt['y'][m]
    w, x, y, z = (np.interp(t, ag['timestamp'], ag[f'q[{i}]']) for i in range(4))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    L, W = FOOT[veh]
    per = []
    for (ax, ay), (bx, by) in zip([(L / 2, W / 2), (L / 2, -W / 2), (-L / 2, -W / 2), (-L / 2, W / 2)],
                                  [(L / 2, -W / 2), (-L / 2, -W / 2), (-L / 2, W / 2), (L / 2, W / 2)]):
        n = max(int(math.hypot(bx - ax, by - ay) / 0.05), 1)
        per += [(ax + (bx - ax) * k / n, ay + (by - ay) * k / n) for k in range(n)]
    per = np.array(per)
    c, s = np.cos(yaw), np.sin(yaw)
    PX = X[:, None] + c[:, None] * per[None, :, 0] - s[:, None] * per[None, :, 1]
    PY = Y[:, None] + s[:, None] * per[None, :, 0] + c[:, None] * per[None, :, 1]
    best = (math.inf, None, None)
    per_obstacle = {}
    for o in lay['obstacles']:
        dist = sd_obstacle(PX, PY, o).min(axis=1)
        k = int(np.argmin(dist))
        per_obstacle[o['name']] = round(float(dist[k]), 3)
        if dist[k] < best[0]:
            best = (float(dist[k]), o['name'], (t[k] - t[0]) * 1e-6)
    gx, gy = lay['waypoints']['GOAL']
    res = {'run': os.path.basename(run.rstrip('/')), 'min_clearance_m': round(best[0], 3), 'nearest': best[1],
           'at_s': round(best[2], 1), 'contact': bool(best[0] < 0.0),
           'goal_dist_m': round(float(math.hypot(X[-1] - gx, Y[-1] - gy)), 2),
           'footprint_m': [L, W], 'nearest5': dict(sorted(per_obstacle.items(), key=lambda kv: kv[1])[:5])}
    if '--json' in sys.argv:
        print(json.dumps(res))
    else:
        print(' | '.join(f'{k} {v}' for k, v in res.items()))
    np.save(os.path.join(run, 'out', 'truth_xy_world.npy'), np.column_stack([X, Y, yaw]))


if __name__ == '__main__':
    main()
