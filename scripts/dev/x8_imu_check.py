#!/usr/bin/env python3
"""Sensor-model check of a SITL flight: IMU specific force and EKF state against Gazebo truth.

A light IMU link or a mesh collision can make Gazebo's IMU read forces that are not there
(the base X8 read 1.1 m/s^2 forward at rest), and PX4's EKF then starts tilted and its
altitude runs off - a fault the flight itself does not show. Passing values are the iris's:
IMU error <= 0.1 m/s^2 per axis, EKF vertical-velocity error <= 0.05 m/s, altitude error
change after take-off <= 0.2 m.

usage: x8_imu_check.py PX4_ULOG [...]
"""
import sys

import numpy as np
from pyulog import ULog
from scipy.spatial.transform import Rotation


def check(path):
    u = ULog(path, ['sensor_accel', 'vehicle_attitude_groundtruth', 'vehicle_local_position_groundtruth',
                    'vehicle_land_detected', 'vehicle_attitude', 'vehicle_local_position'])
    g = {d.name: d.data for d in u.data_list if d.multi_id == 0}
    sa, ag, gt = g['sensor_accel'], g['vehicle_attitude_groundtruth'], g['vehicle_local_position_groundtruth']
    ts, tg, ta = sa['timestamp'] * 1e-6, gt['timestamp'] * 1e-6, ag['timestamp'] * 1e-6
    k = np.ones(5) / 5
    # specific force the IMU should read: truth acceleration (50 Hz velocity, differentiated) minus gravity, in body axes
    a_ned = np.stack([np.convolve(np.gradient(gt[c], tg), k, 'same') for c in ('vx', 'vy', 'vz')], 1)
    grid = np.arange(max(ts[0], tg[0], ta[0]) + 1, min(ts[-1], tg[-1], ta[-1]) - 1, 0.02)
    q = np.stack([ag['q[1]'], ag['q[2]'], ag['q[3]'], ag['q[0]']], 1)
    R = Rotation.from_quat(q[np.searchsorted(ta, grid).clip(0, len(ta) - 1)])
    f_true = R.inv().apply(np.stack([np.interp(grid, tg, a_ned[:, i]) for i in range(3)], 1) - [0, 0, 9.80665])
    f_imu = np.stack([np.convolve(np.interp(grid, ts, sa[c]), k, 'same') for c in 'xyz'], 1)
    ld = g['vehicle_land_detected']
    landed = np.interp(grid, ld['timestamp'] * 1e-6, ld['landed'].astype(float)) > 0.5
    print(f"{path}  (SYS_AUTOSTART {u.initial_parameters.get('SYS_AUTOSTART')})")
    ok = True
    for name, m in (('on ground', landed), ('in air', ~landed)):
        if m.sum() < 50:
            continue
        e = (f_imu - f_true)[m].mean(0)
        ok &= bool(np.all(np.abs(e) <= 0.1))
        print(f'  IMU - truth {name:9s} [{e[0]:+.3f} {e[1]:+.3f} {e[2]:+.3f}] m/s^2')
    lp = g['vehicle_local_position']
    t = lp['timestamp'] * 1e-6
    air = np.interp(t, grid, (~landed).astype(float)) > 0.5
    dz = lp['z'] - np.interp(t, tg, gt['z'])        # includes the frames' origin offset (CG height at rest)
    dvz = lp['vz'] - np.interp(t, tg, gt['vz'])
    va = g['vehicle_attitude']
    tilt_err = []
    for a_ in (va, ag):
        w, x, y, z = (a_[f'q[{i}]'] for i in range(4))
        tilt_err.append((np.degrees(np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))),
                         np.degrees(np.arcsin(np.clip(2 * (w * y - z * x), -1, 1)))))
    tv = va['timestamp'] * 1e-6
    gr = np.interp(tv, grid, landed.astype(float)) > 0.5
    if gr.sum() > 10:
        r_t = np.interp(tv, ta, tilt_err[1][0]); p_t = np.interp(tv, ta, tilt_err[1][1])
        print(f'  EKF attitude - truth on ground: roll {np.mean(tilt_err[0][0][gr] - r_t[gr]):+.2f}, '
              f'pitch {np.mean(tilt_err[0][1][gr] - p_t[gr]):+.2f} deg')
    if air.sum() > 50 and (~air).sum() > 50:
        drift = np.median(dz[air]) - np.median(dz[~air & (t < t[air][0])])
        vz = np.median(dvz[air])
        ok &= abs(drift) <= 0.2 and abs(vz) <= 0.05
        print(f'  EKF in air: altitude error change since the ground {drift:+.2f} m, vz - truth {vz:+.3f} m/s')
    print('  PASS' if ok else '  FAIL')
    return ok


if __name__ == '__main__':
    sys.exit(0 if all([check(p) for p in sys.argv[1:]]) else 1)
