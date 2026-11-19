#!/usr/bin/env python3
"""Export a window of a vehicle flight's attitude setpoints for scripts/ros2/tools/x8_replay.py.

  x8_replay_export.py LOG.ulg T0 T1 OUT.csv        T0, T1 = seconds from the start of the log
  x8_replay_export.py LOG.ulg                      list the busiest airborne windows and the flight's gains

Output columns: t (from 0), roll, pitch (window mean removed), yaw_rel (relative to the window start) [rad], 20 Hz -
the format of test_runs/x8_replay_src/rate_tune_test_25_80.csv.
"""
import sys

import numpy as np
from pyulog import ULog


def topic(u, name):
    for d in u.data_list:
        if d.name == name and d.multi_id == 0:
            return d.data
    return None


def main():
    u = ULog(sys.argv[1], ['vehicle_attitude_setpoint', 'vehicle_land_detected', 'vehicle_rates_setpoint'])
    t0log = u.start_timestamp
    sp = topic(u, 'vehicle_attitude_setpoint')
    t = (sp['timestamp'] - t0log) / 1e6
    if 'roll_body' in sp:
        roll, pitch, yaw = sp['roll_body'], sp['pitch_body'], sp['yaw_body']
    else:
        w, x, y, z = (sp[f'q_d[{i}]'] for i in range(4))
        roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
        yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    land = topic(u, 'vehicle_land_detected')
    tl = (land['timestamp'] - t0log) / 1e6
    air = np.interp(t, tl, (~land['landed'].astype(bool)).astype(float)) > 0.5
    if len(sys.argv) < 5:
        P = u.initial_parameters
        print('gains:', {k: round(float(P[k]), 4) for k in ('MC_ROLLRATE_P', 'MC_ROLLRATE_I', 'MC_ROLLRATE_D', 'MC_PITCHRATE_P',
              'MC_PITCHRATE_I', 'MC_PITCHRATE_D', 'MC_YAWRATE_P', 'MC_YAWRATE_I', 'MC_YAWRATE_D', 'MC_ROLL_P', 'MC_PITCH_P', 'MC_YAW_P')},
              'PWM', P.get('PWM_MAIN_MIN1'), P.get('PWM_MAIN_MAX1'))
        print(f'log {t[-1]:.0f} s, airborne {t[air][0]:.0f}-{t[air][-1]:.0f} s' if air.any() else 'never airborne')
        for a in np.arange(t[air][0] + 5, t[air][-1] - 60, 5.0):
            m = (t >= a) & (t <= a + 55)
            if air[m].all():
                print(f'  window {a:.0f}-{a + 55:.0f} s: roll sp std {np.degrees(np.std(roll[m])):.1f} deg, pitch {np.degrees(np.std(pitch[m])):.1f} deg, '
                      f'yaw range {np.degrees(np.ptp(np.unwrap(yaw[m]))):.0f} deg')
        return
    a, b, out = float(sys.argv[2]), float(sys.argv[3]), sys.argv[4]
    tg = np.arange(a, b, 0.05)
    r = np.interp(tg, t, roll); p = np.interp(tg, t, pitch); y = np.interp(tg, t, np.unwrap(yaw))
    r -= r.mean(); p -= p.mean(); y -= y[0]
    with open(out, 'w') as f:
        f.write('t,roll,pitch,yaw_rel\n')
        for i in range(len(tg)):
            f.write(f'{tg[i] - a:.4f},{r[i]:.4f},{p[i]:.4f},{y[i]:.4f}\n')
    print(f'{out}: {len(tg)} rows ({b - a:.0f} s), roll std {np.degrees(np.std(r)):.1f} deg, pitch std {np.degrees(np.std(p)):.1f} deg')


if __name__ == '__main__':
    main()
