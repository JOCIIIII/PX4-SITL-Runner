#!/usr/bin/env python3
"""How much a SITL flight shakes: body-rate jitter, attitude-setpoint jumps, yaw wobble,
vertical bobbing, and (for noclimb_ca runs) how much the policy's commands jump per step.

Computed over the cruise (horizontal speed above 1 m/s). 'hf' = above ~2 Hz (signal minus its
0.5 s moving average): the wobble a viewer sees, as opposed to the manoeuvres themselves.

usage: shake_metrics.py RUN_DIR [RUN_DIR ...]   (px4_sitl.ulg, optional noclimb_ca.csv)
       shake_metrics.py --ulog LOG.ulg [--policy-csv noclimb_ca.csv]
"""
import argparse
import os

import numpy as np
from pyulog import ULog


def topic(u, name):
    for d in u.data_list:
        if d.name == name and d.multi_id == 0:
            return d.data
    return None


def euler(q0, q1, q2, q3):
    roll = np.arctan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2))
    pitch = np.arcsin(np.clip(2 * (q0 * q2 - q3 * q1), -1, 1))
    yaw = np.arctan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3))
    return roll, pitch, yaw


def hp(x, fs, win_s=0.5):
    n = max(int(win_s * fs), 3)
    return x - np.convolve(x, np.ones(n) / n, mode='same')


def resample(t, x, tg):
    return np.interp(tg, t, x)


def metrics(ulog_path, policy_csv=None):
    u = ULog(ulog_path, ['vehicle_angular_velocity', 'vehicle_attitude_setpoint', 'vehicle_attitude',
                         'vehicle_local_position', 'vehicle_thrust_setpoint'])
    av, asp, att = topic(u, 'vehicle_angular_velocity'), topic(u, 'vehicle_attitude_setpoint'), topic(u, 'vehicle_attitude')
    lp, th = topic(u, 'vehicle_local_position'), topic(u, 'vehicle_thrust_setpoint')
    T = lambda d: d['timestamp'] * 1e-6
    tl = T(lp)
    vh = np.hypot(lp['vx'], lp['vy'])
    cruise = vh > 1.0
    if cruise.sum() < 100:
        return None
    t0, t1 = tl[cruise][0], tl[cruise][-1]
    fs = 100.0
    tg = np.arange(t0, t1, 1 / fs)
    m = np.interp(tg, tl, cruise.astype(float)) > 0.5
    out = {'cruise_s': float(m.sum() / fs)}
    w = np.stack([resample(T(av), av[f'xyz[{i}]'], tg) for i in range(3)], 1)
    for i, ax in enumerate(('roll', 'pitch', 'yaw')):
        out[f'{ax}_rate_hf'] = float(np.degrees(np.std(hp(w[:, i], fs)[m])))
        out[f'{ax}_rate_rms'] = float(np.degrees(np.sqrt(np.mean(w[m, i] ** 2))))
    r, p, y = euler(*[asp[f'q_d[{i}]'] for i in range(4)])
    ts = T(asp)
    for name, x in (('roll_sp', r), ('pitch_sp', p)):
        xg = resample(ts, x, tg)
        out[f'{name}_hf'] = float(np.degrees(np.std(hp(xg, fs)[m])))
        out[f'{name}_rate'] = float(np.degrees(np.sqrt(np.mean(np.gradient(xg, 1 / fs)[m] ** 2))))
    yg = np.unwrap(resample(ts, np.unwrap(y), tg))
    out['yaw_sp_rate'] = float(np.degrees(np.sqrt(np.mean(np.gradient(yg, 1 / fs)[m] ** 2))))
    out['yaw_sp_hf'] = float(np.degrees(np.std(hp(yg, fs)[m])))
    ra, pa, ya = euler(*[att[f'q[{i}]'] for i in range(4)])
    tilt = np.degrees(np.arccos(np.clip(np.cos(ra) * np.cos(pa), -1, 1)))
    out['tilt_hf'] = float(np.std(hp(resample(T(att), tilt, tg), fs)[m]))
    tz = resample(T(th), -th['xyz[2]'], tg)
    out['thrust_hf'] = float(np.std(hp(tz, fs)[m]))
    vz = resample(tl, lp['vz'], tg)
    out['vz_std'] = float(np.std(vz[m]))
    out['vz_hf'] = float(np.std(hp(vz, fs)[m]))
    # roll/pitch rate sign reversals per second in the 1-8 Hz band (wobble count)
    band = hp(w[:, :2], fs) if False else np.stack([hp(w[:, i], fs) for i in range(2)], 1)
    rev = 0
    for i in range(2):
        s = np.sign(band[m, i] * (np.abs(band[m, i]) > np.radians(5)))
        s = s[s != 0]
        rev += int(np.sum(s[1:] != s[:-1]))
    out['wobble_reversals_per_s'] = float(rev / max(out['cruise_s'], 1e-3))
    # worst 1 s of roll+pitch rate jitter: an intermittent shake hides in the whole-flight std
    e = np.sqrt(band[:, 0] ** 2 + band[:, 1] ** 2) * m
    win = int(fs)
    rms1 = np.sqrt(np.convolve(e ** 2, np.ones(win) / win, mode='valid'))
    out['worst_1s_rate_hf'] = float(np.degrees(rms1.max()))
    out['worst_1s_at'] = float(tg[int(np.argmax(rms1)) + win // 2] - t0)
    out['speed_mean'] = float(np.mean(np.interp(tg, tl, vh)[m]))
    # nose vs direction of travel (above 2 m/s)
    course = np.arctan2(lp['vy'], lp['vx'])
    yaw_l = np.interp(tl, T(att), np.unwrap(ya))
    err = np.degrees(np.abs((course - yaw_l + np.pi) % (2 * np.pi) - np.pi))
    fast = vh > 2.0
    if fast.sum() > 10:
        out['yaw_course_err_med'] = float(np.median(err[fast]))
        out['yaw_course_err_p90'] = float(np.percentile(err[fast], 90))
    if policy_csv and os.path.exists(policy_csv):
        c = np.genfromtxt(policy_csv, delimiter=',', names=True, dtype=None, encoding=None)
        ca = c[c['mode'] == 'CA']
        if len(ca) > 5:
            a0, a2 = ca['a0'].astype(float), ca['a2'].astype(float)
            out['policy_da0_mean'] = float(np.mean(np.abs(np.diff(a0))))
            out['policy_da2_mean'] = float(np.mean(np.abs(np.diff(a2))))
            hd = np.degrees(np.unwrap(np.arctan2(ca['wp_y'].astype(float) - ca['y'].astype(float),
                                                 ca['wp_x'].astype(float) - ca['x'].astype(float))))
            out['policy_heading_step_deg'] = float(np.mean(np.abs(np.diff(hd))))
            out['policy_speed_step'] = float(np.mean(np.abs(np.diff(ca['speed_cmd'].astype(float)))))
    return out


KEYS = [('speed_mean', 'm/s'), ('roll_rate_hf', 'deg/s'), ('pitch_rate_hf', 'deg/s'), ('yaw_rate_hf', 'deg/s'),
        ('yaw_rate_rms', 'deg/s'), ('roll_sp_hf', 'deg'), ('pitch_sp_hf', 'deg'), ('roll_sp_rate', 'deg/s'),
        ('pitch_sp_rate', 'deg/s'), ('yaw_sp_rate', 'deg/s'), ('tilt_hf', 'deg'), ('thrust_hf', '-'),
        ('vz_std', 'm/s'), ('vz_hf', 'm/s'), ('wobble_reversals_per_s', '1/s'),
        ('worst_1s_rate_hf', 'deg/s'), ('worst_1s_at', 's'),
        ('yaw_course_err_med', 'deg'), ('yaw_course_err_p90', 'deg'),
        ('policy_heading_step_deg', 'deg'), ('policy_speed_step', 'm/s'), ('policy_da0_mean', '-')]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('runs', nargs='*')
    ap.add_argument('--ulog')
    ap.add_argument('--policy-csv')
    a = ap.parse_args()
    items = [(r.rstrip('/').split('/')[-1], os.path.join(r, 'px4_sitl.ulg'), os.path.join(r, 'noclimb_ca.csv')) for r in a.runs]
    if a.ulog:
        items.append((os.path.basename(a.ulog), a.ulog, a.policy_csv))
    res = [(n, metrics(f, c)) for n, f, c in items]
    print(f'{"metric":26s} {"unit":6s} ' + ' '.join(f'{n[:14]:>14s}' for n, _ in res))
    for k, unit in KEYS:
        vals = [r.get(k) if r else None for _, r in res]
        if all(v is None for v in vals):
            continue
        print(f'{k:26s} {unit:6s} ' + ' '.join(f'{v:14.2f}' if v is not None else f'{"-":>14s}' for v in vals))


if __name__ == '__main__':
    main()
