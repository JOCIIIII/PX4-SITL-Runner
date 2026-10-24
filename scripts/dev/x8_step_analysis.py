#!/usr/bin/env python3
"""Responses of a scripts/ros2/apps/x8_step_test.sh run, in the terms of the CA training note
(dynamics.md 4.1): dead time, 63 % / 90 % times, overshoot, peak tilt and acceleration,
altitude steps, altitude excursion in the +-40 deg zigzag, low-speed tracking, and the
training-equivalent tracker tau (t63 - 0.17 s - dead time).

Responses are the EKF state as the ROS side received it (px4.csv: what the policy sees), on
the ROS clock of the test node's segment log; tilt and Gazebo's ground truth (a check) come
from the PX4 log, mapped onto that clock.

usage: x8_step_analysis.py RUN_DIR [--plot OUT.png]
       RUN_DIR holds x8_step_test.csv and px4.csv (fly_test.sh) and px4_sitl.ulg (the run's PX4 log)
"""
import argparse
import json
import os

import numpy as np
from pyulog import ULog


def load_csv(path):
    return np.genfromtxt(path, delimiter=',', names=True, dtype=None, encoding=None)


def ulog_to_ros(u, px):
    """Map PX4 log time [us] -> ROS time [s] of the run.

    Gazebo's real-time factor wanders (0.97-1.03 here), so the clocks drift apart by up to
    ~0.2 s within a run. px4.csv samples are found in the log by value; the offset is the
    smallest receive delay within +-1 s (receive latency only adds)."""
    lp = [x for x in u.data_list if x.name == 'vehicle_local_position'][0].data
    key = {}
    for i in range(len(lp['timestamp'])):
        k = tuple(round(float(lp[c][i]), 3) for c in ('x', 'y', 'z', 'vx', 'vy'))
        key.setdefault(k, []).append(lp['timestamp'][i])
    pu, pt = [], []
    for i in range(len(px)):
        k = tuple(round(float(px[c][i]), 3) for c in ('x', 'y', 'z', 'vx', 'vy'))
        if len(key.get(k, ())) == 1:
            pu.append(key[k][0] * 1e-6); pt.append(px['t'][i])
    pu, pt = np.array(pu), np.array(pt)
    o = np.argsort(pu); pu, pt = pu[o], pt[o]
    off = pt - pu
    lo = np.searchsorted(pu, pu - 1.0); hi = np.searchsorted(pu, pu + 1.0)
    off_min = np.array([off[a:b].min() for a, b in zip(lo, hi)])
    return lambda ts_us: ts_us * 1e-6 + np.interp(ts_us * 1e-6, pu, off_min)


def crossing(t, r, level):
    above = np.nonzero(r >= level)[0]
    if not len(above):
        return float('nan')
    i = above[0]
    if i == 0:
        return float(t[0])
    return float(t[i - 1] + (level - r[i - 1]) / (r[i] - r[i - 1]) * (t[i] - t[i - 1]))


def step(t, y, ts, te, y1):
    """Step from the pre-step mean to y1 at ts, within [ts, te]."""
    pre = (t >= ts - 0.5) & (t < ts)
    y0, sd = y[pre].mean(), y[pre].std()
    m = (t >= ts) & (t <= te)
    tt, r = t[m] - ts, (y[m] - y0) / (y1 - y0)
    band = max(3 * sd / abs(y1 - y0), 0.03)
    return dict(dead=crossing(tt, r, band), t63=crossing(tt, r, 0.63), t90=crossing(tt, r, 0.9),
                over=max(0.0, r.max() - 1.0) * abs(y1 - y0), y0=y0)


# training tracker + controller (dynamics.md 2.3/2.4): command delay d, 1st-order tau, velocity
# loop kv 6 (P), attitude loop 600/40 (24.5 rad/s, damping 0.82) between commanded and actual
# horizontal acceleration; vertical: vz_ref = clip(kz (z_wp - z), +-vz_max) through the same tau
KV, W_ATT, Z_ATT, DT = 6.0, 24.5, 0.82, 0.01


def train_speed(n, d, tau):
    """unit-step speed responses of the training chain, vectorised over arrays d, tau"""
    shape = np.broadcast(d, tau).shape
    vr, v, a, ad = (np.zeros(shape) for _ in range(4))
    out = np.empty((n,) + shape)
    for k in range(n):
        c = (k * DT >= d).astype(float)
        vr = vr + DT * (c - vr) / tau
        ad = ad + DT * (W_ATT ** 2 * (KV * (vr - v) - a) - 2 * Z_ATT * W_ATT * ad)
        a = a + DT * ad
        v = v + DT * a
        out[k] = v
    return out


def fit_speed(t, y, ts, te, y0, y1, horizon=3.0):
    tt = np.arange(0.0, min(horizon, te - ts), DT)
    r = (np.interp(ts + tt, t, y) - y0) / (y1 - y0)
    D, T = np.meshgrid(np.arange(0.0, 0.31, 0.01), np.arange(0.05, 1.51, 0.01), indexing='ij')
    e = np.sqrt(np.mean((train_speed(len(tt), D, T) - r[:, None, None]) ** 2, axis=0))
    i = np.unravel_index(np.argmin(e), e.shape)
    return float(D[i]), float(T[i]), float(e[i])


def train_alt(n, dz, d, tau, kz, vz_max):
    shape = np.broadcast(kz, vz_max).shape
    z, vz, vr = (np.zeros(shape) for _ in range(3))
    nd = int(round(d / DT))
    hist = [np.zeros(shape)] * (nd + 1)
    out = np.empty((n,) + shape)
    for k in range(n):
        hist.append(np.clip(kz * (dz - z), -vz_max, vz_max))
        vr = vr + DT * (hist[-1 - nd] - vr) / tau
        vz = vz + DT * KV * (vr - vz)
        z = z + DT * vz
        out[k] = z
    return out


def fit_alt(t, z, vz, ts, te, z1, d, tau):
    tt = np.arange(0.0, te - ts, DT)
    z0 = float(np.interp(ts, t, z))
    zm = np.interp(ts + tt, t, z) - z0
    K, V = np.meshgrid(np.arange(0.1, 3.01, 0.05), np.arange(0.1, 2.01, 0.05), indexing='ij')
    e = np.sqrt(np.mean((train_alt(len(tt), z1 - z0, d, tau, K, V) - zm[:, None, None]) ** 2, axis=0)) / abs(z1 - z0)
    i = np.unravel_index(np.argmin(e), e.shape)
    return float(K[i]), float(V[i]), float(e[i])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run_dir')
    ap.add_argument('--plot')
    a = ap.parse_args()
    d = a.run_dir
    st = load_csv(os.path.join(d, 'x8_step_test.csv'))
    px = load_csv(os.path.join(d, 'px4.csv'))
    u = ULog(os.path.join(d, 'px4_sitl.ulg'), ['vehicle_local_position', 'vehicle_local_position_groundtruth',
                                               'vehicle_attitude'])
    g = {x.name: x.data for x in u.data_list if x.multi_id == 0}
    to_ros = ulog_to_ros(u, px)
    seg_start = {}
    for i in range(len(st)):
        seg_start.setdefault(st['seg'][i], float(st['t'][i]))
    names = list(seg_start)
    seg_end = {n: (seg_start[names[k + 1]] if k + 1 < len(names) else float(st['t'][-1])) for k, n in enumerate(names)}
    t_beg, t_end = seg_start[names[0]], seg_end[names[-1]]

    def states(t, lp):
        return dict(t=t, E=lp['y'], N=lp['x'], U=-lp['z'], vE=lp['vy'], vN=lp['vx'], vU=-lp['vz'])
    # EKF state as the ROS side received it (no clock mapping); truth from the log, mapped
    ekf = states(px['t'], px)
    gt = states(to_ros(g['vehicle_local_position_groundtruth']['timestamp']), g['vehicle_local_position_groundtruth'])
    att = g['vehicle_attitude']
    t_att = to_ros(att['timestamp'])
    tilt = np.degrees(np.arccos(np.clip(1 - 2 * (att['q[1]'] ** 2 + att['q[2]'] ** 2), -1, 1)))
    for s in (ekf, gt):
        s['vh'] = np.hypot(s['vE'], s['vN'])
    # horizontal acceleration: the EKF's own estimate (PX4 clock - receive-time derivatives are jittery)
    lp = g['vehicle_local_position']
    acc = dict(t=to_ros(lp['timestamp']), ah=np.hypot(lp['ax'], lp['ay']))

    def peak(y, t, t0, t1):
        m = (t >= t0) & (t <= t1)
        return float(np.abs(y[m]).max())

    def mean(s, key, t0, t1):
        m = (s['t'] >= t0) & (s['t'] <= t1)
        return float(s[key][m].mean())

    print(f'test window {t_end - t_beg:.1f} s; alt offset EKF - truth during the test: '
          f'{mean(ekf, "U", t_beg, t_end) - mean(gt, "U", t_beg, t_end):+.2f} m')

    def seg_cmd(name, col):
        return float(st[col][st['seg'] == name][0])

    def along(s, h_deg):
        h = np.radians(h_deg)
        return s['vE'] * np.cos(h) + s['vN'] * np.sin(h)

    summary = dict(speed=[], altitude=[])
    print(f'{"step":24s} {"dead":>5s} {"t63":>5s} {"t90":>5s} {"over":>5s} {"tilt":>5s} {"acc":>5s} {"end":>5s} '
          f'{"tau_eq":>6s} | {"fit d":>5s} {"tau":>5s} {"rms":>5s} | truth {"t63":>5s} {"t90":>5s}')
    for name, lbl in (('v0to3', 'speed 0 -> 3 m/s'), ('v3to1', 'speed 3 -> 1 m/s'), ('v1to0', 'speed 1 -> 0 m/s'),
                      ('v0to3b', 'speed 0 -> 3 m/s (2)'), ('turn90', 'turn 90 deg at 3 m/s'), ('stop', 'speed 3 -> 0 m/s'),
                      ('v0to5', 'speed 0 -> 5 m/s'), ('rev180', 'reverse at 5 m/s'), ('stop5', 'speed 5 -> 0 m/s')):
        if name not in seg_start:
            continue
        ts, te = seg_start[name], seg_end[name]
        h, y1 = seg_cmd(name, 'heading_cmd'), seg_cmd(name, 'speed_cmd')
        m = step(ekf['t'], along(ekf, h), ts, te, y1)
        mg = step(gt['t'], along(gt, h), ts, te, y1)
        fd, ftau, frms = fit_speed(ekf['t'], along(ekf, h), ts, te, m['y0'], y1)
        end = mean(ekf, 'vh', te - 1.0, te)
        tau = m['t63'] - 0.17 - m['dead']
        row = dict(step=lbl, dead=m['dead'], t63=m['t63'], t90=m['t90'], over=m['over'], tilt=peak(tilt, t_att, ts, te),
                   acc=peak(acc['ah'], acc['t'], ts, te), end=end, tau_eq=tau, fit_delay=fd, fit_tau=ftau, fit_rms=frms,
                   truth_t63=mg['t63'], truth_t90=mg['t90'])
        summary['speed'].append(row)
        print(f'{lbl:24s} {m["dead"]:5.2f} {m["t63"]:5.2f} {m["t90"]:5.2f} {m["over"]:5.2f} {row["tilt"]:5.1f} '
              f'{row["acc"]:5.2f} {end:5.2f} {tau:6.2f} | {fd:5.2f} {ftau:5.2f} {frms:5.3f} |       {mg["t63"]:5.2f} {mg["t90"]:5.2f}')
    print('  dead = first departure from the pre-step band (3 sd, >= 3 %); over in m/s; end = speed over the last 1 s;'
          ' tau_eq = t63 - 0.17 - dead;\n  fit = delay d and tau of the training tracker (1st order, kv 6 velocity loop,'
          ' 24.5 rad/s attitude loop) fitted to the response, rms of the normalised fit error')
    taus = [r['fit_tau'] for r in summary['speed']]; ds = [r['fit_delay'] for r in summary['speed']]
    tau_m, d_m = float(np.median(taus)), float(np.median(ds))
    print(f'  median fit: delay {d_m:.2f} s, tau {tau_m:.2f} s (range {min(taus):.2f}-{max(taus):.2f})')
    print(f'{"altitude step":24s} {"dead":>5s} {"t63":>5s} {"t90":>5s} {"over":>5s} {"vU pk":>5s} {"end":>6s} | '
          f'{"fit kz":>6s} {"vzmax":>5s} {"rms":>5s} | truth {"t63":>5s} {"t90":>5s}')
    for name, lbl in (('alt_up', 'altitude +1 m, hover'), ('alt_down', 'altitude -1 m, hover'),
                      ('alt_up_mv', 'altitude +1 m, 3 m/s'), ('alt_down_mv', 'altitude -1 m, 3 m/s')):
        if name not in seg_start:
            continue
        ts, te = seg_start[name], seg_end[name]
        y0 = mean(ekf, 'U', ts - 0.5, ts)
        y1 = seg_cmd(name, 'alt_cmd')
        m = step(ekf['t'], ekf['U'], ts, te, y1)
        mg = step(gt['t'], gt['U'], ts, te, mean(gt, 'U', ts - 0.5, ts) + (y1 - y0))
        kz, vzm, frms = fit_alt(ekf['t'], ekf['U'], ekf['vU'], ts, te, y1, d_m, tau_m)
        end = mean(ekf, 'U', te - 1.0, te) - y1
        row = dict(step=lbl, size=y1 - y0, dead=m['dead'], t63=m['t63'], t90=m['t90'], over=m['over'],
                   vu_peak=peak(ekf['vU'], ekf['t'], ts, te), end=end, fit_kz=kz, fit_vz_max=vzm, fit_rms=frms,
                   truth_t63=mg['t63'], truth_t90=mg['t90'])
        summary['altitude'].append(row)
        print(f'{lbl:24s} {m["dead"]:5.2f} {m["t63"]:5.2f} {m["t90"]:5.2f} {m["over"]:5.2f} {row["vu_peak"]:5.2f} {end:+6.2f} | '
              f'{kz:6.2f} {vzm:5.2f} {frms:5.3f} |       {mg["t63"]:5.2f} {mg["t90"]:5.2f}   (step {y1 - y0:+.2f} m)')
    print(f'  fit = ctrl_kz and ctrl_vz_max of the training tracker with the median delay/tau above; over in m;'
          ' end = altitude error over the last 1 s')
    ts, te = seg_start['zigzag'], seg_end['zigzag']
    alt_cmd = seg_cmd('zigzag', 'alt_cmd')
    m = (ekf['t'] >= ts + 1.0) & (ekf['t'] <= te)
    hv = np.degrees(np.arctan2(ekf['vN'][m], ekf['vE'][m]))
    summary['zigzag'] = dict(alt_err_max=float(np.abs(ekf['U'][m] - alt_cmd).max()), alt_sd=float(np.std(ekf['U'][m])),
                             speed=float(ekf['vh'][m].mean()), course_swing=float(np.percentile(np.abs(hv), 95)),
                             tilt=peak(tilt, t_att, ts, te))
    z = summary['zigzag']
    print(f'zigzag +-40 deg every 0.5 s at 3 m/s: altitude error {z["alt_err_max"]:.2f} m max, {z["alt_sd"]:.2f} m sd; '
          f'speed {z["speed"]:.2f} m/s; course swing +-{z["course_swing"]:.0f} deg; peak tilt {z["tilt"]:.1f} deg')
    summary['hold'] = {}
    for name, v in (('v3to1', 1.0), ('v1to0', 0.0), ('stop', 0.0), ('stop5', 0.0)):
        if name in seg_start:
            summary['hold'][name] = mean(ekf, 'vh', seg_end[name] - 1.5, seg_end[name])
            print(f'hold {v:g} m/s ({name}): last 1.5 s speed {summary["hold"][name]:.2f} m/s')
    m = (ekf['t'] >= t_beg) & (ekf['t'] <= t_end)
    summary['overall'] = dict(speed=float(ekf['vh'][m].max()), climb=float(ekf['vU'][m].max()),
                              descent=float(-ekf['vU'][m].min()), tilt=peak(tilt, t_att, t_beg, t_end),
                              acc=peak(acc['ah'], acc['t'], t_beg, t_end), fit_delay=d_m, fit_tau=tau_m)
    o = summary['overall']
    print(f'in the test: max speed {o["speed"]:.2f} m/s, climb {o["climb"]:.2f} / descent {o["descent"]:.2f} m/s, '
          f'tilt {o["tilt"]:.1f} deg, horizontal accel {o["acc"]:.2f} m/s^2')
    with open(os.path.join(d, 'x8_step_summary.json'), 'w') as f:
        json.dump(summary, f, indent=1)
    if a.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(4, 1, figsize=(13, 11), sharex=True)
        tc = st['t'] - t_beg
        dU = mean(gt, 'U', t_beg, t_beg + 3) - mean(ekf, 'U', t_beg, t_beg + 3)
        for s, lab, ls, off in ((ekf, 'EKF', '-', 0.0), (gt, 'truth', ':', dU)):
            mm = (s['t'] >= t_beg - 3) & (s['t'] <= t_end + 2)
            ax[0].plot(s['t'][mm] - t_beg, s['vh'][mm], ls, label=f'|v| {lab}')
            ax[2].plot(s['t'][mm] - t_beg, s['U'][mm] - off, ls, label=f'alt {lab}' + (f' - {dU:.2f}' if off else ''))
        ax[0].plot(tc, st['speed_cmd'], 'k--', label='speed cmd'); ax[0].set_ylabel('m/s')
        hv = np.degrees(np.arctan2(ekf['vN'], ekf['vE'])); hv[ekf['vh'] < 0.5] = np.nan
        ax[1].plot(ekf['t'] - t_beg, hv, '.', ms=2, label='course'); ax[1].plot(tc, st['heading_cmd'], 'k--', label='heading cmd')
        ax[1].set_ylabel('deg')
        ax[2].plot(tc, st['alt_cmd'], 'k--', label='alt cmd'); ax[2].set_ylabel('m')
        ma = (t_att >= t_beg - 3) & (t_att <= t_end + 2)
        ax[3].plot(t_att[ma] - t_beg, tilt[ma], label='tilt'); ax[3].set_ylabel('deg')
        for x in ax:
            for n in names:
                x.axvline(seg_start[n] - t_beg, color='gray', lw=0.5)
            x.legend(loc='upper right', fontsize=8); x.grid(alpha=0.3)
        ax[3].set_xlim(-3, t_end - t_beg + 2); ax[3].set_xlabel('s')
        plt.tight_layout(); plt.savefig(a.plot, dpi=90)


if __name__ == '__main__':
    main()
