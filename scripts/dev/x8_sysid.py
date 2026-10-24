#!/usr/bin/env python3
"""Multicopter (X8, X500: motor count from CA_ROTOR_COUNT) dynamics identification from a PX4 ulog - the same estimates for the real vehicle's
flight logs and for SITL logs, so the Gazebo model can be checked against the vehicle.

usage: x8_sysid.py LOG.ulg [LOG.ulg ...] [--axes rpy] [--no-rate]

per log:
  hover    motor command u (actuator_motors, 0..1) and PWM (actuator_outputs, real only),
           normalised thrust setpoint, hover-thrust estimate, battery voltage
  rate     rate plant, output-error fit: the PX4 rate controller (logged gains, gyro 40 Hz and
           D-term 30 Hz filters) closes the loop around  w' = K * lag(tau_m)(torque(t - d)),
           driven by the logged rate setpoints over the 12 busiest 8 s airborne windows;
           K [rad/s^2 per unit normalised torque] = physical torque per unit / inertia
  drag     horizontal specific force (body frame) per body velocity [1/s]
The real vehicle maps u to PWM 1100 + 800 u (2026-01-07) or 1200 + 700 u (2025-07-31): K
scales with that span (see scripts/px4/x8/README.md for the comparison).
"""
import argparse
import os

import numpy as np
from pyulog import ULog
from scipy.optimize import least_squares
from scipy.signal import butter

FS = 400.0


def topic(u, name, mid=0):
    for d in u.data_list:
        if d.name == name and d.multi_id == mid:
            return d.data
    return None


class Log:
    def __init__(self, path):
        self.u = u = ULog(path)
        self.name = os.path.basename(path)
        self.p = u.initial_parameters
        t0 = u.start_timestamp
        self.t = lambda d: (d['timestamp'] - t0) * 1e-6
        ld = topic(u, 'vehicle_land_detected')
        lt = self.t(ld)
        ok = (lt > -1) & (lt < 1e5)
        self.ld_t, self.landed = lt[ok], ld['landed'][ok].astype(float)
        self.sitl = 'sitl' in str(u.msg_info_dict.get('ver_hw', '')).lower() or \
            str(u.msg_info_dict.get('sys_name', '')) != 'PX4' or 'SITL' in str(u.msg_info_dict.get('ver_hw', ''))

    def airborne(self, tg, margin=3.0):
        air = ~(np.interp(tg, self.ld_t, self.landed) > 0.5)
        k = max(int(margin / np.median(np.diff(tg))), 1)
        return np.convolve(air.astype(float), np.ones(2 * k + 1) / (2 * k + 1), mode='same') > 0.999


def hover(L):
    u = L.u
    am = topic(u, 'actuator_motors'); t = L.t(am)
    nm = int(L.p.get('CA_ROTOR_COUNT', 8))
    um = np.stack([am[f'control[{i}]'] for i in range(nm)], 1)
    lp = topic(u, 'vehicle_local_position'); tl = L.t(lp)
    vz = np.interp(t, tl, lp['vz']); az = np.interp(t, tl, lp['az'])
    alt = -np.interp(t, tl, lp['z'])
    air = L.airborne(t) & (alt - np.percentile(alt, 2) > 1.5)
    hov = air & (np.abs(vz) < 0.3) & (np.abs(az) < 0.5)
    out = {'n': int(hov.sum())}
    if hov.sum() < 10:
        return out
    out['u'] = float(um[hov].mean())
    ts = topic(u, 'vehicle_thrust_setpoint')
    out['thrust_sp'] = float(np.mean(np.interp(t[hov], L.t(ts), -ts['xyz[2]'])))
    ao = topic(u, 'actuator_outputs')
    if ao is not None and not L.sitl:
        out['pwm'] = float(np.mean(np.stack([np.interp(t[hov], L.t(ao), ao[f'output[{i}]']) for i in range(nm)], 1)))
    hte = topic(u, 'hover_thrust_estimate')
    if hte is not None and len(hte['timestamp']) > 3:
        out['hte'] = float(np.median(hte['hover_thrust'][hte['valid'] > 0])) if (hte['valid'] > 0).any() else float('nan')
    return out


class LP2:
    def __init__(self, fc, fs):
        self.b, self.a = butter(2, fc / (fs / 2)); self.z = np.zeros(2)

    def __call__(self, x):
        y = self.b[0] * x + self.z[0]
        self.z[0] = self.b[1] * x - self.a[1] * y + self.z[1]
        self.z[1] = self.b[2] * x - self.a[2] * y
        return y


def sim_axis(rsp, gains, K, tm, dl, imax, fc_gyro, fc_d, w0):
    P, I, D = gains
    w, integ, m, wf_prev = w0, 0.0, 0.0, w0
    lp_w, lp_d = LP2(fc_gyro, FS), LP2(fc_d, FS)
    for _ in range(50):                      # settle the filters on the initial rate
        wf_prev = lp_w(w0)
    c = np.exp(-1 / (FS * tm))
    nd = max(int(round(dl * FS)), 0)
    buf = np.zeros(nd + 1)
    out = np.empty(len(rsp))
    for k in range(len(rsp)):
        wf = lp_w(w)
        acc = lp_d((wf - wf_prev) * FS); wf_prev = wf
        e = rsp[k] - wf
        tau = min(max(P * e + integ - D * acc, -1.0), 1.0)
        integ = min(max(integ + I * e / FS, -imax), imax)
        buf = np.roll(buf, 1); buf[0] = tau
        m = c * m + (1 - c) * buf[-1]
        w += K * m / FS
        out[k] = w
    return out


def rate_plant(L, ax, delays=(0.0, 0.01, 0.02, 0.03, 0.04, 0.05), window=None):
    u, p = L.u, L.p
    pre = ('MC_ROLLRATE_', 'MC_PITCHRATE_', 'MC_YAWRATE_')[ax]
    gains = (p[pre + 'P'] * p[pre + 'K'], p[pre + 'I'] * p[pre + 'K'], p[pre + 'D'] * p[pre + 'K'])
    imax = p.get(('MC_RR_INT_LIM', 'MC_PR_INT_LIM', 'MC_YR_INT_LIM')[ax], 0.3)
    rs = topic(u, 'vehicle_rates_setpoint'); trs = L.t(rs)
    av = topic(u, 'vehicle_angular_velocity'); tav = L.t(av)
    tg = np.arange(max(trs[0], tav[0]) + 1, min(trs[-1], tav[-1]) - 1, 1 / FS)
    rsp = np.interp(tg, trs, rs[('roll', 'pitch', 'yaw')[ax]])
    meas = np.interp(tg, tav, av[f'xyz[{ax}]'])
    air = L.airborne(tg)
    if window is not None:
        air &= (tg >= window[0]) & (tg <= window[1])
    L8 = int(8 * FS)
    cands = []
    i = 0
    while i + L8 < len(tg):
        if air[i:i + L8].all():
            cands.append((np.std(np.diff(rsp[i:i + L8])) + np.std(rsp[i:i + L8]), i))
        i += L8 // 2
    cands.sort(reverse=True)
    picks = sorted(s for _, s in cands[:12])
    if not picks:
        return None
    fc_g, fc_d = p.get('IMU_GYRO_CUTOFF', 40.0), p.get('IMU_DGYRO_CUTOFF', 30.0)
    y = np.concatenate([meas[s:s + L8][int(0.5 * FS)::4] for s in picks])
    best = None
    for dl in delays:
        def resid(x):
            return np.concatenate([(sim_axis(rsp[s:s + L8], gains, x[0], x[1], dl, imax, fc_g, fc_d, meas[s])
                                    - meas[s:s + L8])[int(0.5 * FS)::4] for s in picks])
        for K0 in (30.0, 120.0):
            r = least_squares(resid, [K0, 0.02], bounds=([1, 0.002], [800, 0.2]), x_scale=[50, 0.02])
            vaf = 1 - np.var(r.fun) / np.var(y)
            if best is None or vaf > best[3]:
                best = (r.x[0], r.x[1], dl, vaf)
    return best + (len(picks),)


def drag(L):
    u = L.u
    lp = topic(u, 'vehicle_local_position'); tl = L.t(lp)
    tg = np.arange(tl[0] + 2, tl[-1] - 2, 0.05)
    at = topic(u, 'vehicle_attitude'); q = np.stack([np.interp(tg, L.t(at), at[f'q[{i}]']) for i in range(4)], 1)
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    w, x, y, z = q.T
    R = np.stack([np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], 1),
                  np.stack([2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], 1),
                  np.stack([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], 1)], 1)
    v = np.stack([np.interp(tg, tl, lp[k]) for k in ('vx', 'vy', 'vz')], 1)
    vb = np.einsum('nji,nj->ni', R, v)
    sc = topic(u, 'sensor_combined'); ts = L.t(sc)
    acc = np.stack([np.interp(tg, ts, np.convolve(sc[f'accelerometer_m_s2[{i}]'], np.ones(20) / 20, mode='same'))
                    for i in range(2)], 1)
    alt = -np.interp(tg, tl, lp['z'])
    m = L.airborne(tg) & (alt - np.percentile(alt, 2) > 1.5)
    out = []
    for ax in (0, 1):
        big = m & (np.abs(vb[:, ax]) > 0.5)
        out.append(float(np.polyfit(vb[big, ax], acc[big, ax], 1)[0]) if big.sum() > 50 else float('nan'))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('logs', nargs='+')
    ap.add_argument('--axes', default='rpy')
    ap.add_argument('--no-rate', action='store_true')
    ap.add_argument('--window', type=float, nargs=2, metavar=('T0', 'T1'),
                    help='rate fit only inside this log-time window [s] (e.g. the replayed part)')
    a = ap.parse_args()
    for f in a.logs:
        L = Log(f)
        print(f'== {L.name}  ({"SITL" if L.sitl else "vehicle"}, THR_MDL_FAC {L.p.get("THR_MDL_FAC")}, '
              f'PWM_MAIN_MIN1 {L.p.get("PWM_MAIN_MIN1")})')
        h = hover(L)
        print('  hover: ' + '  '.join(f'{k} {v:.4g}' for k, v in h.items()))
        dx = drag(L)
        print(f'  drag: specific force / body velocity  x {dx[0]:+.3f}  y {dx[1]:+.3f} 1/s')
        if a.no_rate:
            continue
        for ax in ('rpy'.index(c) for c in a.axes):
            r = rate_plant(L, ax, window=a.window)
            if r is None:
                print(f'  rate {"RPY"[ax]}: no airborne windows'); continue
            K, tm, dl, vaf, n = r
            print(f'  rate {"RPY"[ax]}: K {K:6.1f} rad/s^2/unit  tau_m {tm * 1e3:5.1f} ms  delay {dl * 1e3:3.0f} ms  '
                  f'VAF {vaf:.3f}  ({n} windows)')


if __name__ == '__main__':
    main()
