#!/usr/bin/env python3
"""Paper figures and table for the A4VAI X8 SITL model (scripts/px4/x8).

  fig_x8_thrust     thrust per rotor vs PWM: datasheet (at the fitted battery voltage), the SITL
                    motor map, the Custom-repo model, and the thrust slope measured in flight
  fig_x8_replay     same attitude commands on the vehicle (2025-07-31 rate-tuning flight) and in
                    SITL (x8_replay.sh): body rates and their setpoints
  fig_x8_ratestep   closed-loop rate-step responses of the identified rate plants (vehicle and
                    SITL), both under the vehicle's controller of that flight
  fig_x8_drag       body-frame horizontal specific force vs body velocity, vehicle and SITL
  fig_x8_speedstep  X8 + PathFollowing speed steps (x8_step_test.sh) with the CA-training
                    tracker model fitted to them
  tab_x8_model.tex  model parameters (new vs Custom repo) and validation numbers

usage: x8_paper_figs.py --vehicle-dir DIR --replay RUN_DIR --step RUN_DIR --out DIR
       DIR = flight-log/X8 (0107/*.ulg, 250731/*/*.ulg); RUN_DIRs from scripts/dev/fly_test.sh
"""
import argparse
import glob
import json
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from pyulog import ULog  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'px4', 'x8'))
import make_x8_model as M  # noqa: E402
import x8_step_analysis as SA  # noqa: E402
import x8_sysid as SY  # noqa: E402

G = 9.80665
C_VEH, C_SIM, C_OLD, C_REF = '#1f3b73', '#d1495b', '#8a8a8a', 'k'
plt.rcParams.update({
    'font.family': 'serif', 'font.serif': ['Times New Roman', 'Times', 'DejaVu Serif'],
    'mathtext.fontset': 'stix', 'font.size': 8, 'axes.labelsize': 8, 'legend.fontsize': 7,
    'xtick.labelsize': 7, 'ytick.labelsize': 7, 'lines.linewidth': 1.0, 'axes.linewidth': 0.6,
    'grid.linewidth': 0.4, 'grid.alpha': 0.4, 'savefig.dpi': 300, 'figure.dpi': 150,
    'legend.frameon': False, 'axes.grid': True})
COL1, COL2 = 3.5, 7.16                               # IEEE single / double column [in]


def save(fig, out, name):
    fig.tight_layout(pad=0.3)
    for ext in ('pdf', 'png'):
        fig.savefig(os.path.join(out, f'{name}.{ext}'))
    plt.close(fig)
    print('wrote', name)


def topic(u, name):
    return SY.topic(u, name)


# ---------------------------------------------------------------- thrust model
def fig_thrust(out, p):
    thr, Tg, rpm, _ = M.DATASHEET.T
    w = rpm * 2 * np.pi / 60
    pw = np.polyfit(thr, w, 2)
    vs = p['v_eff'] / M.DATASHEET_V
    esc_lo, esc_hi = M.ESC_RANGE_US
    lo, hi = M.PWM_CFG_US
    pwm = np.linspace(1100, 1900, 200)
    th = (pwm - esc_lo) / (esc_hi - esc_lo) * 100
    T_ds = p['kT'] * (np.polyval(pw, th) * vs) ** 2                  # datasheet curve at V_eff
    u = (pwm - lo) / (hi - lo)
    w_sitl = np.minimum((u + p['input_offset']) * p['input_scaling'], p['maxRotVelocity'])
    T_sitl = p['kT'] * w_sitl ** 2
    T_old = 1.5e-4 * np.minimum(1000 * u, 885) ** 2                  # Custom repo: omega = 1000 u, k 1.5e-4
    fig, ax = plt.subplots(figsize=(COL1, 2.4))
    ax.plot(pwm, T_old, color=C_OLD, ls='--', label='Custom-repo model')
    ax.plot(pwm, T_ds, color=C_REF, lw=0.8, label=f'datasheet at {p["v_eff"]:.1f} V')
    ax.plot(pwm, T_sitl, color=C_SIM, label='SITL motor map (this work)')
    th_pts = thr
    pwm_pts = esc_lo + th_pts / 100 * (esc_hi - esc_lo)
    ok = (pwm_pts >= 1100) & (pwm_pts <= 1900)
    ax.plot(pwm_pts[ok], (Tg[ok] * G * 1e-3) * vs ** 2, 'o', ms=3, mfc='none', color=C_REF)
    T_h = M.MASS_TOTAL * G / 8
    ax.axhline(T_h, color=C_VEH, lw=0.5, ls=':')
    ax.plot([M.HOVER_PWM_US], [T_h], 's', ms=4, color=C_VEH, label='hover, flight logs (1515 µs)')
    # measured thrust slope at hover: 0.036 (0.034-0.041) m/s^2 per us on the 10.4 kg vehicle
    for slope, ls in ((0.036, '-'), (0.034, ':'), (0.041, ':')):
        k = slope * M.MASS_TOTAL / 8
        x = np.array([M.HOVER_PWM_US - 90, M.HOVER_PWM_US + 90])
        ax.plot(x, T_h + k * (x - M.HOVER_PWM_US), color=C_VEH, ls=ls, lw=1.6 if ls == '-' else 0.6,
                label='thrust slope, flight logs' if ls == '-' else None)
    ax.set_xlim(1100, 1900); ax.set_ylim(0, 60)
    ax.set_xlabel('PWM [µs]'); ax.set_ylabel('thrust per rotor [N]')
    ax.legend(loc='upper left')
    save(fig, out, 'fig_x8_thrust')


# ---------------------------------------------------------------- replay
def euler(q):
    w, x, y, z = q
    return (np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
            np.arcsin(np.clip(2 * (w * y - z * x), -1, 1)))


def rates(L):
    av = topic(L.u, 'vehicle_angular_velocity'); rs = topic(L.u, 'vehicle_rates_setpoint')
    asp = topic(L.u, 'vehicle_attitude_setpoint')
    r_sp = euler([asp[f'q_d[{i}]'] for i in range(4)])
    return dict(t=L.t(av), w=np.stack([av[f'xyz[{i}]'] for i in range(3)], 1),
                ts=L.t(rs), wsp=np.stack([rs[k] for k in ('roll', 'pitch', 'yaw')], 1),
                ta=L.t(asp), roll_sp=r_sp[0], pitch_sp=r_sp[1])


def align(veh, sim, win, search=(-60.0, 120.0)):
    """SITL time of vehicle time 0: cross-correlate the roll/pitch attitude setpoints."""
    dt = 0.02
    tv = np.arange(win[0], win[1], dt)
    sv = np.concatenate([np.interp(tv, veh['ta'], veh['roll_sp']), np.interp(tv, veh['ta'], veh['pitch_sp'])])
    sv -= sv.mean()
    ts = np.arange(sim['ta'][0], sim['ta'][-1], dt)
    ssr, ssp = np.interp(ts, sim['ta'], sim['roll_sp']), np.interp(ts, sim['ta'], sim['pitch_sp'])
    best = (-np.inf, 0.0)
    for off in np.arange(sim['ta'][0] - win[0] + search[0] * 0, sim['ta'][-1] - win[1], dt):
        i0 = int(round((win[0] + off - ts[0]) / dt))
        if i0 < 0 or i0 + len(tv) > len(ts):
            continue
        s = np.concatenate([ssr[i0:i0 + len(tv)], ssp[i0:i0 + len(tv)]]); s = s - s.mean()
        c = float(np.dot(sv, s) / (np.linalg.norm(sv) * np.linalg.norm(s) + 1e-12))
        if c > best[0]:
            best = (c, off)
    return best[1], best[0]


def fig_replay(out, veh_log, sim_log, win, show):
    Lv, Ls = SY.Log(veh_log), SY.Log(sim_log)
    v, s = rates(Lv), rates(Ls)
    off, corr = align(v, s, win)
    print(f'replay alignment: SITL t = vehicle t + {off:.2f} s (setpoint correlation {corr:.3f})')
    fig, axs = plt.subplots(3, 1, figsize=(COL2, 4.2), sharex=True)
    stats = {}
    for ax, i, lab in zip(axs, range(3), ('roll', 'pitch', 'yaw')):
        mv = (v['t'] >= show[0]) & (v['t'] <= show[1])
        ms = (s['t'] - off >= show[0]) & (s['t'] - off <= show[1])
        ax.plot(v['ts'], np.degrees(v['wsp'][:, i]), color=C_VEH, lw=0.6, ls='--', alpha=0.7, label='vehicle setpoint')
        ax.plot(v['t'][mv], np.degrees(v['w'][mv, i]), color=C_VEH, label='vehicle')
        ax.plot(s['ts'] - off, np.degrees(s['wsp'][:, i]), color=C_SIM, lw=0.6, ls='--', alpha=0.7, label='SITL setpoint')
        ax.plot(s['t'][ms] - off, np.degrees(s['w'][ms, i]), color=C_SIM, label='SITL')
        ax.set_ylabel(f'{lab} rate [°/s]')
        ax.set_xlim(*show)
        # statistics over the whole replayed window
        wv = (v['t'] >= win[0]) & (v['t'] <= win[1]); ws = (s['t'] - off >= win[0]) & (s['t'] - off <= win[1])
        stats[lab] = dict(veh_sd=float(np.degrees(np.std(v['w'][wv, i]))), sim_sd=float(np.degrees(np.std(s['w'][ws, i]))))
    axs[0].legend(ncol=4, loc='upper right')
    axs[-1].set_xlabel('time in the vehicle log [s]')
    save(fig, out, 'fig_x8_replay')
    # torque setpoints and motor saturation over the window (yaw is where the stock plugin failed)
    for L, o, key in ((Lv, 0.0, 'veh'), (Ls, off, 'sim')):
        tq = topic(L.u, 'vehicle_torque_setpoint'); tt = L.t(tq) - o
        m = (tt >= win[0]) & (tt <= win[1])
        am = topic(L.u, 'actuator_motors'); ta = L.t(am) - o
        ma = (ta >= win[0]) & (ta <= win[1])
        um = np.stack([am[f'control[{j}]'] for j in range(8)], 1)[ma]
        for j, lab in enumerate(('roll', 'pitch', 'yaw')):
            stats[lab][f'{key}_tq_sd'] = float(np.std(tq[f'xyz[{j}]'][m]))
        stats[f'{key}_sat'] = float(np.mean((um > 0.99) | (um < 0.01)))
    stats['offset'] = off; stats['corr'] = corr
    return stats


# ---------------------------------------------------------------- rate steps
def step_resp(gains, K, tm, dl, imax, fc_g, fc_d, T=0.6):
    r = np.ones(int(T * SY.FS)); r[:int(0.05 * SY.FS)] = 0.0
    return np.arange(len(r)) / SY.FS - 0.05, SY.sim_axis(r, gains, K, tm, dl, imax, fc_g, fc_d, 0.0)


def rise_overshoot(t, y):
    m = t >= 0
    t, y = t[m], y[m]
    t10 = t[np.argmax(y >= 0.1)]; t90 = t[np.argmax(y >= 0.9)]
    return float(t90 - t10), float(max(0.0, y.max() - 1.0))


def fig_ratestep(out, veh_log, sim_log, win, off):
    Lv, Ls = SY.Log(veh_log), SY.Log(sim_log)
    p = Lv.p                                           # the vehicle's controller of that flight
    fig, axs = plt.subplots(1, 3, figsize=(COL2, 1.9), sharey=True)
    res = {}
    for ax_i, (ax, lab) in enumerate(zip(axs, ('roll', 'pitch', 'yaw'))):
        pre = ('MC_ROLLRATE_', 'MC_PITCHRATE_', 'MC_YAWRATE_')[ax_i]
        gains = (p[pre + 'P'] * p[pre + 'K'], p[pre + 'I'] * p[pre + 'K'], p[pre + 'D'] * p[pre + 'K'])
        imax = p.get(('MC_RR_INT_LIM', 'MC_PR_INT_LIM', 'MC_YR_INT_LIM')[ax_i], 0.3)
        fc_g, fc_d = p.get('IMU_GYRO_CUTOFF', 40.0), p.get('IMU_DGYRO_CUTOFF', 30.0)
        res[lab] = {}
        for L, w, key, col in ((Lv, win, 'veh', C_VEH), (Ls, (win[0] + off, win[1] + off), 'sim', C_SIM)):
            K, tm, dl, vaf, n = SY.rate_plant(L, ax_i, window=w)
            t, y = step_resp(gains, K, tm, dl, imax, fc_g, fc_d)
            rise, over = rise_overshoot(t, y)
            res[lab][key] = dict(K=K, tau_m=tm, delay=dl, vaf=vaf, rise=rise, over=over)
            ax.plot(t * 1e3, y, color=col, label=('vehicle' if key == 'veh' else 'SITL') + f' ({rise * 1e3:.0f} ms, {over * 100:.0f} %)')
        ax.axhline(1.0, color='k', lw=0.4)
        ax.set_title(lab, fontsize=8); ax.set_xlabel('time [ms]')
        ax.legend(loc='lower right', fontsize=6)
        ax.set_xlim(-50, 500)
    axs[0].set_ylabel('rate / setpoint')
    save(fig, out, 'fig_x8_ratestep')
    return res


# ---------------------------------------------------------------- drag
def drag_points(L):
    u = L.u
    lp = topic(u, 'vehicle_local_position'); tl = L.t(lp)
    tg = np.arange(tl[0] + 2, tl[-1] - 2, 0.05)
    at = topic(u, 'vehicle_attitude')
    q = np.stack([np.interp(tg, L.t(at), at[f'q[{i}]']) for i in range(4)], 1)
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
    return vb[m, :2], acc[m]


def fig_drag(out, veh_logs, sim_logs):
    fig, ax = plt.subplots(figsize=(COL1, 2.3))
    res = {}
    for logs, col, lab in ((veh_logs, C_VEH, 'vehicle'), (sim_logs, C_SIM, 'SITL')):
        V, A = [], []
        for f in logs:
            vb, acc = drag_points(SY.Log(f))
            V.append(vb.ravel()); A.append(acc.ravel())
        V, A = np.concatenate(V), np.concatenate(A)
        big = np.abs(V) > 0.5
        k = float(np.polyfit(V[big], A[big], 1)[0])
        res[lab] = k
        sel = np.random.default_rng(0).choice(len(V), min(len(V), 4000), replace=False)
        ax.plot(V[sel], A[sel], '.', ms=1.2, color=col, alpha=0.35)
        vv = np.array([-6, 6])
        ax.plot(vv, k * vv, color=col, lw=1.2, label=f'{lab}: {k:+.3f} 1/s')
    ax.plot([-6, 6], [0.66, -0.66], color='k', lw=0.6, ls='--', label='model: −0.110 1/s')
    ax.set_xlim(-6, 6); ax.set_ylim(-1.5, 1.5)
    ax.set_xlabel('body velocity [m/s]'); ax.set_ylabel('specific force [m/s²]')
    ax.legend(loc='upper right')
    save(fig, out, 'fig_x8_drag')
    return res


# ---------------------------------------------------------------- X8 + PF speed steps
def fig_speedstep(out, run):
    st = SA.load_csv(os.path.join(run, 'x8_step_test.csv'))
    px = SA.load_csv(os.path.join(run, 'px4.csv'))
    summ = json.load(open(os.path.join(run, 'x8_step_summary.json')))
    seg = {}
    for i in range(len(st)):
        seg.setdefault(st['seg'][i], float(st['t'][i]))
    fig, axs = plt.subplots(1, 2, figsize=(COL2, 2.0))
    tt = np.arange(0, 3.0, SA.DT)
    for name, lab, col in (('v0to3', '0→3 m/s', C_VEH), ('v3to1', '3→1 m/s', '#2a9d8f'), ('v1to0', '1→0 m/s', '#e9c46a'),
                           ('turn90', '90° turn, 3 m/s', '#f4a261'), ('v0to5', '0→5 m/s', C_SIM)):
        if name not in seg:
            continue
        ts = seg[name]
        h = np.radians(float(st['heading_cmd'][st['seg'] == name][0]))
        y1 = float(st['speed_cmd'][st['seg'] == name][0])
        v = px['vy'] * np.cos(h) + px['vx'] * np.sin(h)
        y0 = float(np.mean(v[(px['t'] >= ts - 0.5) & (px['t'] < ts)]))
        r = (np.interp(ts + tt, px['t'], v) - y0) / (y1 - y0)
        axs[0].plot(tt, r, color=col, label=lab)
    fd, ft = summ['overall']['fit_delay'], summ['overall']['fit_tau']
    axs[0].plot(tt, SA.train_speed(len(tt), np.array(fd), np.array(ft)), 'k--', lw=1.0,
                label=f'training model, τ {ft:.2f} s, delay {fd:.2f} s')
    axs[0].set_xlabel('time after command [s]'); axs[0].set_ylabel('normalised speed along command')
    axs[0].legend(fontsize=6, loc='lower right'); axs[0].set_xlim(0, 3)
    rows = [r for r in summ['speed'] if 'reverse' not in r['step']]
    x = np.arange(len(rows))
    axs[1].bar(x - 0.2, [r['t63'] for r in rows], 0.4, color=C_SIM, label='63 %')
    axs[1].bar(x + 0.2, [r['t90'] for r in rows], 0.4, color=C_VEH, label='90 %')
    axs[1].set_xticks(x); axs[1].set_xticklabels([r['step'].replace('speed ', '').replace(' m/s', '').replace('turn 90 deg at 3', '90° turn')
                                                  .replace(' (2)', '′') for r in rows], rotation=35, ha='right', fontsize=6)
    axs[1].set_ylabel('time [s]'); axs[1].legend(fontsize=6)
    save(fig, out, 'fig_x8_speedstep')
    return summ


def sitl_hover(L):
    """Hover motor command of a SITL flight, as the vehicle's PWM (1100 + 800 u). PX4's SITL
    PWM outputs span 1000-2000 for u 0..1 (after THR_MDL_FAC); the vehicle's span 1100-1900."""
    u = L.u
    ao = topic(u, 'actuator_outputs'); t = L.t(ao)
    out = np.stack([ao[f'output[{i}]'] for i in range(8)], 1)
    lp = topic(u, 'vehicle_local_position'); tl = L.t(lp)
    vz, az = np.interp(t, tl, lp['vz']), np.interp(t, tl, lp['az'])
    vh = np.hypot(np.interp(t, tl, lp['vx']), np.interp(t, tl, lp['vy']))
    alt = -np.interp(t, tl, lp['z'])
    hov = L.airborne(t) & (alt - np.percentile(alt, 2) > 1.5) & (np.abs(vz) < 0.1) & (np.abs(az) < 0.2) & (vh < 0.1)
    um = (out[hov].mean() - 1000.0) / 1000.0
    return dict(n=int(hov.sum()), u=float(um), pwm_equiv=1100.0 + 800.0 * float(um))


# ---------------------------------------------------------------- table
def table(out, p, replay, rate, drag, hover_n):
    def f(x, n=2):
        return f'{x:.{n}f}'
    lines = [
        r'\begin{table}[t]', r'\centering', r'\caption{A4VAI X8 SITL model: parameters and validation against flight logs.}',
        r'\label{tab:x8_model}', r'\small', r'\begin{tabular}{lll}', r'\hline',
        r'Parameter & This model & Custom-repo model \\', r'\hline',
        rf'Mass, inertia & 10.4\,kg; 0.603/0.751/1.005\,kg\,m$^2$ (measured) & 18.4\,kg (rotor links 1\,kg each) \\',
        rf'Thrust & $T = {p["kT"]:.3e}\,\omega^2$, max {p["T_max"]:.1f}\,N/rotor (T/W {p["TW"]:.2f}) & $1.5\times10^{{-4}}\omega^2$, 117\,N/rotor (T/W 9.2) \\',
        rf'Motor map & $\omega = (u + {p["input_offset"]:.4f})\cdot{p["input_scaling"]:.1f}$ rad/s & $\omega = 1000\,u$ \\',
        rf'Hover & $u$ {p["u_hover"]:.3f} (1515\,µs), normalised thrust {p["t_hover"]:.3f} & $\approx 0.33$ \\',
        r'Yaw moment & $0.0199\,T$ + rotor spin-up reaction ($J_r = 2.8\times10^{-4}$\,kg\,m$^2$) & $0.05\,T$ \\',
        rf'Rotor drag & {p["rotorDragCoefficient"]:.3e} (0.11\,s$^{{-1}}$) & 0.008 ($\approx$26$\times$) \\',
        r'Motor time constant & 25\,ms & 9.5/25\,ms \\',
        r'\hline', r'Validation & Vehicle & SITL \\', r'\hline',
    ]
    for lab in ('roll', 'pitch', 'yaw'):
        v, s = rate[lab]['veh'], rate[lab]['sim']
        lines.append(rf'{lab.capitalize()} rate step: rise / overshoot & {v["rise"] * 1e3:.0f}\,ms / {v["over"] * 100:.0f}\,\% & '
                     rf'{s["rise"] * 1e3:.0f}\,ms / {s["over"] * 100:.0f}\,\% \\')
    lines.append(rf'Yaw rate s.d. (same commands) & {replay["yaw"]["veh_sd"]:.1f}\,°/s & {replay["yaw"]["sim_sd"]:.1f}\,°/s \\')
    lines.append(rf'Yaw torque setpoint s.d. & {f(replay["yaw"]["veh_tq_sd"], 3)} & {f(replay["yaw"]["sim_tq_sd"], 3)} \\')
    lines.append(rf'Horizontal drag & {drag["vehicle"]:+.3f}\,s$^{{-1}}$ & {drag["SITL"]:+.3f}\,s$^{{-1}}$ \\')
    lines.append(rf'Hover PWM & 1515\,µs (1479--1533) & {hover_n:.0f}\,µs \\')
    lines += [r'\hline', r'\end{tabular}', r'\end{table}']
    open(os.path.join(out, 'tab_x8_model.tex'), 'w').write('\n'.join(lines) + '\n')
    print('wrote tab_x8_model.tex')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--vehicle-dir', required=True)
    ap.add_argument('--replay', required=True, help='fly_test run dir of x8_replay.sh (px4_sitl.ulg)')
    ap.add_argument('--step', required=True, help='fly_test run dir of x8_step_test.sh')
    ap.add_argument('--out', required=True)
    ap.add_argument('--window', type=float, nargs=2, default=(25.0, 80.0), help='replayed window of the vehicle log [s]')
    ap.add_argument('--show', type=float, nargs=2, default=(40.0, 52.0), help='time-series window shown [s]')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    p = M.derive()
    veh_rate = os.path.join(a.vehicle_dir, '250731', '로그', '2025-7-31-rate_tune_test.ulg')
    sim = os.path.join(a.replay, 'px4_sitl.ulg')
    fig_thrust(a.out, p)
    rp = fig_replay(a.out, veh_rate, sim, a.window, a.show)
    rate = fig_ratestep(a.out, veh_rate, sim, a.window, rp['offset'])
    veh_all = sorted(glob.glob(os.path.join(a.vehicle_dir, '0107', '*.ulg'))) + \
        sorted(glob.glob(os.path.join(a.vehicle_dir, '250731', '*', '*.ulg')))
    dr = fig_drag(a.out, veh_all, [sim, os.path.join(a.step, 'px4_sitl.ulg')])
    fig_speedstep(a.out, a.step)
    h = sitl_hover(SY.Log(os.path.join(a.step, 'px4_sitl.ulg')))
    hover_pwm = h['pwm_equiv']
    table(a.out, p, rp, rate, dr, hover_pwm)
    json.dump(dict(replay=rp, rate=rate, drag=dr, hover=h), open(os.path.join(a.out, 'x8_validation.json'), 'w'), indent=1)
    print(json.dumps(dict(replay=rp, rate=rate, drag=dr, hover=h), indent=1))


if __name__ == '__main__':
    main()
