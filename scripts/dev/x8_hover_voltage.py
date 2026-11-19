#!/usr/bin/env python3
"""Hover command vs battery voltage on the real X8 (read-only, for the paper).

For each ULog: take level-hover windows (airborne, more than 1 m above the take-off height,
|vz| < 0.15 m/s, horizontal speed < 1 m/s for 90 % of a 1 s window, and a steady motor command:
standard deviation of the 8-motor mean PWM within the window below PWM_STD_MAX),
and report the hover PWM (mean of the 8 motors), PX4's normalised thrust setpoint and the battery
voltage. Voltages are converted to one scale: `BAT1_V_DIV` differs between logs (18.0 in the
identification flights, 9.152 on 2026-11-02), so V_true = V_logged * V_DIV_REF / BAT1_V_DIV.

Then fit, over all hover samples of the identification flights, the ESC duty
d = (PWM - 1000) / 1000 against voltage, and predict the hover command at another voltage.

  x8_hover_voltage.py --fit ID1.ulg ID2.ulg ... --check NOV1.ulg NOV2.ulg
"""
import argparse
import numpy as np
from pyulog import ULog

V_DIV_REF = 9.15221881866455      # divider on 2026-11-02 (ground reading 23.8 V on a 6S pack)
THR_MDL_FAC = 0.3
PWM_MIN, PWM_MAX = 1100.0, 1900.0
PWM_STD_MAX = 5.0                # [us] steady-command limit within a hover window (manual throttle work exceeds it)


def topic(u, name, mid=0):
    for d in u.data_list:
        if d.name == name and d.multi_id == mid:
            return d.data
    return None


def thrust_from_pwm(pwm):
    """PX4 normalised thrust for a motor PWM (THR_MDL_FAC curve, 1100-1900 us)."""
    x = (pwm - PWM_MIN) / (PWM_MAX - PWM_MIN)
    return THR_MDL_FAC * x * x + (1 - THR_MDL_FAC) * x


def hover_samples(path, win=1.0):
    u = ULog(path, ['vehicle_local_position', 'vehicle_land_detected', 'actuator_outputs',
                    'vehicle_thrust_setpoint', 'battery_status', 'hover_thrust_estimate'])
    P = u.initial_parameters
    lp = topic(u, 'vehicle_local_position')
    t = lp['timestamp'] / 1e6
    land = topic(u, 'vehicle_land_detected')
    air = np.interp(t, land['timestamp'] / 1e6, (~land['landed'].astype(bool)).astype(float)) > 0.5
    h = -lp['z'] - np.median(-lp['z'][~air]) if (~air).any() else -lp['z'] + lp['z'][0]
    ok = air & (h > 1.0) & (np.abs(lp['vz']) < 0.15) & (np.hypot(lp['vx'], lp['vy']) < 1.0)
    ao = topic(u, 'actuator_outputs')
    pwm = np.mean(np.vstack([ao['output[%d]' % i] for i in range(8)]), axis=0)
    ta = ao['timestamp'] / 1e6
    ts = topic(u, 'vehicle_thrust_setpoint')
    bs = topic(u, 'battery_status')
    scale = V_DIV_REF / float(P['BAT1_V_DIV'])
    out = []
    t0 = t[0]
    while t0 < t[-1]:
        m = (t >= t0) & (t < t0 + win)
        if m.sum() and ok[m].mean() > 0.9:       # a window that is hover throughout
            ma = (ta >= t0) & (ta < t0 + win)
            mt = (ts['timestamp'] / 1e6 >= t0) & (ts['timestamp'] / 1e6 < t0 + win)
            mb = (bs['timestamp'] / 1e6 >= t0) & (bs['timestamp'] / 1e6 < t0 + win)
            if ma.sum() and mt.sum() and mb.sum() and pwm[ma].std() < PWM_STD_MAX:
                out.append((t0 - t[0], float(pwm[ma].mean()), float(-ts['xyz[2]'][mt].mean()),
                            float(bs['voltage_v'][mb].mean() * scale)))
        t0 += win
    hte = topic(u, 'hover_thrust_estimate')
    hte_med = float(np.median(hte['hover_thrust'][hte['valid'] > 0])) if hte is not None and (hte['valid'] > 0).any() else float('nan')
    return np.array(out), dict(v_div=float(P['BAT1_V_DIV']), pwm_min=P.get('PWM_MAIN_MIN1'), pwm_max=P.get('PWM_MAIN_MAX1'),
                               thr_mdl=P.get('THR_MDL_FAC'), bat_scale=P.get('MC_BAT_SCALE_EN'), hte=hte_med)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--fit', nargs='+', required=True)
    ap.add_argument('--check', nargs='*', default=[])
    a = ap.parse_args()
    allv, alld = [], []
    print('log, hover windows(1 s), PWM mean [min-max], thrust sp, voltage mean [min-max], BAT1_V_DIV, PWM range, HTE')
    for group, paths in (('fit', a.fit), ('check', a.check)):
        for p in paths:
            s, info = hover_samples(p)
            name = p.split('/')[-1][:44]
            if not len(s):
                print(f'{group:5s} {name:44s} no level-hover window')
                continue
            print(f'{group:5s} {name:44s} n={len(s):3d}  PWM {s[:,1].mean():6.0f} [{s[:,1].min():.0f}-{s[:,1].max():.0f}]  '
                  f'thr {s[:,2].mean():.3f}  V {s[:,3].mean():5.2f} [{s[:,3].min():.2f}-{s[:,3].max():.2f}]  '
                  f'div {info["v_div"]:.2f}  pwm {info["pwm_min"]}-{info["pwm_max"]}  hte {info["hte"]:.3f}')
            if group == 'fit':
                allv += list(s[:, 3]); alld += list((s[:, 1] - 1000) / 1000)
            else:
                info['s'] = s
                a.__dict__.setdefault('chk', []).append((name, s, info))
    V, D = np.array(allv), np.array(alld)
    k = float(np.sum(D / V) / np.sum(1 / V ** 2))          # d = k / V  (least squares)
    A = np.vstack([V, np.ones_like(V)]).T
    slope, icpt = np.linalg.lstsq(A, D, rcond=None)[0]     # d = slope * V + icpt
    r = np.corrcoef(V, D)[0, 1]
    res_k = D - k / V
    print(f'\nfit on {len(V)} hover windows, V {V.min():.2f}-{V.max():.2f}: '
          f'd = {k:.3f}/V (rms {np.sqrt(np.mean(res_k**2))*1000:.0f} us)   '
          f'linear d = {slope*1000:.1f} us/V * V + {icpt*1000:.0f} us (r = {r:.2f})')
    for name, s, info in getattr(a, 'chk', []):
        v = s[:, 3].mean()
        for label, d in (('1/V law', k / v), ('linear', slope * v + icpt)):
            pwm = 1000 + 1000 * d
            print(f'check {name}: at {v:.2f} V  {label:8s} predicts PWM {pwm:.0f} us, thrust {thrust_from_pwm(pwm):.3f}   '
                  f'measured PWM {s[:,1].mean():.0f} us, thrust sp {s[:,2].mean():.3f}')
    v_id = V.mean()
    print(f'identification flights: mean hover V {v_id:.2f}, mean PWM {1000+1000*D.mean():.0f} us, thrust {thrust_from_pwm(1000+1000*D.mean()):.3f}')


if __name__ == '__main__':
    main()
