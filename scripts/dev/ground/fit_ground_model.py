#!/usr/bin/env python3
"""Fit the PX4 rover_differential response model of the MPPI rollout (mppi_kernel.cu GROUND_VEHICLE) to
ground_step_test.py logs: the logged speed / bearing setpoints drive the model, the time constants that
reproduce the logged forward speed and yaw best (RMS) are reported.
  fit_ground_model.py VEHICLE STEP_CSV [STEP_CSV ...] [--drive-only]     VEHICLE: ugv | usv (PX4 limits below)
  --drive-only  score only the driving regime: leave out PX4's spot turns (|bearing - yaw| > the switch angle)
                and 1 s after each - the regime the MPPI rollout predicts (the UGV's spot turns overshoot the
                120 deg/s limit, which this model does not reproduce)
Model (per 20 ms):
  spot turn  |bearing - yaw| > enter -> speed setpoint 0 until < exit           (RD_TRANS_DRV_TRN / _TRN_DRV)
  yaw        r_sp = clamp(kp * (bearing - yaw), +-r_lim)                      (RO_YAW_P, RO_YAW_RATE_LIM)
             r += clamp((r_sp - r) / tau_r, +-r_acc) dt                        (RO_YAW_ACCEL_LIM)
  speed      v_sp capped by the wheel / thruster differential left over:
             v_sp <= max_thr_speed * (1 - |r_sp| * track / 2 / max_thr_yaw_r)
             v += clamp((v_sp - v) / tau_v, -decel, accel) dt                  (RO_ACCEL_LIM / RO_DECEL_LIM)"""
import csv
import math
import sys

import numpy as np

PX4 = {
    # kp, r_lim, r_acc, accel, decel, enter, exit, max_thr_speed, max_thr_yaw_r, track (airframes 2010 / 2011)
    'ugv': dict(kp=5.0, r_lim=math.radians(120), r_acc=math.radians(120), accel=3.0, decel=5.0,
                enter=0.349066, exit=0.174533, max_thr_speed=2.7, max_thr_yaw_r=2.7, track=1.4),
    'usv': dict(kp=1.5, r_lim=math.radians(40), r_acc=math.radians(40), accel=1.0, decel=1.0,
                enter=1.75, exit=1.0, max_thr_speed=4.5, max_thr_yaw_r=1.0, track=2.0),
}


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def simulate(rows, p, tau_v, tau_r, r_acc_scale=1.0, cap=True):
    v, yaw, r, spot = rows[0]['speed_fwd'], rows[0]['yaw'], rows[0]['yaw_rate'], 0
    out_v, out_y = [], []
    for k, row in enumerate(rows):
        dt = rows[k + 1]['t'] - row['t'] if k + 1 < len(rows) else 0.02
        herr = wrap(row['bearing_cmd'] - yaw)
        if spot and abs(herr) < p['exit']:
            spot = 0
        elif not spot and abs(herr) > p['enter']:
            spot = 1
        r_sp = float(np.clip(p['kp'] * herr, -p['r_lim'], p['r_lim']))
        v_sp = 0.0 if spot else row['speed_cmd']
        if cap and not spot:
            v_sp = min(v_sp, p['max_thr_speed'] * max(1.0 - abs(r_sp) * p['track'] / 2.0 / p['max_thr_yaw_r'], 0.0))
        r += float(np.clip((r_sp - r) / tau_r, -p['r_acc'] * r_acc_scale, p['r_acc'] * r_acc_scale)) * dt
        v += float(np.clip((v_sp - v) / tau_v, -p['decel'], p['accel'])) * dt
        yaw = wrap(yaw + r * dt)
        out_v.append(v)
        out_y.append(yaw)
    return np.array(out_v), np.array(out_y)


def main():
    veh = sys.argv[1]
    p = PX4[veh]
    drive_only = '--drive-only' in sys.argv
    sets, masks = [], []
    for f in [a for a in sys.argv[2:] if not a.startswith('--')]:
        rows = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(open(f))]
        sets.append(rows)
        keep = np.ones(len(rows), bool)
        if drive_only:
            err = np.abs(wrap(np.array([r['bearing_cmd'] - r['yaw'] for r in rows])))
            for i in np.flatnonzero(err > p['enter']):
                keep[i:i + 50] = False
        masks.append(keep)
    best = []
    for tau_v in (0.15, 0.25, 0.35, 0.5, 0.7, 1.0, 1.3, 1.6, 2.0, 2.5, 3.0, 4.0):
        for tau_r in (0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.4, 0.5, 0.7, 1.0):
            for sc in (0.5, 1.0, 1.5, 2.0):
                ev, ey = [], []
                for rows, keep in zip(sets, masks):
                    sv, sy = simulate(rows, p, tau_v, tau_r, sc)
                    ev.append((sv - np.array([r['speed_fwd'] for r in rows]))[keep])
                    ey.append(np.degrees(wrap(sy - np.array([r['yaw'] for r in rows])))[keep])
                ev, ey = np.concatenate(ev), np.concatenate(ey)
                best.append((np.sqrt(np.mean(ev ** 2)) / 0.1 + np.sqrt(np.mean(ey ** 2)) / 2.0,
                             tau_v, tau_r, sc, np.sqrt(np.mean(ev ** 2)), np.sqrt(np.mean(ey ** 2)), np.abs(ey).max()))
    best.sort()
    print('score  tau_speed  tau_yaw_rate  yaw_accel x  speed rms [m/s]  yaw rms [deg]  yaw max [deg]')
    for b in best[:6]:
        print('%5.2f  %9.2f  %12.2f  %11.1f  %15.3f  %13.2f  %13.1f' % b)
    tau_v, tau_r, sc = best[0][1:4]
    sv, sy = simulate(sets[0], p, tau_v, tau_r, sc, cap=False)
    ev = sv - np.array([r['speed_fwd'] for r in sets[0]])
    print('without the speed cap while turning (best taus, first file): speed rms %.3f m/s' % np.sqrt(np.mean(ev ** 2)))


if __name__ == '__main__':
    main()
