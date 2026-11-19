#!/usr/bin/env python3
"""Vehicle flight vs its SITL replay (scripts/ros2/apps/x8_replay.sh), any flight and window (for the paper).

  x8_replay_compare.py VEHICLE.ulg SITL_RUN_DIR T0 T1 OUT_DIR [--show A B] [--label NAME]

T0 T1 = the replayed window in the vehicle log's time [s] (the window given to x8_replay_export.py).
Prints and saves (OUT_DIR/replay_compare.json): body-rate and torque-command spread of both over the
window, motor saturation, and the rate plant identified from each over the same window (x8_sysid.rate_plant:
gain K, lag, delay, fit). The vehicle's K is also given on the SITL's command scale: a unit of motor
command is PWM_MAIN_MAX - PWM_MAIN_MIN microseconds on the vehicle and 800 (1100-1900) in the SITL model,
so K_vehicle * 800 / span is the like-for-like value. Also notes each log's firmware and MC_YAW_TQ_CUTOFF
(PX4 v1.16 low-passes the yaw torque command, 2 Hz by default; v1.14 has no such filter).
The time-series figure fig_x8_replay.png of the pair is written to OUT_DIR as well.
"""
import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import x8_paper_figs as F  # noqa: E402
import x8_sysid as SY  # noqa: E402

SIM_SPAN = 800.0            # the SITL motor map: command 0..1 = 1100..1900 us


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('vehicle')
    ap.add_argument('sitl_run')
    ap.add_argument('t0', type=float)
    ap.add_argument('t1', type=float)
    ap.add_argument('out')
    ap.add_argument('--show', type=float, nargs=2, default=None)
    ap.add_argument('--label', default='')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    win = (a.t0, a.t1)
    show = tuple(a.show) if a.show else (a.t0 + 15.0, a.t0 + 27.0)
    sim = os.path.join(a.sitl_run, 'px4_sitl.ulg')
    Lv, Ls = SY.Log(a.vehicle), SY.Log(sim)
    span = float(Lv.p.get('PWM_MAIN_MAX1', 1900.0) - Lv.p.get('PWM_MAIN_MIN1', 1100.0))
    scale = SIM_SPAN / span
    info = {}
    for key, L in (('vehicle', Lv), ('sitl', Ls)):
        gains = {k: round(float(L.p[k]), 5) for k in (
            'MC_ROLLRATE_P', 'MC_ROLLRATE_I', 'MC_ROLLRATE_D', 'MC_PITCHRATE_P', 'MC_PITCHRATE_I', 'MC_PITCHRATE_D',
            'MC_YAWRATE_P', 'MC_YAWRATE_I', 'MC_YAWRATE_D', 'MC_ROLL_P', 'MC_PITCH_P', 'MC_YAW_P') if k in L.p}
        info[key] = dict(firmware=str(L.u.msg_info_dict.get('ver_sw_release', '')),
                         yaw_tq_cutoff=(float(L.p['MC_YAW_TQ_CUTOFF']) if 'MC_YAW_TQ_CUTOFF' in L.p else None),
                         gains=gains)
    rp = F.fig_replay(a.out, a.vehicle, sim, win, show)
    rate = F.fig_ratestep(a.out, a.vehicle, sim, win, rp['offset'])
    for ax in ('roll', 'pitch', 'yaw'):
        rate[ax]['veh']['K_on_sitl_scale'] = rate[ax]['veh']['K'] * scale
    res = dict(label=a.label, vehicle=os.path.basename(a.vehicle), sitl_run=os.path.basename(os.path.normpath(a.sitl_run)),
               window=win, vehicle_pwm_span=span, info=info, replay=rp, rate=rate)
    json.dump(res, open(os.path.join(a.out, 'replay_compare.json'), 'w'), indent=1)

    def release(v):
        try:
            v = int(v)
            return f'v{(v >> 24) & 255}.{(v >> 16) & 255}.{(v >> 8) & 255}'
        except (TypeError, ValueError):
            return str(v)

    print(f'\n=== {a.label or res["sitl_run"]}: {res["vehicle"]} window {win[0]:g}-{win[1]:g} s vs {res["sitl_run"]}')
    for key in ('vehicle', 'sitl'):
        i = info[key]
        print(f'  {key:7s} firmware {release(i["firmware"])}, MC_YAW_TQ_CUTOFF {i["yaw_tq_cutoff"]}, gains '
              + ' '.join(f'{k[3:]}={v:g}' for k, v in i['gains'].items()))
    print(f'  alignment: setpoint correlation {rp["corr"]:.3f}; motor saturation vehicle {rp["veh_sat"] * 100:.1f} %, '
          f'SITL {rp["sim_sat"] * 100:.1f} %')
    print('  axis  | rate sd [deg/s] veh / SITL | torque cmd sd veh / SITL (veh on SITL scale) | '
          'plant K veh (on SITL scale) / SITL | lag veh / SITL [ms] | delay veh / SITL [ms] | fit veh / SITL')
    for ax in ('roll', 'pitch', 'yaw'):
        s, r = rp[ax], rate[ax]
        print(f'  {ax:5s} | {s["veh_sd"]:5.1f} / {s["sim_sd"]:5.1f} | {s["veh_tq_sd"]:.4f} / {s["sim_tq_sd"]:.4f} '
              f'({s["veh_tq_sd"] / scale:.4f}) | {r["veh"]["K"]:6.1f} ({r["veh"]["K_on_sitl_scale"]:6.1f}) / {r["sim"]["K"]:6.1f} | '
              f'{r["veh"]["tau_m"] * 1e3:4.0f} / {r["sim"]["tau_m"] * 1e3:4.0f} | {r["veh"]["delay"] * 1e3:3.0f} / '
              f'{r["sim"]["delay"] * 1e3:3.0f} | {r["veh"]["vaf"]:.2f} / {r["sim"]["vaf"]:.2f}')


if __name__ == '__main__':
    main()
