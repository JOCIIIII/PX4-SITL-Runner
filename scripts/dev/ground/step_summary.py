#!/usr/bin/env python3
"""Summary of ground_step_test.py's step.csv: per step the speed and yaw responses (steady values, 63 % times,
peak yaw rate) - the numbers the PathFollowing ground output stage and the MPPI rollout plant use."""
import csv
import math
import sys


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


rows = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(open(sys.argv[1]))]
phases = sorted({int(r['phase']) for r in rows})
prev = None
for ph in phases:
    R = [r for r in rows if int(r['phase']) == ph]
    t0 = R[0]['t']
    vc, bc = R[0]['speed_cmd'], R[0]['bearing_cmd']
    v0, y0 = R[0]['speed_fwd'], R[0]['yaw']
    tail = [r for r in R if r['t'] - t0 > 0.7 * (R[-1]['t'] - t0)]
    v_ss = sum(r['speed_fwd'] for r in tail) / len(tail)
    yerr_ss = sum(wrap(bc - r['yaw']) for r in tail) / len(tail)
    r_peak = max(abs(r['yaw_rate']) for r in R)
    r_sign = max(R, key=lambda r: abs(r['yaw_rate']))['yaw_rate']
    line = f'step {ph}: cmd {vc:.2f} m/s, bearing {math.degrees(bc):7.1f} deg | speed {v0:.2f} -> {v_ss:.2f}'
    if abs(vc - v0) > 0.2:
        tgt = v0 + 0.632 * (v_ss - v0)
        t63 = next((r['t'] - t0 for r in R if (r['speed_fwd'] - tgt) * (v_ss - v0) >= 0), float('nan'))
        line += f' (63 % in {t63:.2f} s)'
    dy = wrap(bc - y0)
    if abs(dy) > math.radians(5):
        t63 = next((r['t'] - t0 for r in R if abs(wrap(bc - r['yaw'])) <= 0.368 * abs(dy)), float('nan'))
        vmin = min(r['speed_fwd'] for r in R)
        line += f' | yaw step {math.degrees(dy):.1f} deg: 63 % in {t63:.2f} s, peak rate {math.degrees(r_peak):.1f} deg/s ' \
                f'(sign {"+" if r_sign > 0 else "-"}), min speed {vmin:.2f}'
    line += f' | yaw err end {math.degrees(yerr_ss):.1f} deg'
    print(line)
print(f'path end: x {rows[-1]["x"]:.1f} y {rows[-1]["y"]:.1f}, armed {int(rows[-1]["armed"])}, offboard {int(rows[-1]["offboard"])}, nav {int(rows[-1]["nav_state"])}')
