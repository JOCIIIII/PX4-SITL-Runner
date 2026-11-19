#!/usr/bin/env python3
"""Monte Carlo plan for the RFP path-following verification (docs/montecarlo-plan-2026-11-05.md): one CSV row per run
- the course (line / circle / square / spiral, scripts/dev/pf_shapes.py sizes enlarged for the wind), the cruise speed
and the wind (mean speed, direction, Ornstein-Uhlenbeck gust std / time constant, body CdA), Latin hypercube per course.

usage: mc_gen.py OUT_CSV [--n 75] [--courses line circle square spiral] [--vmin 3 --vmax 7] [--wmin 0 --wmax 10]
                 [--seed 20261105] [--first 0]
  --n      runs per course: (speed, wind) uniform over the operating envelope below, a Latin hypercube over
           (direction, gust std, gust tau, CdA factor) for each course
  --first  index of the first run (to append a second block, e.g. the pilot's strong-wind corner)

Operating envelope (2026-11-05: conditions the vehicle cannot fly are left out, "말이 안 되는 실패 조건은 빼"):
speed + mean wind <= AIRSPEED_MAX = 15.5 m/s, the airspeed at which flying straight into the wind already takes 25 deg
of the 30 deg tilt limit (X8 10.4 kg, rotor drag 1.16 N s/m on the airspeed + body drag 0.5 rho 0.2 m^2 airspeed^2) -
5 deg stay for gusts and control. The sample is uniform over (speed, wind) inside it (rejection), Latin hypercube in
the other variables.
Course sizes: pf_shapes.py's size for the speed, but every turn radius at least v^2 / a_avail, a_avail = the lateral
acceleration left at the 30 deg tilt limit flying v straight into the strongest wind allowed at that speed,
min(10, AIRSPEED_MAX - v). Circle: radius R, 2 laps. Square: corner radius R, side max(pf_shapes side, 2 R + 3 v).
Spiral: R -> R + 8 m, 2 turns. Line: pf_shapes length (accelerate, ~4 s at speed, brake).
"""
import argparse
import csv
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from pf_shapes import default_size  # noqa: E402

M, G, RHO, CDA, KR, TILT = 10.4, 9.81, 1.225, 0.2, 1.16, 30.0
TILT_ENV = 25.0


def drag(a):
    return KR * a + 0.5 * RHO * CDA * a * a


def airspeed_at(tilt_deg):
    """the airspeed whose drag takes tilt_deg of tilt"""
    f = M * G * math.tan(math.radians(tilt_deg))
    return (-KR + math.sqrt(KR * KR + 2.0 * RHO * CDA * f)) / (RHO * CDA)


AIRSPEED_MAX = round(airspeed_at(TILT_ENV), 1)            # 15.5 m/s


def feasible(v, w):
    return v + w <= AIRSPEED_MAX


def turn_radius(v, wmax=10.0):
    """the smallest turn radius flown at v inside the 30 deg tilt limit in the strongest headwind allowed at v"""
    a = v + min(wmax, AIRSPEED_MAX - v)
    f_max = M * G * math.tan(math.radians(TILT))
    lat = math.sqrt(max(f_max * f_max - drag(a) ** 2, 0.0)) / M
    return round(max(default_size('circle', v), v * v / lat), 1)


def course(shape, v):
    """(size, corner, laps) for pf_shapes.py"""
    R = turn_radius(v)
    if shape == 'line':
        return default_size('line', v), '', 1
    if shape == 'circle':
        return R, '', 2
    if shape == 'square':
        return round(max(default_size('square', v), 2.0 * R + 3.0 * v), 1), R, 1
    return round(R + 8.0, 1), '', 2                       # spiral: inner R (pf_shapes: size - 8), 2 turns


def lhs(rng, n, dims):
    """n points in [0,1)^dims, one per stratum in every dimension"""
    u = (rng.random((n, dims)) + np.arange(n)[:, None]) / n
    for d in range(dims):
        u[:, d] = u[rng.permutation(n), d]
    return u


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('out')
    ap.add_argument('--n', type=int, default=75)
    ap.add_argument('--courses', nargs='+', default=['line', 'circle', 'square', 'spiral'])
    ap.add_argument('--vmin', type=float, default=3.0)
    ap.add_argument('--vmax', type=float, default=7.0)
    ap.add_argument('--wmin', type=float, default=0.0)
    ap.add_argument('--wmax', type=float, default=10.0)
    ap.add_argument('--gust', nargs=2, type=float, default=[0.5, 1.5])
    ap.add_argument('--tau', nargs=2, type=float, default=[1.0, 4.0])
    ap.add_argument('--cda', nargs=2, type=float, default=[0.7, 1.3], help='factor on the body CdA 0.2 m^2')
    ap.add_argument('--seed', type=int, default=20261105)
    ap.add_argument('--first', type=int, default=0)
    a = ap.parse_args()
    rng = np.random.default_rng(a.seed)
    rows = []
    idx = a.first
    for shape in a.courses:
        vw = []                                   # uniform over the envelope: draw, keep the feasible ones
        while len(vw) < a.n:
            v_, w_ = a.vmin + (a.vmax - a.vmin) * rng.random(), a.wmin + (a.wmax - a.wmin) * rng.random()
            if feasible(v_, w_):
                vw.append((v_, w_))
        u = lhs(rng, a.n, 6)
        for k in range(a.n):
            v, w = vw[k]
            size, corner, laps = course(shape, v)
            rows.append(dict(
                run=idx, course=shape, speed=round(v, 2), wind=round(w, 2),
                wind_toward_deg=round(360.0 * u[k, 2], 1),           # compass bearing the wind blows toward
                gust_std=round(a.gust[0] + (a.gust[1] - a.gust[0]) * u[k, 3], 2),
                gust_tau=round(a.tau[0] + (a.tau[1] - a.tau[0]) * u[k, 4], 2),
                cda=round(CDA * (a.cda[0] + (a.cda[1] - a.cda[0]) * u[k, 5]), 3),
                size=size, corner=corner, laps=laps, seed=a.seed * 1000 + idx))
            idx += 1
    with open(a.out, 'w', newline='') as f:
        wr = csv.DictWriter(f, fieldnames=list(rows[0]))
        wr.writeheader()
        wr.writerows(rows)
    print(f'{len(rows)} runs -> {a.out}')
    for shape in a.courses:
        r = [x for x in rows if x['course'] == shape]
        print(f'  {shape:6s} {len(r):3d} runs, speed {min(x["speed"] for x in r):.1f}-{max(x["speed"] for x in r):.1f} '
              f'm/s, wind {min(x["wind"] for x in r):.1f}-{max(x["wind"] for x in r):.1f} m/s, size '
              f'{min(x["size"] for x in r):g}-{max(x["size"] for x in r):g} m')


if __name__ == '__main__':
    main()
