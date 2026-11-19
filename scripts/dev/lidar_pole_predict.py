#!/usr/bin/env python3
"""Hits on a thin vertical pole predicted from the recorded Mid-360 scan pattern (for the lidar bench test).

The simulator lidar shoots along the recorded pattern with zero-width rays, so the number of returns from a
pole is geometry only: a shot hits if its direction passes within the pole's radius, between the pole's bottom
and top. This script counts that per 0.1 s frame over every frame phase of the pattern, for a table of
distances and diameters, so the same poles measured with the real sensor (lidar_scene_check.py: returns per
frame, share of frames that saw the pole) can be compared with what the simulator gives.

  lidar_pole_predict.py [--pattern FILE] [--dist 3 5 7 10 12] [--diam 0.02 0.03 0.1 0.5]
                        [--sensor-height 1.0] [--pole-bottom 0.0] [--pole-top 2.0] [--bearing 0]
                        [--pitch 0 --roll 0] [--beam-mrad 0] [--csv OUT.csv]

--sensor-height   lidar above the ground [m]
--pole-bottom/top the part of the pole that counts, above the ground [m] (lidar_scene_check.py drops returns
                  below 0.25 m as ground: use --pole-bottom 0.25 to compare with its numbers)
--bearing         pole direction from the sensor's x axis [deg], positive to the left
--pitch/--roll    sensor mount angles [deg] (nose-up pitch positive; as lidar_scene_check.py --pitch/--roll)
--beam-mrad       full beam divergence [mrad]: a shot also counts if its centre passes within half the beam
                  width of the pole surface. 0 = the simulator's zero-width ray. The real sensor's
                  output-beam divergence is not published; this is for sensitivity only.
"""
import os
import argparse
import struct

import numpy as np

DEFAULT_PATTERN = os.path.expanduser('~/Documents/InhouseSim-deps/mid360_pattern/mid360_pattern_2s.bin')


def load_pattern(path):
    with open(path, 'rb') as f:
        assert f.read(8) == b'MID360PT'
        ver, n, pps, nl = struct.unpack('<IIfI', f.read(16))
        a = np.frombuffer(f.read(n * 8), dtype='<f4').reshape(n, 2).astype(np.float64)
    return a[:, 0], a[:, 1], pps


def rot(roll, pitch):
    """Sensor -> level frame (x forward, y left, z up). Positive pitch = nose up, positive roll = right side down."""
    r, p = np.radians(roll), np.radians(pitch)
    Rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    Ry = np.array([[np.cos(p), 0, -np.sin(p)], [0, 1, 0], [np.sin(p), 0, np.cos(p)]])
    return Ry @ Rx


def pole_hits(az, el, pps, dist, diam, sensor_h, bottom, top, bearing, R, beam_mrad, frame=0.1):
    """Per-frame hit counts over frame phases (frames start every 0.01 s through the pattern, wrapping)."""
    a, e = np.radians(az), np.radians(el)
    d = np.stack([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)], 1) @ R.T      # level frame
    ok = np.isfinite(d).all(1)
    b = np.radians(bearing)
    c = np.array([dist * np.cos(b), dist * np.sin(b)])                                   # pole axis (x, y)
    # closest approach of the horizontal ray to the pole axis
    h = np.hypot(d[:, 0], d[:, 1])
    ux, uy = d[:, 0] / np.maximum(h, 1e-12), d[:, 1] / np.maximum(h, 1e-12)
    along = c[0] * ux + c[1] * uy                    # horizontal range to the closest point
    perp = np.abs(c[0] * uy - c[1] * ux)
    rng = along / np.maximum(h, 1e-12)               # slant range there
    z = sensor_h + rng * d[:, 2]
    rad = diam / 2 + 0.5 * beam_mrad * 1e-3 * np.maximum(rng, 0)
    hit = ok & (along > 0) & (perp <= rad) & (z >= bottom) & (z <= top)
    n = len(hit)
    per = int(round(frame * pps))
    cs = np.concatenate([[0], np.cumsum(np.concatenate([hit, hit[:per]]).astype(np.int64))])
    starts = np.arange(0, n, int(round(0.01 * pps)))
    return cs[starts + per] - cs[starts]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--pattern', default=DEFAULT_PATTERN)
    ap.add_argument('--dist', type=float, nargs='+', default=[3, 5, 7, 10, 12])
    ap.add_argument('--diam', type=float, nargs='+', default=[0.02, 0.03, 0.1, 0.5])
    ap.add_argument('--sensor-height', type=float, default=1.0)
    ap.add_argument('--pole-bottom', type=float, default=0.0)
    ap.add_argument('--pole-top', type=float, default=2.0)
    ap.add_argument('--bearing', type=float, default=0.0)
    ap.add_argument('--pitch', type=float, default=0.0)
    ap.add_argument('--roll', type=float, default=0.0)
    ap.add_argument('--beam-mrad', type=float, default=0.0)
    ap.add_argument('--csv', default=None)
    a = ap.parse_args()
    az, el, pps = load_pattern(a.pattern)
    R = rot(a.roll, a.pitch)
    print(f'pattern {a.pattern.split("/")[-1]}: {len(az)} shots, {pps:.0f}/s; sensor {a.sensor_height} m above ground, '
          f'pole {a.pole_bottom}-{a.pole_top} m, bearing {a.bearing} deg, mount pitch {a.pitch} roll {a.roll} deg, beam {a.beam_mrad} mrad')
    print('diameter [m] | distance [m] | hits per 0.1 s frame: mean (min-max) | frames with >= 1 hit [%] | frames with >= 2 hits [%]')
    rows = []
    for dm in a.diam:
        for ds in a.dist:
            k = pole_hits(az, el, pps, ds, dm, a.sensor_height, a.pole_bottom, a.pole_top, a.bearing, R, a.beam_mrad)
            rows.append((dm, ds, k.mean(), k.min(), k.max(), 100 * np.mean(k >= 1), 100 * np.mean(k >= 2)))
            print(f'{dm:g} | {ds:g} | {k.mean():.2f} ({k.min()}-{k.max()}) | {100 * np.mean(k >= 1):.0f} | {100 * np.mean(k >= 2):.0f}')
    if a.csv:
        with open(a.csv, 'w') as f:
            f.write('diameter_m,distance_m,hits_mean,hits_min,hits_max,frames_ge1_pct,frames_ge2_pct\n')
            for r in rows:
                f.write(','.join(f'{x:g}' for x in r) + '\n')


if __name__ == '__main__':
    main()
