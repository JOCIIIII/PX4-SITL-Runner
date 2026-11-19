#!/usr/bin/env python3
"""Returns from thin vertical poles in a recording of a real Mid-360, next to what the recorded scan pattern
(the simulator's lidar: zero-width rays) predicts for the same geometry. For the lidar bench test.

Recording: Livox SDK2 point packets, cartesian (data_type 1) or spherical (data_type 3), from a sensor standing still:
  ~/Documents/InhouseSim-deps/mid360_tests/record_direct.py SECONDS OUT.npz   (PC wired to the sensor)
or a raw_packets_*.npy (N x 1380) as in ~/Documents/InhouseSim-deps/mid360_pattern/.

  lidar_pole_measure.py REC --sensor-height H --pole DIST BEARING DIAM [--pole ...]
                        [--pole-top 2.0] [--cut 0.25] [--pitch 0 --roll 0] [--search 0.6] [--assoc 0.10]
                        [--pattern FILE] [--csv OUT.csv]

--pole DIST BEARING DIAM   where the pole was put: distance [m] and bearing [deg, + = left of the sensor's x
                           axis] by tape measure, and its diameter [m]. The script looks for the pole within
                           --search m of that spot and uses the position it finds.
--sensor-height            lidar optical centre above the ground [m]
--cut                      returns lower than this above the ground are not counted (ground) [m]
--pole-top                 pole top above the ground [m]
--pitch/--roll             sensor tilt [deg] (0 if the sensor stands level)
--assoc                    a return belongs to the pole if it is within DIAM/2 + assoc of its axis [m]

Per pole: returns per 0.1 s frame (20 000 shots), share of frames with at least 1 and 2 returns, the apparent
width of the pole in the returns, and the same from the pattern (lidar_pole_predict.py) at the found position.
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lidar_pole_predict as PP  # noqa: E402

PT = np.dtype([('x', '<i4'), ('y', '<i4'), ('z', '<i4'), ('r', 'u1'), ('t', 'u1')])
SP = np.dtype([('d', '<u4'), ('th', '<u2'), ('ph', '<u2'), ('r', 'u1'), ('t', 'u1')])
PER_FRAME = 20000


def load_points(path):
    raw = np.load(path, allow_pickle=False)
    if hasattr(raw, 'files'):
        raw = raw['pts']
    raw = np.asarray(raw)
    if raw.shape[1] not in (1380, 996):
        sys.exit(f'{path}: packets of {raw.shape[1]} bytes; need point packets of 1380 (cartesian) or 996 (spherical) bytes')
    cnt = raw[:, 7].astype(np.int64) + 256 * raw[:, 8].astype(np.int64)           # udp_cnt
    lost = int(np.sum((np.diff(cnt) % 65536) != 1))
    if raw.shape[1] == 996:                                                        # spherical (data_type 3)
        S = np.frombuffer(np.ascontiguousarray(raw[:, 36:]).tobytes(), dtype=SP)
        d = S['d'].astype(np.float64) / 1000.0
        th, ph = np.radians(S['th'] / 100.0), np.radians(S['ph'] / 100.0)
        X = np.stack([d * np.sin(th) * np.cos(ph), d * np.sin(th) * np.sin(ph), d * np.cos(th)], 1)
        return X, S['r'].astype(np.int32), lost, len(raw)
    P = np.frombuffer(np.ascontiguousarray(raw[:, 36:]).tobytes(), dtype=PT)
    X = np.stack([P['x'], P['y'], P['z']], 1).astype(np.float64) / 1000.0
    return X, P['r'].astype(np.int32), lost, len(raw)


def synthetic(a, R):
    """A recording as the simulator would give it: every pattern shot is a zero-width ray that returns from the
    nearest of the poles (cylinders, ground to --pole-top) and the ground plane. Used to check this script
    against lidar_pole_predict.py: measured / pattern must come out as 1."""
    az, el, pps = PP.load_pattern(a.pattern)
    az, el = np.radians(az), np.radians(el)
    d = np.stack([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)], 1)      # sensor frame
    dl = d @ R.T                                                                          # level frame
    rng = np.full(len(d), np.inf)
    down = dl[:, 2] < -1e-6
    rng[down] = -a.sensor_height / dl[down, 2]                                            # ground
    h = np.hypot(dl[:, 0], dl[:, 1])
    for dist, bearing, diam in a.pole:
        b = np.radians(bearing)
        c = np.array([dist * np.cos(b), dist * np.sin(b)])
        ux, uy = dl[:, 0] / np.maximum(h, 1e-12), dl[:, 1] / np.maximum(h, 1e-12)
        along = c[0] * ux + c[1] * uy
        perp = np.abs(c[0] * uy - c[1] * ux)
        hit = (along > 0) & (perp <= diam / 2)
        s_h = along - np.sqrt(np.maximum((diam / 2) ** 2 - perp ** 2, 0.0))                # to the cylinder surface
        r = s_h / np.maximum(h, 1e-12)
        z = a.sensor_height + r * dl[:, 2]
        hit &= (z >= 0) & (z <= a.pole_top) & (r < rng)
        rng[hit] = r[hit]
    rng[~np.isfinite(rng) | (rng > 70.0)] = 0.0
    X = d * rng[:, None]                                                                  # sensor frame, like the sensor
    X[~np.isfinite(X).all(1)] = 0.0
    return X, np.full(len(X), 50, dtype=np.int32), 0, len(X) // 96


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('rec', help="recording, or 'selftest': a synthetic recording made from the pattern with the given poles")
    ap.add_argument('--sensor-height', type=float, required=True)
    ap.add_argument('--pole', type=float, nargs=3, action='append', required=True, metavar=('DIST', 'BEARING', 'DIAM'))
    ap.add_argument('--pole-top', type=float, default=2.0)
    ap.add_argument('--cut', type=float, default=0.25)
    ap.add_argument('--pitch', type=float, default=0.0)
    ap.add_argument('--roll', type=float, default=0.0)
    ap.add_argument('--search', type=float, default=0.6)
    ap.add_argument('--assoc', type=float, default=0.10)
    ap.add_argument('--pattern', default=PP.DEFAULT_PATTERN)
    ap.add_argument('--csv', default=None)
    a = ap.parse_args()

    R = PP.rot(a.roll, a.pitch)
    if a.rec == 'selftest':
        X, refl, lost, npk = synthetic(a, R)
    else:
        X, refl, lost, npk = load_points(a.rec)
    valid = np.linalg.norm(X, axis=1) > 0.05
    L = X @ R.T                                         # level frame, origin at the sensor
    hgt = L[:, 2] + a.sensor_height                     # above the ground
    nfr = len(X) // PER_FRAME
    print(f'{os.path.basename(a.rec)}: {npk} packets, {len(X)} shots = {len(X) / 200000:.1f} s, {nfr} frames; '
          f'returns {100 * valid.mean():.1f} %, lost packets {lost}')
    if lost:
        print('  WARNING: packets were lost; frames are counted by shot index and will be shifted after each loss')
    g = valid & (np.hypot(L[:, 0], L[:, 1]) > 1.0) & (np.hypot(L[:, 0], L[:, 1]) < 6.0) & (hgt < 0.3)
    if g.sum() > 200:
        print(f'  ground check (returns 1-6 m away, below 0.3 m): median height {np.median(hgt[g]):+.3f} m '
              f'(0 = sensor height and tilt are right), ahead {np.median(hgt[g & (L[:, 0] > 0)]):+.3f}, '
              f'behind {np.median(hgt[g & (L[:, 0] < 0)]):+.3f}, left {np.median(hgt[g & (L[:, 1] > 0)]):+.3f}, '
              f'right {np.median(hgt[g & (L[:, 1] < 0)]):+.3f}')
    az, el, pps = PP.load_pattern(a.pattern)
    print('pole: placed (dist, bearing) -> found | diameter | measured returns/frame mean (min-max), frames >=1, >=2 [%] | '
          'apparent width [m] | pattern: returns/frame mean, frames >=1, >=2 [%] | measured/pattern')
    rows = []
    for dist, bearing, diam in a.pole:
        b = np.radians(bearing)
        c0 = np.array([dist * np.cos(b), dist * np.sin(b)])
        inh = valid & (hgt >= a.cut) & (hgt <= a.pole_top)
        near = inh & (np.hypot(L[:, 0] - c0[0], L[:, 1] - c0[1]) <= a.search)
        if near.sum() < 5:
            print(f'  ({dist:g} m, {bearing:+g} deg) d {diam:g}: no returns within {a.search} m of the placed spot')
            continue
        Q = L[near, :2]
        # densest 5 cm cell, then the median of the returns around it
        ij = np.floor((Q - c0) / 0.05).astype(int)
        keys, cnts = np.unique(ij, axis=0, return_counts=True)
        peak = c0 + (keys[np.argmax(cnts)] + 0.5) * 0.05
        c = np.median(Q[np.hypot(Q[:, 0] - peak[0], Q[:, 1] - peak[1]) <= 0.15], axis=0)
        # the returns lie on the near side of the pole: move from their median to the axis (about 0.8 radius further)
        c = c + 0.8 * (diam / 2) * c / max(np.hypot(*c), 1e-9)
        fd, fb = float(np.hypot(*c)), float(np.degrees(np.arctan2(c[1], c[0])))
        on = inh & (np.hypot(L[:, 0] - c[0], L[:, 1] - c[1]) <= diam / 2 + a.assoc)
        k = np.add.reduceat(on[:nfr * PER_FRAME].astype(np.int64), np.arange(0, nfr * PER_FRAME, PER_FRAME))
        # apparent width: across the line of sight
        u = np.array([-np.sin(np.radians(fb)), np.cos(np.radians(fb))])
        lat = (L[on, :2] - c) @ u
        width = float(np.percentile(lat, 97.5) - np.percentile(lat, 2.5)) if on.sum() >= 20 else float('nan')
        hs = hgt[on]
        kp = PP.pole_hits(az, el, pps, fd, diam, a.sensor_height, a.cut, a.pole_top, fb, R, 0.0)
        ratio = k.mean() / kp.mean() if kp.mean() > 0 else float('nan')
        print(f'  ({dist:g} m, {bearing:+g} deg) -> ({fd:.2f} m, {fb:+.1f} deg) | {diam:g} | {k.mean():.2f} ({k.min()}-{k.max()}), '
              f'{100 * np.mean(k >= 1):.0f}, {100 * np.mean(k >= 2):.0f} | {width:.3f} | {kp.mean():.2f}, '
              f'{100 * np.mean(kp >= 1):.0f}, {100 * np.mean(kp >= 2):.0f} | {ratio:.2f}'
              + (f'   returns span {hs.min():.2f}-{hs.max():.2f} m above ground, reflectivity median {int(np.median(refl[on]))}' if on.sum() else ''))
        rows.append((dist, bearing, diam, fd, fb, k.mean(), 100 * np.mean(k >= 1), 100 * np.mean(k >= 2), width,
                     kp.mean(), 100 * np.mean(kp >= 1), 100 * np.mean(kp >= 2), ratio))
    if rows:
        r = np.array(rows)
        ok = r[:, 9] > 0
        mape = 100 * np.mean(np.abs(r[ok, 5] - r[ok, 9]) / r[ok, 9])
        print(f'mean absolute percentage error of returns per frame, pattern vs measured, over {int(ok.sum())} poles: {mape:.1f} %')
    if a.csv and rows:
        with open(a.csv, 'w') as f:
            f.write('placed_dist_m,placed_bearing_deg,diameter_m,found_dist_m,found_bearing_deg,meas_hits_mean,meas_ge1_pct,'
                    'meas_ge2_pct,apparent_width_m,pattern_hits_mean,pattern_ge1_pct,pattern_ge2_pct,meas_over_pattern\n')
            for r in rows:
                f.write(','.join(f'{x:g}' for x in r) + '\n')


if __name__ == '__main__':
    main()
