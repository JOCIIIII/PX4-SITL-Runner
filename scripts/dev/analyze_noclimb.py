#!/usr/bin/env python3
"""Summarise the noclimb_ca per-step log of a fly_test.sh run (noclimb_ca.csv).

usage: analyze_noclimb.py RUN_DIR [RUN_DIR ...]

Per run: CA / final-approach time, commanded vs flown speed, the follower's horizontal
velocity response fitted as a first-order lag with dead time (the policy was trained for
tau 0.5-1.2 s or 0.3-0.8 s and 0-0.1 s command delay, agent.md section 9), stall timer
(training ended an episode at 4 s), altitude vs the path, avoidance activity and the
speed governor.
"""
import os
import sys

import numpy as np


def load(path):
    a = np.genfromtxt(path, delimiter=',', names=True, dtype=None, encoding=None)
    return a if a.size > 1 else None


def fit_tau(t, v, vc, max_delay=0.4):
    """v' = (vc(t - d) - v) / tau on the horizontal velocity vector; best d by residual."""
    best = None
    dt = np.median(np.diff(t))
    dv = np.gradient(v, t, axis=0)
    for k in range(int(max_delay / dt) + 1):
        e = vc[:len(vc) - k] - v[k:]
        g = dv[k:]
        den = float(np.sum(e * g))
        if den <= 0:
            continue
        tau = float(np.sum(e * e)) / den
        res = float(np.sum((g - e / tau) ** 2)) / max(float(np.sum(g * g)), 1e-9)
        if best is None or res < best[2]:
            best = (tau, k * dt, res)
    return best


def main():
    for run in sys.argv[1:]:
        f = os.path.join(run, 'noclimb_ca.csv')
        if not os.path.exists(f):
            print(f'== {run}: no noclimb_ca.csv')
            continue
        a = load(f)
        print(f'== {os.path.basename(run.rstrip("/"))}')
        if a is None:
            print('  empty log')
            continue
        ca = a[a['mode'] == 'CA']
        fin = a[a['mode'] == 'FINAL']
        if not ca.size:
            print('  CA never engaged')
            continue
        t = ca['t']
        P = np.stack([ca['x'], ca['y'], ca['z']], 1)
        V = np.stack([ca['vx'], ca['vy']], 1)
        vh = np.linalg.norm(V, axis=1)
        wp = np.stack([ca['wp_x'], ca['wp_y']], 1)
        u = wp - P[:, :2]
        u /= np.maximum(np.linalg.norm(u, axis=1, keepdims=True), 1e-6)
        vc = u * ca['speed_cmd'][:, None]
        gaps = np.diff(t)
        print(f'  CA {t[-1] - t[0]:5.1f}s ({len(t)} steps, dt med {1e3 * np.median(gaps):.0f} ms, '
              f'max {1e3 * gaps.max():.0f} ms)  final approach {fin["t"][-1] - fin["t"][0] if fin.size > 1 else 0:.1f}s')
        print(f'  speed cmd mean {ca["speed_cmd"].mean():.2f} (policy {ca["speed_policy"].mean():.2f})  '
              f'flown mean {vh.mean():.2f} max {vh.max():.2f}  '
              f'governor-limited {100 * np.mean(ca["speed_policy"] < 4.99):.0f}%  a2<0.9 {100 * np.mean(ca["a2"] < 0.9):.0f}%')
        fit = fit_tau(t, V, vc)
        if fit:
            print(f'  follower fit: tau {fit[0]:.2f}s  dead time {fit[1]:.2f}s  (residual {fit[2]:.2f})')
        print(f'  stall_t max {ca["stall_t"].max():.1f}s  >2s {100 * np.mean(ca["stall_t"] > 2):.0f}%  '
              f'>=4s {100 * np.mean(ca["stall_t"] >= 4):.0f}%')
        print(f'  alt vs path: vdev mean {ca["vdev"].mean():+.2f} min {ca["vdev"].min():+.2f} '
              f'max {ca["vdev"].max():+.2f}  wp_z {ca["wp_z"].min():.2f}..{ca["wp_z"].max():.2f}')
        print(f'  xte |max| {np.abs(ca["cross"]).max():.2f}  |a0|>0.3 {100 * np.mean(np.abs(ca["a0"]) > 0.3):.0f}%  '
              f'a0 sign flips/s {np.sum(np.diff(np.sign(ca["a0"])) != 0) / max(t[-1] - t[0], 1e-3):.2f}')
        print(f'  d_img_min min {ca["d_img_min"].min():.2f} p5 {np.percentile(ca["d_img_min"], 5):.2f}  '
              f'density mean {ca["density"].mean():.2f} max {ca["density"].max():.2f}  n_pts med {np.median(ca["n_pts"]):.0f}')
        last = a[-1]
        print(f'  last row: mode {last["mode"]} d_goal {last["d_goal"]:.2f} m')


if __name__ == '__main__':
    main()
