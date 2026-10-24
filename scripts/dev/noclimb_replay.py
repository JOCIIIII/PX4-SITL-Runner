#!/usr/bin/env python3
"""Counterfactual replay of a noclimb_ca flight (noclimb_ca_dump.npz from fly_test.sh).

For each recorded policy step the range image is rebuilt from the recorded scans with other
scan-memory settings and swapped into the recorded observation (density recomputed from the
same points); the policy then runs once. State, previous action and stall terms stay as
flown, so a change in the action is caused by how the image was built alone.

usage (ROS2 image, ROS + workspace sourced - imports noclimb_ca_node for count_obstacles):
  noclimb_replay.py DUMP.npz POLICY_DIR [--policy NAME] [--memory 0 1 2] [--x0 X --x1 X]
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'ros2', 'apps'))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('dump')
    ap.add_argument('policy_dir')
    ap.add_argument('--policy', default='ca_noclimb_tau05-12')
    ap.add_argument('--memory', type=float, nargs='+', default=[0.0, 1.0, 2.0])
    ap.add_argument('--x0', type=float, default=-1e9)
    ap.add_argument('--x1', type=float, default=1e9)
    ap.add_argument('--every', type=int, default=2)
    a = ap.parse_args()

    sys.path.insert(0, a.policy_dir)
    import onnxruntime as ort
    from ca_policy_ref import NoClimbPolicy, pointcloud_to_image
    from noclimb_ca_node import count_obstacles

    pol = NoClimbPolicy(os.path.join(a.policy_dir, a.policy + '.onnx'),
                        os.path.join(a.policy_dir, 'cfg.json'), [[0, 0], [1, 0]], [4, 4])
    c, d_max = pol.c, float(pol.c['d_sensor_max'])
    sess = ort.InferenceSession(os.path.join(a.policy_dir, a.policy + '.onnx'),
                                providers=['CPUExecutionProvider'])
    d = np.load(a.dump)
    scan_t, off = d['scan_t'], np.r_[0, np.cumsum(d['scan_n'])]
    xyz = d['scan_xyz'].astype(np.float64)
    print(f'flown with scan memory {float(d["memory_s"]):.1f} s; columns: logged a0 | a0 (dmin, density) '
          f'per memory {a.memory}')

    def image_at(k, mem):
        t = d['t'][k]
        last = int(np.searchsorted(scan_t, t, side='right')) - 1
        if last < 0:
            return np.full((c['lidar_n_el'], c['lidar_n_az']), d_max, np.float32), 0
        first = last if mem <= 0 else int(np.searchsorted(scan_t, t - mem, side='right'))
        pts = xyz[off[min(first, last)]:off[last + 1]]
        rel = pts - d['pos'][k]
        near = np.einsum('ij,ij->i', rel, rel) <= (d_max + 0.5) ** 2
        rel, zw = rel[near], pts[near, 2]
        img = pointcloud_to_image(rel, float(d['ref_bearing'][k]), c)
        return np.minimum(img, d_max), count_obstacles(rel, zw)

    flips = {m: 0 for m in a.memory}
    n = 0
    for k in range(0, len(d['t']), a.every):
        x, y = d['pos'][k, :2]
        if not a.x0 <= x <= a.x1:
            continue
        cols = []
        a0s = []
        for mem in a.memory:
            img, n_obj = image_at(k, mem)
            obs = d['obs'][k].copy()
            obs[15:207] = np.clip(img / d_max, 0, 1).reshape(-1)
            obs[207] = min(n_obj / 4.0, 1.0)
            act = sess.run(None, {'obs': obs[None].astype(np.float32)})[0][0]
            a0s.append(act[0])
            cols.append(f'{act[0]:+.2f} ({img.min():.2f},{obs[207]:.2f})')
        logged = float(d['action'][k, 0])
        for mem, a0 in zip(a.memory, a0s):
            flips[mem] += int(np.sign(a0) != np.sign(logged))
        n += 1
        print(f't+{d["t"][k] - d["t"][0]:5.1f} ({x:5.1f},{y:4.1f})  logged {logged:+.2f} | ' + '  '.join(cols))
    print('steps with a0 sign opposite to the flown one: ' +
          '  '.join(f'mem {m:.1f}s: {flips[m]}/{n}' for m in a.memory))


if __name__ == '__main__':
    main()
