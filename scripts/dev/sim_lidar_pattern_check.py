#!/usr/bin/env python3
"""Does the simulator lidar put out what the recorded Mid-360 pattern predicts? (for the paper)

Run with Unreal up on the test map with the pattern lidar and NO Gazebo (the vehicle pose is set through
the AirSim API):   $SCENARIO_DIR/run_unreal.sh settings/settings_gazebo_mid360.json
Nothing in the Unreal folder changes: the pole is rescaled in the running game only and put back at the end.

  sim_lidar_pattern_check.py [--pole Pole_C1] [--dist 3 5 7 10 12 15] [--diam 0.02 0.03 0.1]
                             [--sensor-height 1.0] [--seconds 4] [--out OUT.json]

Part A - pattern: with the vehicle at rest, every returned point of every frame must lie on the pattern
  shot of the same index (order and direction), the pattern phase must advance with the frame stamps at the
  file's rate, and the shots per second must be the file's.
Part B - thin pole: for each diameter and distance, the returns the simulator labels as the pole, per frame,
  against the count predicted for the very same shots from geometry alone (a shot hits if its direction
  passes within the pole's radius between the ground and the pole's top - what lidar_pole_predict.py
  computes for the paper's table).
"""
import argparse
import json
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'ros2', 'tools'))
sys.path.insert(0, HERE)
from airsim_rpc import Rpc  # noqa: E402
from lidar_pole_predict import DEFAULT_PATTERN, load_pattern  # noqa: E402

LIDAR, VEHICLE = 'RPLIDAR_A3', ''


def quat_R(q):
    w, x, y, z = q['w_val'], q['x_val'], q['y_val'], q['z_val']
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def grab(rpc, seconds, settle=0.6):
    """Frames (stamp [s], points n x 3, labels, sensor pose) read for `seconds` after `settle`."""
    time.sleep(settle)
    frames, last, t_end = [], None, time.time() + seconds
    while time.time() < t_end:
        d = rpc.call('getLidarData', LIDAR, VEHICLE)
        ts = d['time_stamp']
        if ts != last and len(d['point_cloud']) >= 3:
            last = ts
            p = np.array(d['point_cloud'], dtype=np.float64).reshape(-1, 3)
            gt = d.get('groundtruth') or []
            if len(gt) == len(p):
                frames.append((ts * 1e-9, p, np.array(gt), d['pose']))
    return frames[1:]            # the first one may predate the pose or scale change


def frame_start(p, dirs):
    """Pattern index of the frame's first shot: the candidate (from the first hits) that fits all hits best."""
    hits = np.flatnonzero(~np.isnan(p[:, 0]))
    if len(hits) == 0:
        return None, None
    u = p[hits] / np.linalg.norm(p[hits], axis=1, keepdims=True)
    N = len(dirs)
    best = (np.inf, None, None)
    for j in hits[:3]:
        d = p[j] / np.linalg.norm(p[j])
        for k in np.argsort(dirs @ d)[-4:]:
            s = int((k - j) % N)
            err = np.degrees(np.arccos(np.clip(np.sum(u * dirs[(s + hits) % N], axis=1), -1.0, 1.0)))
            if err.mean() < best[0]:
                best = (err.mean(), s, err)
    return best[1], best[2]


def predict_hits(dirs_world, sensor, pole_xy, radius, top, ground_z=0.0):
    """Shots whose (zero-width) ray passes within `radius` of the pole axis between the ground and its top.
    World = NED: z down, the ground at ground_z."""
    c = np.asarray(pole_xy) - sensor[:2]
    h = np.hypot(dirs_world[:, 0], dirs_world[:, 1])
    ux, uy = dirs_world[:, 0] / np.maximum(h, 1e-12), dirs_world[:, 1] / np.maximum(h, 1e-12)
    along = c[0] * ux + c[1] * uy
    perp = np.abs(c[0] * uy - c[1] * ux)
    rng = along / np.maximum(h, 1e-12)
    z = sensor[2] + rng * dirs_world[:, 2]                 # NED z at the closest approach
    height = ground_z - z
    return (along > 0) & (perp <= radius) & (height >= 0.0) & (height <= top)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--pattern', default=DEFAULT_PATTERN)
    ap.add_argument('--pole', default='Pole_C1')
    ap.add_argument('--dist', type=float, nargs='+', default=[3, 5, 7, 10, 12, 15])
    ap.add_argument('--diam', type=float, nargs='+', default=[0.02, 0.03, 0.1])
    ap.add_argument('--sensor-height', type=float, default=1.0, help='lidar above the ground [m]')
    ap.add_argument('--sensor-up', type=float, default=0.3, help='lidar above the vehicle origin (settings Z -0.3) [m]')
    ap.add_argument('--seconds', type=float, default=4.0)
    ap.add_argument('--out', default=None)
    a = ap.parse_args()

    az, el, pps = load_pattern(a.pattern)
    ar, er = np.radians(az), np.radians(el)
    # Livox (x fwd, y left, z up) -> AirSim sensor frame (x fwd, y right, z down)
    dirs = np.stack([np.cos(er) * np.cos(ar), -np.cos(er) * np.sin(ar), -np.sin(er)], axis=1)
    N = len(dirs)

    rpc = Rpc(a.host, timeout=30.0)
    print('AirSim:', rpc.call('ping'))
    pp = rpc.call('simGetObjectPose', a.pole, False)['position']
    scale0 = rpc.call('simGetObjectScale', a.pole)
    pole_xy = np.array([pp['x_val'], pp['y_val']])
    top = float(scale0['z_val'])                 # the map's actors: unit cylinder, scale = size [m]
    print(f'{a.pole}: at x {pole_xy[0]:.2f} y {pole_xy[1]:.2f} (z {pp["z_val"]:.2f}), scale '
          f'{scale0["x_val"]:.3f} {scale0["y_val"]:.3f} {scale0["z_val"]:.3f}')

    def place(dist):
        pose = {'position': {'x_val': float(pole_xy[0] - dist), 'y_val': float(pole_xy[1]),
                             'z_val': float(-(a.sensor_height - a.sensor_up))},
                'orientation': {'w_val': 1.0, 'x_val': 0.0, 'y_val': 0.0, 'z_val': 0.0}}
        rpc.call('simSetVehiclePose', pose, True, VEHICLE)

    res = dict(pattern=os.path.basename(a.pattern), pattern_shots=N, pps=pps, pole=a.pole, pole_top=top,
               sensor_height=a.sensor_height, A={}, B=[])
    try:
        # ---------------- Part A: the pattern itself ----------------
        place(7.0)
        fr = grab(rpc, max(a.seconds, 6.0), settle=1.5)
        n = np.array([len(p) for _, p, _, _ in fr])
        ts = np.array([t for t, _, _, _ in fr])
        starts, errs, nan = [], [], []
        for _, p, _, _ in fr:
            s, e = frame_start(p, dirs)
            starts.append(s)
            errs.append(e)
            nan.append(np.isnan(p[:, 0]).mean())
        err = np.concatenate(errs)
        # phase: the last shot of a frame (index s + n - 1) left at the frame's stamp
        last = (np.array(starts) + n - 1).astype(np.int64)
        adv = np.diff(last) % N
        exp = np.round(np.diff(ts) * pps).astype(np.int64) % N
        dphi = (adv - exp + N // 2) % N - N // 2
        consecutive = np.array([(starts[i] + n[i]) % N == starts[i + 1] for i in range(len(fr) - 1)])
        sizes, counts = np.unique(n, return_counts=True)
        sensor = fr[0][3]['position']
        A = dict(frames=len(fr), shots_per_frame=dict(zip(map(int, sizes), map(int, counts))),
                 returned_points=int(len(err)), nan_share=float(np.mean(nan)),
                 dir_err_deg=dict(max=float(err.max()), p999=float(np.percentile(err, 99.9)), mean=float(err.mean())),
                 phase_err_shots=dict(max_abs=int(np.abs(dphi).max()), mean=float(dphi.mean())),
                 frames_read_back_to_back=int(consecutive.sum()),
                 sensor_pose=dict(x=sensor['x_val'], y=sensor['y_val'], z=sensor['z_val']))
        # shots per second over runs of back-to-back frames (a skipped frame is not a gap in the pattern)
        if consecutive.any():
            A['shots_per_s_back_to_back'] = float(np.sum(n[1:][consecutive]) / np.sum(np.diff(ts)[consecutive]))
        res['A'] = A
        print(f'\nPart A: {len(fr)} frames read ({int(consecutive.sum())} back to back), shots per frame {A["shots_per_frame"]}, '
              f'{len(err)} returned points, no-return shots {np.mean(nan) * 100:.1f} %')
        print(f'  direction of every returned point vs its pattern shot: max {err.max():.5f} deg, '
              f'99.9th percentile {np.percentile(err, 99.9):.5f} deg')
        print(f'  pattern phase vs frame stamps (expected advance = stamp difference x {pps:.0f}/s): '
              f'largest deviation {np.abs(dphi).max()} shots')
        if 'shots_per_s_back_to_back' in A:
            print(f'  shots per second over back-to-back frames: {A["shots_per_s_back_to_back"]:.0f}')
        print(f'  sensor in the vehicle frame: x {sensor["x_val"]:.2f} y {sensor["y_val"]:.2f} z {sensor["z_val"]:.2f} (NED)')

        # ---------------- Part B: thin pole ----------------
        print('\nPart B: returns from the pole, simulator vs the count predicted for the same shots')
        print('  diam [m] | dist [m] | frames | per 20 000 shots: sim / predicted | frames with >= 1: sim / predicted | '
              'frames with equal count')
        for diam in a.diam:
            rpc.call('simSetObjectScale', a.pole, {'x_val': diam, 'y_val': diam, 'z_val': scale0['z_val']})
            for dist in a.dist:
                place(dist)
                fr = grab(rpc, a.seconds)
                vp = rpc.call('simGetVehiclePose', VEHICLE)
                sim, pre, nn = [], [], []
                for _, p, gt, pose in fr:
                    s, _ = frame_start(p, dirs)
                    if s is None:
                        continue
                    idx = (s + np.arange(len(p))) % N
                    # the frame's pose is the sensor in the vehicle frame: put it into the world with the vehicle pose
                    Rv = quat_R(vp['orientation'])
                    sp = pose['position']
                    sensor_w = np.array([vp['position']['x_val'], vp['position']['y_val'], vp['position']['z_val']]) \
                        + Rv @ np.array([sp['x_val'], sp['y_val'], sp['z_val']])
                    dw = dirs[idx] @ (Rv @ quat_R(pose['orientation'])).T
                    hit = predict_hits(dw, sensor_w, pole_xy, diam / 2, top)
                    sim.append(int(np.sum(gt == a.pole)))
                    pre.append(int(hit.sum()))
                    nn.append(len(p))
                sim, pre, nn = np.array(sim), np.array(pre), np.array(nn)
                row = dict(diam=diam, dist=dist, frames=int(len(sim)), shots=int(nn.sum()),
                           sim_per_20k=float(sim.sum() / nn.sum() * 20000), pred_per_20k=float(pre.sum() / nn.sum() * 20000),
                           sim_seen=float(np.mean(sim > 0)), pred_seen=float(np.mean(pre > 0)),
                           equal_frames=float(np.mean(sim == pre)), sim_hits=int(sim.sum()), pred_hits=int(pre.sum()))
                res['B'].append(row)
                print(f'  {diam:7.3f} | {dist:7.1f} | {len(sim):6d} | {row["sim_per_20k"]:6.2f} / {row["pred_per_20k"]:6.2f} | '
                      f'{row["sim_seen"] * 100:5.1f} % / {row["pred_seen"] * 100:5.1f} % | {row["equal_frames"] * 100:5.1f} %')
    finally:
        rpc.call('simSetObjectScale', a.pole, scale0)
        s = rpc.call('simGetObjectScale', a.pole)
        print(f'\n{a.pole} scale put back: {s["x_val"]:.3f} {s["y_val"]:.3f} {s["z_val"]:.3f}')
    if a.out:
        json.dump(res, open(a.out, 'w'), indent=1)
        print('saved', a.out)


if __name__ == '__main__':
    main()
