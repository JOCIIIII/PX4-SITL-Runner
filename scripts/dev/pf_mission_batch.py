#!/usr/bin/env python3
"""The vehicle's PathFollowing-only missions (scripts/ros2/apps/mission.sh: flight_driver + PathFollowing MPPI with
the corner limit) on the Gazebo-only SITL stack (fly_test.sh, FLY_PROFILE gazebo-classic-sitl), every shape x speed
(x wind) once, paths from scripts/dev/pf_shapes.py. One CSV row per run; envs/gazebo-classic.env and envs/ros2.env
are restored at the end (also on Ctrl+C).

usage: pf_mission_batch.py PREFIX [--shapes line square circle spiral] [--speeds 1 3 5 7 10]
                           [--wind 0] [--wind-dir "0 -1 0"] [--airframe x8] [--alt 5] [--env "KEY=VAL ..."]
  --wind      steady wind [m/s] for every run (Gazebo gust from t = 0, as pf_wind_batch.py); with the lines heading
              north (pf_shapes.py --heading 0), --wind-dir "0 -1 0" (blowing south) is a headwind, "0 1 0" a
              tailwind, "1 0 0" a crosswind
  --env       more mission.sh knobs (FLY_ENV), e.g. "MISSION_CORNER=0"
Per run: completed (flight_driver "path complete"), the path time (FOLLOW -> LANDING), cross-track error to the
polyline (mean / p95 / max), height error, speed on the path (mean, p95 over the middle 60 % of the path) and the
flight_driver abort line if any.
"""
import argparse
import math
import os
import re
import subprocess
import sys
import time

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
RUNS = os.path.expanduser('~/Documents/A4VAI-SITL/ROS2/test_runs')
COLS = ['shape', 'speed', 'wind', 'wind_dir', 'run', 'result', 'completed', 'path_time_s', 'path_len_m',
        'xte_mean', 'xte_p95', 'xte_max', 'alt_err_mean', 'alt_err_max', 'v_mean', 'v_mid_p50', 'v_max', 'abort']


def set_env(text, key, value):
    return re.sub(rf'^{key}=.*$', f'{key}={value}', text, flags=re.M)


def seg_dist(P, a, b):
    d = b - a
    L2 = float(d @ d)
    t = np.clip(((P - a) @ d) / L2, 0.0, 1.0) if L2 > 0 else np.zeros(len(P))
    return np.linalg.norm(P - (a + t[:, None] * d), axis=1)


def analyze(run, wp_csv, alt):
    res = {'completed': False}
    fd = os.path.join(run, 'flight_driver.log')
    t_follow = t_end = None
    abort = ''
    if os.path.exists(fd):
        for line in open(fd, errors='ignore'):
            m = re.search(r'\[(\d+\.\d+)\] \[flight_driver\]: (.*)', line)
            if not m:
                continue
            t, msg = float(m.group(1)), m.group(2)
            if '-> FOLLOW' in msg and t_follow is None:
                t_follow = t
            if 'FOLLOW -> LANDING' in msg and t_end is None:
                t_end = t
                res['completed'] = 'path complete' in msg
            if 'ABORT' in msg and not abort:
                abort = msg[:120].replace(',', ';')
    res['abort'] = abort
    px_path = os.path.join(run, 'px4.csv')
    if t_follow is None or not os.path.exists(px_path):
        res['note'] = 'no path phase'
        return res
    px = np.genfromtxt(px_path, delimiter=',', names=True)
    t1 = t_end if t_end else px['t'][-1]
    m = (px['t'] >= t_follow) & (px['t'] <= t1)
    P = np.column_stack([px['x'][m], px['y'][m]])
    if len(P) < 10:
        res['note'] = 'no path phase'
        return res
    wp = np.genfromtxt(wp_csv, delimiter=',', names=True, comments='#')
    W = np.column_stack([wp['x'], wp['y']])
    if len(W) == 1:
        W = np.vstack([W, W])
    D = np.min(np.vstack([seg_dist(P, W[i], W[i + 1]) for i in range(len(W) - 1)]), axis=0)
    zerr = np.abs(-px['z'][m] - alt)
    v = np.hypot(px['vx'][m], px['vy'][m])
    n = len(v)
    mid = v[int(0.2 * n):int(0.8 * n)] if n > 10 else v
    res.update(path_time_s=float(t1 - t_follow), path_len_m=float(np.sum(np.hypot(*np.diff(W, axis=0).T))),
               xte_mean=float(D.mean()), xte_p95=float(np.percentile(D, 95)), xte_max=float(D.max()),
               alt_err_mean=float(zerr.mean()), alt_err_max=float(zerr.max()),
               v_mean=float(v.mean()), v_mid_p50=float(np.median(mid)), v_max=float(v.max()))
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('prefix')
    ap.add_argument('--shapes', nargs='+', default=['line', 'square', 'circle', 'spiral'])
    ap.add_argument('--speeds', nargs='+', type=float, default=[1, 3, 5, 7, 10])
    ap.add_argument('--wind', type=float, default=0.0)
    ap.add_argument('--wind-dir', default='0 -1 0')
    ap.add_argument('--airframe', default='x8')
    ap.add_argument('--alt', type=float, default=5.0)
    ap.add_argument('--env', default='')
    a = ap.parse_args()

    gz_env, ros2_env = os.path.join(REPO, 'envs/gazebo-classic.env'), os.path.join(REPO, 'envs/ros2.env')
    backup = {p: open(p).read() for p in (gz_env, ros2_env)}
    csv_path = os.path.join(RUNS, f'{a.prefix}_summary.csv')
    new = not os.path.exists(csv_path)
    out = open(csv_path, 'a')
    if new:
        out.write(','.join(COLS) + '\n')
    fails_in_row = 0
    try:
        gz = backup[gz_env]
        gz = set_env(gz, 'SITL_AIRFRAME', f'"{a.airframe}"')
        gz = set_env(gz, 'SITL_WORLD', '"windy"')
        for k, v in (('windVelocityMean', '0.0'), ('windVelocityMax', '0.0'), ('windVelocityVariance', '0'),
                     ('windDirectionMean', f'"{a.wind_dir}"'), ('windDirectionVariance', '0'),
                     ('windGustStart', '0'), ('windGustDuration', '3600' if a.wind > 0 else '0'),
                     ('windGustVelocityMean', f'{a.wind:.1f}'), ('windGustVelocityMax', f'{a.wind + 3.0:.1f}'),
                     ('windGustVelocityVariance', '0'), ('windGustDirectionMean', f'"{a.wind_dir}"'),
                     ('windGustDirectionVariance', '0')):
            gz = set_env(gz, k, v)
        open(gz_env, 'w').write(gz)
        for shape in a.shapes:
            for v in a.speeds:
                name = f'{a.prefix}_{shape}_v{v:g}' + (f'_w{a.wind:g}' if a.wind > 0 else '')
                wp_file = os.path.join(RUNS, f'{name}_wp.csv')
                subprocess.run([sys.executable, os.path.join(REPO, 'scripts/dev/pf_shapes.py'), shape, f'{v:g}',
                                '--alt', f'{a.alt:g}', '--out', wp_file], check=True)
                wp = np.genfromtxt(wp_file, delimiter=',', names=True, comments='#')
                wp_len = float(np.sum(np.hypot(np.diff(wp['x']), np.diff(wp['y'])))) if wp.size > 1 else 0.0
                timeout = int(max(200, 100 + 1.6 * wp_len / max(v, 0.5) + 60))
                env = dict(os.environ, FLY_PROFILE='gazebo-classic-sitl',
                           FLY_ENV=f'MISSION_CSV=/home/user/workspace/ros2/test_runs/{os.path.basename(wp_file)} '
                                   f'MISSION_TYPE=wp_local MISSION_SPEED={v:g} MISSION_ALT={a.alt:g} MISSION_EKF_SETTLE=5'
                                   + (f' {a.env}' if a.env else ''))
                t0 = time.time()
                log = os.path.join(RUNS, f'{name}.batch.log')
                with open(log, 'w') as fh:
                    subprocess.run(['timeout', str(timeout + 240), os.path.join(REPO, 'scripts/dev/fly_test.sh'), name,
                                    '0', '0', f'{a.alt:g}', str(timeout), 'mission'], env=env, stdout=fh,
                                   stderr=subprocess.STDOUT)
                run_dir = os.path.join(RUNS, name)
                res_file = os.path.join(run_dir, 'result.txt')
                result = open(res_file).read().strip() if os.path.exists(res_file) else 'no_result'
                if os.path.isdir(run_dir):
                    os.replace(log, os.path.join(run_dir, 'batch.log'))
                    os.replace(wp_file, os.path.join(run_dir, 'wp.csv'))
                    wp_file = os.path.join(run_dir, 'wp.csv')
                try:
                    m = analyze(run_dir, wp_file, a.alt)
                except Exception as e:  # noqa: BLE001
                    m = {'note': f'analysis failed: {e}'}
                m.update(shape=shape, speed=v, wind=a.wind, wind_dir=a.wind_dir.replace(' ', '_'), run=name,
                         result=result.replace(',', ';'))
                out.write(','.join(f'{m[c]:.3f}' if isinstance(m.get(c), float) else str(m.get(c, '')) for c in COLS)
                          + '\n')
                out.flush()
                ok = result.startswith('landed') and m.get('completed')
                fails_in_row = 0 if result.startswith('landed') else fails_in_row + 1
                nan = float('nan')
                print(f'RUN {shape} {v:g} m/s wind {a.wind:g}: {result}, completed {m.get("completed")} in '
                      f'{m.get("path_time_s", nan):.0f} s | xte mean {m.get("xte_mean", nan):.2f} p95 '
                      f'{m.get("xte_p95", nan):.2f} max {m.get("xte_max", nan):.2f} m | alt err max '
                      f'{m.get("alt_err_max", nan):.2f} m | v mid {m.get("v_mid_p50", nan):.1f} max {m.get("v_max", nan):.1f}'
                      f' | {m.get("abort", "")} | {time.time() - t0:.0f} s{"" if ok else "  <-- CHECK"}', flush=True)
                if fails_in_row >= 3:
                    print('STOPPED: 3 runs in a row did not land (stack broken?)', flush=True)
                    return
    finally:
        for p, text in backup.items():
            open(p, 'w').write(text)
        out.close()
        print('BATCH DONE', flush=True)


if __name__ == '__main__':
    main()
