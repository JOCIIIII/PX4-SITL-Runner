#!/usr/bin/env python3
"""PathFollowing (MPPI) alone in steady wind: every wind x speed pair once, on the Gazebo-only
stack (fly_test.sh, FLY_PROFILE gazebo-classic-sitl, ROS2_APP pf_test) with a waypoint CSV, then
scripts/dev/pf_wind_metrics.py on each run. One CSV row per run; envs/gazebo-classic.env and
envs/ros2.env are restored at the end (also on Ctrl+C).

usage: pf_wind_batch.py PREFIX [--winds 2 4 6 8 10] [--speeds 1 3 5 7 10] [--airframe x8]
                        [--wind-dir "0 1 0"] [--wp ~/Documents/A4VAI-SITL/ROS2/test_runs/wp_wind_square.csv]
                        [--sim-yaml sim_ca_mppi_x8.yaml] [--wind-start-s 0]
  wind direction = Gazebo windDirectionMean (ENU, the way it blows: "0 1 0" = toward north)
"""
import argparse
import math
import os
import re
import subprocess
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
RUNS = os.path.expanduser('~/Documents/A4VAI-SITL/ROS2/test_runs')
sys.path.insert(0, os.path.join(REPO, 'scripts', 'dev'))
import pf_wind_metrics  # noqa: E402

COLS = ['wind', 'speed', 'run', 'result', 'completed', 'path_time_s', 'xte_mean', 'xte_p95', 'xte_max',
        'alt_err_mean', 'alt_err_max', 'v_head', 'v_tail', 'v_cross', 'tilt_max', 'tilt_p95', 'tilt_ge34_pct',
        'thrust_max', 'thrust_sat_pct']


def set_env(text, key, value):
    return re.sub(rf'^{key}=.*$', f'{key}={value}', text, flags=re.M)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('prefix')
    ap.add_argument('--winds', nargs='+', type=float, default=[2, 4, 6, 8, 10])
    ap.add_argument('--speeds', nargs='+', type=float, default=[1, 3, 5, 7, 10])
    ap.add_argument('--airframe', default='x8')
    ap.add_argument('--wind-dir', default='0 1 0')
    ap.add_argument('--wp', default=os.path.join(RUNS, 'wp_wind_square.csv'))
    ap.add_argument('--sim-yaml', default='sim_ca_mppi_x8.yaml', help="PF/MPPI config ('' = pf_test's sim.yaml)")
    ap.add_argument('--wind-start-s', type=float, default=0.0, help='Gazebo sim time the wind starts [s]')
    ap.add_argument('--env', default='', help="more pf_test settings for FLY_ENV, e.g. 'PF_TEST_CORNER=1 "
                    "PF_TEST_PF_ARGS=corner_max_error_m:=1.0' (KEY=VAL, space-separated, no spaces in values)")
    a = ap.parse_args()

    gz_env, ros2_env = os.path.join(REPO, 'envs/gazebo-classic.env'), os.path.join(REPO, 'envs/ros2.env')
    backup = {p: open(p).read() for p in (gz_env, ros2_env)}
    wx, wy = (float(c) for c in a.wind_dir.split()[:2])
    wind_dir_deg = math.degrees(math.atan2(wx, wy))              # compass: ENU (x E, y N) -> deg from north
    wp_len = 0.0
    import numpy as np
    wp = np.genfromtxt(a.wp, delimiter=',', names=True)
    wp_len = float(np.sum(np.hypot(np.diff(wp['x']), np.diff(wp['y']))))
    wp_in_container = '/home/user/workspace/ros2/test_runs/' + os.path.basename(a.wp)
    csv_path = os.path.join(RUNS, f'{a.prefix}_summary.csv')
    new = not os.path.exists(csv_path)
    out = open(csv_path, 'a')
    if new:
        out.write(','.join(COLS) + '\n')
    fails_in_row = 0
    try:
        for w in a.winds:
            gz = backup[gz_env]
            gz = set_env(gz, 'SITL_AIRFRAME', f'"{a.airframe}"')
            # the wind as the plugin's 'gust' from wind_start_s (sim time) on - 0 (default): steady from the start.
            # (The x8_mppi_check* runs that never left the ground were not the wind: the x8 spawned underground
            # and stuck in windy.world's asphalt box - GAZEBO_POSE -z, scripts/include/commonEnv.sh. With that
            # fixed it took off and held 2.5 m in 10 m/s from the start: 6.5 deg into the wind, thrust 0.46.)
            gz = set_env(gz, 'windVelocityMean', '0.0')
            gz = set_env(gz, 'windVelocityMax', '0.0')
            gz = set_env(gz, 'windVelocityVariance', '0')
            gz = set_env(gz, 'windDirectionMean', f'"{a.wind_dir}"')
            gz = set_env(gz, 'windDirectionVariance', '0')
            gz = set_env(gz, 'windGustStart', f'{a.wind_start_s:g}')
            gz = set_env(gz, 'windGustDuration', '3600')
            gz = set_env(gz, 'windGustVelocityMean', f'{w:.1f}')
            gz = set_env(gz, 'windGustVelocityMax', f'{w + 3.0:.1f}')
            gz = set_env(gz, 'windGustVelocityVariance', '0')
            gz = set_env(gz, 'windGustDirectionMean', f'"{a.wind_dir}"')
            gz = set_env(gz, 'windGustDirectionVariance', '0')
            open(gz_env, 'w').write(gz)
            for v in a.speeds:
                name = f'{a.prefix}_w{w:g}_v{v:g}'
                timeout = int(max(180, 70 + 1.4 * wp_len / max(v, 0.5) + 40))
                env = dict(os.environ, FLY_PROFILE='gazebo-classic-sitl',
                           FLY_ENV=f'PF_TEST_WP_CSV={wp_in_container} PF_TEST_SPEED={v:g}'
                           + (f' PF_TEST_SIM_YAML={a.sim_yaml}' if a.sim_yaml else '')
                           + (f' {a.env}' if a.env else ''))
                t0 = time.time()
                log = os.path.join(RUNS, f'{name}.batch.log')
                with open(log, 'w') as fh:
                    subprocess.run(['timeout', str(timeout + 240), os.path.join(REPO, 'scripts/dev/fly_test.sh'), name,
                                    '0', '0', '5', str(timeout), 'pf_test'], env=env, stdout=fh, stderr=subprocess.STDOUT)
                res_file = os.path.join(RUNS, name, 'result.txt')
                result = open(res_file).read().strip() if os.path.exists(res_file) else 'no_result'
                if os.path.isdir(os.path.join(RUNS, name)):
                    os.replace(log, os.path.join(RUNS, name, 'batch.log'))
                try:
                    m = pf_wind_metrics.analyze(os.path.join(RUNS, name), a.wp, wind_dir_deg, v)
                except Exception as e:  # noqa: BLE001
                    m = {'note': f'analysis failed: {e}'}
                m.update(wind=w, speed=v, run=name, result=result.replace(',', ';'))
                out.write(','.join(f'{m[c]:.3f}' if isinstance(m.get(c), float) else str(m.get(c, '')) for c in COLS) + '\n')
                out.flush()
                ok = result.startswith('landed') and m.get('completed')
                fails_in_row = 0 if result.startswith('landed') else fails_in_row + 1
                print(f'RUN wind {w:g} speed {v:g}: {result}, completed {m.get("completed")} in {m.get("path_time_s", float("nan")):.0f} s'
                      f' | xte mean {m.get("xte_mean", float("nan")):.2f} max {m.get("xte_max", float("nan")):.2f} m'
                      f' | alt err max {m.get("alt_err_max", float("nan")):.2f} m'
                      f' | v head/tail/cross {m.get("v_head", float("nan")):.1f}/{m.get("v_tail", float("nan")):.1f}/{m.get("v_cross", float("nan")):.1f}'
                      f' | tilt max {m.get("tilt_max", float("nan")):.0f} deg, thrust sat {m.get("thrust_sat_pct", float("nan")):.1f}%'
                      f' | {time.time() - t0:.0f} s{"" if ok else "  <-- CHECK"}', flush=True)
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
