#!/usr/bin/env python3
"""Run a Monte Carlo plan (mc_gen.py) on the Gazebo-only SITL stack, one run after another: the vehicle's
PathFollowing mission app (flight_driver + PathFollowing MPPI with the corner limit, as on the vehicle) flies the course
from scripts/dev/pf_shapes.py in the run's wind, then mc_eval.py judges it. Resumable: runs that already have an
mc_result.json (not a simulator fault) are skipped. envs/gazebo-classic.env and envs/ros2.env are restored and the wind
file is removed at the end (also on Ctrl+C).

usage: mc_run.py PLAN_CSV PREFIX [--runs 0 5 9 | --first 0 --last 299] [--alt 5] [--tilt 30] [--airspeed-drag 1]
                 [--retries 2] [--stop-after 5] [--dry-run]
  run directories: ~/Documents/A4VAI-SITL/ROS2/test_runs/PREFIX_NNN_COURSE (mc_params.json, wp.csv, zone_wind.txt,
  the flight logs, px4_sitl.ulg, mc_result.json); summary: test_runs/PREFIX_summary.csv

Wind: the vehicle plugin scripts/px4/plugins/gazebo_zone_wind_plugin.cpp with one zone over the whole world (rotor
drag through PX4's motor model + body drag 0.5 rho CdA |w - v| (w - v) with --airspeed-drag 1, Ornstein-Uhlenbeck
gusts, the run's seed). The Gazebo container reads it from ZONE_WIND_FILE (envs/gazebo-classic.env, set here only);
Gazebo's own world wind stays 0. Direction: the wind blows TOWARD the compass bearing wind_toward_deg (Gazebo ENU
x = east, y = north).
"""
import argparse
import csv
import json
import math
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))
sys.path.insert(0, HERE)
from mc_eval import evaluate  # noqa: E402

RUNS = os.path.expanduser('~/Documents/A4VAI-SITL/ROS2/test_runs')
WIND_HOST = os.path.expanduser('~/Documents/A4VAI-SITL/PX4-Autopilot/mc_zone_wind.txt')
WIND_CONT = '/home/user/workspace/px4/mc_zone_wind.txt'
COLS = ['run', 'course', 'speed', 'wind', 'wind_toward_deg', 'gust_std', 'gust_tau', 'cda', 'size', 'seed', 'result',
        'success', 'fail', 'e95', 'e_rms', 'e_max', 'e95_true', 'e_max_true', 'h_min', 'land_err_m', 'follow_s', 'path_len_m',
        'aborts', 'sim_fault', 'wall_s']


def set_env(text, key, value):
    if re.search(rf'^{key}=', text, flags=re.M):
        return re.sub(rf'^{key}=.*$', f'{key}={value}', text, flags=re.M)
    return text.rstrip('\n') + f'\n{key}={value}\n'


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('plan')
    ap.add_argument('prefix')
    ap.add_argument('--runs', nargs='+', type=int, default=None)
    ap.add_argument('--first', type=int, default=None)
    ap.add_argument('--last', type=int, default=None)
    ap.add_argument('--alt', type=float, default=5.0)
    ap.add_argument('--tilt', type=float, default=30.0)
    ap.add_argument('--airspeed-drag', type=int, default=1)
    ap.add_argument('--retries', type=int, default=2)
    ap.add_argument('--stop-after', type=int, default=5, help='stop after this many simulator faults in a row')
    ap.add_argument('--dry-run', action='store_true')
    a = ap.parse_args()
    plan = list(csv.DictReader(open(a.plan)))
    if a.runs is not None:
        plan = [r for r in plan if int(r['run']) in a.runs]
    if a.first is not None:
        plan = [r for r in plan if int(r['run']) >= a.first]
    if a.last is not None:
        plan = [r for r in plan if int(r['run']) <= a.last]
    gz_env, ros2_env = os.path.join(REPO, 'envs/gazebo-classic.env'), os.path.join(REPO, 'envs/ros2.env')
    backup = {p: open(p).read() for p in (gz_env, ros2_env)}
    csv_path = os.path.join(RUNS, f'{a.prefix}_summary.csv')
    new = not os.path.exists(csv_path)
    out = open(csv_path, 'a')
    if new:
        out.write(','.join(COLS) + '\n')
    faults_in_row = 0
    n_done = 0
    t_start = time.time()
    try:
        gz = backup[gz_env]
        for k, v in (('SITL_AIRFRAME', '"x8"'), ('SITL_WORLD', '"windy"'), ('windVelocityMean', '0.0'),
                     ('windVelocityMax', '0.0'), ('windGustDuration', '0'), ('windGustVelocityMean', '0'),
                     ('windGustVelocityMax', '0'), ('ZONE_WIND_FILE', f'"{WIND_CONT}"')):
            gz = set_env(gz, k, v)
        if not a.dry_run:
            open(gz_env, 'w').write(gz)
        for row in plan:
            idx, shape, v = int(row['run']), row['course'], float(row['speed'])
            name = f'{a.prefix}_{idx:03d}_{shape}'
            run_dir = os.path.join(RUNS, name)
            done = os.path.join(run_dir, 'mc_result.json')
            if os.path.exists(done) and not json.load(open(done)).get('sim_fault'):
                continue
            wp_file = os.path.join(RUNS, f'{name}_wp.csv')
            cmd = [sys.executable, os.path.join(REPO, 'scripts/dev/pf_shapes.py'), shape, f'{v:g}', '--alt', f'{a.alt:g}',
                   '--size', row['size'], '--out', wp_file]
            if row.get('corner'):
                cmd += ['--corner', row['corner']]
            if shape in ('circle', 'spiral'):
                cmd += ['--laps', row['laps']]
            subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL)
            n_wp = sum(1 for line in open(wp_file) if line[:1] not in ('#', 'x'))
            th = math.radians(float(row['wind_toward_deg']))
            U = float(row['wind'])
            wx, wy = U * math.sin(th), U * math.cos(th)                       # ENU: east, north
            wind_text = (f'# mc_run.py {name}: {U:.2f} m/s toward {row["wind_toward_deg"]} deg\n'
                         f'seed {row["seed"]}\n'
                         f'MC -3000 3000 -3000 3000 -50 1000 {wx:.3f} {wy:.3f} 0.0 {row["gust_std"]} {row["gust_tau"]} '
                         f'{row["cda"]} 10.0 {a.airspeed_drag}\n')
            import numpy as np
            wp = np.genfromtxt(wp_file, delimiter=',', names=True, comments='#')
            plen = float(np.sum(np.hypot(np.diff(wp['x']), np.diff(wp['y']))))
            timeout = int(160 + 2.5 * plen / max(v, 0.5))
            params = dict(row, alt=a.alt, tilt_deg=a.tilt, airspeed_drag=a.airspeed_drag, wind_enu=[wx, wy],
                          path_len_m=round(plen, 1), waypoints=n_wp, timeout_s=timeout)
            print(f'START {name}: {shape} {v:.2f} m/s, path {plen:.0f} m, wind {U:.2f} m/s toward '
                  f'{row["wind_toward_deg"]} deg, gust {row["gust_std"]}/{row["gust_tau"]} s, CdA {row["cda"]}, '
                  f'timeout {timeout} s', flush=True)
            if a.dry_run:
                os.remove(wp_file)
                continue
            for attempt in range(a.retries + 1):
                open(WIND_HOST, 'w').write(wind_text)
                env = dict(os.environ, FLY_PROFILE='gazebo-classic-sitl',
                           FLY_ENV=f'MISSION_CSV=/home/user/workspace/ros2/test_runs/{os.path.basename(wp_file)} '
                                   f'MISSION_TYPE=wp_local MISSION_SPEED={v:.2f} MISSION_ALT={a.alt:g} '
                                   f'MISSION_EKF_SETTLE=5 MISSION_MAX_TILT_DEG={a.tilt:g}')
                t0 = time.time()
                log = os.path.join(RUNS, f'{name}.batch.log')
                with open(log, 'w') as fh:
                    subprocess.run(['timeout', str(timeout + 300), os.path.join(REPO, 'scripts/dev/fly_test.sh'), name,
                                    '0', '0', f'{a.alt:g}', str(timeout), 'mission'], env=env, stdout=fh,
                                   stderr=subprocess.STDOUT)
                os.makedirs(run_dir, exist_ok=True)
                os.replace(log, os.path.join(run_dir, 'batch.log'))
                import shutil
                shutil.copy(wp_file, os.path.join(run_dir, 'wp.csv'))
                open(os.path.join(run_dir, 'zone_wind.txt'), 'w').write(wind_text)
                json.dump(params, open(os.path.join(run_dir, 'mc_params.json'), 'w'), indent=1)
                try:
                    res = evaluate(run_dir)
                except Exception as e:                                                # noqa: BLE001
                    res = {'run': name, 'sim_fault': True, 'success': False, 'note': f'evaluation failed: {e}'}
                res['wall_s'] = round(time.time() - t0, 1)
                res['attempt'] = attempt
                json.dump(res, open(done, 'w'), indent=1)
                if not res.get('sim_fault') or attempt == a.retries:
                    break
                print(f'  {name}: simulator fault ({res.get("note")}), retry {attempt + 1}', flush=True)
                os.rename(run_dir, f'{run_dir}_fault{attempt}')
            os.remove(wp_file)
            faults_in_row = faults_in_row + 1 if res.get('sim_fault') else 0
            row_out = dict(res, **{k: row[k] for k in ('run', 'course', 'speed', 'wind', 'wind_toward_deg', 'gust_std',
                                                       'gust_tau', 'cda', 'size', 'seed')})
            row_out['fail'] = ' '.join(res.get('fail', []))
            row_out['aborts'] = ' | '.join(res.get('aborts', [])).replace(',', ';')
            out.write(','.join(str(row_out.get(c, '')).replace(',', ';') for c in COLS) + '\n')
            out.flush()
            n_done += 1
            el = (time.time() - t_start) / n_done
            print(f'RUN {name}: {"OK  " if res.get("success") else "FAIL"} {res.get("result")} | e95 {res.get("e95")} '
                  f'max {res.get("e_max")} m (true e95 {res.get("e95_true")}) | landing {res.get("land_err_m")} m | '
                  f'{" ".join(res.get("fail", [])) or res.get("note", "")} | {res["wall_s"]:.0f} s, '
                  f'avg {el:.0f} s/run', flush=True)
            if faults_in_row >= a.stop_after:
                print(f'STOPPED: {faults_in_row} simulator faults in a row (stack broken?)', flush=True)
                return
    finally:
        if not a.dry_run:
            for p, text in backup.items():
                open(p, 'w').write(text)
        if os.path.exists(WIND_HOST):
            os.remove(WIND_HOST)
        out.close()
        print(f'BATCH DONE ({n_done} runs, {(time.time() - t_start) / 60:.0f} min)', flush=True)


if __name__ == '__main__':
    main()
