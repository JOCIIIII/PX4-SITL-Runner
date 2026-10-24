#!/usr/bin/env python3
"""Fly the same configuration many times (scripts/dev/fly_test.sh) and tabulate the outcome.

Each run: fly_test.sh NAME GX GY ALT TIMEOUT APP with FLY_ENV, then analyze_flight.py for the
clearance to the trees (own map), goal error, thrust at its limit and the height reached. One CSV
row per run; a PROGRESS line every 10 runs and a FAIL line for every run that did not land or came
closer than --warn-clr. envs/ros2.env and envs/gazebo-classic.env are restored at the end.

usage: batch_fly.py PREFIX N --goals 100,0 100,15 [--alt 4] [--app noclimb_ca] [--airframe x8]
                    [--fly-env "K=V ..."] [--timeout 420] [--start 1] [--warn-clr 1.0]
"""
import argparse
import os
import re
import shutil
import statistics
import subprocess
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
RUNS = os.path.expanduser('~/Documents/A4VAI-SITL/ROS2/test_runs')
REFS = ['survey2_e100', 'survey_e100', 's7s_e100', 's7t_e100', 's7w_e100', 't1000_p1', 't1000_p3',
        't1000_p4', 't1000_p5', 'nc_mppi_d', 'nc_mppi_e', 'nc_mppi_g', 'x8_nc_mppi_f', 'x8_nc_mppi_g']


def num(pat, text, cast=float):
    m = re.search(pat, text)
    return cast(m.group(1)) if m else None


def analyze(name, gx, gy, alt):
    refs = [os.path.join(RUNS, r, 'cloud_map.npz') for r in REFS if os.path.exists(os.path.join(RUNS, r, 'cloud_map.npz'))]
    out = subprocess.run([sys.executable, os.path.join(REPO, 'scripts/dev/analyze_flight.py'), os.path.join(RUNS, name),
                          '--goal', f'{gx},{gy},{alt}', '--max-obs-range', '10', '--confirm', '1', '--ref', *refs],
                         capture_output=True, text=True, timeout=600).stdout
    return {
        'clr': num(r'clearance\[own\] min ([0-9.]+)m', out),
        'n_lt1': num(r'clearance\[own\].*<1m: (\d+)', out, int),
        'goal_err': num(r'goal err xy ([0-9.]+)m', out),
        'avg': num(r'cruise .*avg\s+([0-9.]+)m/s', out),
        'vmax': num(r'cruise .*max\s+([0-9.]+)', out),
        'alt_max': num(r'truth alt\[min [0-9.]+ max ([0-9.]+)\]', out),
        'thrust_sat': num(r'thrust max [0-9.]+ >=0.99\s+([0-9.]+)%', out),
        'tilt_lim': num(r'tilt@limit\s+([0-9.]+)%', out),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('prefix')
    ap.add_argument('n', type=int)
    ap.add_argument('--goals', nargs='+', default=['100,0'])
    ap.add_argument('--alt', default='4')
    ap.add_argument('--app', default='noclimb_ca')
    ap.add_argument('--airframe', default='x8')
    ap.add_argument('--fly-env', default='')
    ap.add_argument('--timeout', default='420')
    ap.add_argument('--start', type=int, default=1)
    ap.add_argument('--warn-clr', type=float, default=1.0)
    a = ap.parse_args()

    ros2_env = os.path.join(REPO, 'envs/ros2.env')
    gz_env = os.path.join(REPO, 'envs/gazebo-classic.env')
    backup = {p: open(p).read() for p in (ros2_env, gz_env)}
    gz = re.sub(r'^SITL_AIRFRAME=".*?"', f'SITL_AIRFRAME="{a.airframe}"', backup[gz_env], flags=re.M)
    open(gz_env, 'w').write(gz)
    csv_path = os.path.join(RUNS, f'{a.prefix}_summary.csv')
    new = not os.path.exists(csv_path)
    csv = open(csv_path, 'a')
    if new:
        csv.write('i,name,goal,result,secs,goal_err,clr_own,n_lt1m,thrust_sat_pct,tilt_lim_pct,alt_max,avg_speed,vmax\n')
    rows, fails_in_row = [], 0
    try:
        for i in range(a.start, a.start + a.n):
            gx, gy = a.goals[(i - 1) % len(a.goals)].split(',')
            name = f'{a.prefix}_{i:03d}'
            env = dict(os.environ, FLY_ENV=a.fly_env)
            log = os.path.join(RUNS, f'{name}.batch.log')
            t0 = time.time()
            with open(log, 'w') as fh:
                subprocess.run(['timeout', '900', os.path.join(REPO, 'scripts/dev/fly_test.sh'), name, gx, gy, a.alt,
                                a.timeout, a.app, '7.0'], env=env, stdout=fh, stderr=subprocess.STDOUT)
            text = open(log).read()
            m = re.search(r'\[fly_test\] \S+: (\S+)', text)
            result = m.group(1) if m else 'no_result'
            res = os.path.join(RUNS, name, 'result.txt')
            if os.path.exists(res):
                result = open(res).read().strip().replace(',', ';') or result
            secs = round(time.time() - t0)
            st = analyze(name, gx, gy, a.alt) if result.startswith('landed') else {}
            shutil.move(log, os.path.join(RUNS, name, 'batch.log')) if os.path.isdir(os.path.join(RUNS, name)) else None
            row = dict(i=i, name=name, goal=f'({gx};{gy})', result=result, secs=secs, **st)
            rows.append(row)
            csv.write(','.join(str(row.get(k, '')) for k in ('i', 'name', 'goal', 'result', 'secs', 'goal_err', 'clr', 'n_lt1',
                                                              'thrust_sat', 'tilt_lim', 'alt_max', 'avg', 'vmax')) + '\n')
            csv.flush()
            landed = result.startswith('landed')
            fails_in_row = 0 if landed else fails_in_row + 1
            print(f'RUN {i:03d} ({gx},{gy}): {result} {secs}s | clr {st.get("clr")} m (<1m: {st.get("n_lt1")}) | '
                  f'err {st.get("goal_err")} | thrust_sat {st.get("thrust_sat")}% | alt max {st.get("alt_max")} | avg {st.get("avg")}',
                  flush=True)
            if not landed or (st.get('clr') is not None and st['clr'] < a.warn_clr):
                print(f'FAIL {i:03d} ({gx},{gy}): {result}, clearance {st.get("clr")} m', flush=True)
            if i % 10 == 0:
                done = [r for r in rows]
                ok = [r for r in done if r['result'].startswith('landed')]
                clr = [r['clr'] for r in ok if r.get('clr') is not None]
                print(f'PROGRESS {i}/{a.start + a.n - 1}: landed {len(ok)}/{len(done)}'
                      + (f' | clr min {min(clr):.2f} median {statistics.median(clr):.2f} m, runs < {a.warn_clr} m: '
                         f'{sum(c < a.warn_clr for c in clr)}, < 0.77 m (prop tips): {sum(c < 0.77 for c in clr)}' if clr else ''),
                      flush=True)
            if fails_in_row >= 4:
                print(f'STOPPED after {i}: {fails_in_row} runs in a row did not land (stack broken?)', flush=True)
                break
    finally:
        for p, text in backup.items():
            open(p, 'w').write(text)
        subprocess.run(['git', '-C', os.path.expanduser('~/Documents/A4VAI-SITL/ROS2/ros2_ws/src'), 'checkout', '--',
                        'realgazebo_fastplanner_adapters/config/goals_super_far.csv'], capture_output=True)
        csv.close()
        print('BATCH DONE', flush=True)


if __name__ == '__main__':
    main()
