#!/usr/bin/env python3
"""UGV / USV batch: the course and planned-path runs of the port review, up to --parallel ground_run.sh instances
at a time (own docker network each), then one summary (CSV + markdown) from ground_metrics.py (path error,
MPPI share, speed) and, for the planned-path runs, ground_clearance.py (true footprint vs obstacles).
  ground_batch.py PREFIX [--parallel 4] [--only SUBSTR] [--reps N]
Runs that already have out/ground_driver_result.txt are skipped (re-running the same command resumes)."""
import argparse
import csv
import json
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
GS = os.path.expanduser('~/Documents/A4VAI-SITL/ground_sim')
LAYOUTS = os.path.join(HERE, 'layouts')

# name, vehicle, course CSV (courses/), speed, layout (planned path) or None, timeout [s]
COURSES = [
    ('line', 'ugv', 'ugv_line.csv', 1.5, None, 240),
    ('circle', 'ugv', 'ugv_circle.csv', 1.5, None, 240),
    ('square_r2', 'ugv', 'ugv_square_r2.csv', 1.5, None, 240),
    ('square_sharp', 'ugv', 'ugv_square_sharp.csv', 1.5, None, 240),
    ('slalom', 'ugv', 'ugv_slalom.csv', 1.5, None, 240),
    ('offset_start', 'ugv', 'ugv_line_offset.csv', 1.5, None, 240),
    ('pp_yard', 'ugv', 'pp_ugv_yard.csv', 1.5, 'ugv_yard', 300),
    ('line', 'usv', 'usv_line.csv', 2.5, None, 360),
    ('circle', 'usv', 'usv_circle.csv', 2.5, None, 420),
    ('square_r12', 'usv', 'usv_square_r12.csv', 2.5, None, 420),
    ('slalom', 'usv', 'usv_slalom.csv', 2.5, None, 420),
    ('offset_start', 'usv', 'usv_line_offset.csv', 2.5, None, 360),
    ('pp_harbour', 'usv', 'pp_usv_harbour.csv', 2.5, 'usv_harbour', 480),
]


def cases(a):
    out = []
    for name, veh, csv_name, speed, layout, timeout in COURSES:
        for guid in (0, 2):
            for rep in range(a.reps if layout else 1):
                run = f'{a.prefix}_{veh}_{name}_g{guid}' + (f'_r{rep}' if layout and a.reps > 1 else '')
                if a.only and a.only not in run:
                    continue
                out.append(dict(run=run, veh=veh, csv=csv_name, speed=speed, guid=guid, layout=layout, timeout=timeout))
    return out


def start(c, k):
    env = dict(os.environ, GROUND_INSTANCE=str(k))
    extra = []
    if c['layout']:
        world = f"{c['layout']}.world"
        if not os.path.exists(os.path.join(GS, 'worlds', world)):
            subprocess.run([sys.executable, os.path.join(HERE, 'ground_world.py'), os.path.join(LAYOUTS, c['layout'] + '.json'),
                            c['veh'], os.path.join(GS, 'worlds', world)], check=True, capture_output=True)
        lay = json.load(open(os.path.join(LAYOUTS, c['layout'] + '.json')))
        sx, sy = lay['waypoints']['START']
        env['GROUND_WORLD'] = world
        env['GROUND_SPAWN_POSE'] = f"-x {sy} -y {sx} -z {0.10 if c['veh'] == 'usv' else 0.08} -Y 1.570796"
        extra = ['--frame', 'local', '--origin', f'{sx},{sy}']
    cmd = ['timeout', str(c['timeout'] + 120), os.path.join(HERE, 'ground_run.sh'), c['veh'], c['run'], '--',
           '/ground_scripts/ground_mission.sh', f"/ground_courses/{c['csv']}", f"{c['speed']}", str(c['guid']),
           '--timeout', str(c['timeout'])] + extra
    log = open(os.path.join(GS, 'runs', c['run'] + '.batch.log'), 'w')
    return subprocess.Popen(cmd, env=env, stdout=log, stderr=subprocess.STDOUT, cwd=os.path.expanduser('~'))


def summarise(a, cs):
    rows = []
    for c in cs:
        run_dir = os.path.join(GS, 'runs', c['run'])
        r = dict(run=c['run'], vehicle=c['veh'], course=c['csv'], speed=c['speed'], guidance='MPPI' if c['guid'] else 'pursuit')
        try:
            m = json.loads(subprocess.run([sys.executable, os.path.join(HERE, 'ground_metrics.py'), run_dir, '--json'],
                                          capture_output=True, text=True).stdout)[0]
            r.update({k: m.get(k) for k in ('result', 'path_len_m', 'follow_s', 'e_p95', 'e_max', 'e_true_p95', 'e_true_max',
                                            'mppi_pct', 'e_p95_mppi', 'speed_mean', 'end_err_true_m', 'est_err_max')})
        except Exception as ex:
            r['result'] = f'no metrics ({type(ex).__name__})'
        if c['layout']:
            try:
                cl = json.loads(subprocess.run([sys.executable, os.path.join(HERE, 'ground_clearance.py'), run_dir,
                                                os.path.join(LAYOUTS, c['layout'] + '.json'), c['veh'], '--json'],
                                               capture_output=True, text=True).stdout)
                r.update(min_clearance_m=cl['min_clearance_m'], nearest=cl['nearest'], contact=cl['contact'], goal_dist_m=cl['goal_dist_m'])
            except Exception as ex:
                r['min_clearance_m'] = f'n/a ({type(ex).__name__})'
        rows.append(r)
    keys = []
    for r in rows:
        keys += [k for k in r if k not in keys]
    path = os.path.join(GS, 'runs', f'{a.prefix}_summary.csv')
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)
    md = os.path.join(GS, 'runs', f'{a.prefix}_summary.md')
    with open(md, 'w') as f:
        f.write('| run | result | path m | time s | e p95 est / true m | e max true m | MPPI % | speed m/s | end err m | clearance m |\n')
        f.write('|---|---|---|---|---|---|---|---|---|---|\n')
        for r in rows:
            f.write(f"| {r['run']} | {r.get('result')} | {r.get('path_len_m')} | {r.get('follow_s')} | {r.get('e_p95')} / {r.get('e_true_p95')} | "
                    f"{r.get('e_true_max')} | {r.get('mppi_pct')} | {r.get('speed_mean')} | {r.get('end_err_true_m')} | {r.get('min_clearance_m', '')} |\n")
    print(open(md).read())
    print('->', path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('prefix')
    ap.add_argument('--parallel', type=int, default=4)
    ap.add_argument('--only', default='')
    ap.add_argument('--reps', type=int, default=1)
    ap.add_argument('--summary-only', action='store_true')
    a = ap.parse_args()
    cs = cases(a)
    todo = [c for c in cs if not os.path.exists(os.path.join(GS, 'runs', c['run'], 'out', 'ground_driver_result.txt'))]
    print(f'[ground_batch] {len(cs)} runs, {len(todo)} to fly, {a.parallel} at a time', flush=True)
    free, running = list(range(a.parallel)), {}
    while (todo or running) and not a.summary_only:
        while todo and free:
            c, k = todo.pop(0), free.pop(0)
            running[k] = (c, start(c, k), time.time())
            print(f"[ground_batch] start {c['run']} on instance {k}", flush=True)
        time.sleep(2)
        for k, (c, p, t0) in list(running.items()):
            if p.poll() is not None:
                res = os.path.join(GS, 'runs', c['run'], 'out', 'ground_driver_result.txt')
                print(f"[ground_batch] done {c['run']} ({open(res).read().strip() if os.path.exists(res) else 'NO RESULT'}, "
                      f"{time.time() - t0:.0f} s)", flush=True)
                del running[k]
                free.append(k)
    summarise(a, cs)


if __name__ == '__main__':
    main()
