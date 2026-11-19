#!/usr/bin/env python3
"""Index of every run under test_runs/: environment, SILS/PILS, airframe, app, policy, result, and for C-Track /
Yard runs the ctrack_clearance numbers (min prop-tip margin, speeds). Rows are appended as they are made.

    python3 scripts/dev/sils_record/runs_index.py OUT.csv        (resumes OUT.csv if it exists; ~10 min for 636 runs)
    python3 scripts/dev/sils_record/campaigns.py DIR             (DIR/runs_index.csv -> DIR/runs_index_campaign.csv, DIR/campaigns.csv)
Margins use the C-Track layout of the run's time: the poles were 0.1 m until the night of 2026-10-12 (backup layouts in
the A4VAI-Unreal folder's backup/), 0.5 m after. Written for docs/sils-pils-test-record.md (2026-11-02)."""
import csv, os, re, subprocess, sys, datetime
TR = os.path.expanduser('~/Documents/A4VAI-SITL/ROS2/test_runs')
CLR = os.path.expanduser('~/PX4-SITL-Runner-v1.0/scripts/dev/ctrack_clearance.py')
UNREAL = os.path.expanduser(os.environ.get('CTRACK_DIR', '~/Desktop/00_Project/00_A4VAI/00_Development/00_Simulator/02_InhouseSim/00_Unreal/00_Map'))
OUT = sys.argv[1]
PILS = re.compile(r'^(pils_|gate_pils|j2_|llm_pils|ctrack_pils_loop)')
LAYOUT_OLD = {r'^ctrack_sp_': 'ctrack_layout.20261010_before_123poles.json',
              r'^ctrack_(123|hold|s900|v2|big|x8r2|x8s)_': 'ctrack_layout.20261012_before_poles05.json'}
FIELDS = ['name', 'date', 'time', 'env', 'mode', 'onboard', 'airframe', 'app', 'from', 'goal', 'alt', 'policy', 'bundle',
          'result', 'reason', 'margin', 'margin_obj', 'contact', 'v_mean', 'v_max', 'goal_err', 'h_median', 'note', 'layout']

def read(p, n=4000):
    try:
        with open(p, 'rb') as f:
            return f.read(n).replace(b'\0', b'').decode('utf-8', 'replace')
    except OSError:
        return ''

done = set()
if os.path.exists(OUT):
    done = {r['name'] for r in csv.DictReader(open(OUT))}
new = not os.path.exists(OUT)
out = open(OUT, 'a', newline='')
w = csv.DictWriter(out, FIELDS)
if new:
    w.writeheader()
for name in sorted(os.listdir(TR)):
    d = os.path.join(TR, name)
    if not os.path.isdir(d) or name in done:
        continue
    row = dict.fromkeys(FIELDS, '')
    row['name'] = name
    mt = datetime.datetime.fromtimestamp(os.path.getmtime(d))
    row['date'], row['time'] = mt.strftime('%m-%d'), mt.strftime('%H:%M')
    run = read(os.path.join(d, 'ctrack_run.txt')).split()
    files = set(os.listdir(d))
    if run:
        row['from'], row['goal'] = run[0], run[1]
        row['alt'] = run[6] if len(run) > 6 else ''
        row['airframe'] = run[8] if len(run) > 8 else 'x8'
        row['app'] = run[9] if len(run) > 9 else 'super_pf'
        row['env'] = run[10] if len(run) > 10 else 'CTrack'
    elif name.startswith('pfm_') or name.startswith('pf'):
        row['env'] = 'forest-sim(PF only)'
    elif 'airsim_odom.csv' in files or 'stack.log' in files or 'batch.log' in files:
        row['env'] = 'forest(AirSim)'
    else:
        row['env'] = 'other'
    row['mode'] = 'PILS' if PILS.match(name) else 'SILS'
    if row['mode'] == 'PILS':
        row['onboard'] = ('jetson002' if name.startswith(('j2_', 'gate_pils02')) else
                          'zedbox' if name.startswith('llm_pils') else
                          'loopback(PC)' if name.startswith('ctrack_pils_loop') else 'jetson001')
    log = read(os.path.join(d, 'noclimb_ca.log'), 20000)
    m = re.search(r'policy (\S+) \((\S+)\)', log)
    if m:
        row['policy'] = m.group(1)
        b = m.group(2)
        k = b.find('Inhouse-collision-avoidance/')
        row['bundle'] = os.path.dirname(b[k + 28:]) if k >= 0 else os.path.dirname(b)
        if not row['app']:
            row['app'] = 'noclimb_ca'
    elif 't1000' in name and not row['app']:
        row['app'] = 't1000'
    res = read(os.path.join(d, 'result.txt')).strip().splitlines()
    if res:
        row['result'] = res[0].split()[0] if res[0].split() else ''
        row['reason'] = res[0][len(row['result']):].strip()[:160]
    if name.endswith('INVALID'):
        row['note'] = 'INVALID (named so at the time)'
    if run and row['env'] in ('CTrack', 'Yard'):
        args = [sys.executable, CLR, d]
        old = [f for rx, f in LAYOUT_OLD.items() if re.match(rx, name)]
        if row['env'] == 'Yard':
            args += ['--layout', os.path.join(UNREAL, 'layout', 'yard_slam_layout.json')]
            row['layout'] = 'yard (no trees: margin not meaningful)'
        elif old:
            args += ['--layout', os.path.join(UNREAL, 'backup', old[0])]
            row['layout'] = old[0]
        elif os.path.exists(os.path.join(d, 'layout_x500.json')):
            args += ['--layout', os.path.join(d, 'layout_x500.json')]
        try:
            o = subprocess.run(args, capture_output=True, text=True, timeout=180).stdout
            m = re.search(r'minimum margin to the prop tips: ([+-][0-9.]+) m \(([^)]*)\)(\s+CONTACT)?', o)
            if m:
                row['margin'], row['margin_obj'], row['contact'] = m.group(1), m.group(2), 'yes' if m.group(3) else ''
            m = re.search(r'speed mean ([0-9.]+) max ([0-9.]+) m/s, end \([^)]*\) = ([0-9.]+) m', o)
            if m:
                row['v_mean'], row['v_max'], row['goal_err'] = m.groups()
            m = re.search(r'centre height above ground while moving[^:]*: min [0-9.-]+ median ([0-9.-]+)', o)
            if m:
                row['h_median'] = m.group(1)
            if not row['margin']:
                row['note'] = (row['note'] + ' clearance: ' + (o.strip().splitlines() or ['no output'])[-1])[:200]
        except Exception as e:
            row['note'] = (row['note'] + f' clearance error {e}')[:200]
    w.writerow(row)
    out.flush()
    print(name, row['env'], row['mode'], row['result'], row['margin'], flush=True)
