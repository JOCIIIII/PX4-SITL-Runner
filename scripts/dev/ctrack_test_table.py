#!/usr/bin/env python3
"""The test matrix at a glance: the latest run of each of the 12 cells of scripts/dev/ctrack_test.sh.

  python3 scripts/dev/ctrack_test_table.py [test_matrix.csv]      (or: ./scripts/ctrack.sh test table)

Cells: pp; pf; ca, pf-ca and all x {inha, jbnu, super}; pp-pf. Path following is PathFollowing's MPPI throughout
(the CA unit test flies each avoidance with its own default chain). A cell never run shows "-"; runs of earlier
variants (e.g. the dropped PX4 default follower) are not shown.
"""
import csv
import os
import sys

path = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser('~/Documents/A4VAI-SITL/ROS2/test_runs/test_matrix.csv')
PFS, CAS = ('mppi',), ('inha', 'jbnu', 'super')
cells = [('pp', '-', '-')]
cells += [('pf', p, '-') for p in PFS]
cells += [('ca', '-', c) for c in CAS]
cells += [('pp-pf', p, '-') for p in PFS]
cells += [('pf-ca', p, c) for c in CAS for p in PFS]
cells += [('all', p, c) for c in CAS for p in PFS]

latest = {}
if os.path.exists(path):
    for r in csv.DictReader(open(path)):
        kind = r['kind']
        key = (kind, '-' if kind in ('pp', 'ca') else r['pf'], r['ca'] if kind in ('ca', 'pf-ca', 'all') else '-')
        latest[key] = r                       # the file is in run order: the last one wins

KEEP = ('length', 'planning', 'p95', 'max', 'margin', 'goal', 'landed')
print(f'{"test":6} {"PF":5} {"CA":6} {"verdict":7} {"run":22} key values')
for key in cells:
    r = latest.get(key)
    if r is None:
        print(f'{key[0]:6} {key[1]:5} {key[2]:6} {"-":7}')
        continue
    vals = [kv for kv in r['checks'].split('; ') if any(k in kv for k in KEEP)]
    print(f'{key[0]:6} {key[1]:5} {key[2]:6} {r["verdict"]:7} {r["name"][:22]:22} {"; ".join(vals)}')
done = [latest[k] for k in cells if k in latest]
print(f'\n{sum(r["verdict"] == "PASS" for r in done)} PASS, {sum(r["verdict"] == "FAIL" for r in done)} FAIL, '
      f'{len(cells) - len(done)} not run (of {len(cells)})')
