#!/usr/bin/env python3
"""Map of a person search run: the field and its obstacles, the flown track, the people found (person_search_node.py
search_results.json) against the ones placed (scripts/dev/spawn_people.py ground truth), with the photos.

    python3 scripts/dev/search_report.py RUN_DIR [--truth people.json] [--out RUN_DIR/search_map.png]
prints found / missed / false alarms and the position error of each match (within --match m, default 5).
"""
import argparse
import csv
import json
import math
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.image as mpimg      # noqa: E402
import matplotlib.pyplot as plt       # noqa: E402
from matplotlib import font_manager   # noqa: E402

for _f in ('NanumGothic', 'NanumSquare', 'Noto Sans CJK KR', 'Noto Sans CJK JP'):   # Hangul in the descriptions
    if any(_f == x.name for x in font_manager.fontManager.ttflist):
        plt.rcParams['font.family'] = _f
        break

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from scenario_clearance import DEF_LAYOUT  # noqa: E402


def track(run):
    """flown track in the layout frame from noclimb_ca.csv (PX4 local ENU from the take-off point) + scenario_run.txt"""
    meta = open(os.path.join(run, 'scenario_run.txt')).read().split()
    layout = json.load(open(DEF_LAYOUT))
    frm = meta[0]
    fx, fy = layout['waypoints'][frm] if frm in layout['waypoints'] else map(float, frm.split(','))
    xs, ys = [], []
    with open(os.path.join(run, 'noclimb_ca.csv')) as f:
        for r in csv.DictReader(f):
            xs.append(fx + float(r['x']))
            ys.append(fy - float(r['y']))
    return xs, ys, (fx, fy)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run')
    ap.add_argument('--truth', default=None)
    ap.add_argument('--out', default=None)
    ap.add_argument('--match', type=float, default=5.0)
    a = ap.parse_args()
    res = json.load(open(os.path.join(a.run, 'search_results.json')))
    truth = json.load(open(a.truth))['people'] if a.truth else []
    layout = json.load(open(DEF_LAYOUT))
    found = res['people']

    # matches: each true person to the nearest report within --match
    used, rows = set(), []
    for t in truth:
        best = min(((math.dist(t['xy'], f['xy']), f['id']) for f in found if f['id'] not in used), default=None)
        if best and best[0] <= a.match:
            used.add(best[1])
            rows.append((t, best[1], best[0]))
        else:
            rows.append((t, None, None))
    extra = [f for f in found if f['id'] not in used]
    print(f'people placed {len(truth)}, reported {len(found)}: found {sum(1 for r in rows if r[1])}, '
          f'missed {sum(1 for r in rows if not r[1])}, reports with nobody there {len(extra) if truth else "?"}, '
          f'rejected on a closer look {len(res["false_alarms"])}; {res["frames"]} frames')
    for t, fid, err in rows:
        f = next((x for x in found if x['id'] == fid), None)
        print(f'  {t["name"]} {t["pose"]:8s} ({t["xy"][0]:.1f}, {t["xy"][1]:.1f}): '
              + (f'#{fid} at {err:.2f} m, score {f["score"]:.2f} - {f["description"]}' if f else 'MISSED'))

    n = len(found)
    fig = plt.figure(figsize=(14, 6 + 2.6 * ((n + 3) // 4)))
    ax = fig.add_axes([0.05, 0.38 if n else 0.08, 0.9, 0.58 if n else 0.88])
    fxr, fyr = layout['field']['x_range'], layout['field']['y_range']
    ax.add_patch(plt.Rectangle((fxr[0], fyr[0]), fxr[1] - fxr[0], fyr[1] - fyr[0], fill=False, ec='0.6'))
    for o in layout['obstacles']:
        (x, y), (sx, sy) = o['pos'], o['size'][:2]
        ax.add_patch(plt.Rectangle((x - sx / 2, y - sy / 2), sx, sy, fc='0.55', ec='0.3'))
    xs, ys, origin = track(a.run)
    ax.plot(xs, ys, '-', color='tab:blue', lw=1.2, label='flown track')
    ax.plot(*origin, 'k^', ms=10, label='take-off')
    for t in truth:
        ax.plot(*t['xy'], 'o', ms=14, mfc='none', mec='tab:green', mew=2)
        ax.annotate(f'{t["name"]} ({t["pose"]})', t['xy'], (6, -14), textcoords='offset points', color='tab:green', fontsize=8)
    for f in found:
        ax.plot(*f['xy'], 'x', color='tab:red', ms=12, mew=3)
        ax.annotate(f'#{f["id"]}', f['xy'], (6, 6), textcoords='offset points', color='tab:red', fontsize=11, weight='bold')
    for fa in res['false_alarms']:
        ax.plot(*fa['xy'], '+', color='tab:orange', ms=10, mew=2)
    ax.set_aspect('equal')
    ax.invert_yaxis()
    ax.set_xlabel('layout X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_title(f'Person search {os.path.basename(os.path.normpath(a.run))}: green = placed, red = reported, '
                 f'orange = rejected on a closer look')
    ax.legend(loc='lower left', fontsize=8)
    for i, f in enumerate(found):
        if not os.path.exists(f['photo']):
            f['photo'] = os.path.join(a.run, 'search', os.path.basename(f['photo']))
        if not os.path.exists(f['photo']):
            continue
        r, c = divmod(i, 4)
        axi = fig.add_axes([0.05 + c * 0.235, 0.33 - (r + 1) * 0.3 * (0.38 / 0.33) / max(1, (n + 3) // 4) * 0.9,
                            0.22, 0.28 / max(1, (n + 3) // 4)])
        axi.imshow(mpimg.imread(f['photo']))
        axi.set_title(f'#{f["id"]} {f["score"]:.2f}: {f["description"][:60]}', fontsize=7)
        axi.axis('off')
    out = a.out or os.path.join(a.run, 'search_map.png')
    fig.savefig(out, dpi=110)
    print(f'map -> {out}')


if __name__ == '__main__':
    main()
