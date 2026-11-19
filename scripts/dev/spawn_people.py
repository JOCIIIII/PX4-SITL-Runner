#!/usr/bin/env python3
"""People to find in the simulator: spawns the A4VAI-Unreal mannequins (SM_Person_Manny / SM_Person_Quinn,
branch people and later) standing or lying at random spots of a search area, away from the layout's obstacles and the
take-off point, and writes the ground truth for scripts/dev/search_report.py.

    python3 scripts/dev/spawn_people.py [--area 시험장 전체] [--n 3] [--lying 0.5] [--seed 1] [--out people.json]
    python3 scripts/dev/spawn_people.py --clear          # remove the ones spawned before
Positions are test map layout X, Y [m] (= AirSim NED of the Scenario map). Unreal must be up (API 41451).
"""
import argparse
import json
import math
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'ros2', 'tools'))
from airsim_rpc import Rpc  # noqa: E402

ASSETS = ['SM_Person_Manny', 'SM_Person_Quinn']


def quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return {'w_val': cr * cp * cy + sr * sp * sy, 'x_val': sr * cp * cy - cr * sp * sy,
            'y_val': cr * sp * cy + sr * cp * sy, 'z_val': cr * cp * sy - sr * sp * cy}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--places', default=os.path.join(HERE, '..', 'ros2', 'apps', 'llm_places_scenario.json'))
    ap.add_argument('--layout', default=None, help='obstacles to keep clear of (default: the installed A4VAI-Unreal layout)')
    ap.add_argument('--area', default='시험장 전체')
    ap.add_argument('--n', type=int, default=3)
    ap.add_argument('--lying', type=float, default=0.5, help='fraction lying on the ground')
    ap.add_argument('--seed', type=int, default=None)
    ap.add_argument('--origin', default='79,26', help='take-off point X,Y - nobody within 10 m of it')
    ap.add_argument('--clearance', type=float, default=4.0, help='metres from every obstacle')
    ap.add_argument('--out', default='people.json')
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--clear', action='store_true')
    a = ap.parse_args()
    rpc = Rpc(a.host, timeout=30.0)
    for name in rpc.call('simListSceneObjects', 'person_.*'):
        rpc.call('simDestroyObject', name)
    if a.clear:
        print('removed the spawned people')
        return
    poly = json.load(open(a.places))['areas'][a.area]['polygon']
    xs, ys = [p[0] for p in poly], [p[1] for p in poly]
    layout = a.layout
    if layout is None:
        for d in (os.environ.get('SCENARIO_DIR', ''), os.path.expanduser('~/Documents/A4VAI-SITL/A4VAI-Unreal'),
                  os.path.expanduser('~/Desktop/00_Project/00_A4VAI/00_Development/00_Simulator/02_InhouseSim/00_Unreal/00_Map')):
            if d and os.path.exists(os.path.join(d, 'layout', 'scenario_layout.json')):
                layout = os.path.join(d, 'layout', 'scenario_layout.json')
                break
    obstacles = []
    if layout:
        L = json.load(open(layout))
        obstacles = [(o['pos'][0], o['pos'][1], max(o['size'][0], o['size'][1]) / 2) for o in L['obstacles']]
        obstacles += [(m['start'][0], (m['start'][1] + m['end'][1]) / 2, abs(m['start'][1] - m['end'][1]) / 2 + 1)
                      for m in L.get('moving_obstacles', [])]
    ox, oy = (float(v) for v in a.origin.split(','))
    rnd = random.Random(a.seed)
    people = []
    for i in range(a.n):
        for _ in range(2000):
            x, y = rnd.uniform(min(xs) + 2, max(xs) - 2), rnd.uniform(min(ys) + 2, max(ys) - 2)
            if math.hypot(x - ox, y - oy) < 10.0:
                continue
            if any(math.hypot(x - cx, y - cy) < r + a.clearance for cx, cy, r in obstacles):
                continue
            if any(math.hypot(x - p['xy'][0], y - p['xy'][1]) < 8.0 for p in people):
                continue
            break
        else:
            sys.exit(f'no free spot for person {i + 1}')
        lying = rnd.random() < a.lying
        yaw = rnd.uniform(-math.pi, math.pi)
        asset = ASSETS[i % len(ASSETS)]
        name = f'person_{i + 1}'
        pose = {'position': {'x_val': x, 'y_val': y, 'z_val': -0.15 if lying else 0.0},
                'orientation': quat(math.pi / 2 if lying else 0.0, 0.0, yaw)}
        got = rpc.call('simSpawnObject', name, asset, pose, {'x_val': 1.0, 'y_val': 1.0, 'z_val': 1.0}, False, False)
        people.append({'name': got or name, 'asset': asset, 'xy': [round(x, 2), round(y, 2)],
                       'pose': 'lying' if lying else 'standing', 'yaw_deg': round(math.degrees(yaw), 1)})
        print(f'{got or name}: {asset} {"lying" if lying else "standing"} at ({x:.1f}, {y:.1f})')
    json.dump({'area': a.area, 'people': people}, open(a.out, 'w'), ensure_ascii=False, indent=2)
    print(f'ground truth -> {a.out}')


if __name__ == '__main__':
    main()
