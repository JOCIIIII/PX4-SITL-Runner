#!/usr/bin/env python3
"""Open-sided 3x6 m canopy tent (6 legs, sloped roof eave 2.1 m -> ridge 3.0 m) spawned into the running C-Track Unreal.
Centre CX,CY (layout X,Y = AirSim NED x,y), 3 m along X (the flight direction), 6 m across (Y). --clear removes it.
Only the Unreal lidar sees it (Gazebo physics does not) - judge contact from the track."""
import sys, json
import os
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..', 'ros2', 'tools'))
from airsim_rpc import Rpc
CX, CY = 52.0, -8.0
L, W, EAVE, RIDGE, LEG = 3.0, 6.0, 2.1, 3.0, 0.05
r = Rpc('127.0.0.1', timeout=30.0)
names = [n for n in r.call('simListSceneObjects', 'Tent17_.*')]
for n in names:
    r.call('simDestroyObject', n)
if '--clear' in sys.argv:
    print('removed', names); sys.exit()
parts = []
for i, dx in enumerate((-L / 2, L / 2)):
    for j, dy in enumerate((-W / 2, 0.0, W / 2)):
        parts.append((f'Tent17_leg{i}{j}', 'Cylinder', CX + dx, CY + dy, EAVE / 2, (LEG, LEG, EAVE)))
# roof: three stacked slabs approximating the slope (full width at the eave, 1 m at the ridge), ridge along Y
for k, (lx, z0, z1) in enumerate(((L, EAVE, EAVE + 0.3), (2.0, EAVE + 0.3, EAVE + 0.6), (1.0, EAVE + 0.6, RIDGE))):
    parts.append((f'Tent17_roof{k}', 'Cube', CX, CY, (z0 + z1) / 2, (lx, W, z1 - z0)))
q = {'w_val': 1.0, 'x_val': 0.0, 'y_val': 0.0, 'z_val': 0.0}
for name, asset, x, y, zc, (sx, sy, sz) in parts:
    pose = {'position': {'x_val': x, 'y_val': y, 'z_val': -zc}, 'orientation': q}
    got = r.call('simSpawnObject', name, asset, pose, {'x_val': sx, 'y_val': sy, 'z_val': sz}, False, False)
    print(name, got)
json.dump({'centre': [CX, CY], 'size': [L, W], 'eave': EAVE, 'ridge': RIDGE, 'leg': LEG,
           'parts': [p[:1] + p[2:] for p in parts]}, open(os.path.join(HERE, 'tent17.json'), 'w'))
