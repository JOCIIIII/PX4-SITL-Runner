#!/usr/bin/env python3
"""Make the C-Track poles thinner in the running Unreal, for SILS with the field's real poles (20 mm).

The Unreal map is built from the layout with 0.5 m poles; this scales the pole actors' X/Y in the running game
(simSetObjectScale), so the lidar sees the thin pole. Nothing in the Unreal folder changes: restarting Unreal
brings the map's poles back. Fly with CTRACK_LAYOUT pointing at a layout copy with the same pole size, so that
Gazebo (collisions) and scripts/dev/ctrack_clearance.py agree with what the lidar sees.

usage: ctrack_thin_poles.py [--diameter 0.02] [--pattern '^Pole_'] [--layout LAYOUT]   (Unreal must be up, API 41451)
       ctrack_thin_poles.py --check                                                    (print the poles' scales)
"""
import argparse
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'ros2', 'tools'))
from airsim_rpc import Rpc  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--diameter', type=float, default=0.02)
    ap.add_argument('--pattern', default=r'^Pole_')
    ap.add_argument('--layout', default=None, help='the layout the map was built from (default: the installed one)')
    ap.add_argument('--check', action='store_true')
    ap.add_argument('--host', default='127.0.0.1')
    a = ap.parse_args()
    if a.layout is None:
        for d in (os.environ.get('CTRACK_DIR', ''), '~/Documents/A4VAI-SITL/A4VAI-Unreal',
                  '~/Desktop/00_Project/00_A4VAI/00_Development/00_Simulator/02_InhouseSim/00_Unreal/00_Map'):
            if d and os.path.isdir(os.path.expanduser(d)):
                a.layout = os.path.join(os.path.expanduser(d), 'layout', 'ctrack_layout.json')
                break
    lay = json.load(open(a.layout))
    poles = {o['name']: o for o in lay['obstacles'] if re.search(a.pattern, o['name']) and o['shape'] == 'cylinder'}
    rpc = Rpc(a.host, timeout=30.0)
    names = rpc.call('simListSceneObjects', a.pattern.lstrip('^') + '.*')
    for n in sorted(poles):
        if n not in names:
            print(f'[thin_poles] {n}: not in the scene ({names})')
            continue
        s = rpc.call('simGetObjectScale', n)
        if a.check:
            print(f'[thin_poles] {n}: scale {s}, layout diameter {poles[n]["size"][0]} m')
            continue
        # the map's actor: scale = the layout size (unit cylinder); the new diameter keeps the height
        k = a.diameter / float(poles[n]['size'][0])
        new = {'x_val': poles[n]['size'][0] * k, 'y_val': poles[n]['size'][1] * k, 'z_val': s['z_val']}
        if abs(s['x_val'] - poles[n]['size'][0]) > 1e-3:      # not the map's scale: assume unit-size mesh anyway
            print(f'[thin_poles] {n}: current scale {s} differs from the layout size {poles[n]["size"]}')
        ok = rpc.call('simSetObjectScale', n, new)
        s2 = rpc.call('simGetObjectScale', n)
        print(f'[thin_poles] {n}: scale {s["x_val"]:.3f},{s["y_val"]:.3f},{s["z_val"]:.3f} -> '
              f'{s2["x_val"]:.3f},{s2["y_val"]:.3f},{s2["z_val"]:.3f} ({"ok" if ok else "FAILED"})')


if __name__ == '__main__':
    main()
