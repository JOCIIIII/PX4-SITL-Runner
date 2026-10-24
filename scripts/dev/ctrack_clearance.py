#!/usr/bin/env python3
"""Clearance of a scripts/dev/ctrack_super.sh run to the C-Track obstacles, from the layout
(the Unreal map and the Gazebo world are both generated from it, so it is the ground truth).

The vehicle is a vertical slab around its centre: from the skid bottom to the top plate
(X8 -0.44 / +0.09 m), radius to the prop tips (X8 0.77 m). Per obstacle: the closest horizontal
distance from the centre to the obstacle outline while the slab overlaps the obstacle's height;
'over' when the vehicle passed above it (gap from the skid bottom to the obstacle top).
The RC cart is replayed from its plugin rule (starts when the vehicle comes within
trigger_distance of trigger_point, shuttles start <-> end at speed_mps).

Positions: the simulator's pose (airsim_odom.csv) when recorded, else PX4's estimate (--estimate
forces it); PX4's local z is relative to the vehicle centre resting on its skids, so its height
above ground is -z + rest (x8 0.44 m).

usage: ctrack_clearance.py RUN_DIR [--layout LAYOUT.json] [--radius 0.77] [--below 0.44] [--above 0.09]
                           [--rest 0.44] [--estimate]
"""
import argparse
import json
import math
import os

import numpy as np

def _ctrack_dir():
    """CTRACK_DIR, else the installed A4VAI-Unreal (scripts/install.sh), else the developers' checkout"""
    for d in (os.environ.get('CTRACK_DIR', ''), '~/Documents/A4VAI-SITL/A4VAI-Unreal',
              '~/Desktop/00_Project/00_A4VAI/00_Development/00_Simulator/02_InhouseSim/Unreal'):
        if d and os.path.isdir(os.path.expanduser(d)):
            return os.path.expanduser(d)
    return os.path.expanduser('~/Documents/A4VAI-SITL/A4VAI-Unreal')


DEF_LAYOUT = os.path.join(_ctrack_dir(), 'layout', 'ctrack_layout.json')


def box_dist(px, py, cx, cy, sx, sy, yaw_deg):
    """horizontal distance from points to a yawed rectangle (0 inside)"""
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    dx, dy = px - cx, py - cy
    lx, ly = c * dx + s * dy, -s * dx + c * dy
    ex = np.maximum(np.abs(lx) - sx / 2.0, 0.0)
    ey = np.maximum(np.abs(ly) - sy / 2.0, 0.0)
    return np.hypot(ex, ey)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run_dir')
    ap.add_argument('--layout', default=DEF_LAYOUT)
    # vehicle geometry (prop-tip radius, body below / above the centre, centre height at rest): by default the
    # run's airframe (ctrack_run.txt field 9) - X8 0.77 / 0.44 / 0.09 / 0.44, X500 0.386 / 0.23 / 0.07 / 0.23.
    # (The X8 values on X500 runs understated their margins by 0.38 m, 2026-10-16.)
    ap.add_argument('--radius', type=float, default=None)
    ap.add_argument('--below', type=float, default=None)
    ap.add_argument('--above', type=float, default=None)
    ap.add_argument('--rest', type=float, default=None)
    ap.add_argument('--estimate', action='store_true', help='use PX4\'s estimate even if airsim_odom.csv exists')
    ap.add_argument('--alt', type=float, default=1.5, help='target centre height above ground [m]')
    ap.add_argument('--alt-tol', type=float, default=0.2)
    a = ap.parse_args()
    lay = json.load(open(a.layout))
    run = open(os.path.join(a.run_dir, 'ctrack_run.txt')).read().split()
    frm, goal_wp = run[:2]
    airframe = run[8] if len(run) > 8 else 'x8'
    geo = {'x500': (0.386, 0.23, 0.07, 0.23)}.get(airframe, (0.77, 0.44, 0.09, 0.44))
    for name, value in zip(('radius', 'below', 'above', 'rest'), geo):
        if getattr(a, name) is None:
            setattr(a, name, value)
    wps = lay['waypoints']
    fx, fy = wps[frm] if frm in wps else map(float, frm.split(','))       # name or X,Y
    gx, gy = wps[goal_wp] if goal_wp in wps else map(float, goal_wp.split(','))
    p = np.genfromtxt(os.path.join(a.run_dir, 'px4.csv'), delimiter=',', names=True)
    st = np.genfromtxt(os.path.join(a.run_dir, 'status.csv'), delimiter=',', names=True)
    t = p['t']
    X, Y, H = fx + p['y'], fy - p['x'], a.rest - p['z']  # PX4 local NED -> track frame, height AGL
    truth = os.path.join(a.run_dir, 'airsim_odom.csv')
    if os.path.exists(truth) and not a.estimate:
        # the simulator's pose (Unreal mirrors Gazebo; x = X, y = -Y, z = centre height): PX4's
        # estimate is 0.2-0.7 m off in a clean flight and meaningless after a collision
        g = np.genfromtxt(truth, delimiter=',', names=True)
        tx, ty, th = (np.interp(t, g['t'], g[k]) for k in ('x', 'y', 'z'))
        err = np.hypot(tx - X, -ty - Y)
        X, Y, H = tx, -ty, th
        src = f'simulator pose (PX4 estimate off by median {np.median(err):.2f} / max {err.max():.2f} m)'
    else:
        src = 'PX4 estimate'
    armed = st['t'][st['arming'] == 2]
    fly = (H > a.rest + 0.3) & (t >= (armed[0] if len(armed) else t[0]))
    lo, hi = H - a.below, H + a.above                     # vehicle slab

    rows = []
    for o in lay['obstacles']:
        sx, sy, sz = o['size']
        if o['shape'] == 'cylinder':
            d = np.maximum(np.hypot(X - o['pos'][0], Y - o['pos'][1]) - sx / 2.0, 0.0)
        else:
            d = box_dist(X, Y, o['pos'][0], o['pos'][1], sx, sy, o.get('yaw_deg', 0.0))
        rows.append((o['name'], d, sz))

    # moving obstacles replayed from the plugin rule
    for mo in lay.get('moving_obstacles', []):
        s0, s1 = np.array(mo['start'], float), np.array(mo['end'], float)
        L = float(np.linalg.norm(s1 - s0))
        near = np.hypot(X - mo['trigger_point'][0], Y - mo['trigger_point'][1]) <= mo['trigger_distance']
        cx, cy = np.full(len(t), s0[0]), np.full(len(t), s0[1])
        if near.any():
            tt = np.maximum(t - t[np.argmax(near)], 0.0) * mo['speed_mps']
            ph = np.mod(tt, 2 * L) if mo.get('loop', True) else np.minimum(tt, L)
            f = np.where(ph <= L, ph, 2 * L - ph) / L
            cx, cy = s0[0] + f * (s1[0] - s0[0]), s0[1] + f * (s1[1] - s0[1])
        sx = max(pt['size'][0] for pt in mo['parts'])
        sy = max(pt['size'][1] for pt in mo['parts'])
        sz = max(pt['offset'][2] + pt['size'][2] for pt in mo['parts'])
        d = box_dist(X, Y, cx, cy, sx, sy, 0.0)
        rows.append((mo['name'] + ' (moving)', d, sz))

    print(f'{os.path.basename(a.run_dir.rstrip("/"))}: {frm} -> {goal_wp}, slab {-a.below:+.2f}/{a.above:+.2f} m, '
          f'prop radius {a.radius} m; {src}')
    worst = (None, float('inf'))
    for name, d, top in rows:
        overlap = fly & (lo < top)
        over = fly & ~(lo < top)
        line = f'  {name:22s}'
        if overlap.any():
            i = np.argmin(np.where(overlap, d, np.inf))
            c = float(d[i])
            line += (f' closest {c:5.2f} m (margin {c - a.radius:+.2f}) at ({X[i]:5.1f},{Y[i]:5.1f}) '
                     f'h {H[i]:.1f} m t+{t[i] - t[fly][0]:.1f}s')
            if c - a.radius < worst[1]:
                worst = (name, c - a.radius)
        else:
            line += ' never at its height'
        if over.any():
            j = np.argmin(np.where(over, d, np.inf))
            if d[j] < a.radius:
                line += f' | passed over it, skid gap {lo[j] - top:.2f} m'
        print(line)
    v = np.hypot(p['vx'], p['vy'])
    end = fly.nonzero()[0][-1] if fly.any() else -1
    print(f'  minimum margin to the prop tips: {worst[1]:+.2f} m ({worst[0]})'
          f'{"  CONTACT" if worst[1] < 0 else ""}')
    # not the final approach / descent onto the goal (3.5 m: flight_manager's handover radius 3 m)
    cr = fly & (v > 1.0) & (np.hypot(X - gx, Y - gy) > 3.5)
    print(f'  centre height above ground while moving (>1 m/s): min {H[cr].min():.2f} median {np.median(H[cr]):.2f} '
          f'p90 {np.percentile(H[cr], 90):.2f} max {H[cr].max():.2f} m')
    # the pole field (C1..C6, X 40-70): height while flying through it
    pz = fly & (X > 40.0) & (X < 70.0) & (H > a.rest + 0.5)
    if pz.any():
        ok = np.abs(H[pz] - a.alt) <= a.alt_tol
        print(f'  pole field X 40-70 m: height min {H[pz].min():.2f} median {np.median(H[pz]):.2f} max {H[pz].max():.2f} m, '
              f'{100.0 * ok.mean():.0f}% within {a.alt} +- {a.alt_tol} m, Y {Y[pz].min():.1f}..{Y[pz].max():.1f}')
    print(f'  flight: speed mean {v[cr].mean():.2f} max {v[fly].max():.2f} m/s, end ({X[-1]:.1f},{Y[-1]:.1f}) = {math.hypot(X[-1] - gx, Y[-1] - gy):.2f} m '
          f'from {goal_wp}, last airborne ({X[end]:.1f},{Y[end]:.1f})')


if __name__ == '__main__':
    main()
