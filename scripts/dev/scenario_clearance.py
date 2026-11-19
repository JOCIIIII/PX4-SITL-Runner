#!/usr/bin/env python3
"""Clearance of a scripts/dev/scenario_super.sh run to the test map's obstacles, from the layout
(the Unreal map and the Gazebo world are both generated from it, so it is the ground truth).

The vehicle is a vertical slab around its centre: from the skid bottom to the top plate
(X8 -0.44 / +0.09 m), radius to the prop tips (X8 0.77 m). Per obstacle: the closest horizontal
distance from the centre to the obstacle outline while the slab overlaps the obstacle's height;
'over' when the vehicle passed above it (gap from the skid bottom to the obstacle top).
The RC cart shuttles start <-> end at speed_mps from the moment it started (see moving_track): the
start in the run's gzserver log on PX4's clock (= Gazebo sim time; computer-clock tracks are mapped
through the uXRCE-DDS offset in px4_sitl.ulg), else its plugin rule (layouts with a trigger: when the
vehicle comes within trigger_distance of trigger_point), else the simulation start (no trigger).

Positions: the simulator's pose (airsim_odom.csv) when recorded, else PX4's estimate (--estimate
forces it); PX4's local z is relative to the vehicle centre resting on its skids, so its height
above ground is -z + rest (x8 0.44 m).

usage: scenario_clearance.py RUN_DIR [--layout LAYOUT.json] [--radius 0.77] [--below 0.44] [--above 0.09]
                           [--rest 0.44] [--estimate]
"""
import argparse
import json
import math
import os
import re

import numpy as np

def _scenario_dir():
    """SCENARIO_DIR, else the installed A4VAI-Unreal (scripts/install.sh), else the developers' checkout"""
    for d in (os.environ.get('SCENARIO_DIR', ''), '~/Documents/A4VAI-SITL/A4VAI-Unreal',
              '~/Desktop/00_Project/00_A4VAI/00_Development/00_Simulator/02_InhouseSim/00_Unreal/00_Map'):
        if d and os.path.isdir(os.path.expanduser(d)):
            return os.path.expanduser(d)
    return os.path.expanduser('~/Documents/A4VAI-SITL/A4VAI-Unreal')


DEF_LAYOUT = os.path.join(_scenario_dir(), 'layout', 'scenario_layout.json')


def px4_offset(run_dir):
    """(PX4 times [s], offsets [s]) from px4_sitl.ulg's timesync_status: PX4 time = computer-clock time + offset
    (uXRCE-DDS timesync; in SITL, PX4's clock is the Gazebo sim time). None without the log or the topic."""
    ulg = os.path.join(run_dir, 'px4_sitl.ulg')
    if not os.path.exists(ulg):
        return None
    try:
        from pyulog import ULog
        ts = ULog(ulg, ['timesync_status']).get_dataset('timesync_status').data
    except Exception:  # noqa: BLE001 - old logs without the topic
        return None
    return ts['timestamp'] / 1e6, ts['estimated_offset'] / 1e6


def to_px4_time(t, off):
    """track times t -> PX4 times: unchanged when they already are (< 1e9 s), else computer-clock times mapped
    through the offset; None when that is needed but unknown"""
    if t[0] < 1e9:
        return t
    if off is None:
        return None
    return t + np.interp(t + off[1][0], off[0], off[1])


def cart_starts(run_dir):
    """{moving obstacle: Gazebo sim time [s] it started} from the run's gzserver log (scenario_logs/gzserver.log, where
    the moving-obstacle plugin prints '[<name>_motion] started at t=...')"""
    p = os.path.join(run_dir, 'scenario_logs', 'gzserver.log')
    if not os.path.exists(p):
        return {}
    txt = re.sub(r'\x1b\[[0-9;]*m', '', open(p, errors='replace').read())
    return {name: float(t0) for name, t0 in re.findall(r'\[(\S+?)_motion\] started at t=([0-9.]+)', txt)}


def moving_track(mo, run_dir, t, X, Y):
    """(cx, cy, how): where a moving obstacle was at the track times t (vehicle at X, Y). Exact when the gzserver log
    has its start and PX4's clock is known; else its plugin rule (a layout with a trigger: the first time the vehicle
    came within trigger_distance of trigger_point); else the simulation start (no trigger: it moves from the start).
    cx, cy are None when the start cannot be told."""
    s0, s1 = np.array(mo['start'], float), np.array(mo['end'], float)
    L = float(np.linalg.norm(s1 - s0))
    clock = to_px4_time(t, px4_offset(run_dir))
    t0 = cart_starts(run_dir).get(mo['name'])
    if clock is not None and t0 is not None:
        el, how = clock - t0, f'started at sim t={t0:.2f} s (gzserver log)'
    elif mo.get('trigger_point') is not None:
        near = np.hypot(X - mo['trigger_point'][0], Y - mo['trigger_point'][1]) <= mo['trigger_distance']
        if not near.any():
            return np.full(len(t), s0[0]), np.full(len(t), s0[1]), 'never triggered (stayed at its start)'
        el, how = t - t[np.argmax(near)], 'trigger rule replayed'
    elif clock is not None:
        el, how = clock, 'moving since the simulation start (no start in a gzserver log)'
    else:
        return None, None, 'start unknown (no gzserver log, no PX4 clock)'
    tt = np.maximum(el, 0.0) * mo['speed_mps']
    ph = np.mod(tt, 2 * L) if mo.get('loop', True) else np.minimum(tt, L)
    f = np.where(ph <= L, ph, 2 * L - ph) / L if L > 0 else np.zeros(len(t))
    return s0[0] + f * (s1[0] - s0[0]), s0[1] + f * (s1[1] - s0[1]), how


def box_dist(px, py, cx, cy, sx, sy, yaw_deg):
    """horizontal distance from points to a yawed rectangle (0 inside)"""
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    dx, dy = px - cx, py - cy
    lx, ly = c * dx + s * dy, -s * dx + c * dy
    ex = np.maximum(np.abs(lx) - sx / 2.0, 0.0)
    ey = np.maximum(np.abs(ly) - sy / 2.0, 0.0)
    return np.hypot(ex, ey)


def load_track(run_dir):
    """PX4's local position over the run: (t [s], north, east, down, armed times). From the flight_recorder's px4.csv
    + status.csv when it ran (epoch time, so airsim_odom.csv can be matched), else from the PX4 log px4_sitl.ulg
    (SILS / PILS; its own time base), else from the avoidance node's noclimb_ca.csv (20 Hz, epoch time). The recorder
    is off by default since 2026-11-01 (ONBOARD_RECORDER)."""
    px4, st = os.path.join(run_dir, 'px4.csv'), os.path.join(run_dir, 'status.csv')
    if os.path.exists(px4) and os.path.exists(st):
        p = np.genfromtxt(px4, delimiter=',', names=True)
        s = np.genfromtxt(st, delimiter=',', names=True)
        return p['t'], p['x'], p['y'], p['z'], s['t'][s['arming'] == 2], np.hypot(p['vx'], p['vy'])
    ulg = os.path.join(run_dir, 'px4_sitl.ulg')
    if os.path.exists(ulg):
        from pyulog import ULog
        u = ULog(ulg, ['vehicle_local_position', 'vehicle_status'])
        lp = u.get_dataset('vehicle_local_position').data
        t = lp['timestamp'] / 1e6
        armed = np.array([])
        try:
            vs = u.get_dataset('vehicle_status').data
            armed = vs['timestamp'][vs['arming_state'] == 2] / 1e6
        except (KeyError, ValueError):
            pass
        return t, lp['x'], lp['y'], lp['z'], armed, np.hypot(lp['vx'], lp['vy'])
    ca = np.genfromtxt(os.path.join(run_dir, 'noclimb_ca.csv'), delimiter=',', names=True)
    # the node logs ENU: x east, y north, z up
    return ca['t'], ca['y'], ca['x'], -ca['z'], ca['t'][:1], np.hypot(ca['vx'], ca['vy'])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run_dir')
    ap.add_argument('--layout', default=DEF_LAYOUT)
    # vehicle geometry (prop-tip radius, body below / above the centre, centre height at rest): by default the
    # run's airframe (scenario_run.txt field 9) - X8 0.77 / 0.44 / 0.09 / 0.44, X500 0.386 / 0.23 / 0.07 / 0.23.
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
    run = open(os.path.join(a.run_dir, 'scenario_run.txt')).read().split()
    frm, goal_wp = run[:2]
    airframe = run[8] if len(run) > 8 else 'x8'
    geo = {'x500': (0.386, 0.23, 0.07, 0.23)}.get(airframe, (0.77, 0.44, 0.09, 0.44))
    for name, value in zip(('radius', 'below', 'above', 'rest'), geo):
        if getattr(a, name) is None:
            setattr(a, name, value)
    wps = lay['waypoints']
    fx, fy = wps[frm] if frm in wps else map(float, frm.split(','))       # name or X,Y
    gx, gy = wps[goal_wp] if goal_wp in wps else map(float, goal_wp.split(','))
    t, pn, pe, pd, armed, v = load_track(a.run_dir)
    X, Y, H = fx + pe, fy - pn, a.rest - pd                # PX4 local NED -> track frame, height AGL
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

    # moving obstacles where they were (moving_track); start unknown: anywhere on their travel
    notes = []
    for mo in lay.get('moving_obstacles', []):
        sx = max(pt['size'][0] for pt in mo['parts'])
        sy = max(pt['size'][1] for pt in mo['parts'])
        sz = max(pt['offset'][2] + pt['size'][2] for pt in mo['parts'])
        cx, cy, how = moving_track(mo, a.run_dir, t, X, Y)
        if cx is None:
            s0, s1 = np.array(mo['start'], float), np.array(mo['end'], float)
            mid = (s0 + s1) / 2
            d = box_dist(X, Y, mid[0], mid[1], abs(s1[0] - s0[0]) + sx, abs(s1[1] - s0[1]) + sy, 0.0)
            how += ' - distance to its whole travel'
        else:
            d = box_dist(X, Y, cx, cy, sx, sy, 0.0)
        rows.append((mo['name'] + ' (moving)', d, sz))
        notes.append(f'{mo["name"]}: {how}')

    print(f'{os.path.basename(a.run_dir.rstrip("/"))}: {frm} -> {goal_wp}, slab {-a.below:+.2f}/{a.above:+.2f} m, '
          f'prop radius {a.radius} m; {src}')
    for n in notes:
        print(f'  moving obstacle {n}')
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
