#!/usr/bin/env python3
"""Closed-loop check of scripts/ros2/apps/super_speed_guard.py (GuardCore) on the C-Track layout,
without the simulator.

lidar: the AirSim one of the X8 (0.3 m above the centre, 32 channels -7..+52 deg, 192 azimuths
= 1.875 deg, 10 Hz, 40 m), ray-cast against the layout's boxes / cylinders and the RC cart
(its plugin rule: starts when the vehicle comes within trigger_distance of trigger_point,
shuttles start <-> end). Vehicle: on a straight path at a fixed height; its speed follows the
guarded command after a delay with acceleration limits (PathFollowing guid 0 in ctrack_sp_c/d:
a /desired_speed drop shows after ~0.5 s, then ~3 m/s^2).

  crossing  - fly START -> GOAL (default the 123_h line 54,29 -> 20,30, where the cart crosses the
              path as the vehicle arrives); prints the cart's moving verdict, when the guard
              braked, the closest approach to the panel, where it stopped and whether it went on.
  sweep     - crossing from a range of start points / speeds (timing phases), one line each
  replay    - a recorded run (test_runs/NAME, AirSim truth): the guard on its actual track with the
              static layout + the replayed cart; counts the ticks the guard would have limited
              (false-positive check; the path ahead = the recorded track)
  dump      - a recorded run's real guard inputs (super_speed_guard_scans.pkl), open loop: the
              moving verdicts and the brake / release timeline the current code would give
  eval      - recorded runs' guard inputs: how soon the cart was called moving, and anything else
              called moving (false positives) - run it over every recorded run after a change
  leaks     - recorded runs: the cart's returns that would reach SUPER after the guard called the
              cart moving (a trail in SUPER's map); exit status 1 if any

Run these inside the ROS image before flying a change (numpy 2.2 there, 1.26 on the host: a
removed ndarray method passed here and blinded the guard in flight, ctrack_123_n), together with
scripts/dev/speed_guard_node_test.py. The Desktop layout path does not bind-mount into docker -
copy the layout next to the run dirs and pass --layout:
  cp $LAYOUT ~/Documents/A4VAI-SITL/ROS2/ctrack_layout.json
  docker run --rm -e HOME=$HOME -v $HOME/PX4-SITL-Runner:$HOME/PX4-SITL-Runner:ro \
      -v $HOME/Documents/A4VAI-SITL/ROS2:$HOME/Documents/A4VAI-SITL/ROS2 --entrypoint bash \
      jociiiii/a4vai:inhousesim-v0.9 -c "cd $HOME/PX4-SITL-Runner; python3 scripts/dev/speed_guard_sim.py eval \
      $HOME/Documents/A4VAI-SITL/ROS2/test_runs/ctrack_123_[i-r] --layout $HOME/Documents/A4VAI-SITL/ROS2/ctrack_layout.json"

usage: speed_guard_sim.py crossing [--start 54,29] [--goal 20,30] [--speed 5] [--height 1.6]
       speed_guard_sim.py sweep
       speed_guard_sim.py replay RUN_DIR
       speed_guard_sim.py dump RUN_DIR
       speed_guard_sim.py eval RUN_DIR...
       speed_guard_sim.py leaks RUN_DIR...
"""
import argparse
import math
import os
import sys
import warnings

import numpy as np

warnings.filterwarnings('ignore')
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'ros2', 'apps'))
from super_speed_guard import GuardCore  # noqa: E402

def _ctrack_dir():
    """CTRACK_DIR, else the installed A4VAI-Unreal (scripts/install.sh), else the developers' checkout"""
    for d in (os.environ.get('CTRACK_DIR', ''), '~/Documents/A4VAI-SITL/A4VAI-Unreal',
              '~/Desktop/00_Project/00_A4VAI/00_Development/00_Simulator/02_InhouseSim/Unreal'):
        if d and os.path.isdir(os.path.expanduser(d)):
            return os.path.expanduser(d)
    return os.path.expanduser('~/Documents/A4VAI-SITL/A4VAI-Unreal')


LAYOUT = os.path.join(_ctrack_dir(), 'layout', 'ctrack_layout.json')
AZ = np.radians(np.arange(192) * 360.0 / 192)
EL = np.radians(-7.0 + np.arange(32) * 59.0 / 31)
DIRS = np.stack([np.cos(EL)[:, None] * np.cos(AZ)[None], np.cos(EL)[:, None] * np.sin(AZ)[None],
                 np.sin(EL)[:, None] * np.ones_like(AZ)[None]], axis=-1).reshape(-1, 3)
PROP_R = 0.77


def load(layout):
    import json
    lay = json.load(open(layout))
    static = []
    for o in lay['obstacles']:
        sx, sy, sz = o['size']
        static.append((o['shape'], o['pos'][0], o['pos'][1], 0.0, sx, sy, sz, o.get('yaw_deg', 0.0)))
    return lay, static


def cart_pose(mo, t, t_trig):
    s0, s1 = np.array(mo['start'], float), np.array(mo['end'], float)
    if t_trig is None:
        return s0
    L = float(np.linalg.norm(s1 - s0))
    ph = ((t - t_trig) * mo['speed_mps']) % (2 * L)
    f = (ph if ph <= L else 2 * L - ph) / L
    return s0 + f * (s1 - s0)


def cart_boxes(mo, c):
    return [('box', c[0] + p['offset'][0], c[1] + p['offset'][1], p['offset'][2], *p['size'], 0.0) for p in mo['parts']]


def raycast(o, objs, rng=40.0):
    d = DIRS
    best = np.full(len(d), rng)
    for shape, cx, cy, z0, sx, sy, sz, yaw in objs:
        if shape == 'cylinder':
            r = sx / 2.0
            ox, oy = o[0] - cx, o[1] - cy
            A = d[:, 0] ** 2 + d[:, 1] ** 2
            B = 2 * (ox * d[:, 0] + oy * d[:, 1])
            C = ox * ox + oy * oy - r * r
            disc = B * B - 4 * A * C
            ok = (disc >= 0) & (A > 1e-12)
            t1 = np.where(ok, (-B - np.sqrt(np.maximum(disc, 0))) / (2 * np.maximum(A, 1e-12)), np.inf)
            z = o[2] + t1 * d[:, 2]
            hit = ok & (t1 > 0) & (z >= z0) & (z <= z0 + sz)
        else:
            c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
            ox, oy = o[0] - cx, o[1] - cy
            lo_ = np.array([c * ox + s * oy, -s * ox + c * oy, o[2]])
            ld = np.stack([c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1], d[:, 2]], axis=1)
            lo = np.array([-sx / 2, -sy / 2, z0]); hi = np.array([sx / 2, sy / 2, z0 + sz])
            with np.errstate(divide='ignore', invalid='ignore'):
                ta = (lo[None] - lo_[None]) / ld
                tb = (hi[None] - lo_[None]) / ld
            tmin = np.nanmax(np.minimum(ta, tb), axis=1)
            tmax = np.nanmin(np.maximum(ta, tb), axis=1)
            t1 = np.where(tmin > 0, tmin, tmax)
            hit = (tmin <= tmax) & (tmax > 0)
        best = np.where(hit & (t1 < best), t1, best)
    m = best < rng
    return o[None] + best[m, None] * d[m]


def panel_margin(mo, c, x, y):
    """prop-tip margin to the cart's parts that reach the vehicle's height (panel)"""
    m = np.inf
    for p in mo['parts']:
        if p['offset'][2] + p['size'][2] < 1.0:       # body under the vehicle's slab
            continue
        px, py = c[0] + p['offset'][0], c[1] + p['offset'][1]
        ex = max(abs(x - px) - p['size'][0] / 2, 0.0)
        ey = max(abs(y - py) - p['size'][1] / 2, 0.0)
        m = min(m, math.hypot(ex, ey) - PROP_R)
    return m


def crossing(start, goal, speed, height, lay, static, quiet=False, delay=0.5, brake=2.7, t_end=30.0):
    mo = lay['moving_obstacles'][0]
    S, G = np.array(start, float), np.array(goal, float)
    u = (G - S) / np.linalg.norm(G - S)
    path = S[None] + np.arange(0, np.linalg.norm(G - S) + 0.5, 0.5)[:, None] * u[None]
    g = GuardCore(nominal_speed=speed)
    g.set_path(path)
    dt = 0.02
    n_delay = int(round(delay / dt))
    hist_cmd = [0.0] * (n_delay + 1)
    s = 0.0; v = 0.0; t_trig = None; lim = None; cmd = 0.0
    log = {'moving': None, 'brake': None, 'min_margin': np.inf, 'min_at': None, 'stop': None, 'passed': None}
    for k in range(int(t_end / dt)):
        t = k * dt
        pos2 = S + s * u
        if t_trig is None and math.hypot(pos2[0] - mo['trigger_point'][0], pos2[1] - mo['trigger_point'][1]) <= mo['trigger_distance']:
            t_trig = t
        c = cart_pose(mo, t, t_trig)
        if k % 5 == 0:                                   # 10 Hz scans
            pts = raycast(np.array([pos2[0], pos2[1], height + 0.3]), static + cart_boxes(mo, c))
            g.add_scan(t, pts, np.array([pos2[0], pos2[1], height]))
            for e in g.events:
                if 'moving' in e and log['moving'] is None and t_trig is not None:
                    log['moving'] = (t - t_trig, e)
            g.events.clear()
        raw = min(speed, 2.5 * t)                        # the bridge ramps up after the hover
        if k % 2 == 1 or lim is None:                    # 25 Hz speed ticks
            cmd, lim = g.command(t, np.array([pos2[0], pos2[1], height]), v, raw)
        if lim and cmd < raw and log['brake'] is None and t_trig is not None:
            log['brake'] = (t - t_trig, lim, cmd)
        hist_cmd.append(cmd)
        target = hist_cmd[-1 - n_delay]
        a = max(min(2.5 * (target - v), 2.2), -brake)
        v = max(v + a * dt, 0.0)
        s += v * dt
        mg = panel_margin(mo, c, pos2[0], pos2[1])
        if mg < log['min_margin']:
            log['min_margin'], log['min_at'] = mg, (round(t - (t_trig or 0), 2), round(pos2[0], 2), round(pos2[1], 2), round(v, 2))
        if log['brake'] is not None and log['stop'] is None and v < 0.05 and t > 3:
            log['stop'] = (t - t_trig, round(pos2[0], 2), round(pos2[1], 2))
        if pos2[0] < mo['start'][0] - 2.0 and log['passed'] is None:
            log['passed'] = t - (t_trig or 0)
        if s >= np.linalg.norm(G - S) - 0.5:
            break
    if not quiet:
        print(f'start {start} -> goal {goal}, {speed} m/s, height {height} m, PF lag {delay} s / {brake} m/s^2')
        print(f'  cart triggered at t={t_trig:.2f} s')
        print(f'  moving verdict: ' + (f't_trig+{log["moving"][0]:.2f} s  {log["moving"][1]}' if log['moving'] else 'never'))
        print(f'  first brake:    ' + (f't_trig+{log["brake"][0]:.2f} s, command {log["brake"][2]:.2f} m/s ({log["brake"][1][2]}, '
                                       f'{log["brake"][1][1]:.1f} m ahead, {log["brake"][1][3]:.2f} m off)' if log['brake'] else 'never'))
        print(f'  stopped:        ' + (f't_trig+{log["stop"][0]:.2f} s at ({log["stop"][1]}, {log["stop"][2]})' if log['stop'] else 'no'))
        print(f'  closest to the panel: margin {log["min_margin"]:+.2f} m at (t_trig+s, x, y, v) {log["min_at"]}')
        print(f'  passed the cart line: ' + (f't_trig+{log["passed"]:.1f} s' if log['passed'] else 'no (within the sim time)'))
    return log


def replay(run_dir, lay, static):
    mo = lay['moving_obstacles'][0]
    frm = open(os.path.join(run_dir, 'ctrack_run.txt')).read().split()[0]
    a = np.genfromtxt(os.path.join(run_dir, 'airsim_odom.csv'), delimiter=',', names=True)
    st = np.genfromtxt(os.path.join(run_dir, 'status.csv'), delimiter=',', names=True)
    t_arm = st['t'][np.argmax(st['arming'] == 2)]
    X, Y, H = a['x'], -a['y'], a['z']
    t = a['t']
    fly = (t > t_arm) & (H > 1.0)
    g = GuardCore()
    t_trig = None
    n_lim = {'static': 0, 'moving': 0}
    first = {}
    ts = np.arange(t[fly][0], t[fly][-1], 0.1)
    for tt in ts:
        x, y, h = (float(np.interp(tt, t, q)) for q in (X, Y, H))
        if t_trig is None and math.hypot(x - mo['trigger_point'][0], y - mo['trigger_point'][1]) <= mo['trigger_distance']:
            t_trig = tt
        c = cart_pose(mo, tt, t_trig)
        # path ahead: the recorded track from here on, 25 m
        k = np.searchsorted(t, tt)
        ahead = np.column_stack([X[k:], Y[k:]])
        arc = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(ahead, axis=0).T))])
        ahead = ahead[arc <= 25.0]
        if len(ahead) < 2 or arc[min(len(arc) - 1, len(ahead) - 1)] < 1.0:
            continue
        g.set_path(ahead[::max(1, len(ahead) // 60)])
        pts = raycast(np.array([x, y, h + 0.3]), static + cart_boxes(mo, c))
        g.add_scan(tt, pts, np.array([x, y, h]))
        for e in g.events:
            print(f'  t+{tt - t[fly][0]:5.1f}s {e}')
        g.events.clear()
        v = float(np.hypot(np.interp(tt + 0.05, t, X) - np.interp(tt - 0.05, t, X), np.interp(tt + 0.05, t, Y) - np.interp(tt - 0.05, t, Y)) / 0.1)
        lim = g.limit(tt, np.array([x, y, h]), v, v)
        if lim is not None and lim[0] < max(v, 0.5):
            kind = lim[2].split()[0]
            n_lim[kind] += 1
            if kind not in first:
                first[kind] = f't+{tt - t[fly][0]:.1f}s at ({x:.1f},{y:.1f}) {lim[2]} block {lim[1]:.1f} m ahead {lim[3]:.2f} m off, limit {lim[0]:.2f} < v {v:.2f}'
    print(f'{os.path.basename(run_dir.rstrip("/"))} ({frm}): guard would have braked on {n_lim["static"]} static / '
          f'{n_lim["moving"]} moving ticks (of {len(ts)} at 10 Hz)')
    for k_, s_ in first.items():
        print(f'  first {k_}: {s_}')


def dump_replay(run_dir, raw=5.0):
    """the guard's recorded inputs (super_speed_guard_scans.pkl) through GuardCore, open loop:
    the vehicle moves as recorded, so this shows classification and block / release timing"""
    import pickle
    recs = []
    with open(os.path.join(run_dir, 'super_speed_guard_scans.pkl'), 'rb') as f:
        while True:
            try:
                recs.append(pickle.load(f))
            except EOFError:
                break
    g = GuardCore()
    t0 = recs[0]['t']
    prev = None
    state = None
    for k, r in enumerate(recs):
        if r['path'] is not None:
            g.set_path(r['path'])
        g.add_scan(r['t'], r['xyz'].astype(float), r['pos'])
        for e in g.events:
            print(f'  t+{r["t"] - t0:5.1f}s {e}')
        g.events.clear()
        v = 0.0 if prev is None else float(np.hypot(*(r['pos'][:2] - prev['pos'][:2])) / max(r['t'] - prev['t'], 1e-3))
        prev = r
        out, lim = g.command(r['t'], r['pos'], v, raw)
        st = ('BRAKE' if g.braking else ('cap' if out < raw else 'free'), None if lim is None else lim[2])
        if st != state:
            print(f'  t+{r["t"] - t0:5.1f}s v {v:4.2f} -> {out:4.2f} {st[0]:5s} ' +
                  ('' if lim is None else f'{lim[2]} block {lim[1]:.1f} m ahead, {lim[3]:.2f} m off'))
            state = st
    print(f'{os.path.basename(run_dir.rstrip("/"))}: {len(recs)} recorded scans')


def eval_runs(run_dirs, lay):
    """the guard's recorded inputs of each run through GuardCore: when the RC cart (replayed from
    its plugin rule, which matches the lidar within 0.2 m) was first called moving after it set
    off, and every other object called moving (false positives)"""
    import pickle
    mo = lay['moving_obstacles'][0]
    for run_dir in run_dirs:
        frm = open(os.path.join(run_dir, 'ctrack_run.txt')).read().split()[0]
        fx, fy = lay['waypoints'][frm] if frm in lay['waypoints'] else map(float, frm.split(','))
        recs = []
        with open(os.path.join(run_dir, 'super_speed_guard_scans.pkl'), 'rb') as f:
            while True:
                try:
                    recs.append(pickle.load(f))
                except EOFError:
                    break
        g = np.genfromtxt(os.path.join(run_dir, 'airsim_odom.csv'), delimiter=',', names=True)
        X, Y = g['x'], -g['y']
        near = np.hypot(X - mo['trigger_point'][0], Y - mo['trigger_point'][1]) <= mo['trigger_distance']
        t_trig = g['t'][np.argmax(near)] if near.any() else None
        core = GuardCore()
        cart_first, false = None, []
        for r in recs:
            if r['path'] is not None:
                core.set_path(r['path'])
            core.add_scan(r['t'], r['xyz'].astype(float), r['pos'])
            for e in core.events:
                x, y = (float(v) for v in e.split('at (')[1].split(')')[0].split(','))
                Xe, Ye = fx + x, fy - y
                c = cart_pose(mo, r['t'], t_trig)
                if t_trig is not None and r['t'] >= t_trig and abs(Xe - c[0]) < 1.5 and abs(Ye - c[1]) < 2.0:
                    cart_first = r['t'] - t_trig if cart_first is None else cart_first
                else:
                    false.append(f'({Xe:.1f},{Ye:.1f}) t+{r["t"] - recs[0]["t"]:.1f}')
            core.events.clear()
        print(f'{os.path.basename(run_dir.rstrip("/"))}: cart moving ' +
              (f'{cart_first:.2f} s after it set off' if cart_first is not None else 'never') +
              f'; other objects called moving: {len(false)} {false[:4]}')


def leaks(run_dirs, lay):
    """what SUPER would have mapped of the RC cart: each recorded run's guard inputs through
    GuardCore, /cloud_static rebuilt (moving_mask, the 25 m range and the height floor of a held
    flight: 0.5 m under the centre) and the cart's returns in it counted - before the guard first
    called the cart moving (it is parked there: fine) and after (a trail SUPER never clears:
    ctrack_s900_f / g). Exit status 1 when any run leaks after the verdict."""
    import pickle
    mo = lay['moving_obstacles'][0]
    bad = 0
    for run_dir in run_dirs:
        f = open(os.path.join(run_dir, 'ctrack_run.txt')).read().split()
        frm, alt = f[0], float(f[6])
        fx, fy = lay['waypoints'][frm] if frm in lay['waypoints'] else map(float, frm.split(','))
        min_z = alt - 0.44 - 0.5
        recs = []
        with open(os.path.join(run_dir, 'super_speed_guard_scans.pkl'), 'rb') as fh:
            while True:
                try:
                    recs.append(pickle.load(fh))
                except EOFError:
                    break
        g = np.genfromtxt(os.path.join(run_dir, 'airsim_odom.csv'), delimiter=',', names=True)
        X, Y = g['x'], -g['y']
        near = np.hypot(X - mo['trigger_point'][0], Y - mo['trigger_point'][1]) <= mo['trigger_distance']
        t_trig = g['t'][np.argmax(near)] if near.any() else None
        core = GuardCore()
        verdict, before, after, scans_after = None, 0, 0, 0
        for r in recs:
            if r['path'] is not None:
                core.set_path(r['path'])
            xyz = r['xyz'].astype(float)
            core.add_scan(r['t'], xyz, r['pos'])
            for e in core.events:
                if t_trig is not None and r['t'] >= t_trig and verdict is None:
                    x, y = (float(v) for v in e.split('at (')[1].split(')')[0].split(','))
                    c = cart_pose(mo, r['t'], t_trig)
                    if abs(fx + x - c[0]) < 1.5 and abs(fy - y - c[1]) < 2.0:
                        verdict = r['t']
            core.events.clear()
            if t_trig is None or r['t'] < t_trig:
                continue
            keep = ~core.moving_mask(xyz) & (xyz[:, 2] >= min_z) & (np.linalg.norm(xyz - r['pos'], axis=1) <= 25.0)
            c = cart_pose(mo, r['t'], t_trig)
            on = keep & (np.abs(fx + xyz[:, 0] - c[0]) < 0.45) & (np.abs(fy - xyz[:, 1] - c[1]) < 1.0)
            if verdict is None:
                before += int(on.sum())
            else:
                after += int(on.sum())
                scans_after += int(on.any())
        bad += after > 0
        print(f'{os.path.basename(run_dir.rstrip("/"))}: cart called moving ' +
              (f'{verdict - t_trig:.2f} s after it set off' if verdict else 'never') +
              f'; cart returns to SUPER before {before}, after {after} (in {scans_after} scans)')
    sys.exit(1 if bad else 0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('mode', choices=['crossing', 'sweep', 'replay', 'dump', 'eval', 'leaks'])
    ap.add_argument('run_dir', nargs='*')
    ap.add_argument('--start', default='54,29')
    ap.add_argument('--goal', default='20,30')
    ap.add_argument('--speed', type=float, default=5.0)
    ap.add_argument('--height', type=float, default=1.6)
    ap.add_argument('--layout', default=LAYOUT)
    a = ap.parse_args()
    lay, static = load(a.layout)
    if a.mode == 'crossing':
        crossing([float(v) for v in a.start.split(',')], [float(v) for v in a.goal.split(',')],
                 a.speed, a.height, lay, static)
    elif a.mode == 'sweep':
        print('start      speed  moving@  brake@   min margin  stopped at        passed')
        # lines between the pole rows (C5 (46,26) / C6 (46,32), C3 (56,29) behind the starts): the
        # straight path does not avoid static obstacles; the start sets the cart's timing
        for speed in (4.0, 5.0):
            for sx in (47.0, 49.0, 51.0, 53.0, 55.0):
                for sy in (28.0, 29.0, 30.0):
                    L = crossing([sx, sy], [20.0, sy], speed, 1.6, lay, static, quiet=True)
                    mv = f'{L["moving"][0]:5.2f}' if L['moving'] else '  -  '
                    br = f'{L["brake"][0]:5.2f}' if L['brake'] else '  -  '
                    stp = f'({L["stop"][1]:5.1f},{L["stop"][2]:4.1f})' if L['stop'] else '      -     '
                    ps = f'{L["passed"]:5.1f}' if L['passed'] else '  no '
                    print(f'{sx:4.0f},{sy:4.1f}  {speed:4.1f}  {mv}   {br}   {L["min_margin"]:+6.2f}     {stp}  {ps}')
    elif a.mode == 'dump':
        dump_replay(a.run_dir[0])
    elif a.mode == 'eval':
        eval_runs(a.run_dir, lay)
    elif a.mode == 'leaks':
        leaks(a.run_dir, lay)
    else:
        replay(a.run_dir[0], lay, static)


if __name__ == '__main__':
    main()
