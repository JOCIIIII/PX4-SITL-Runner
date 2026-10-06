#!/usr/bin/env python3
"""Summarise a fly_test.sh run.

usage: analyze_flight.py RUN_DIR [--goal X,Y,Z] [--ref MAP.npz ...] [--quiet]

Clearance is measured from the flown trajectory (/odom, ENU) to obstacle reference
points (voxel centres from cloud_map.npz of slow survey runs; points below
--ground-z are treated as ground and checked separately as altitude).
"""
import argparse
import glob
import os
import re

import numpy as np
from scipy.spatial import cKDTree


def load(path):
    if not os.path.exists(path) or os.path.getsize(path) == 0:
        return None
    try:
        a = np.genfromtxt(path, delimiter=',', names=True, invalid_raise=False)
    except Exception:
        return None
    return a if a.size > 1 else None


def air_offset(run):
    lg = os.path.join(run, 'px4_lidar_to_odom_cloud.log')
    if os.path.exists(lg):
        m = re.search(r'AirSim->PX4 offset \[([-\d.e]+), ([-\d.e]+), ([-\d.e]+)\]', open(lg, errors='ignore').read())
        if m:
            return np.array([float(v) for v in m.groups()])
    # adapter never calibrated: estimate from the recorded streams (PX4 /odom vs AirSim
    # pose at standstill/hover, matched by receive time)
    od, ao = load(os.path.join(run, 'odom.csv')), load(os.path.join(run, 'airsim_odom.csv'))
    if od is None or ao is None:
        return None
    still = np.linalg.norm(np.stack([od['vx'], od['vy'], od['vz']], 1), axis=1) < 0.1
    if still.sum() < 50:
        return None
    t = od['t'][still]
    P = np.stack([od['x'], od['y'], od['z']], 1)[still]
    A = np.stack([np.interp(t, ao['t'], ao[k]) for k in ('x', 'y', 'z')], 1)
    ok = (t > ao['t'][0]) & (t < ao['t'][-1])
    return np.median(P[ok] - A[ok], axis=0) if ok.sum() >= 50 else None


MAX_OBS_RANGE = 0.0   # >0: keep only voxels observed from within this range (needs r_min)


def truth_map(run, lidar_z=0.3, phantom_r=0.35):
    # cloud_map.npz is in the run's PX4 frame = AirSim(Gazebo) truth + offset.
    # Before the adapter got min_range, no-return rays (AirSim reports them at the
    # sensor origin) were published at the lidar position of the scan pose; drop the
    # voxels lying on the run's own lidar track so they do not count as obstacles.
    f, T = os.path.join(run, 'cloud_map.npz'), air_offset(run)
    if not os.path.exists(f) or T is None:
        return None
    npz = np.load(f)
    m = npz['xyz'] - T.astype(np.float32)
    if MAX_OBS_RANGE > 0.0 and 'r_min' in npz.files:
        m = m[npz['r_min'] <= MAX_OBS_RANGE]
    ao = load(os.path.join(run, 'airsim_odom.csv'))
    if ao is not None and m.size:
        track = np.stack([ao['x'], ao['y'], ao['z'] + lidar_z], 1)
        d, _ = cKDTree(track).query(m, distance_upper_bound=phantom_r)
        m = m[~np.isfinite(d)]
    return m


def consensus_map(runs, r=0.3, k=2):
    # keep a voxel only if at least k runs (itself included) saw something within r:
    # one-off points (mis-registered scans during hard manoeuvres) would otherwise
    # show up as phantom near-misses for every later run flying through that spot
    maps = [m for m in (truth_map(x) for x in runs) if m is not None and m.size]
    if not maps:
        return None
    trees = [cKDTree(m) for m in maps]
    keep = []
    for i, m in enumerate(maps):
        votes = np.ones(len(m), dtype=int)
        for j, t in enumerate(trees):
            if j != i:
                d, _ = t.query(m, distance_upper_bound=r)
                votes += np.isfinite(d)
        keep.append(m[votes >= k])
    return np.concatenate(keep)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run')
    ap.add_argument('--goal')
    ap.add_argument('--ref', nargs='*', default=[], help='run dirs whose maps form the reference')
    ap.add_argument('--ground-z', type=float, default=0.5)
    ap.add_argument('--drone-r', type=float, default=0.35)
    ap.add_argument('--max-obs-range', type=float, default=0.0,
                    help='only obstacles seen from within this range [m] (maps with r_min); '
                         'far-range registration error places phantom voxels near the path')
    ap.add_argument('--phase', choices=('all', 'flight', 'landing'), default='flight',
                    help='clearance over the offboard flight (default), the AUTO_LAND descent, or both')
    a = ap.parse_args()
    global MAX_OBS_RANGE
    MAX_OBS_RANGE = a.max_obs_range
    d = a.run
    res = open(os.path.join(d, 'result.txt')).read().strip() if os.path.exists(os.path.join(d, 'result.txt')) else '?'
    print(f'== {os.path.basename(d.rstrip("/"))}  result={res}')

    st = load(os.path.join(d, 'status.csv'))
    od = load(os.path.join(d, 'odom.csv'))
    if od is None:
        print('  no odom'); return
    t_arm = t_disarm = None
    if st is not None:
        arm = st[st['arming'] == 2]
        if arm.size:
            t_arm = float(np.atleast_1d(arm['t'])[0])
            dis = st[(st['arming'] == 1) & (st['t'] > t_arm)]
            if dis.size:
                t_disarm = float(np.atleast_1d(dis['t'])[0])
    t = od['t']
    P = np.stack([od['x'], od['y'], od['z']], 1)
    V = np.stack([od['vx'], od['vy'], od['vz']], 1)
    vh = np.linalg.norm(V[:, :2], axis=1)
    armed = (t >= (t_arm or t[0])) & (t <= (t_disarm or t[-1]))
    cruise = armed & (P[:, 2] > 2.0)
    if not cruise.any():
        print('  never left the ground'); return
    tc = t[cruise]
    Pc = P[cruise]
    seg = np.linalg.norm(np.diff(Pc, axis=0), axis=1)
    dist = float(seg[seg < 5].sum())
    dur = float(tc[-1] - tc[0])
    print(f'  cruise {dur:5.1f}s  path {dist:6.1f}m  avg {dist / max(dur, 1e-3):5.2f}m/s  '
          f'max {vh[cruise].max():5.2f}  p90 {np.percentile(vh[cruise], 90):5.2f}  '
          f'>=6m/s {100 * np.mean(vh[cruise] >= 6):4.1f}%  >8m/s {100 * np.mean(vh[cruise] > 8):4.1f}%  '
          f'alt[min {Pc[:, 2].min():.1f} max {Pc[:, 2].max():.1f}]')
    fin = P[armed][-1]
    msg = f'  final ({fin[0]:.1f},{fin[1]:.1f},{fin[2]:.1f})'
    if a.goal:
        g = np.array([float(v) for v in a.goal.split(',')])
        msg += f'  goal err xy {np.linalg.norm(fin[:2] - g[:2]):.1f}m'
        # progress along straight line start->goal
        prog = np.dot(Pc[:, :2] - Pc[0, :2], (g[:2] - Pc[0, :2]) / np.linalg.norm(g[:2] - Pc[0, :2]))
        msg += f'  max progress {prog.max():.1f}/{np.linalg.norm(g[:2] - Pc[0, :2]):.1f}m'
    print(msg)

    li = load(os.path.join(d, 'lidar.csv'))
    if li is not None:
        lt = li['t']
        span = lt[-1] - lt[0]
        nd = li[li['dup'] == 0]
        off = li['t'] - li['stamp']
        lc = (lt >= tc[0]) & (lt <= tc[-1]) & (li['minr'] >= 0)
        minr = li['minr'][lc]
        print(f'  lidar {li.size / span:5.1f}Hz uniq {nd.size / span:5.1f}Hz npts~{np.median(li["npts"]):.0f}  '
              f'recv-stamp off[first {off[0]:.2f} med {np.median(off):.2f} last {off[-1]:.2f}]  '
              f'cruise minr {minr.min() if minr.size else -1:.2f}m p1 {np.percentile(minr, 1) if minr.size else -1:.2f}')

    # clearance in the AirSim (= Gazebo) truth frame: trajectory from the AirSim
    # pose stream, maps converted back with each run's calibrated offset
    ao = load(os.path.join(d, 'airsim_odom.csv'))
    if ao is not None and air_offset(d) is not None:
        _, iu = np.unique(ao['stamp'], return_index=True)
        ao = ao[np.sort(iu)]
        at = ao['t']
        A = np.stack([ao['x'], ao['y'], ao['z']], 1)
        cr = (at >= tc[0]) & (at <= tc[-1]) & (A[:, 2] > a.ground_z + 1.0)
        # landing (PX4 AUTO_LAND, nav_state 18) is reported apart from the avoidance flight
        t_land = None
        if st is not None:
            lnd = st[(st['nav'] == 18) & (st['t'] > (t_arm or 0))]
            if lnd.size:
                t_land = float(np.atleast_1d(lnd['t'])[0])
        if t_land is not None and a.phase == 'flight':
            cr &= at < t_land
        elif t_land is not None and a.phase == 'landing':
            cr &= at >= t_land
        At, Ac = at[cr], A[cr]
        if not Ac.size:
            print(f'  ({a.phase}: no samples)')
            Ac = np.zeros((0, 3))
        if Ac.size:
            print(f'  [{a.phase}] truth alt[min {Ac[:, 2].min():.1f} max {Ac[:, 2].max():.1f}]  offset {np.round(air_offset(d), 2).tolist()}')
        refs = [r for r in a.ref if os.path.abspath(r) != os.path.abspath(d)]
        for label, pts in (('ref', consensus_map(refs) if refs else None), ('own', truth_map(d))):
            if pts is None or not pts.size or not Ac.size:
                continue
            obs = pts[pts[:, 2] > a.ground_z]
            tree = cKDTree(obs)
            dd, ii = tree.query(Ac)
            k = int(np.argmin(dd))
            print(f'  clearance[{label}] min {dd.min():.2f}m at t+{At[k] - tc[0]:.1f}s truth ({Ac[k, 0]:.1f},{Ac[k, 1]:.1f},{Ac[k, 2]:.1f}) '
                  f'obs ({obs[ii[k], 0]:.1f},{obs[ii[k], 1]:.1f},{obs[ii[k], 2]:.1f})  '
                  f'<{a.drone_r}m: {int(np.sum(dd < a.drone_r))}  <0.6m: {int(np.sum(dd < 0.6))}  <1m: {int(np.sum(dd < 1.0))}')
            # detection latency of that obstacle in THIS run's own lidar map
            npz = np.load(os.path.join(d, 'cloud_map.npz'))
            if 't_first' in npz.files:
                own_xyz = npz['xyz'] - air_offset(d).astype(np.float32)
                near = np.linalg.norm(own_xyz - obs[ii[k]], axis=1) < 0.4
                if near.any():
                    tf = float(np.nanmin(npz['t_first'][near]))
                    j = int(np.argmin(abs(At - tf)))
                    print(f'      {label}: obstacle first seen by this run {At[k] - tf:.1f}s before closest approach, '
                          f'from {np.linalg.norm(Ac[j] - obs[ii[k]]):.1f}m')
                else:
                    print(f'      {label}: obstacle never in this run\'s own lidar map')

    # PathFollowing internal log (NED): planned-path altitude vs flown, command saturation
    pf = load(os.path.join(d, 'pf_log.csv'))
    if pf is not None:
        sp = np.hypot(pf['vel_x'], pf['vel_y'])
        m = (-pf['pos_z'] > 2.0) & (sp > 0.5)
        if m.any():
            tilt_lim = np.degrees(np.maximum(abs(pf['cmd_roll'][m]), abs(pf['cmd_pitch'][m]))) > 34.9
            print(f'  pf  target alt[min {-pf["vt_z"][m].max():.1f}]  flown alt[min {-pf["pos_z"][m].max():.1f}]  '
                  f'tilt@limit {100 * tilt_lim.mean():4.1f}%  thrust max {pf["thrust_norm_cmd"][m].max():.2f} '
                  f'>=0.99 {100 * np.mean(pf["thrust_norm_cmd"][m] >= 0.99):4.1f}%')
            xte = pf['cross_track_error'][m]
            msg = f'  pf  xte p50 {np.median(xte):.2f} p90 {np.percentile(xte, 90):.2f} max {xte.max():.2f}'
            # split the margin: how close SUPER's path came (closest point on the PF path)
            # vs how close the vehicle came, both against the reference map (truth frame)
            T = air_offset(d)
            ref = consensus_map(a.ref) if a.ref else None
            if T is not None and ref is not None and ref.size:
                tree = cKDTree(ref[ref[:, 2] > a.ground_z])
                path = np.stack([pf['closest_y'], pf['closest_x'], -pf['closest_z']], 1)[m] - T
                dp, _ = tree.query(path)
                msg += (f'  | path clearance[ref] min {dp.min():.2f} p1 {np.percentile(dp, 1):.2f} '
                        f'<0.4m {100 * np.mean(dp < 0.4):.1f}%')
            print(msg)

    fsm = os.path.join(d, 'super_fsm.log')
    if os.path.exists(fsm):
        txt = open(fsm, errors='ignore').read()
        txt = re.sub(r'\x1b\[[0-9;]*m', '', txt)
        cnt = {k: len(re.findall(k, txt)) for k in
               ['ReplanOnce succeed', 'failed', 'TIME_OUT', 'Unfinished frame', 'No point cloud', 'Emergency',
                'below virtual ground', 'above virtual ceil']}
        print('  fsm ' + '  '.join(f'{k.replace(" virtual", "").replace(" ", "_")}={v}' for k, v in cnt.items()))


if __name__ == '__main__':
    main()
