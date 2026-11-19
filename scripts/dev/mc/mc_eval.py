#!/usr/bin/env python3
"""Monte Carlo evaluation (docs/montecarlo-plan-2026-11-05.md sections 2-3): the path error and the mission success
of one run, and the campaign statistics.

  mc_eval.py run RUN_DIR [--speed V]          one run -> RUN_DIR/mc_result.json (and printed)
  mc_eval.py report PREFIX [--out DIR]        every test_runs/PREFIX_* run with an mc_result.json -> report + figures

Path error (our definition): the position against the course polyline (the take-off point + wp.csv, height above the
take-off point): e = sqrt(horizontal distance^2 + height error^2). Judged on the estimated position
(vehicle_local_position, what PathFollowing flies on and what the field tests measure); also reported on the true
position (PX4 SITL ground truth vehicle_local_position_groundtruth, 50 Hz, re-projected into the estimator's frame),
which adds the SITL GPS / estimator error (~0.3-0.4 m at p95, no RTK) - fields *_true.
Window: FOLLOW (flight_driver) up to the first pass of the end point (the overshoot past it - and in a tailwind the
way back to it - is along the path, not off it). The nearest point is searched only within -5 / +25 m of the progress along the path, so a circle's
second lap or the spiral's next turn never stands in for the right part of the path. E = p95 of e.

Success (all): S1 path complete without ABORT, S2 E <= 1.0 m, S3 max e <= 3.0 m, S4 lowest height in FOLLOW > 0.5 m,
S5 landing <= 3.0 m from the end point, S6 FOLLOW time <= 2 x path length / speed + 10 s. A run that never armed or
left no log is a simulator fault: excluded and re-run (not a failure).
"""
import argparse
import glob
import json
import math
import os
import re
import sys

import numpy as np

RUNS = os.path.expanduser('~/Documents/A4VAI-SITL/ROS2/test_runs')
R_EARTH = 6371000.0
E_LIMIT, E_MAX, H_MIN, LAND_MAX = 1.0, 3.0, 0.5, 3.0


def project(lat, lon, ref_lat, ref_lon):
    """PX4 MapProjection (azimuthal equidistant): lat/lon [deg] -> north, east [m] about the reference"""
    la, lo = np.radians(lat), np.radians(lon)
    rla, rlo = math.radians(ref_lat), math.radians(ref_lon)
    sl, cl, cd = np.sin(la), np.cos(la), np.cos(lo - rlo)
    arg = np.clip(math.sin(rla) * sl + math.cos(rla) * cl * cd, -1.0, 1.0)
    c = np.arccos(arg)
    k = np.where(np.abs(c) < 1e-12, 1.0, c / np.where(np.abs(c) < 1e-12, 1.0, np.sin(c)))
    return (k * (math.cos(rla) * sl - math.sin(rla) * cl * cd) * R_EARTH,
            k * cl * np.sin(lo - rlo) * R_EARTH)


def unproject(x, y, ref_lat, ref_lon):
    """the inverse of project(): north, east [m] about the reference -> lat, lon [deg]"""
    rla, rlo = math.radians(ref_lat), math.radians(ref_lon)
    xr, yr = np.asarray(x) / R_EARTH, np.asarray(y) / R_EARTH
    c = np.hypot(xr, yr)
    sc, cc = np.sin(c), np.cos(c)
    cs = np.where(c > 1e-12, c, 1.0)
    lat = np.where(c > 1e-12, np.arcsin(cc * math.sin(rla) + xr * sc * math.cos(rla) / cs), rla)
    lon = np.where(c > 1e-12, rlo + np.arctan2(yr * sc, cs * math.cos(rla) * cc - xr * math.sin(rla) * sc), rlo)
    return np.degrees(lat), np.degrees(lon)


def driver_events(path):
    ev = {'aborts': []}
    for line in open(path, errors='ignore'):
        m = re.search(r'\[(\d+\.\d+)\] \[flight_driver\]: (.*)', line)
        if not m:
            continue
        t, msg = float(m.group(1)), m.group(2)
        if 'ARMING -> TAKEOFF' in msg and 'armed' not in ev:
            ev['armed'] = t
            h = re.search(r'armed at N (\S+) E (\S+?),', msg)
            if h:
                ev['home'] = (float(h.group(1)), float(h.group(2)))
        elif '-> FOLLOW' in msg and 'follow' not in ev:
            ev['follow'] = t
        elif 'FOLLOW -> LANDING' in msg and 'end' not in ev:
            ev['end'], ev['complete'] = t, 'path complete' in msg
            ev['end_msg'] = msg[:160]
        elif 'LANDING -> DONE' in msg:
            ev['done'] = t
        if 'ABORT' in msg:
            ev['aborts'].append(msg[:160])
    return ev


def match(P, path, back=5.0, ahead=25.0):
    """progress-aware distance of each point to the polyline: (dist, station, segment, u)"""
    seg_a, seg_b = path[:-1], path[1:]
    d = seg_b - seg_a
    L = np.linalg.norm(d, axis=1)
    S = np.r_[0.0, np.cumsum(L)][:-1]
    s_cur = 0.0
    out = np.zeros((len(P), 4))
    for k, p in enumerate(P):
        cand = np.nonzero((S <= s_cur + ahead) & (S + L >= s_cur - back))[0]
        if len(cand) == 0:
            cand = np.arange(len(L))
        a, dd, LL = seg_a[cand], d[cand], L[cand]
        u = np.where(LL > 1e-9, np.einsum('ij,ij->i', p - a, dd) / np.maximum(LL * LL, 1e-18), 0.0)
        uc = np.clip(u, 0.0, 1.0)
        dist = np.linalg.norm(a + uc[:, None] * dd - p, axis=1)
        j = int(np.argmin(dist))
        st = S[cand[j]] + uc[j] * LL[j]
        s_cur = max(s_cur, st)
        out[k] = (dist[j], st, cand[j], u[j])
    return out


def evaluate(run, speed=None):
    res = {'run': os.path.basename(run.rstrip('/'))}
    pj = os.path.join(run, 'mc_params.json')
    params = json.load(open(pj)) if os.path.exists(pj) else {}
    res.update({k: params[k] for k in ('course', 'speed', 'wind', 'wind_toward_deg', 'gust_std', 'gust_tau', 'cda',
                                        'seed', 'size') if k in params})
    v = float(speed if speed is not None else params.get('speed', 0.0))
    rt = os.path.join(run, 'result.txt')
    res['result'] = open(rt).read().strip() if os.path.exists(rt) else 'no_result'
    fd = os.path.join(run, 'flight_driver.log')
    ulg = os.path.join(run, 'px4_sitl.ulg')
    ev = driver_events(fd) if os.path.exists(fd) else {'aborts': []}
    if 'armed' not in ev or not os.path.exists(ulg):
        res.update(sim_fault=True, success=False, note='never armed or no ulog')
        return res
    res['sim_fault'] = False
    res['aborts'] = ev['aborts']
    from pyulog import ULog
    u = ULog(ulg, ['vehicle_local_position_groundtruth', 'vehicle_local_position', 'vehicle_status'])
    gt = u.get_dataset('vehicle_local_position_groundtruth').data                  # 50 Hz, the simulator's truth
    lp = u.get_dataset('vehicle_local_position').data                              # the estimator: its frame
    ok = lp['xy_global'] > 0
    ref_lat, ref_lon = float(lp['ref_lat'][ok][-1]), float(lp['ref_lon'][ok][-1])
    tg = gt['timestamp'] / 1e6
    glat, glon = unproject(gt['x'], gt['y'], float(gt['ref_lat'][-1]), float(gt['ref_lon'][-1]))
    gn, ge = project(glat, glon, ref_lat, ref_lon)
    # wall clock (flight_driver log) -> PX4 boot time (ulog): arming and disarming seen by both
    vs = u.get_dataset('vehicle_status').data
    ts, armed = vs['timestamp'] / 1e6, vs['arming_state'] == 2
    if not armed.any():
        res.update(sim_fault=True, success=False, note='the ulog shows no arming')
        return res
    i_arm = int(np.argmax(armed))
    arm_px4 = ts[i_arm]
    after = np.nonzero(~armed[i_arm:])[0]
    if len(after) and 'done' in ev:
        dis_px4 = ts[i_arm + after[0]]
        rate = (dis_px4 - arm_px4) / max(ev['done'] - ev['armed'], 1e-6)
    else:
        rate = 1.0
    to_px4 = lambda t: arm_px4 + (t - ev['armed']) * rate                        # noqa: E731
    res['clock_rate'] = round(float(rate), 4)
    pre = (tg > arm_px4 - 2.0) & (tg <= arm_px4)
    ground = float(np.median(-gt['z'][pre])) if pre.any() else float(-gt['z'][0])
    h = -gt['z'] - ground
    wp = np.genfromtxt(os.path.join(run, 'wp.csv'), delimiter=',', names=True, comments='#')
    home = ev.get('home', (0.0, 0.0))
    path = np.column_stack([home[0] + np.atleast_1d(wp['x']), home[1] + np.atleast_1d(wp['y'])])
    zpath = np.atleast_1d(wp['z']).astype(float)
    keep = np.r_[True, np.linalg.norm(np.diff(path, axis=0), axis=1) > 1e-6]
    path, zpath = path[keep], zpath[keep]
    plen = float(np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1)))
    res.update(path_len_m=round(plen, 1), completed=bool(ev.get('complete', False)))
    if 'follow' not in ev:
        res.update(success=False, note='no FOLLOW phase')
        return res
    t0 = to_px4(ev['follow'])
    t1 = to_px4(ev['end']) if 'end' in ev else tg[-1]
    w = (tg >= t0) & (tg <= t1)
    P = np.column_stack([gn[w], ge[w]])
    H = h[w]
    if len(P) < 20:
        res.update(success=False, note='too few samples in FOLLOW')
        return res
    m = match(P, path)
    last = len(path) - 2
    # up to the first pass of the end point: in a tailwind PathFollowing overshoots it and comes back before
    # "path complete" (mcp_100: 2.35 m out and back) - along the path, not off it
    past = (m[:, 2] == last) & (m[:, 3] > 1.0)
    n_in = int(np.argmax(past)) if past.any() else len(P)
    seg = m[:n_in, 2].astype(int)
    uu = np.clip(m[:n_in, 3], 0.0, 1.0)
    zref = zpath[seg] + uu * (zpath[seg + 1] - zpath[seg])
    e_xy = m[:n_in, 0]
    e_z = H[:n_in] - zref
    e = np.hypot(e_xy, e_z)
    # the same on the estimator's own position (what PathFollowing sees) for comparison
    tl = lp['timestamp'] / 1e6
    wl = (tl >= t0) & (tl <= t1)
    if wl.sum() < 20:
        res.update(success=False, note='too few estimator samples in FOLLOW')
        return res
    ml = match(np.column_stack([lp['x'][wl], lp['y'][wl]]), path)
    pl = (ml[:, 2] == last) & (ml[:, 3] > 1.0)
    nl = int(np.argmax(pl)) if pl.any() else len(ml)
    sl_ = ml[:nl, 2].astype(int)
    ul_ = np.clip(ml[:nl, 3], 0.0, 1.0)
    zl = zpath[sl_] + ul_ * (zpath[sl_ + 1] - zpath[sl_])
    hl = -(lp['z'][wl][:nl] - float(np.interp(arm_px4, tl, lp['z'])))
    e_est = np.hypot(ml[:nl, 0], hl - zl)
    end_pt = path[-1]
    land = np.array([gn[-1], ge[-1]]) if 'done' not in ev else \
        np.array([np.interp(to_px4(ev['done']), tg, gn), np.interp(to_px4(ev['done']), tg, ge)])
    follow_s = (ev['end'] if 'end' in ev else ev.get('done', ev['follow'])) - ev['follow']
    res.update(
        # judged: the estimated position (what PathFollowing flies on; the field tests measure the same way)
        e95=round(float(np.percentile(e_est, 95)), 3), e_rms=round(float(np.sqrt(np.mean(e_est * e_est))), 3),
        e_max=round(float(e_est.max()), 3), e_mean=round(float(e_est.mean()), 3),
        within_1m_pct=round(100.0 * float(np.mean(e_est <= 1.0)), 1),
        # reported: the simulator's true position (adds the SITL GPS / estimator error, ~0.3-0.4 m at p95)
        e95_true=round(float(np.percentile(e, 95)), 3), e_max_true=round(float(e.max()), 3),
        e_rms_true=round(float(np.sqrt(np.mean(e * e))), 3),
        exy95_true=round(float(np.percentile(e_xy, 95)), 3), ez_max_true=round(float(np.abs(e_z).max()), 3),
        h_min=round(float(H[:n_in].min()), 2),
        land_err_m=round(float(np.linalg.norm(land - end_pt)), 2), follow_s=round(follow_s, 1),
        progress_m=round(float(m[:n_in, 1].max()), 1), samples=int(n_in))
    nominal = plen / max(v, 0.1)
    s = {'S1_complete': bool(ev.get('complete', False)) and not ev['aborts'] and res['result'].startswith('landed'),
         'S2_e95': res['e95'] <= E_LIMIT, 'S3_emax': res['e_max'] <= E_MAX, 'S4_height': res['h_min'] > H_MIN,
         'S5_landing': res['land_err_m'] <= LAND_MAX, 'S6_time': follow_s <= 2.0 * nominal + 10.0}
    res['checks'] = s
    res['success'] = all(s.values())
    res['fail'] = [k for k, ok_ in s.items() if not ok_]
    return res


def clopper_pearson_lower(k, n, alpha=0.05):
    if n == 0 or k == 0:
        return 0.0
    try:
        from scipy.stats import beta
        return float(beta.ppf(alpha, k, n - k + 1))
    except Exception:                                                              # noqa: BLE001
        p = k / n                                                                  # Wilson fallback
        z = 1.6449
        return (p + z * z / (2 * n) - z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n))) / (1 + z * z / n)


def report(prefix, out_dir):
    files = sorted(glob.glob(os.path.join(RUNS, f'{prefix}_*', 'mc_result.json')))
    R = [json.load(open(f)) for f in files]
    os.makedirs(out_dir, exist_ok=True)
    valid = [r for r in R if not r.get('sim_fault')]
    faults = [r for r in R if r.get('sim_fault')]
    n, ks = len(valid), sum(1 for r in valid if r.get('success'))
    k2 = sum(1 for r in valid if r.get('checks', {}).get('S2_e95'))
    lines = [f'# 몬테카를로 결과 {prefix}', '',
             f'- 유효 {n}회 (시뮬레이터 고장 제외 {len(faults)}회)',
             f'- **운행 성공 {ks}/{n} = {100.0 * ks / max(n, 1):.1f} %**, 95 % 신뢰 하한 '
             f'{100.0 * clopper_pearson_lower(ks, n):.1f} % (판정: 하한 ≥ 90 %)',
             f'- **경로 오차(95 백분위) ≤ 1 m: {k2}/{n} = {100.0 * k2 / max(n, 1):.1f} %**, 95 % 신뢰 하한 '
             f'{100.0 * clopper_pearson_lower(k2, n):.1f} %', '']

    def table(title, key, bins):
        out = [f'## {title}', '', '| 구간 | 회수 | 성공 | 성공률 | 경로 오차 95 백분위 중앙값 / 최대 |', '|---|---|---|---|---|']
        for name, f in bins:
            rr = [r for r in valid if f(r)]
            if not rr:
                continue
            e = [r['e95'] for r in rr if 'e95' in r]
            out.append(f'| {name} | {len(rr)} | {sum(r["success"] for r in rr)} | '
                       f'{100.0 * sum(r["success"] for r in rr) / len(rr):.0f} % | '
                       + (f'{np.median(e):.2f} / {max(e):.2f} m |' if e else '- |'))
        return out + ['']
    lines += table('코스별', 'course', [(c, lambda r, c=c: r.get('course') == c)
                                     for c in ('line', 'circle', 'square', 'spiral')])
    lines += table('평균 풍속별', 'wind', [(f'{a:g}–{b:g} m/s', lambda r, a=a, b=b: a <= r.get('wind', -1) < b)
                                      for a, b in ((0, 3.34), (3.34, 6.67), (6.67, 10.01))])
    lines += table('속도별', 'speed', [(f'{a:g}–{b:g} m/s', lambda r, a=a, b=b: a <= r.get('speed', -1) < b)
                                    for a, b in ((3, 4.34), (4.34, 5.67), (5.67, 7.01))])
    fails = [r for r in valid if not r.get('success')]
    lines += ['## 실패 목록', '', '| 실행 | 코스 | 속도 | 풍속 | 풍향 | 경로 오차 95 % / 최대 | 실패 조건 | ABORT |',
              '|---|---|---|---|---|---|---|---|']
    for r in fails:
        lines.append(f'| {r["run"]} | {r.get("course")} | {r.get("speed")} | {r.get("wind")} | '
                     f'{r.get("wind_toward_deg")}° | {r.get("e95", "-")} / {r.get("e_max", "-")} m | '
                     f'{", ".join(r.get("fail", [])) or r.get("note", "")} | {"; ".join(r.get("aborts", []))[:80]} |')
    lines += ['', '## 시뮬레이터 고장 (제외)', ''] + [f'- {r["run"]}: {r.get("note", "")}' for r in faults]
    md = os.path.join(out_dir, f'{prefix}_report.md')
    open(md, 'w').write('\n'.join(lines) + '\n')
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 2, figsize=(12, 4.6))
        for c, col in (('line', 'C0'), ('circle', 'C1'), ('square', 'C2'), ('spiral', 'C3')):
            e = np.sort([r['e95'] for r in valid if r.get('course') == c and 'e95' in r])
            if len(e):
                ax[0].step(e, np.arange(1, len(e) + 1) / len(e), where='post', color=col, label=c)
        ax[0].axvline(E_LIMIT, color='k', ls='--', lw=1)
        ax[0].set_xlabel('path error p95 per run [m]')
        ax[0].set_ylabel('fraction of runs')
        ax[0].legend()
        ax[0].grid(alpha=0.3)
        for r in valid:
            ax[1].scatter(r.get('speed', 0), r.get('wind', 0), c='tab:green' if r.get('success') else 'tab:red',
                          marker={'line': 'o', 'circle': 's', 'square': 'D', 'spiral': '^'}.get(r.get('course'), 'x'),
                          s=22)
        ax[1].set_xlabel('speed [m/s]')
        ax[1].set_ylabel('mean wind [m/s]')
        ax[1].set_title('green = success, red = failure')
        ax[1].grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig(os.path.join(out_dir, f'{prefix}_report.png'), dpi=110)
    except Exception as ex:                                                        # noqa: BLE001
        print('figure failed:', ex)
    print(open(md).read())
    print('->', md)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    r1 = sub.add_parser('run')
    r1.add_argument('run_dir')
    r1.add_argument('--speed', type=float, default=None)
    r2 = sub.add_parser('report')
    r2.add_argument('prefix')
    r2.add_argument('--out', default=None)
    a = ap.parse_args()
    if a.cmd == 'run':
        res = evaluate(a.run_dir, a.speed)
        json.dump(res, open(os.path.join(a.run_dir, 'mc_result.json'), 'w'), indent=1)
        print(json.dumps(res, indent=1, ensure_ascii=False))
    else:
        report(a.prefix, a.out or os.path.join(RUNS, f'{a.prefix}_report'))


if __name__ == '__main__':
    sys.exit(main())
