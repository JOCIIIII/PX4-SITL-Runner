import csv, re, collections, sys
SP = sys.argv[1]
rows = list(csv.DictReader(open(f'{SP}/runs_index.csv')))
C = [  # (id, regex, title)
 ('F1', r'^(s7|v10|base_|survey|sensor|probe_nd|demo_7ms|demo_e|demo$|pftest)', 'SUPER+PF 7 m/s 튜닝 (AirSim 숲, iris)'),
 ('F2', r'^(t1000|demo_t1000)', 'T1000(전북대 정책) 통합 (AirSim 숲)'),
 ('F3', r'^(nc_|demo_noclimb)', '인하우스 정책 noclimb + MPPI 통합 (AirSim 숲, iris)'),
 ('F4', r'^(x8_step|x8_replay|x8_super|x8_sp[0-9]|iris_|x8_nc|x8_t1000)', 'X8 모델·제어·SUPER/정책 X8 적용 (AirSim 숲)'),
 ('F5', r'^x8_big', 'X8 ca_big 정책 반복 비행 (AirSim 숲)'),
 ('F6', r'^(x8_mppi|x8_corner)', 'PF 단독 바람 매트릭스·모서리 감속 (X8)'),
 ('F7', r'^pfm_', 'PF 단독 경로 모양 1–10 m/s·바람 (X8)'),
 ('F8', r'^x500_step', 'X500 모델 계단 응답 (Gazebo)'),
 ('C1', r'^(ctrack_123|ctrack_hold|ctrack_sp_|ctrack_s900|ctrack_v2)', 'C-Track SUPER X8·RC카트 대응'),
 ('C2', r'^(ctrack_big|ctrack_x8r2|ctrack_x8s|ctrack_m_new|ctrack_m_r2)', 'C-Track 학습 정책 적용 (ca_big → X8 학습판)'),
 ('C3', r'^(ctrack_m_soft|ctrack_mem|ctrack_tau|ctrack_capfade|ctrack_a[0-9]j|ctrack_hud)', 'C-Track 명령 평활·점 누적·제동 규칙 비교'),
 ('C4', r'^ctrack_pkg', 'C-Track 패키징판 검증'),
 ('C5', r'^(ctrack_speed|ctrack_m360|ctrack_x500|ctrack_t1000_[0-9]|ctrack_t1000$)', 'C-Track 속도별 정책·Mid-360 라이다·X500'),
 ('P1', r'^(pils_x500|ctrack_pils_loop)', 'PILS 젯슨 001 (X500 5개 묶음)'),
 ('C6', r'^gate_sils', 'flight_driver 관문 SILS'),
 ('P2', r'^gate_pils_', 'flight_driver 관문 PILS (젯슨 001, LAN·5G)'),
 ('C7', r'^ctrack_t1000_(pf|x8)', 'C-Track T1000+PF'),
 ('C8', r'^cand', '실비행 정책 후보 비교 (X8)'),
 ('P3', r'^gate_pils02', 'PILS 젯슨 002 첫 시험'),
 ('C9', r'^x500cmp', 'X500 데모 구성 비교'),
 ('C10', r'^(llm_test|search)', 'LLM 임무·사람 수색 SILS'),
 ('P4', r'^llm_pils', 'LLM 임무 PILS (ZED Box)'),
 ('C11', r'^x8cmp_x8', 'X8 5 m/s 후보 5회 비교'),
 ('P5', r'^j2_', 'PILS 젯슨 002 (X8 3/5 m/s, X500 7 m/s)'),
 ('C12', r'^(x500_10|x500_7)', 'X500 10 m/s·7 m/s C-Track'),
 ('Y1', r'^yard_', 'Yard(인하대 교정) 맵'),
 ('C13', r'^tent17', '천막 충돌 재현 (11-02 실비행)'),
 ('R', r'^(shadow_fly_1016|ca_x500_3_1016|real_1017|gx8_)', '실기체 비행 기록 (시뮬레이션 아님)'),
 ('A', r'^x8cmp_summary$', '분석 결과 폴더'),
]
def camp(n):
    for cid, rx, t in C:
        if re.match(rx, n):
            return cid
    return 'Z'
TITLE = {c[0]: c[2] for c in C}; TITLE['Z'] = '기타 SILS (flight_driver 고장 주입 7, 추적 카메라 녹화 2)'
g = collections.OrderedDict((c[0], []) for c in C); g['Z'] = []
for r in rows:
    r['campaign'] = camp(r['name']); g[r['campaign']].append(r)
w = csv.DictWriter(open(f'{SP}/runs_index_campaign.csv', 'w', newline=''), list(rows[0].keys()))
w.writeheader(); [w.writerow(r) for r in sorted(rows, key=lambda r: (r['campaign'], r['date'], r['time'], r['name']))]
out = []
for cid, rs in g.items():
    if not rs: continue
    dates = sorted({r['date'] for r in rs})
    res = collections.Counter(r['result'] or '?' for r in rs)
    simfault = sum(r['result'] in ('airsim_frozen', 'no_lidar') or 'INVALID' in r['name'] for r in rs)
    valid_m = [r for r in rs if r['margin'] and r['env'] == 'CTrack' and cid != 'C13']
    m = [float(r['margin']) for r in valid_m]
    v = [float(r['v_mean']) for r in rs if r['v_mean']]
    vx = [float(r['v_max']) for r in rs if r['v_max']]
    modes = '/'.join(sorted({r['mode'] for r in rs}))
    ob = ','.join(sorted({r['onboard'] for r in rs if r['onboard']}))
    af = ','.join(sorted({r['airframe'] or '?' for r in rs}))
    envs = ','.join(sorted({r['env'] for r in rs}))
    out.append(dict(id=cid, title=TITLE[cid], dates=f"{dates[0]}~{dates[-1]}" if len(dates) > 1 else dates[0], env=envs,
                    mode=modes + (f' ({ob})' if ob else ''), airframe=af, n=len(rs), landed=res.get('landed', 0),
                    aborted=res.get('aborted', 0) + res.get('crashed', 0), timeout=res.get('timeout', 0) + res.get('stalled', 0),
                    simfault=simfault, unknown=res.get('?', 0) + res.get('stopped', 0),
                    contact=sum(x < 0 for x in m), n_margin=len(m),
                    margin_min=f'{min(m):+.2f}' if m else '', margin_med=f'{sorted(m)[len(m)//2]:+.2f}' if m else '',
                    v_mean=f'{sum(v)/len(v):.2f}' if v else '', v_max=f'{max(vx):.1f}' if vx else ''))
w = csv.DictWriter(open(f'{SP}/campaigns.csv', 'w', newline=''), list(out[0].keys()))
w.writeheader(); [w.writerow(o) for o in out]
for o in out:
    print(' | '.join(str(o[k]) for k in o))
