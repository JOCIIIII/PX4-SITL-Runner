#!/bin/bash
# Simulator tests on the demonstration scenario test map (실증 시나리오 시험맵), one run per call:
#   pp      path planning unit: Hybrid PP plans scenario A's route (START -> WP1 around the known tents), no flight
#   pf      path following unit: scenario B's straight line through the fan wind
#   ca      collision avoidance unit: TEST_CA flies WP3 -> GOAL (poles, RC cart, wall, balloon) in its own default
#           chain - inha / super through PathFollowing, jbnu T1000's own velocity setpoints to PX4
#   pp-pf   path planning + following: the scenario A plan flown
#   pf-ca   path following + avoidance: TEST_CA's commands flown by PathFollowing along scenario C's legs
#           WP3 -> WP5 -> WP6 -> GOAL
#   all     integrated: scenario D - the Hybrid PP plan flown by PathFollowing to WP3, then TEST_CA to GOAL
#   Path following is PathFollowing's MPPI guidance throughout (sim_ca_mppi(_x8).yaml under the avoidance).
# usage: scripts/dev/scenario_test.sh KIND NAME [PROFILE]        (or: ./scripts/scenario.sh test KIND NAME [PROFILE])
#   TEST_CA  inha (default) = the learned policy of PROFILE (scripts/ros2/apps/noclimb_ca.sh)
#            jbnu           = T1000 Control 445 (CA unit: t1000.sh; with PathFollowing: t1000_pf.sh)
#            super          = SUPER (super_pf.sh, PathFollowing in MPPI: SUPER_PF_GUID=2)
#   PROFILE  flight profile (default x8_5sm2): the airframe and, for inha, the policy
#   TEST_SPEED (5 m/s), TEST_WIND (fan wind, 5 m/s; 0 = off), TEST_ALT (1.7 m), TEST_TIMEOUT (600 s);
#   the DEMO_* knobs of scripts/dev/scenario_demo.sh pass through.
# Result: test_runs/NAME/test_summary.txt with PASS / FAIL against these criteria (the RFP items, 2026-11-05):
#   pp          path found, length <= 120 % of the shortest, planning <= 1.0 s (the RFP measures this on the Jetson)
#   pf, pp-pf   landed at the end, cross-track error p95 <= 1.0 m and max <= 3.0 m
#   ca, pf-ca   landed within 2 m of the goal, every obstacle margin > 0 (no contact)
#   all         both: the planned prefix within p95 1.0 / max 3.0 m, no contact, landed within 2 m of the goal
# Every run is also appended to test_runs/test_matrix.csv; "scenario_test.sh table" shows the latest of each cell.
set -u
# "table": the latest result of every cell (scripts/dev/scenario_test_table.py)
[ "${1:-}" = table ] && exec python3 "$(dirname "$(readlink -f "$0")")/scenario_test_table.py"
KIND=${1:?usage: scenario_test.sh pp|pf|ca|pp-pf|pf-ca|all NAME [PROFILE] | table}
NAME=${2:?usage: scenario_test.sh KIND NAME [PROFILE]}
PROFILE=${3:-x8_5sm2}
REPO=$(dirname $(dirname $(dirname $(readlink -f "$0"))))
W=~/Documents/A4VAI-SITL
OUT=$W/ROS2/test_runs/$NAME
DEMO=$REPO/scripts/dev/scenario_demo.sh
PFW=mppi; CA=${TEST_CA:-inha}
case "$CA" in inha|jbnu|super) ;; *) echo "[test] TEST_CA: inha | jbnu | super"; exit 1 ;; esac
export DEMO_SPEED=${TEST_SPEED:-${DEMO_SPEED:-5}} DEMO_WIND=${TEST_WIND:-${DEMO_WIND:-5}}
export DEMO_ALT=${TEST_ALT:-${DEMO_ALT:-1.7}} DEMO_TIMEOUT=${TEST_TIMEOUT:-${DEMO_TIMEOUT:-600}}
# what is flown: the avoidance app per cell
case "$KIND" in
    pp)    LABEL="PP unit (Hybrid PP)"; "$DEMO" P "$NAME" "$PROFILE" ;;
    pf)    LABEL="PF unit"; "$DEMO" B "$NAME" "$PROFILE" ;;
    pp-pf) LABEL="PP-PF"; "$DEMO" A "$NAME" "$PROFILE" ;;
    ca)    LABEL="CA unit, CA=$CA"
           case "$CA" in
               inha)  NOCLIMB_GUID=2 DEMO_C_LEGS=GOAL "$DEMO" C "$NAME" "$PROFILE" ;;
               jbnu)  DEMO_CA_APP=t1000 "$DEMO" C "$NAME" "$PROFILE" ;;
               super) SUPER_PF_GUID=2 DEMO_C_LEGS=GOAL DEMO_CA_APP=super_pf "$DEMO" C "$NAME" "$PROFILE" ;;
           esac ;;
    pf-ca) LABEL="PF-CA, CA=$CA"
           case "$CA" in
               inha)  NOCLIMB_GUID=2 "$DEMO" C "$NAME" "$PROFILE" ;;
               jbnu)  NOCLIMB_GUID=2 DEMO_CA_APP=t1000_pf "$DEMO" C "$NAME" "$PROFILE" ;;
               super) SUPER_PF_GUID=2 DEMO_CA_APP=super_pf "$DEMO" C "$NAME" "$PROFILE" ;;
           esac ;;
    all)   LABEL="integrated PP+PF+CA, CA=$CA"
           case "$CA" in
               inha)  NOCLIMB_GUID=2 "$DEMO" D "$NAME" "$PROFILE" ;;
               jbnu)  NOCLIMB_GUID=2 DEMO_CA_APP=t1000_pf "$DEMO" D "$NAME" "$PROFILE" ;;
               super) SUPER_PF_GUID=2 DEMO_CA_APP=super_pf "$DEMO" D "$NAME" "$PROFILE" ;;
           esac ;;
    *)     sed -n '2,/^set -u/p' "$0" | sed '$d'; exit 1 ;;
esac
RC=$?
[ -f "$OUT/demo_summary.txt" ] || { echo "[test] $NAME: no summary (exit $RC)"; exit 1; }

python3 - "$OUT/demo_summary.txt" "$KIND" "$LABEL" "$PROFILE" "$NAME" "$W/ROS2/test_runs/test_matrix.csv" "$PFW" "$CA" <<'PYEOF' | tee "$OUT/test_summary.txt"
import datetime, os, re, sys
summ, kind, label, profile, name, matrix, pfw, ca = sys.argv[1:9]
t = open(summ).read()
def num(pat):
    m = re.search(pat, t)
    return [float(g) for g in m.groups()] if m else None
res = (re.search(r'^result: (.*)$', t, re.M) or [None, '?'])[1].strip()
checks = []                                     # (criterion, value text, ok)
if kind == 'pp':
    pp = re.search(r'Hybrid PP: (ok|FAILED), ([\d.]+) m vs shortest ([\d.]+) m \(ratio ([\d.]+)\), max planning ([\d.]+) ms', t)
    checks.append(('path found', pp.group(1) if pp else 'no plan', bool(pp) and pp.group(1) == 'ok'))
    if pp:
        checks.append(('length <= 120 % of the shortest', f'{float(pp.group(4)) * 100:.1f} %', float(pp.group(4)) <= 1.2))
        checks.append(('planning <= 1.0 s', f'{float(pp.group(5)):.0f} ms', float(pp.group(5)) <= 1000.0))
else:
    checks.append(('landed', res, res == 'landed'))
if kind in ('pf', 'pp-pf', 'all'):
    e = num(r'cross-track error mean ([\d.]+) / p95 ([\d.]+) / max ([\d.]+) m')
    reached = 'reached the end' in t and 'did NOT reach the end' not in t
    if kind != 'all':
        checks.append(('reached the end of the path', 'yes' if reached else 'no', reached))
    checks.append(('cross-track p95 <= 1.0 m', f'{e[1]:.2f} m' if e else 'n/a', bool(e) and e[1] <= 1.0))
    checks.append(('cross-track max <= 3.0 m', f'{e[2]:.2f} m' if e else 'n/a', bool(e) and e[2] <= 3.0))
if kind in ('ca', 'pf-ca', 'all'):
    mg = num(r'minimum margin to the prop tips: ([+-]?[\d.]+) m')
    checks.append(('no contact (margin > 0)', f'{mg[0]:+.2f} m' if mg else 'n/a', bool(mg) and mg[0] > 0.0))
    end = num(r'end \(([-\d.]+),([-\d.]+)\) = ([\d.]+) m')
    checks.append(('landed within 2 m of the goal', f'{end[2]:.2f} m' if end else 'n/a', bool(end) and end[2] <= 2.0))
ok = all(c[2] for c in checks)
head = t.splitlines()[0] if t else ''
print(f'{name}: {label} - {"PASS" if ok else "FAIL"}')
print(f'  {head}')
for c, v, g in checks:
    print(f'  [{"ok" if g else "XX"}] {c}: {v}')
new = not os.path.exists(matrix)
with open(matrix, 'a') as f:
    if new:
        f.write('time,name,kind,pf,ca,profile,verdict,result,' + 'checks\n')
    f.write(f'{datetime.datetime.now():%Y-%m-%d %H:%M},{name},{kind},{pfw if kind != "pp" else "-"},'
            f'{ca if kind in ("ca", "pf-ca", "all") else "-"},{profile},{"PASS" if ok else "FAIL"},'
            f'"{res}","{"; ".join(f"{c}={v}" for c, v, g in checks)}"\n')
PYEOF
