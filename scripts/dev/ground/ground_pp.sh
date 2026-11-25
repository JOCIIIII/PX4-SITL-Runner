#!/bin/bash
# Hybrid PP (hybrid-learning-path-planning 0.3.2, the A4VAI planner) for a ground / surface vehicle on a layout
# of scripts/dev/ground/layouts: one height layer, the vehicle's radius, every obstacle in the height map - via
# scripts/dev/scenario_pp.py (planning time, length, the visibility-graph reference path and a figure), then the
# path densified into a ground_driver.py mission CSV (rows north,east in the layout frame).
#   ground_pp.sh ugv|usv LAYOUT OUT_DIR [scenario_pp.py options]
# defaults  ugv: --alt 0.5 --vclear 0.3 --res 0.1 --radius 0.65 (body 0.35 + tracking 0.3), spacing 0.5 m
#           usv: --alt 1.0 --vclear 0.5 --res 0.5 --radius 5.0  (body 2.7 + tracking 2.3), spacing 2.0 m
set -e
HERE=$(dirname $(readlink -f "$0"))
VEH=$1; LAYOUT=$(readlink -f "$2"); OUT=$3; shift 3
case "$VEH" in
    ugv) DEF="--alt 0.5 --vclear 0.3 --res 0.1 --radius 0.65"; SP=0.5 ;;
    usv) DEF="--alt 1.0 --vclear 0.5 --res 0.5 --radius 5.0"; SP=2.0 ;;
    *) echo "ugv|usv"; exit 1 ;;
esac
SCENARIO_LAYOUT=$LAYOUT $HERE/../scenario_pp.sh $OUT START,GOAL --known . $DEF "$@"
python3 - "$OUT/pp_path.csv" "$OUT/mission.csv" $SP <<'PY'
import math, sys
rows = [l.strip().split(',') for l in open(sys.argv[1])][1:]
P = [(float(r[0]), float(r[1])) for r in rows]
d = float(sys.argv[3])
out = [P[0]]
for a, b in zip(P, P[1:]):
    n = max(int(math.ceil(math.hypot(b[0] - a[0], b[1] - a[1]) / d)), 1)
    out += [(a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n) for i in range(1, n + 1)]
with open(sys.argv[2], 'w') as f:
    for x, y in out:
        f.write(f'{x:.3f},{y:.3f}\n')
L = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(P, P[1:]))
print(f'[ground_pp] {len(P)} vertices, {L:.1f} m -> {len(out)} mission waypoints ({d} m) -> {sys.argv[2]}')
PY
