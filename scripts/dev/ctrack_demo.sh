#!/bin/bash
# The final demonstration scenarios (2026-11-06 workshop deck, slides 17-20) on the C-Track field, one run each:
#   A          path planning: Hybrid PP START -> WP1 around the pre-known tents, flown by PathFollowing alone at DEMO_SPEED
#   B          path following in the fan wind: WP2 -> WP3 straight at DEMO_SPEED through the DEMO_WIND cross wind
#   B-square / B-circle / B-spiral   PathFollowing courses inside the fan zone at DEMO_SPEED, wind on
#   C          collision avoidance: the policy (PROFILE) WP3 -> WP5 -> WP6 -> GOAL (poles, RC cart, wall, balloon)
#   D          integrated: Hybrid PP over START -> WP2 -> WP3 -> WP5 -> WP6 -> GOAL; START -> WP3 flown by
#              PathFollowing alone at DEMO_SPEED (no avoidance, fan wind on), then the policy WP3 -> WP5 -> WP6 -> GOAL
# usage: scripts/dev/ctrack_demo.sh SCENARIO NAME [PROFILE]     (PROFILE default x8_5sm2: its airframe and, for C / D,
#        its policy). Knobs: DEMO_SPEED (7), DEMO_ALT (path height above the ground, 1.7 = the C-Track runs' height),
#        DEMO_WIND (5 m/s; 0 = off), DEMO_TIMEOUT (600 s), DEMO_COURSE_R / DEMO_COURSE_C (B courses, 15 m at 65,5),
#        DEMO_SQUARE_FILLET (B-square corner radius [m], default 0 = sharp corners; 7 m/s at 30 deg tilt turns on >= ~9 m),
#        DEMO_D_COURSE (D: B courses flown at WP2 before WP3, default circle,square,spiral; none = the old D),
#        DEMO_B_LINE (B straight "X0,Y0 X1,Y1", default "79,0 79,26"), DEMO_C_LEGS (C legs after WP3, default
#        WP5,WP6,GOAL), DEMO_D_POINTS (D plan points, default START,WP2,WP3,WP5,WP6,GOAL; the prefix = legs up to
#        DEMO_D_PREFIX_TO, default WP3), DEMO_RECT (B-rect: "W,H" [m], default 30,40, corner DEMO_SQUARE_FILLET),
#        DEMO_COURSE_C per course via DEMO_COURSE_C_<square|circle|spiral|rect> (e.g. DEMO_COURSE_C_circle=69,7),
#        DEMO_D_FILLET (D square corner radius, default 5 m),
#        and those of ctrack_super.sh / the profile.
# The deck's waypoints are the layout's: deck WP1 = WP2 (79,0), deck WP2 = WP3 (79,26), deck WP3 = WP5 (42,26),
# deck WP4 = WP6 (26,23); START (0,0), GOAL (0,26). Results: test_runs/NAME (the run) with pp/ (the plan) and
# demo_summary.txt (path error, wind-zone error, clearance).
set -u
SC=${1:?usage: ctrack_demo.sh A|B|B-square|B-circle|B-spiral|C|D NAME [PROFILE]}
NAME=${2:?usage: ctrack_demo.sh SCENARIO NAME [PROFILE]}
PROFILE=${3:-x8_5sm2}
REPO=$(dirname $(dirname $(dirname $(readlink -f "$0"))))
W=~/Documents/A4VAI-SITL
OUT=$W/ROS2/test_runs/$NAME
PLAN=$W/ROS2/test_runs/${NAME}_plan
SPEED=${DEMO_SPEED:-7}; ALT=${DEMO_ALT:-1.7}; WIND=${DEMO_WIND:-5}; TIMEOUT=${DEMO_TIMEOUT:-600}
PF=$REPO/scripts/dev/flight_profiles/$PROFILE.env
[ -f "$PF" ] || { echo "[demo] no profile $PF"; exit 1; }
AF=$(sed -nE 's/^PROFILE_AIRFRAME="?([a-z0-9]+)"?.*/\1/p' "$PF" | head -1)
if [ -z "${CTRACK_DIR:-}" ]; then
    CTRACK_DIR=$W/A4VAI-Unreal
    [ -d "$CTRACK_DIR" ] || CTRACK_DIR=$HOME/Desktop/00_Project/00_A4VAI/00_Development/00_Simulator/02_InhouseSim/00_Unreal/00_Map
fi
export CTRACK_DIR
LAYOUT=${CTRACK_LAYOUT:-$CTRACK_DIR/layout/ctrack_layout.json}
ZONE=$(python3 -c "import json,sys; z=json.load(open(sys.argv[1])).get('wind_zones',[]); \
print(','.join(str(v) for v in z[0]['x_range'] + z[0]['y_range']) if z else '')" "$LAYOUT")
WIND_ENV=(); [ "$WIND" != 0 ] && WIND_ENV=(CTRACK_WIND=$WIND)
rm -rf "$PLAN"; mkdir -p "$PLAN"

plan() {   # Hybrid PP over the given points -> $PLAN/pp_path.csv
    "$REPO/scripts/dev/ctrack_pp.sh" "$PLAN" "$1" --alt "$ALT" "${@:2}" || { echo "[demo] path planning failed"; exit 1; }
}
course() {  # a PathFollowing course in the fan zone -> $PLAN/course.csv (layout X,Y), downwind of the fans
    python3 - "$1" "$ZONE" "$PLAN/course.csv" <<'PYEOF'
import math, sys
kind, zone, out = sys.argv[1], [float(v) for v in sys.argv[2].split(',')], sys.argv[3]
# DEMO_COURSE_R (default 15 m: 7 m/s needs 3.3 m/s^2 of lateral acceleration on it) around DEMO_COURSE_C (default
# 65,5): the east part runs through the fan zone downwind of the fans (X 84), >= 3.5 m from them and the poles
import os
cx, cy = (float(v) for v in os.environ.get(f'DEMO_COURSE_C_{kind}', os.environ.get('DEMO_COURSE_C', '65,5')).split(','))
half = float(os.environ.get(f'DEMO_COURSE_R_{kind}', os.environ.get('DEMO_COURSE_R', '15')))
if kind == 'rect':      # W x H rectangle about the centre, corners of radius DEMO_SQUARE_FILLET, flown clockwise
    W, Hh = (float(v) / 2.0 for v in os.environ.get('DEMO_RECT', '30,40').split(','))
    r = min(max(float(os.environ.get('DEMO_SQUARE_FILLET', '0')), 0.0), min(W, Hh))
    n = max(4, math.ceil(r * math.pi / 2)) if r > 0 else 1
    pts = [(cx + W, cy + Hh - r)]
    for ox, oy, a0 in ((W - r, -(Hh - r), 0.0), (-(W - r), -(Hh - r), -90.0), (-(W - r), Hh - r, 180.0), (W - r, Hh - r, 90.0)):
        pts += [(cx + ox + r * math.cos(math.radians(a0 - 90.0 * i / n)), cy + oy + r * math.sin(math.radians(a0 - 90.0 * i / n)))
                for i in range(n + 1)]
    pts.append(pts[0])
elif kind == 'square':
    s = half * 0.9
    r = min(max(float(os.environ.get('DEMO_SQUARE_FILLET', '0')), 0.0), s)
    if r <= 0.0:
        pts = [(cx + s, cy + s), (cx + s, cy - s), (cx - s, cy - s), (cx - s, cy + s), (cx + s, cy + s)]
    else:   # rounded corners: the east side down, then clockwise; each corner a quarter circle of radius r
        n = max(4, math.ceil(r * math.pi / 2))                    # about 1 m between the arc points
        pts = [(cx + s, cy + s - r)]
        for ox, oy, a0 in ((s - r, -(s - r), 0.0), (-(s - r), -(s - r), -90.0),
                           (-(s - r), s - r, 180.0), (s - r, s - r, 90.0)):
            pts += [(cx + ox + r * math.cos(math.radians(a0 - 90.0 * i / n)),
                     cy + oy + r * math.sin(math.radians(a0 - 90.0 * i / n))) for i in range(n + 1)]
        pts.append(pts[0])
elif kind == 'circle':
    R = half
    pts = [(cx + R * math.cos(-2 * math.pi * i / 72), cy + R * math.sin(-2 * math.pi * i / 72)) for i in range(73)]
else:                                                               # spiral: 2 turns, R half/3 -> half
    pts = [(cx + (half / 3 + (half - half / 3) * i / 144) * math.cos(-4 * math.pi * i / 144),
            cy + (half / 3 + (half - half / 3) * i / 144) * math.sin(-4 * math.pi * i / 144)) for i in range(145)]
with open(out, 'w') as f:
    f.write('X,Y\n')
    for x, y in pts:
        f.write(f'{x:.3f},{y:.3f}\n')
print(f'{pts[0][0]:.2f},{pts[0][1]:.2f} {pts[-1][0]:.2f},{pts[-1][1]:.2f}')
PYEOF
}

case "$SC" in
    A)
        plan START,WP1
        "$REPO/scripts/ctrack.sh" unreal "$AF" > /dev/null || exit 1
        CTRACK_APP=mission CTRACK_PATH=$PLAN/pp_path.csv "$REPO/scripts/dev/ctrack_super.sh" "$NAME" START WP1 "$ALT" "$SPEED" "$TIMEOUT" "$AF"
        PATH_CSV=$PLAN/pp_path.csv ;;
    B)
        read B0 B1 <<< "${DEMO_B_LINE:-79,0 79,26}"
        printf 'X,Y\n%s\n%s\n' "$B0" "$B1" > "$PLAN/course.csv"
        "$REPO/scripts/ctrack.sh" unreal "$AF" > /dev/null || exit 1
        env "${WIND_ENV[@]}" CTRACK_APP=mission CTRACK_PATH=$PLAN/course.csv \
            "$REPO/scripts/dev/ctrack_super.sh" "$NAME" "$B0" "$B1" "$ALT" "$SPEED" "$TIMEOUT" "$AF"
        PATH_CSV=$PLAN/course.csv ;;
    B-square|B-circle|B-spiral|B-rect)
        [ -n "$ZONE" ] || { echo "[demo] the layout has no wind zone"; exit 1; }
        read A0 A1 <<< "$(course "${SC#B-}")"
        "$REPO/scripts/ctrack.sh" unreal "$AF" > /dev/null || exit 1
        env "${WIND_ENV[@]}" CTRACK_APP=mission CTRACK_PATH=$PLAN/course.csv \
            "$REPO/scripts/dev/ctrack_super.sh" "$NAME" "$A0" "$A1" "$ALT" "$SPEED" "$TIMEOUT" "$AF"
        PATH_CSV=$PLAN/course.csv ;;
    C)
        CTRACK_LEGS=${DEMO_C_LEGS:-WP5,WP6,GOAL} "$REPO/scripts/ctrack.sh" fly "$PROFILE" "$NAME" WP3 GOAL "$ALT" 5.0 "$TIMEOUT"
        PATH_CSV= ;;
    D)
        plan "${DEMO_D_POINTS:-START,WP2,WP3,WP5,WP6,GOAL}"
        # the part flown by PathFollowing alone: the plan's first two legs (START -> WP2 -> WP3)
        python3 - "$PLAN/pp_result.json" "$PLAN/prefix.csv" <<'PYEOF' || exit 1
import json, math, os, sys
legs = json.load(open(sys.argv[1]))['legs']
# DEMO_D_COURSE (default circle,square,spiral; none = straight on): the B courses flown by PathFollowing at WP2
# before WP3, in that order. Each passes through WP2 itself around DEMO_COURSE_C (65,5), all counter-clockwise
# in the layout frame (= the turn of the corner at WP2 toward WP3): the circle and the square (corner radius
# DEMO_D_FILLET, default 5 m) come back to WP2, the spiral (2 turns, radius / 3) ends inside heading at WP3.
# the prefix: every leg up to DEMO_D_PREFIX_TO (default WP3); the courses hang off the leg end before it
to_ = os.environ.get('DEMO_D_PREFIX_TO', 'WP3')
n_pre = next(i for i, l in enumerate(legs) if l['to'] == to_) + 1
wp2 = legs[n_pre - 2]['waypoints'][-1] if n_pre >= 2 else legs[0]['waypoints'][0]
cx, cy = (float(v) for v in os.environ.get('DEMO_COURSE_C', '65,5').split(','))
R = math.hypot(wp2[0] - cx, wp2[1] - cy)
a0 = math.atan2(wp2[1] - cy, wp2[0] - cx)
course = []
kinds = [k for k in ('circle', 'square', 'spiral')
         if k in os.environ.get('DEMO_D_COURSE', 'circle,square,spiral').split(',')]
for kind in kinds:
    if kind == 'circle':
        course += [(cx + R * math.cos(a0 + 2 * math.pi * i / 72), cy + R * math.sin(a0 + 2 * math.pi * i / 72))
                   for i in range(1, 73)]
    elif kind == 'square':              # east side through WP2 (x = wp2 x), half side = that offset
        h = wp2[0] - cx
        r = min(max(float(os.environ.get('DEMO_D_FILLET', '5')), 0.0), h)
        n = max(4, math.ceil(r * math.pi / 2)) if r > 0 else 0
        sq = [(cx + h, cy + h - r)]
        for ox, oy, b0 in ((h - r, h - r, 0.0), (-(h - r), h - r, 90.0), (-(h - r), -(h - r), 180.0),
                           (h - r, -(h - r), 270.0)):
            sq += [(cx + ox + r * math.cos(math.radians(b0 + 90.0 * i / max(n, 1))),
                    cy + oy + r * math.sin(math.radians(b0 + 90.0 * i / max(n, 1)))) for i in range(n + 1)]
        sq.append(tuple(wp2))
        course += sq
    else:
        course += [(cx + (R - (R - R / 3) * i / 144) * math.cos(a0 + 4 * math.pi * i / 144),
                    cy + (R - (R - R / 3) * i / 144) * math.sin(a0 + 4 * math.pi * i / 144)) for i in range(1, 145)]
pts = [q for l in legs[:n_pre - 1] for q in l['waypoints']] + [list(q) for q in course] + legs[n_pre - 1]['waypoints'][1:]
if n_pre < 2:
    pts = legs[0]['waypoints']
L = sum(math.dist(a, b) for a, b in zip(pts[:-1], pts[1:]))
print(f'[demo] D prefix: courses {",".join(kinds) or "none"} at WP2, {len(pts)} points, {L:.0f} m', file=sys.stderr)
with open(sys.argv[2], 'w') as f:
    f.write('X,Y\n')
    for x, y in pts:
        f.write(f'{x:.3f},{y:.3f}\n')
PYEOF
        env "${WIND_ENV[@]}" CTRACK_LEGS=${DEMO_D_LEGS:-WP3,WP5,WP6,GOAL} CTRACK_PREFIX=$PLAN/prefix.csv NOCLIMB_PREFIX_SPEED=$SPEED \
            "$REPO/scripts/ctrack.sh" fly "$PROFILE" "$NAME" START GOAL "$ALT" 5.0 "$TIMEOUT"
        PATH_CSV=$PLAN/prefix.csv ;;
    *)
        sed -n 2,16p "$0"; exit 1 ;;
esac

[ -d "$OUT" ] || { echo "[demo] no run dir $OUT"; exit 1; }
mkdir -p "$OUT/plan" && cp -r "$PLAN"/. "$OUT/plan/" && rm -rf "$PLAN"
{
    echo "scenario $SC, profile $PROFILE ($AF), speed $SPEED m/s, path height $ALT m, wind $WIND m/s"
    echo "result: $(cat "$OUT/result.txt" 2>/dev/null)"
    [ -f "$OUT/plan/pp_result.json" ] && python3 -c "
import json, sys; r = json.load(open(sys.argv[1]))
print(f\"Hybrid PP: {'ok' if r['success'] else 'FAILED'}, {r['length_m']} m vs shortest {r['reference_m']} m (ratio {r['ratio']}), \"
      f\"max planning {r['max_planning_ms']} ms\")
for l in r['legs']:
    print(f\"  {l['from']} -> {l['to']}: {l.get('length_m')} m / {l['reference_m']} m (x{l.get('ratio')}), {l['planning_ms']} ms\")" "$OUT/plan/pp_result.json"
    if [ -n "$PATH_CSV" ]; then
        python3 "$REPO/scripts/dev/ctrack_path_metrics.py" "$OUT" "$OUT/plan/$(basename "$PATH_CSV")" \
            ${ZONE:+--zone $ZONE} --json "$OUT/path_metrics.json"
    fi
    case "$SC" in C|D) python3 "$REPO/scripts/dev/ctrack_clearance.py" "$OUT" --layout "$LAYOUT" ;; esac
} 2>&1 | tee "$OUT/demo_summary.txt"
