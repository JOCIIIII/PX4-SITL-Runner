#!/bin/bash
# ON THE VEHICLE'S JETSON: one PathFollowing-only test flight (no avoidance) - the path from scripts/dev/pf_shapes.py
# relative to where the vehicle stands, flown by scripts/dev/onboard_real.sh mission (flight_driver + PathFollowing,
# the corner limit on). The heading turns the shape into the free part of the field.
#
#   scripts/dev/pf_field.sh SHAPE SPEED HEADING [NAME]   SHAPE line | square | circle | spiral, SPEED m/s,
#                                                        HEADING compass deg (the line / first side / toward the centre)
#   scripts/dev/pf_field.sh table                        sizes and the space each needs
#   then: onboard_real.sh state (READY) -> onboard_real.sh go; after landing: onboard_real.sh stop NAME
# env: PF_FIELD_PROFILE (default x8_5s: airframe, hover thrust, tilt limit), MISSION_ALT (default 5 m),
#      PF_FIELD_SIZE / PF_FIELD_LAPS / PF_FIELD_TURN (pf_shapes.py --size / --laps / --turn),
#      ONBOARD_NO_LIDAR (default 1: no Mid-360 driver - PathFollowing does not use it)
set -eu
REPO=$(dirname $(dirname $(dirname $(readlink -f "$0"))))
if [ "${1:-}" = table ]; then python3 "$REPO/scripts/dev/pf_shapes.py" table; exit 0; fi
SHAPE=${1:?usage: pf_field.sh SHAPE SPEED HEADING [NAME]}; SPEED=${2:?speed m/s}; HEADING=${3:?heading deg}
NAME=${4:-pf_${SHAPE}_v${SPEED}_$(date +%H%M%S)}
ALT=${MISSION_ALT:-5}
CSV=/tmp/$NAME.csv
ARGS=(--heading "$HEADING" --alt "$ALT" --out "$CSV")
[ -n "${PF_FIELD_SIZE:-}" ] && ARGS+=(--size "$PF_FIELD_SIZE")
[ -n "${PF_FIELD_LAPS:-}" ] && ARGS+=(--laps "$PF_FIELD_LAPS")
[ -n "${PF_FIELD_TURN:-}" ] && ARGS+=(--turn "$PF_FIELD_TURN")
python3 "$REPO/scripts/dev/pf_shapes.py" "$SHAPE" "$SPEED" "${ARGS[@]}"
MISSION_TYPE=wp_local MISSION_SPEED=$SPEED MISSION_ALT=$ALT ONBOARD_NO_LIDAR=${ONBOARD_NO_LIDAR:-1} \
    "$REPO/scripts/dev/onboard_real.sh" mission "${PF_FIELD_PROFILE:-x8_5s}" "$NAME" "$CSV"
echo "[pf_field] $NAME: $SHAPE $SPEED m/s, heading $HEADING deg, $ALT m up. onboard_real.sh state -> go; afterwards onboard_real.sh stop $NAME"
