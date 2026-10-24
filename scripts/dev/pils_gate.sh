#!/bin/bash
# The gate before a real flight: a flight profile (scripts/dev/flight_profiles/PROFILE.env - the file
# scripts/dev/onboard_real.sh fly sources on the vehicle) flown on C-Track with the apps on the onboard computer
# (PILS: scripts/dev/pils_onboard.sh there), flight_driver included.
#   scripts/dev/pils_gate.sh PROFILE NAME [FROM GOAL ALT SPEED TIMEOUT]   (default WP3 GOAL 1.7 5.0 480)
# env: CTRACK_MODE (pils, default | sils = the same apps on this PC), ONBOARD_IP / ONBOARD_SSH (default the Jetson
#      over Tailscale, 100.64.0.135 - then PILS_LINK=wan, see ctrack_super.sh; on the lab LAN 192.168.219.152),
#      PILS_DOMAIN (default 37)
# Unreal must run with the airframe's lidar mount: X500 settings_gazebo_mid360_x500.json, X8 settings_gazebo_mid360.json
# (A4VAI-Unreal run_unreal.sh). The Jetson needs the current scripts and flight_driver built:
# scripts/dev/pils_jetson_setup.sh.
set -u
REPO=$(dirname $(dirname $(dirname $(readlink -f "$0"))))
PROFILE=${1:?usage: pils_gate.sh PROFILE NAME [FROM GOAL ALT SPEED TIMEOUT]}
NAME=${2:?usage: pils_gate.sh PROFILE NAME [FROM GOAL ALT SPEED TIMEOUT]}
P=$REPO/scripts/dev/flight_profiles/$PROFILE.env
[ -f "$P" ] || { echo "[pils_gate] no profile $P"; exit 1; }
# the profile's knobs; a value already set by the caller wins
while IFS='=' read -r k v; do
    [[ "$k" =~ ^[A-Z_][A-Z0-9_]*$ ]] || continue
    [ -n "${!k+x}" ] && { echo "[pils_gate] $k=${!k} (profile: ${v%%#*})"; continue; }
    v=$(echo "${v%%#*}" | sed -E 's/[[:space:]]+$//; s/^"(.*)"$/\1/')
    export "$k=$v"
done < "$P"
export CTRACK_APP=noclimb_ca SIM_LIDAR=${SIM_LIDAR:-mid360} CTRACK_MODE=${CTRACK_MODE:-pils}
if [ "$CTRACK_MODE" = pils ]; then
    export ONBOARD_IP=${ONBOARD_IP:-100.64.0.135} ONBOARD_SSH=${ONBOARD_SSH:-jetson@100.64.0.135} PILS_DOMAIN=${PILS_DOMAIN:-37}
    # Tailscale (100.64/10, e.g. the Jetson on its 5G tether): the routed-link PILS (ctrack_super.sh PILS_LINK)
    case "$ONBOARD_IP" in 100.*) export PILS_LINK=${PILS_LINK:-wan} ;; *) export PILS_LINK=${PILS_LINK:-lan} ;; esac
fi
echo "[pils_gate] $PROFILE ($PROFILE_AIRFRAME, $NOCLIMB_POLICY_DIR/$NOCLIMB_POLICY, driver $NOCLIMB_DRIVER) -> $NAME, $CTRACK_MODE ${PILS_LINK:-}"
exec "$REPO/scripts/dev/ctrack_super.sh" "$NAME" "${3:-WP3}" "${4:-GOAL}" "${5:-1.7}" "${6:-5.0}" "${7:-480}" "$PROFILE_AIRFRAME"
