#!/bin/bash
# C-TRACK UNREAL FIELD - THE ENTRY POINT (README.md "C-Track")
#
#   ./scripts/ctrack.sh unreal [x8|x500] [CTrack|Yard]   start the simulator with the airframe's lidar mount
#                                                         (restarts it when another airframe / map is running)
#   ./scripts/ctrack.sh fly PROFILE NAME [FROM GOAL ALT SPEED TIMEOUT]
#                       SILS: the profile's apps run on this PC (learned avoidance -> PathFollowing -> flight_driver)
#   ./scripts/ctrack.sh pils PROFILE NAME [FROM GOAL ALT SPEED TIMEOUT]
#                       PILS: the same apps on the onboard computer - ONBOARD_IP (required) and ONBOARD_SSH=user@host
#                       (the Jetson set up with scripts/dev/jetson_base_setup.sh + pils_jetson_setup.sh)
#   ./scripts/ctrack.sh profiles                          the flight profiles (scripts/dev/flight_profiles/*.env)
#   ./scripts/ctrack.sh stop                              stop the simulator and its containers
#
# FROM / GOAL: layout waypoint names (default WP3 -> GOAL) or field coordinates "X,Y"; ALT [m above ground]
# (default 1.7), SPEED (SUPER max_vel, unused by the learned avoidance), TIMEOUT [s] (default 480).
# Results: ~/Documents/A4VAI-SITL/ROS2/test_runs/NAME (logs, px4_sitl.ulg, noclimb_ca.csv); analyse with
#   python3 scripts/dev/ctrack_clearance.py RUN_DIR      (closest approach to every obstacle, prop-tip margin)
#   python3 scripts/dev/shake_metrics.py RUN_DIR         (roll / pitch shake)
# env: CTRACK_MAP (CTrack default | Yard), CTRACK_DIR (the A4VAI-Unreal folder; default the installed one), any
#   NOCLIMB_* / profile knob (wins over the profile), and the knobs of scripts/dev/ctrack_super.sh / pils_gate.sh.
set -u
BASE_DIR=$(dirname "$(readlink -f "$0")")
REPO=$(dirname "$BASE_DIR")
source "$BASE_DIR/include/commonEnv.sh"
CTRACK=${CTRACK_DIR:-$CTRACK_UNREAL_DIR}
[ -d "$CTRACK" ] || CTRACK=$HOME/Desktop/00_Project/00_A4VAI/00_Development/00_Simulator/02_InhouseSim/Unreal
STATE=${XDG_RUNTIME_DIR:-/tmp}/ctrack_unreal.state      # "airframe map" of the simulator this script started
LOG_DIR=$SITL_DEPLOY_DIR/logs
PROFILES=$REPO/scripts/dev/flight_profiles

api_up() { timeout 2 bash -c "exec 3<>/dev/tcp/127.0.0.1/41451" 2>/dev/null; }
unreal_pids() { pgrep -f "[C]TrackSim-Linux-Shipping|[U]nrealEditor .*CTrackSim.uproject"; }

settings_for() {
    case "$1" in
        x8)   echo "$CTRACK/settings/settings_gazebo_mid360.json" ;;
        x500) echo "$REPO/scripts/dev/unreal_settings/settings_gazebo_mid360_x500.json" ;;
        *)    echo "[ctrack] airframe '$1': x8 or x500" >&2; return 1 ;;
    esac
}

unreal_stop() {
    local p
    p=$(unreal_pids)
    if [ -n "$p" ]; then
        echo "[ctrack] stopping Unreal ($(echo $p | tr '\n' ' '))"
        kill $p 2>/dev/null
        for _ in $(seq 30); do [ -z "$(unreal_pids)" ] && break; sleep 1; done
        p=$(unreal_pids); [ -n "$p" ] && kill -9 $p 2>/dev/null
    fi
    rm -f "$STATE"
}

# unreal_start AIRFRAME MAP: (re)start Unreal unless this script already runs it with the same airframe and map
unreal_start() {
    local af=$1 map=$2 settings
    settings=$(settings_for "$af") || return 1
    [ -f "$settings" ] || { echo "[ctrack] $settings missing - is A4VAI-Unreal installed (scripts/install.sh)?"; return 1; }
    [ -x "$CTRACK/run_unreal.sh" ] || { echo "[ctrack] no $CTRACK/run_unreal.sh - run scripts/install.sh (or set CTRACK_DIR)"; return 1; }
    if api_up && [ "$(cat "$STATE" 2>/dev/null)" = "$af $map" ]; then
        echo "[ctrack] Unreal is up ($af, $map)"; return 0
    fi
    if api_up || [ -n "$(unreal_pids)" ]; then
        echo "[ctrack] Unreal runs with other settings (or was started by hand) - restarting it for $af / $map"
        unreal_stop
        for _ in $(seq 20); do api_up || break; sleep 1; done
    fi
    mkdir -p "$LOG_DIR"
    echo "[ctrack] starting Unreal: map $map, $(basename "$settings") (log $LOG_DIR/ctrack_unreal.log)"
    (cd "$CTRACK" && CTRACK_MAP=$map setsid nohup ./run_unreal.sh "$settings" > "$LOG_DIR/ctrack_unreal.log" 2>&1 < /dev/null &)
    local t0=$SECONDS
    until api_up; do
        if [ $((SECONDS - t0)) -gt 240 ]; then
            echo "[ctrack] Unreal API (41451) not up after 240 s - see $LOG_DIR/ctrack_unreal.log"; return 1
        fi
        sleep 3
    done
    sleep 25                        # the map and the AirSim vehicle finish loading after the port opens
    echo "$af $map" > "$STATE"
    echo "[ctrack] Unreal up ($af, $map)"
}

profile_airframe() {
    local p=$PROFILES/$1.env
    [ -f "$p" ] || { echo "[ctrack] no profile $p - see: ./scripts/ctrack.sh profiles" >&2; return 1; }
    sed -nE 's/^PROFILE_AIRFRAME="?([a-z0-9]+)"?.*/\1/p' "$p" | head -1
}

CMD=${1:-}
case "$CMD" in
    unreal)
        unreal_start "${2:-x8}" "${3:-${CTRACK_MAP:-CTrack}}" ;;
    fly|pils)
        PROFILE=${2:?usage: ctrack.sh $CMD PROFILE NAME [FROM GOAL ALT SPEED TIMEOUT]}
        NAME=${3:?usage: ctrack.sh $CMD PROFILE NAME [FROM GOAL ALT SPEED TIMEOUT]}
        AF=$(profile_airframe "$PROFILE") || exit 1
        if [ "$CMD" = pils ] && [ -z "${ONBOARD_IP:-}" ]; then
            echo "[ctrack] pils needs ONBOARD_IP=<onboard computer address> (and ONBOARD_SSH=user@host to start it from here)"
            exit 1
        fi
        unreal_start "$AF" "${CTRACK_MAP:-CTrack}" || exit 1
        export CTRACK_MAP=${CTRACK_MAP:-CTrack} CTRACK_DIR=$CTRACK
        shift 3
        [ "$CTRACK_MAP" = Yard ] && [ $# -eq 0 ] && set -- START GOAL      # the yard layout's waypoints
        CTRACK_MODE=$([ "$CMD" = fly ] && echo sils || echo pils) exec "$REPO/scripts/dev/pils_gate.sh" "$PROFILE" "$NAME" "$@" ;;
    profiles)
        printf "%-8s %-6s %s\n" PROFILE FRAME "POLICY (knobs: $PROFILES/PROFILE.env, notes: $PROFILES/README.md)"
        for p in "$PROFILES"/*.env; do
            printf "%-8s %-6s %s\n" "$(basename "$p" .env)" "$(profile_airframe "$(basename "$p" .env)")" \
                "$(sed -nE 's/^NOCLIMB_POLICY_DIR="?([^" #]+)"?.*/\1/p' "$p" | head -1)"
        done ;;
    stop)
        unreal_stop
        docker rm -f ctrack-sitl ros2-env pils-airsim > /dev/null 2>&1
        docker network rm ctrack-ros2 > /dev/null 2>&1       # its 172.70/16 blocks the compose stack's network
        echo "[ctrack] stopped" ;;
    *)
        sed -n '2,22p' "$0"; exit 1 ;;
esac
