#!/bin/bash
# DEMONSTRATION SCENARIO TEST MAP (실증 시나리오 시험맵) ON UNREAL - THE ENTRY POINT (README.md, section 4)
#
#   ./scripts/scenario.sh unreal [x8|x500] [Scenario|Yard]  start the simulator with the airframe's lidar mount
#                                                         (restarts it when another airframe / map is running)
#   ./scripts/scenario.sh fly PROFILE NAME [FROM GOAL ALT SPEED TIMEOUT]
#                       SILS: the profile's apps run on this PC (learned avoidance -> PathFollowing -> flight_driver)
#   ./scripts/scenario.sh pils PROFILE NAME [FROM GOAL ALT SPEED TIMEOUT]
#                       PILS: the same apps on the onboard computer - ONBOARD_IP (required) and ONBOARD_SSH=user@host
#                       (the Jetson set up with scripts/dev/jetson_base_setup.sh + pils_jetson_setup.sh)
#   SCENARIO_LLM=1 ./scripts/scenario.sh fly|pils PROFILE NAME
#                       the mission from the operator's words: a local LLM (Ollama, LLM_URL / LLM_MODEL) turns a
#                       Korean command into legs over the place table (LLM_PLACES, default: the test map places); then
#   ./scripts/scenario.sh llm                             (another terminal) type the command, "확인" to fly
#   SCENARIO_SEARCH=1 ./scripts/scenario.sh fly PROFILE NAME  the same with the search camera, 6 m up: "시험장에서 사람
#                       찾아" -> a lawnmower, YOLOX on the camera, stop + photo + description per person, home;
#                       people to find: scripts/dev/spawn_people.py
#   ./scripts/scenario.sh demo A|B|B-square|B-circle|B-spiral|C|D NAME [PROFILE]
#                       the final demonstration scenarios (2026-11-06 workshop): A Hybrid PP around the tents, B path
#                       following through the fan wind, C avoidance WP3 -> GOAL, D all in one flight (PP path flown by
#                       PathFollowing to WP3, then the policy) - scripts/dev/scenario_demo.sh; Hybrid PP needs
#                       scripts/dev/scenario_pp.sh --install once
#   ./scripts/scenario.sh test pp|pf|ca|pp-pf|pf-ca|all NAME [PROFILE]  (test table: the latest result of each cell)
#                       the test matrix: PP / PF / CA unit tests, the PP-PF and PF-CA pair tests and the integrated
#                       test (path following: PathFollowing MPPI), with TEST_CA=inha|jbnu|super (learned policy |
#                       T1000 | SUPER) - scripts/dev/scenario_test.sh
#   ./scripts/scenario.sh profiles                        the flight profiles (scripts/dev/flight_profiles/*.env)
#   ./scripts/scenario.sh stop                            stop the simulator and its containers
#
# FROM / GOAL: layout waypoint names (default WP3 -> GOAL) or field coordinates "X,Y"; ALT [m above ground]
# (default 1.7), SPEED (SUPER max_vel, unused by the learned avoidance), TIMEOUT [s] (default 480).
# Results: ~/Documents/A4VAI-SITL/ROS2/test_runs/NAME (logs, px4_sitl.ulg, noclimb_ca.csv); analyse with
#   python3 scripts/dev/scenario_clearance.py RUN_DIR    (closest approach to every obstacle, prop-tip margin)
#   python3 scripts/dev/shake_metrics.py RUN_DIR         (roll / pitch shake)
# env: SCENARIO_MAP (Scenario default | Yard), SCENARIO_DIR (the A4VAI-Unreal folder; default the installed one), any
#   NOCLIMB_* / profile knob (wins over the profile), and the knobs of scripts/dev/scenario_super.sh / pils_gate.sh.
set -u
BASE_DIR=$(dirname "$(readlink -f "$0")")
REPO=$(dirname "$BASE_DIR")
source "$BASE_DIR/include/commonEnv.sh"
UNREAL=${SCENARIO_DIR:-$SCENARIO_UNREAL_DIR}
[ -d "$UNREAL" ] || UNREAL=$HOME/Desktop/00_Project/00_A4VAI/00_Development/00_Simulator/02_InhouseSim/00_Unreal/00_Map
STATE=${XDG_RUNTIME_DIR:-/tmp}/scenario_unreal.state    # "airframe map" of the simulator this script started
LOG_DIR=$SITL_DEPLOY_DIR/logs
PROFILES=$REPO/scripts/dev/flight_profiles

api_up() { timeout 2 bash -c "exec 3<>/dev/tcp/127.0.0.1/41451" 2>/dev/null; }
unreal_pids() { pgrep -f "[A]4VAISIM-Linux-Shipping|[U]nrealEditor .*A4VAISIM\.uproject"; }

# SCENARIO_CAMERA=1: the same with a 1280x720 camera 'search_cam' looking 50 deg down (person search)
settings_for() {
    local cam=$([ "${SCENARIO_CAMERA:-0}" = 1 ] && echo _cam)
    case "$1" in
        x8)   [ -n "$cam" ] && echo "$REPO/scripts/dev/unreal_settings/settings_gazebo_mid360_x8_cam.json" \
                            || echo "$UNREAL/settings/settings_gazebo_mid360.json" ;;
        x500) echo "$REPO/scripts/dev/unreal_settings/settings_gazebo_mid360_x500${cam}.json" ;;
        *)    echo "[scenario] airframe '$1': x8 or x500" >&2; return 1 ;;
    esac
}

unreal_stop() {
    local p
    p=$(unreal_pids)
    if [ -n "$p" ]; then
        echo "[scenario] stopping Unreal ($(echo $p | tr '\n' ' '))"
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
    [ -f "$settings" ] || { echo "[scenario] $settings missing - is A4VAI-Unreal installed (scripts/install.sh)?"; return 1; }
    [ -x "$UNREAL/run_unreal.sh" ] || { echo "[scenario] no $UNREAL/run_unreal.sh - run scripts/install.sh (or set SCENARIO_DIR)"; return 1; }
    if api_up && [ "$(cat "$STATE" 2>/dev/null)" = "$af $map $(basename "$settings")" ]; then
        echo "[scenario] Unreal is up ($af, $map)"; return 0
    fi
    if api_up || [ -n "$(unreal_pids)" ]; then
        echo "[scenario] Unreal runs with other settings (or was started by hand) - restarting it for $af / $map"
        unreal_stop
        for _ in $(seq 20); do api_up || break; sleep 1; done
    fi
    mkdir -p "$LOG_DIR"
    echo "[scenario] starting Unreal: map $map, $(basename "$settings") (log $LOG_DIR/scenario_unreal.log)"
    (cd "$UNREAL" && SCENARIO_MAP=$map setsid nohup ./run_unreal.sh "$settings" > "$LOG_DIR/scenario_unreal.log" 2>&1 < /dev/null &)
    local t0=$SECONDS
    until api_up; do
        if [ $((SECONDS - t0)) -gt 240 ]; then
            echo "[scenario] Unreal API (41451) not up after 240 s - see $LOG_DIR/scenario_unreal.log"; return 1
        fi
        sleep 3
    done
    sleep 25                        # the map and the AirSim vehicle finish loading after the port opens
    echo "$af $map $(basename "$settings")" > "$STATE"
    echo "[scenario] Unreal up ($af, $map)"
}

profile_airframe() {
    local p=$PROFILES/$1.env
    [ -f "$p" ] || { echo "[scenario] no profile $p - see: ./scripts/scenario.sh profiles" >&2; return 1; }
    sed -nE 's/^PROFILE_AIRFRAME="?([a-z0-9]+)"?.*/\1/p' "$p" | head -1
}

CMD=${1:-}
case "$CMD" in
    unreal)
        unreal_start "${2:-x8}" "${3:-${SCENARIO_MAP:-Scenario}}" ;;
    fly|pils)
        PROFILE=${2:?usage: scenario.sh $CMD PROFILE NAME [FROM GOAL ALT SPEED TIMEOUT]}
        NAME=${3:?usage: scenario.sh $CMD PROFILE NAME [FROM GOAL ALT SPEED TIMEOUT]}
        AF=$(profile_airframe "$PROFILE") || exit 1
        # SCENARIO_SEARCH=1: person search session = the LLM mission + the camera, flown 6 m up by default
        [ "${SCENARIO_SEARCH:-0}" = 1 ] && export SCENARIO_LLM=1 SCENARIO_CAMERA=1
        if [ "$CMD" = pils ] && [ -z "${ONBOARD_IP:-}" ]; then
            echo "[scenario] pils needs ONBOARD_IP=<onboard computer address> (and ONBOARD_SSH=user@host to start it from here)"
            exit 1
        fi
        # the compose stack's network (scripts/run.sh, also left by install.sh's PX4 build) takes the same 172.70/16
        if docker network inspect "$SITL_NETWORK_NAME" > /dev/null 2>&1; then
            if [ "$(docker network inspect -f '{{len .Containers}}' "$SITL_NETWORK_NAME")" = 0 ]; then
                docker network rm "$SITL_NETWORK_NAME" > /dev/null
            else
                echo "[scenario] the A4VAI-SITL compose stack is running ($SITL_NETWORK_NAME) - stop it first: ./scripts/stop.sh <profile>"
                exit 1
            fi
        fi
        unreal_start "$AF" "${SCENARIO_MAP:-Scenario}" || exit 1
        export SCENARIO_MAP=${SCENARIO_MAP:-Scenario} SCENARIO_DIR=$UNREAL
        shift 3
        [ "$SCENARIO_MAP" = Yard ] && [ $# -eq 0 ] && set -- START GOAL    # the yard layout's waypoints
        [ "${SCENARIO_SEARCH:-0}" = 1 ] && [ $# -eq 0 ] && set -- WP3 GOAL 6.0 5.0 1800  # above the obstacles
        [ "${SCENARIO_LLM:-0}" = 1 ] && [ $# -eq 0 ] && set -- WP3 GOAL 1.7 5.0 1200  # time to talk and fly
        SCENARIO_MODE=$([ "$CMD" = fly ] && echo sils || echo pils) exec "$REPO/scripts/dev/pils_gate.sh" "$PROFILE" "$NAME" "$@" ;;
    demo)
        # the final demonstration scenarios A (path planning) / B (path following in the fan wind) / C (avoidance) /
        # D (integrated), see scripts/dev/scenario_demo.sh
        shift
        exec "$REPO/scripts/dev/scenario_demo.sh" "$@" ;;
    llm)
        # the operator console of the mission LLM, where the apps run (PILS: ONBOARD_SSH)
        CMD_IN='source /opt/ros/humble/setup.bash; source /home/user/workspace/ros2/ros2_ws/install/setup.bash; python3 /home/user/workspace/ros2/scripts/tools/llm_console.py'
        if [ -n "${ONBOARD_SSH:-}" ]; then
            exec ssh -t "$ONBOARD_SSH" "docker exec -it ros2-onboard bash -c '$CMD_IN'"
        elif docker ps --format '{{.Names}}' | grep -qx ros2-onboard; then
            exec docker exec -it ros2-onboard bash -c "$CMD_IN"
        else
            exec docker exec -it ros2-env bash -c "$CMD_IN"
        fi ;;
    profiles)
        printf "%-8s %-6s %s\n" PROFILE FRAME "POLICY (knobs: $PROFILES/PROFILE.env, notes: $PROFILES/README.md)"
        for p in "$PROFILES"/*.env; do
            printf "%-8s %-6s %s\n" "$(basename "$p" .env)" "$(profile_airframe "$(basename "$p" .env)")" \
                "$(sed -nE 's/^NOCLIMB_POLICY_DIR="?([^" #]+)"?.*/\1/p' "$p" | head -1)"
        done ;;
    stop)
        unreal_stop
        docker rm -f scenario-sitl ros2-env pils-airsim > /dev/null 2>&1
        # its 172.70/16 blocks the compose stack's network (also one left by an earlier release under another name)
        for n in $(docker network ls -q); do
            [ "$(docker network inspect -f '{{range .IPAM.Config}}{{.Subnet}}{{end}}' "$n" 2>/dev/null)" = 172.70.0.0/16 ] \
                && [ "$(docker network inspect -f '{{.Name}}' "$n")" != "$SITL_NETWORK_NAME" ] && docker network rm "$n" > /dev/null 2>&1
        done
        echo "[scenario] stopped" ;;
    test)
        # the simulator test matrix (unit / pair / integrated tests, PF and CA choices): scripts/dev/scenario_test.sh
        shift
        exec "$REPO/scripts/dev/scenario_test.sh" "$@" ;;
    *)
        sed -n '2,28p' "$0"; exit 1 ;;
esac
