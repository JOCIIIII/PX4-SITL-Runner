#!/bin/bash
# THE VEHICLE: the apps on the real vehicle's onboard computer (Jetson; image a4vai-onboard:humble from
# scripts/dev/onboard_livox_setup.sh). PX4 (Pixhawk 6X TELEM2: UXRCE_DDS_CFG 102, 921600) reaches the agent
# over serial; the Mid-360 is wired to eno1 (this computer 192.168.1.50/24, the lidar 192.168.1.100).
#
#   scripts/dev/onboard_real.sh agent start [DEV [BAUD]]  resident uXRCE-DDS agent (default /dev/ttyUSB0 921600).
#                                                         Leave it up: PX4 1.16 does not reconnect to a restarted
#                                                         agent (reboot the flight controller after a restart)
#   scripts/dev/onboard_real.sh agent stop | status
#   scripts/dev/onboard_real.sh shadow NAME   the chain on the carried vehicle (props off, outdoors with GPS): the
#                                             policy runs on the real lidar and PX4 pose, its commands go to
#                                             /shadow/... only. Watch /cloud, /noclimb/path in Foxglove
#                                             (ws://<this computer>:8765). New run (nose at the course first):
#                                             docker exec ros2-real bash -c "source /opt/ros/humble/setup.bash;
#                                             ros2 topic pub --once /shadow/reset std_msgs/msg/Bool '{data: true}'"
#   scripts/dev/onboard_real.sh fly PROFILE NAME      FLIGHT with the learned avoidance: the profile
#                                             (scripts/dev/flight_profiles/PROFILE.env - the PILS gate's file) gives
#                                             the policy and every knob; the goal is NOCLIMB_GOAL "x,y" = east, north
#                                             metres from the take-off point, or NOCLIMB_GOAL_GPS "lat,lon";
#                                             NOCLIMB_ALT the path height above the take-off point [m]
#   scripts/dev/onboard_real.sh fly-t1000 PROFILE NAME FLIGHT with JBNU's T1000 policy through PathFollowing
#                                             (scripts/ros2/apps/t1000_pf.sh): the same goal / NOCLIMB_ALT / go as fly;
#                                             from the profile only the airframe, lidar mount, follower knobs count
#                                             (T1000 flies 3 m/s; T1000_* knobs as t1000.sh)
#   scripts/dev/onboard_real.sh mission PROFILE NAME CSV   FLIGHT along GPS waypoints with PathFollowing alone
#                                             (scripts/ros2/apps/mission.sh): CSV rows lat,lon,alt (alt above the
#                                             take-off point; check it first: scripts/dev/gps_mission.py check CSV),
#                                             MISSION_SPEED [m/s] (default 3), MISSION_ALT take-off height (default 5);
#                                             MISSION_TYPE=wp_local for rows north,east,height
#   scripts/dev/onboard_real.sh calib NAME    the Mid-360 driver alone (props off) for the mount calibration, then
#   scripts/dev/onboard_real.sh tool SCRIPT [ARGS]   scripts/dev/SCRIPT run inside the container: livox_bench_check.py,
#                                             lidar_mount_calib.py shake | rest | target (prints NOCLIMB_LIDAR_* lines)
#   scripts/dev/onboard_real.sh state         the flight driver's state line (READY = waiting for go)
#   scripts/dev/onboard_real.sh go            the operator's go: the driver arms and takes off (only after READY)
#   scripts/dev/onboard_real.sh land          the driver lands in place now (the pilot's RC always overrides)
#   scripts/dev/onboard_real.sh stop NAME     stop, collect the logs into test_runs/NAME (the agent stays up)
# Flights: the driver (ros2_ws/src/flight_driver) arms only once everything is ready AND on `go`; it yields to the
# pilot for good on any mode change (RC: Position mode), never re-arms, lands in place on lost inputs, the fence
# (mission reach + 20 m, height + 10 m) or over speed.
# env: ONBOARD_DOMAIN (ROS_DOMAIN_ID = PX4's UXRCE_DDS_DOM_ID, default 0 as the vehicle is set now; the lab LAN
#      has other ROS2 hosts on 0 - set both to 37), ONBOARD_WS (default ~/Documents/A4VAI-SITL/ROS2),
#      ONBOARD_NO_LIDAR=1 (mission without the Mid-360 driver), LIVOX_LIDAR_IP (the lidar's address, default
#      192.168.1.100 = the X500's; the x8 profiles set the X8's 192.168.1.182 - for calib / shadow export it); shadow: SHADOW_AIRFRAME (x500 default | x8),
#      NOCLIMB_POLICY_DIR / NOCLIMB_POLICY (default
#      x500_speed_onnx/v1 / ca_x500_v1), NOCLIMB_SHADOW_AHEAD / NOCLIMB_SHADOW_MASK and the other NOCLIMB_* knobs
set -u
WS=${ONBOARD_WS:-$HOME/Documents/A4VAI-SITL/ROS2}
IMAGE=${ONBOARD_REAL_IMAGE:-a4vai-onboard:humble}
DOM=${ONBOARD_DOMAIN:-0}
A=a4vai-agent
C=ros2-real
CW=/home/user/workspace/ros2
REPO=$(dirname $(dirname $(dirname $(readlink -f "$0"))))
GPU=(); docker info 2>/dev/null | grep -qi nvidia && GPU=(--runtime nvidia)
ros() { docker exec $C bash -c "source /opt/ros/humble/setup.bash; $1"; }

# start_app NAME -e K=V ...: the app container (Mid-360 driver, foxglove, the app) + recorder + load logging
start_app() {
    local NAME=$1; shift
    local R=$WS/test_runs/$NAME
    if docker ps --format '{{.Names}}' | grep -qx $C; then
        echo "[onboard_real] $C is running - stop the last run first (onboard_real.sh stop NAME)"; exit 1
    fi
    mkdir -p "$R" "$WS/logs" "$WS/scripts/include"
    cp -r "$REPO"/scripts/ros2/* "$WS/scripts/" && cp -r "$REPO"/scripts/include/* "$WS/scripts/include/"
    docker rm -f $C > /dev/null 2>&1
    docker run -d --name $C --hostname $C --network host --ipc host "${GPU[@]}" --privileged -e ROS_DOMAIN_ID=$DOM \
        -e LIVOX_LIDAR_IP=${LIVOX_LIDAR_IP:-192.168.1.100} "$@" \
        -v "$WS":$CW -w /home/user/workspace --entrypoint bash "$IMAGE" $CW/scripts/onboard-real.sh > /dev/null \
        || { echo "[onboard_real] docker run failed"; exit 1; }
    docker exec -d $C bash -c "source /opt/ros/humble/setup.bash; source $CW/ros2_ws/install/setup.bash; \
        python3 $CW/scripts/tools/flight_recorder.py $CW/test_runs/$NAME > $CW/test_runs/$NAME/recorder.log 2>&1"
    rm -f "$R/.load_pids"
    if command -v tegrastats > /dev/null; then
        nohup tegrastats --interval 1000 --logfile "$R/tegrastats.log" > /dev/null 2>&1 & echo $! >> "$R/.load_pids"
    fi
    nohup top -b -c -d 2 -w 400 > "$R/top.log" 2>&1 & echo $! >> "$R/.load_pids"
}

# the profile's knobs (NOCLIMB_*, PROFILE_AIRFRAME) into this shell; a value already set by the caller wins
# (e.g. NOCLIMB_HOVER_THRUST=0.68 NOCLIMB_LIDAR_YAW_DEG=178 measured in the morning)
load_profile() {
    local P=$REPO/scripts/dev/flight_profiles/$1.env k v
    [ -f "$P" ] || { echo "[onboard_real] no profile $P ($(ls $REPO/scripts/dev/flight_profiles/*.env | xargs -n1 basename | tr '\n' ' '))"; exit 1; }
    while IFS='=' read -r k v; do
        [[ "$k" =~ ^[A-Z_][A-Z0-9_]*$ ]] || continue
        [ -n "${!k+x}" ] && { echo "[onboard_real] $k=${!k} (profile: ${v%%#*})"; continue; }
        v=$(echo "${v%%#*}" | sed -E 's/[[:space:]]+$//; s/^"(.*)"$/\1/')
        export "$k=$v"
    done < "$P"
}

case "${1:-}" in
agent)
    case "${2:-status}" in
    start)
        DEV=${3:-/dev/ttyUSB0}; BAUD=${4:-921600}
        if docker ps --format '{{.Names}}' | grep -qx $A; then
            echo "[onboard_real] the agent is already up - keep it (PX4 does not reconnect to a new one)"; exit 0
        fi
        [ -e "$DEV" ] || { echo "[onboard_real] $DEV not found (USB-serial to the Pixhawk's TELEM2?)"; exit 1; }
        docker rm -f $A > /dev/null 2>&1
        # --ipc host (both containers): Fast DDS talks over shared memory between processes on one host, and
        # each container has its own /dev/shm - without it the apps never see the agent's /fmu topics
        # UDP only (scripts/dev/fastdds_udp_only.xml): the agent's own Fast DDS did not meet the Humble apps over
        # shared memory on orin-nano-02
        docker run -d --restart unless-stopped --name $A --network host --ipc host --privileged -v /dev:/dev \
            -v "$REPO/scripts/dev/fastdds_udp_only.xml":/fastdds_udp_only.xml:ro -e FASTRTPS_DEFAULT_PROFILES_FILE=/fastdds_udp_only.xml \
            --entrypoint MicroXRCEAgent "$IMAGE" serial --dev "$DEV" -b "$BAUD" > /dev/null \
            && echo "[onboard_real] agent up on $DEV at $BAUD (DDS domain = PX4's UXRCE_DDS_DOM_ID)" ;;
    stop)
        docker rm -f $A > /dev/null 2>&1
        echo "[onboard_real] agent stopped - reboot the flight controller before the next start" ;;
    *)
        docker ps -a --filter name=^/$A$ --format '{{.Names}} {{.Status}}'; docker logs --tail 5 $A 2>&1 ;;
    esac ;;
shadow)
    NAME=${2:?usage: onboard_real.sh shadow NAME}
    ENV=(-e ROS2_APP=noclimb_ca -e SITL_AIRFRAME=${SHADOW_AIRFRAME:-x500} -e NOCLIMB_SHADOW=1
         -e NOCLIMB_POLICY_DIR=${NOCLIMB_POLICY_DIR:-x500_speed_onnx/v1} -e NOCLIMB_POLICY=${NOCLIMB_POLICY:-ca_x500_v1})
    for k in $(compgen -e | grep -E '^NOCLIMB_' | grep -vE '^NOCLIMB_(POLICY_DIR|POLICY|SHADOW)$'); do
        ENV+=(-e "$k=${!k}")
    done
    start_app "$NAME" "${ENV[@]}"
    echo "[onboard_real] $NAME: shadow run up (domain $DOM, ${NOCLIMB_POLICY:-ca_x500_v1}); logs $WS/logs;" \
         "Foxglove ws://$(hostname -I | awk '{print $1}'):8765" ;;
fly)
    PROFILE=${2:?usage: onboard_real.sh fly PROFILE NAME}; NAME=${3:?usage: onboard_real.sh fly PROFILE NAME}
    [ -n "${NOCLIMB_GOAL:-}${NOCLIMB_GOAL_GPS:-}" ] || { echo "[onboard_real] set NOCLIMB_GOAL \"east,north\" (m from the take-off point) or NOCLIMB_GOAL_GPS \"lat,lon\""; exit 1; }
    : ${NOCLIMB_ALT:?set NOCLIMB_ALT, the path height above the take-off point [m]}
    load_profile "$PROFILE"
    ENV=(-e ROS2_APP=noclimb_ca -e SITL_AIRFRAME=$PROFILE_AIRFRAME -e NOCLIMB_REQUIRE_GO=1 -e NOCLIMB_EKF_SETTLE=${NOCLIMB_EKF_SETTLE:-30})
    for k in $(compgen -e | grep -E '^NOCLIMB_' | grep -vE '^NOCLIMB_(SHADOW|REQUIRE_GO|EKF_SETTLE)$'); do ENV+=(-e "$k=${!k}"); done
    start_app "$NAME" "${ENV[@]}"
    echo "[onboard_real] $NAME: $PROFILE ($PROFILE_AIRFRAME, $NOCLIMB_POLICY) to ${NOCLIMB_GOAL_GPS:-${NOCLIMB_GOAL}} at" \
         "$NOCLIMB_ALT m; wait for READY (onboard_real.sh state), then onboard_real.sh go" ;;
fly-t1000)
    PROFILE=${2:?usage: onboard_real.sh fly-t1000 PROFILE NAME}; NAME=${3:?usage: onboard_real.sh fly-t1000 PROFILE NAME}
    [ -n "${NOCLIMB_GOAL:-}${NOCLIMB_GOAL_GPS:-}" ] || { echo "[onboard_real] set NOCLIMB_GOAL \"east,north\" (m from the take-off point) or NOCLIMB_GOAL_GPS \"lat,lon\""; exit 1; }
    : ${NOCLIMB_ALT:?set NOCLIMB_ALT, the path height above the take-off point [m]}
    load_profile "$PROFILE"
    ENV=(-e ROS2_APP=t1000_pf -e SITL_AIRFRAME=$PROFILE_AIRFRAME -e NOCLIMB_REQUIRE_GO=1 -e NOCLIMB_EKF_SETTLE=${NOCLIMB_EKF_SETTLE:-30})
    for k in $(compgen -e | grep -E '^(NOCLIMB|T1000)_' | grep -vE '^NOCLIMB_(SHADOW|REQUIRE_GO|EKF_SETTLE)$'); do ENV+=(-e "$k=${!k}"); done
    start_app "$NAME" "${ENV[@]}"
    echo "[onboard_real] $NAME: T1000 + PathFollowing ($PROFILE_AIRFRAME) to ${NOCLIMB_GOAL_GPS:-${NOCLIMB_GOAL}} at" \
         "$NOCLIMB_ALT m; wait for READY (onboard_real.sh state), then onboard_real.sh go" ;;
mission)
    PROFILE=${2:?usage: onboard_real.sh mission PROFILE NAME CSV}; NAME=${3:?usage: onboard_real.sh mission PROFILE NAME CSV}
    CSV=${4:?usage: onboard_real.sh mission PROFILE NAME CSV}
    [ -f "$CSV" ] || { echo "[onboard_real] no $CSV"; exit 1; }
    load_profile "$PROFILE"
    mkdir -p "$WS/missions" && cp "$CSV" "$WS/missions/$NAME.csv"
    [ "${MISSION_TYPE:-wp_gps}" = wp_gps ] && { python3 "$REPO/scripts/dev/gps_mission.py" check "$CSV" 2>/dev/null || true; }
    start_app "$NAME" -e ROS2_APP=mission -e SITL_AIRFRAME=$PROFILE_AIRFRAME -e MISSION_CSV=$CW/missions/$NAME.csv \
        -e MISSION_TYPE=${MISSION_TYPE:-wp_gps} -e MISSION_SPEED=${MISSION_SPEED:-3.0} -e MISSION_ALT=${MISSION_ALT:-5.0} \
        -e MISSION_REQUIRE_GO=1 -e MISSION_EKF_SETTLE=${MISSION_EKF_SETTLE:-30} \
        -e MISSION_HOVER_THRUST=${NOCLIMB_HOVER_THRUST:-0} -e MISSION_MAX_TILT_DEG=${NOCLIMB_MAX_TILT_DEG:-0} \
        -e MISSION_CORNER=${MISSION_CORNER:-1} -e ONBOARD_NO_LIDAR=${ONBOARD_NO_LIDAR:-0}
    echo "[onboard_real] $NAME: $PROFILE_AIRFRAME mission $CSV at ${MISSION_SPEED:-3.0} m/s; wait for READY" \
         "(onboard_real.sh state), then onboard_real.sh go" ;;
calib)
    NAME=${2:?usage: onboard_real.sh calib NAME}
    start_app "$NAME" -e ROS2_APP=idle
    echo "[onboard_real] $NAME: Mid-360 driver up, no app. PROPS OFF. Then, in order:"
    echo "  onboard_real.sh tool livox_bench_check.py 10            (frames, own returns by azimuth -> NOCLIMB_SELF_RADIUS)"
    echo "  onboard_real.sh tool lidar_mount_calib.py shake         (rock it about all axes: latency, roll/pitch/yaw)"
    echo "  onboard_real.sh tool lidar_mount_calib.py rest --yaw Y --centre-height H   (still: roll/pitch, height)"
    echo "  onboard_real.sh tool lidar_mount_calib.py target --yaw Y --roll R --pitch P (pole on the nose axis: yaw check)"
    echo "  onboard_real.sh stop $NAME" ;;
tool)
    T=${2:?usage: onboard_real.sh tool SCRIPT.py [ARGS] (scripts/dev/SCRIPT.py run inside $C)}; shift 2
    [ -f "$REPO/scripts/dev/$T" ] || { echo "[onboard_real] no scripts/dev/$T"; exit 1; }
    docker exec -i $C bash -c "source /opt/ros/humble/setup.bash; source $CW/livox_ws/install/setup.bash 2>/dev/null; \
        source $CW/ros2_ws/install/setup.bash; python3 - $*" < "$REPO/scripts/dev/$T" ;;
state)
    ros "timeout 10 ros2 topic echo --once /flight_driver/state --field data" 2>/dev/null | head -1 \
        || echo "[onboard_real] no state (driver not up yet?)" ;;
go)
    S=$(ros "timeout 10 ros2 topic echo --once /flight_driver/state --field data" 2>/dev/null | head -1)
    case "$S" in
    *phase=WAIT_GO*) ros "ros2 topic pub -t 3 -r 2 /flight_driver/go std_msgs/msg/Bool '{data: true}'" > /dev/null
                     echo "[onboard_real] go sent"; sleep 2; ros "timeout 10 ros2 topic echo --once /flight_driver/state --field data" | head -1 ;;
    *) echo "[onboard_real] not READY - no go sent: ${S:-no state}" ;;
    esac ;;
land)
    ros "ros2 topic pub -t 3 -r 2 /flight_driver/land std_msgs/msg/Bool '{data: true}'" > /dev/null && echo "[onboard_real] land sent" ;;
stop)
    NAME=${2:?usage: onboard_real.sh stop NAME}
    R=$WS/test_runs/$NAME
    [ -f "$R/.load_pids" ] && { kill $(cat "$R/.load_pids") 2>/dev/null; rm -f "$R/.load_pids"; }
    docker exec $C pkill -INT -f flight_recorder.py 2>/dev/null
    # the recorder writes cloud_map.npz on SIGINT; a long run takes over 30 s, and removing the container while it
    # writes leaves an empty file (shadow_x8_fly5, 10-17) - wait until the recorder has exited (up to 3 min)
    for i in $(seq 1 180); do docker exec $C pgrep -f flight_recorder.py > /dev/null 2>&1 || break; sleep 1; done
    docker exec $C bash -c "cp $CW/logs/*.log $CW/logs/noclimb_ca.csv $CW/logs/noclimb_ca_dump.npz $CW/logs/t1000_pf.csv $CW/test_runs/$NAME/ 2>/dev/null; \
        f=\$(ls -t $CW/logs/flight_driver_*.csv 2>/dev/null | head -1); [ -n \"\$f\" ] && cp \"\$f\" $CW/test_runs/$NAME/flight_driver.csv; \
        f=\$(ls -t $CW/logs/[0-9]*.csv 2>/dev/null | head -1); [ -n \"\$f\" ] && cp \"\$f\" $CW/test_runs/$NAME/pf_log.csv; \
        chmod -R a+rw $CW/test_runs/$NAME"
    docker rm -f $C > /dev/null 2>&1
    echo "[onboard_real] $NAME: logs in $R (the agent stays up; the PX4 log is on the flight controller's SD card)" ;;
*)
    sed -n 2,48p "$0"; exit 1 ;;
esac
