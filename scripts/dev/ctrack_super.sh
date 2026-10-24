#!/bin/bash
# Dev harness: SUPER (ROS2_APP=super_pf) on the C-Track field of the InhouseSim Unreal sim
# (UE 5.8 + Cosys-AirSim 3.5 + its own Gazebo/PX4 container ctrack-sitl) instead of the
# A4VAI-SITL compose stack.
#
#   1. start the Unreal map first:  $CTRACK_DIR/run_unreal.sh   (API port 41451 on the host)
#   2. scripts/dev/ctrack_super.sh NAME [FROM] [GOAL] [ALT] [SPEED] [TIMEOUT] [AIRFRAME]
#        FROM / GOAL: layout waypoint names (default WP3 = the test drawing's WP2 -> GOAL) or
#        track coordinates X,Y (e.g. 70,29.5 -> 15,29.5 runs into the RC cart: guard test)
#        ALT [m above ground] (default 1.5: below the 2 m poles / 3 m wall); PX4's local z is
#        relative to the vehicle centre resting on the ground (x8 0.44 m up), so the take-off
#        height / goal z / SUPER band use ALT - 0.44. SPEED = SUPER max_vel (5.0)
#        SUPER's band (ROG-Map, x8 config: resolution 0.15, inflation 0.162 -> rounded up to
#        0.3 x 3 steps = 0.9 m >= robot_r 0.8, so A* paths keep the corridor's robot_r from raw
#        points - 1 step let A* pass 0.6 m from a pole and every corridor failed). Effective ground
#        = (floor(g/0.3)+3)*0.3+0.15, ceiling = (floor(c/0.3)-3)*0.3-0.15; A* keeps (ground+1.2,
#        ceiling-1.2), the corridor only (ground+0.8, ceiling-0.8): dodging a pole the trajectory
#        climbed above the A* band and SUPER could not search from there (hovered beside the first
#        pole, ctrack_123_a). SUPER's corridor_z_margin (super_planner patch) clamps the corridor to
#        the A* band: 1.15, a hair wider than A*'s 1.2 - A* paths run right up to their band edge
#        and a corridor narrower than that (1.35) failed every seed reaching it: SUPER stopped
#        replanning and the vehicle braked to a halt twice (ctrack_123_c). Ground -1.5, ceiling
#        z+2.94 (z = ALT-0.44): trajectory centre within z-0.36 .. z+0.64, A* far below the 2.46 m
#        it needs to hop an inflated 2 m pole - lateral avoidance only. (Virtual ground 0 = effective 1.05: SUPER saw
#        a 1.5 m hover as inside the ground and never planned.)
#        CTRACK_GROUND / CTRACK_CEIL / CTRACK_ZMARGIN override.
#        The RC cart crosses the lane at 1 m/s into SUPER's committed trajectory; SUPER's static map
#        cannot stop for it (ctrack_123_d: hit at 3-5 m/s, flipped) - super_speed_guard.py brakes
#        for it (SUPER_PF_SPEED_GUARD=1; CTRACK_GUARD=0 turns it off).
#        Height: SUPER's band still lets it plan 1.5-2.1 m while dodging (ctrack_123_q median
#        1.74 m at ALT 1.5); the waypoints to PF are held at ALT (SUPER_PF_HOLD_Z; CTRACK_HOLD=0 off),
#        and lidar returns lower than 0.5 m under the held centre (below the skids) are kept from
#        SUPER (SUPER_PF_STATIC_MIN_Z: mis-registered ground stalled ctrack_s900_d).
# CTRACK_APP=noclimb_ca flies the learned avoidance policy instead of SUPER (the same ROS2 app as
#   in the forest, scripts/ros2/apps/noclimb_ca.sh: straight path FROM -> GOAL at ALT, the policy
#   steers around): its NOCLIMB_* knobs come from envs/ros2.env, overridden by the caller's
#   environment (NOCLIMB_POLICY_DIR=ca_big_onnx NOCLIMB_POLICY=ca_big for the 1 m class bundle);
#   the lidar tilt gate is on (NOCLIMB_MAX_TILT_RATE 0.8, as SUPER here). No speed guard: the
#   policy meets the RC cart on its own. Unreal for these runs: run_unreal.sh
#   settings/settings_gazebo_policy10m.json (lidar Range 10 m = the policy's d_sensor_max, the
#   node drops returns beyond 10.5 m anyway; lidar hits drawn green) - SUPER needs the default
#   settings_gazebo.json (Range 40: /cloud_static keeps 25 m).
#
# - ctrack-sitl (Gazebo + PX4, host network) runs CTRACK_DIR's own entry script with only the
#   spawn changed: the vehicle stands on FROM facing GOAL (the original spawns at Start).
#   PX4's local origin is then FROM.
# - Our ROS2 image runs alone (no px4-env / gz-sim / airsim-binary) on a 172.70.0.0/16 bridge at
#   172.70.0.5: this PX4 build's uXRCE-DDS client targets 172.70.0.5:8888 (rcS), and ctrack-sitl
#   sets no ROS_DOMAIN_ID, so the DDS domain is 0. AirSim is the host, 172.70.0.1:41451. The
#   AirSim ROS2 bridge (Cosys 3.2) talks to the Cosys 3.5 plugin: the RPC adaptor headers of the
#   two versions are identical.
# - the goal CSV gets GOAL - FROM in PX4 local ENU (east = dX, north = -dY); restored at the end.
# CTRACK_MODE=pils (Processor-In-the-Loop): the PC runs only the simulation (Unreal, ctrack-sitl); the
#   ROS2 apps run on the onboard computer (scripts/dev/pils_onboard.sh there), reached through the two
#   links the vehicle has: PX4's uXRCE-DDS client is pointed at ONBOARD_IP (a copy of PX4's rcS, the
#   build stays as it is) and the onboard AirSim bridge pulls the lidar from Unreal over RPC 41451.
#   ONBOARD_IP (required; 127.0.0.1 = loopback test on this PC), PILS_DOMAIN (ROS_DOMAIN_ID, default 0),
#   ONBOARD_SSH user@host (optional: start/stop the onboard side over ssh and copy its run dir back;
#   ONBOARD_REPO = the PX4-SITL-Runner checkout there, default PX4-SITL-Runner in its home). Without ONBOARD_SSH
#   the harness prints the onboard command and waits for the arm. The goal goes to the onboard app as
#   NOCLIMB_GOAL / NOCLIMB_ALT (no goal file; super_pf would read the onboard goal CSV). Arm/land is read from PX4 itself (px4-listener).
#   PILS_LINK=wan (the onboard computer only reachable over a routed link, e.g. Tailscale on its 5G tether, ~40 ms
#   round trip; scripts/dev/pils_gate.sh sets it for 100.x addresses): PX4's uXRCE time sync off (UXRCE_DDS_SYNCT 0 -
#   PX4 drops samples over 10 ms RTT and never converged, so it never created a topic; without it PX4 stamps
#   incoming setpoints on receipt), and the AirSim bridge runs here next to Unreal (container pils-airsim; its
#   RPC polling over the link timed out) with its lidar and odometry relayed to the onboard computer over one TCP
#   connection (scripts/ros2/tools/topic_relay.py, port PILS_RELAY_PORT, default 9750).
# output: ~/Documents/A4VAI-SITL/ROS2/test_runs/NAME (as fly_test.sh) + px4_sitl.ulg; analyse with
#   scripts/dev/ctrack_clearance.py RUN_DIR and scripts/dev/shake_metrics.py RUN_DIR
set -u
NAME=$1; FROM=${2:-WP3}; GOALWP=${3:-GOAL}; ALT=${4:-1.5}; SPEED=${5:-5.0}; TIMEOUT=${6:-420}
AIRFRAME=${7:-x8}
APP=${CTRACK_APP:-super_pf}
MODE=${CTRACK_MODE:-sils}
if [ "$MODE" = pils ]; then
    : ${ONBOARD_IP:?CTRACK_MODE=pils needs ONBOARD_IP (the onboard computer, e.g. Tailscale 100.64.0.135)}
    PILS_DOMAIN=${PILS_DOMAIN:-0}
fi
REPO=$(dirname $(dirname $(dirname $(readlink -f "$0"))))
W=~/Documents/A4VAI-SITL
# the C-Track Unreal folder (A4VAI-Unreal): CTRACK_DIR, else the one scripts/install.sh put next to the workspace,
# else the developers' editor checkout
if [ -z "${CTRACK_DIR:-}" ]; then
    CTRACK_DIR=$W/A4VAI-Unreal
    [ -d "$CTRACK_DIR" ] || CTRACK_DIR=$HOME/Desktop/00_Project/00_A4VAI/00_Development/00_Simulator/02_InhouseSim/Unreal
fi
CTRACK=$CTRACK_DIR
# CTRACK_MAP: CTrack (default) or Yard - the Inha aerospace campus courtyard from a SLAM scan (waypoints START,
# GOAL; tree canopies from 2-3 m up). Start Unreal on the same map: CTRACK_MAP=Yard $CTRACK/run_unreal.sh ...
MAP=$(basename "${CTRACK_MAP:-CTrack}")
case "$MAP" in
    Yard) LAYOUT=${CTRACK_LAYOUT:-$CTRACK/layout/yard_slam_layout.json} ;;
    *)    LAYOUT=${CTRACK_LAYOUT:-$CTRACK/layout/ctrack_layout.json} ;;
esac
IMAGE=$(grep '^ROS2_ENV_IMAGE=' $REPO/envs/ros2.env | cut -d= -f2 | tr -d '"')
OUT=$W/ROS2/test_runs/$NAME
CMNT=/home/user/workspace/ros2/test_runs/$NAME
GOALS=$W/ROS2/ros2_ws/src/realgazebo_fastplanner_adapters/config/goals_super_far.csv
mkdir -p "$OUT"; rm -rf "$OUT"/*

if ! timeout 2 bash -c "exec 3<>/dev/tcp/127.0.0.1/41451" 2>/dev/null; then
    echo "[ctrack] Unreal API (41451) is not up - start $CTRACK/run_unreal.sh first"; exit 1
fi
if docker ps --format '{{.Names}}' | grep -qx px4-env; then
    echo "[ctrack] the A4VAI-SITL stack is running - stop it first (scripts/stop.sh)"; exit 1
fi

case "$AIRFRAME" in x8) REST=0.44 ;; x500) REST=0.23 ;; *) REST=0.10 ;; esac   # centre above ground at rest (skid bottom)
Z=$(python3 -c "print(round($ALT - $REST, 3))")
CEIL=${CTRACK_CEIL:-$(python3 -c "print(round($Z + 2.94, 2))")}
GROUND=${CTRACK_GROUND:--1.5}
ZMARGIN=${CTRACK_ZMARGIN:-1.15}
read SX SY SYAW GX GY <<< $(python3 - "$LAYOUT" "$FROM" "$GOALWP" <<'EOF'
import json, math, sys
wp = json.load(open(sys.argv[1]))['waypoints']
fx, fy = wp[sys.argv[2]] if sys.argv[2] in wp else map(float, sys.argv[2].split(','))
gx, gy = wp[sys.argv[3]] if sys.argv[3] in wp else map(float, sys.argv[3].split(','))
# Gazebo ENU: x = X, y = -Y; facing the goal; goal relative to FROM in PX4 local ENU
print(fx, -fy, round(math.atan2(-(gy - fy), gx - fx), 4), gx - fx, -(gy - fy))
EOF
)
APP_ENV=()
if [ "$APP" = noclimb_ca ]; then
    : ${NOCLIMB_MAX_TILT_RATE:=0.8}; export NOCLIMB_MAX_TILT_RATE
    while IFS='=' read -r k v; do
        [ -n "${!k+x}" ] || APP_ENV+=(-e "$k=$v")
    done < <(grep -E '^NOCLIMB_[A-Z_]+=' $REPO/envs/ros2.env | sed -E 's/[[:space:]]+#.*$//; s/"//g')
    for k in $(compgen -e | grep '^NOCLIMB_'); do APP_ENV+=(-e "$k=${!k}"); done
elif [ "$APP" = t1000 ] || [ "$APP" = t1000_pf ]; then
    # T1000 (JBNU) flies its own lifecycle at T1000_FLIGHT_ALTITUDE_M (PX4 local z, so ALT - rest height);
    # t1000_pf: its command through PathFollowing + flight_driver, which take the NOCLIMB_* follower knobs
    : ${T1000_FLIGHT_ALTITUDE_M:=$Z}; export T1000_FLIGHT_ALTITUDE_M
    for k in $(compgen -e | grep '^T1000_'); do APP_ENV+=(-e "$k=${!k}"); done
    if [ "$APP" = t1000_pf ]; then
        : ${NOCLIMB_MAX_TILT_RATE:=0.8}; export NOCLIMB_MAX_TILT_RATE
        for k in $(compgen -e | grep '^NOCLIMB_'); do APP_ENV+=(-e "$k=${!k}"); done
    fi
fi
# SIM_LIDAR=mid360: the A4VAI-Unreal Mid-360 pattern lidar (run_unreal.sh settings/settings_gazebo_mid360.json);
# the apps de-skew it by shot order instead of azimuth
[ -n "${SIM_LIDAR:-}" ] && APP_ENV+=(-e "SIM_LIDAR=$SIM_LIDAR")
echo "[ctrack] $FROM -> $GOALWP ($AIRFRAME, $APP): spawn Gazebo ($SX, $SY) yaw $SYAW; goal PX4 local ENU" \
     "($GX, $GY, $Z) = $ALT m above ground; SUPER max_vel $SPEED m/s, band ground $GROUND ceil $CEIL corridor z-margin $ZMARGIN"
[ ${#APP_ENV[@]} -gt 0 ] && echo "[ctrack] app env: ${APP_ENV[*]}"
echo "$FROM $GOALWP $SX $SY $GX $GY $ALT $SPEED $AIRFRAME $APP $MAP" > "$OUT/ctrack_run.txt"

docker rm -f ctrack-sitl ros2-env pils-airsim >/dev/null 2>&1

# 1) Gazebo + PX4 + GazeboDrone (CTRACK_DIR/gazebo/run_gazebo_px4.sh with the spawn on FROM)
# the RC cart and the wind zone find the vehicle by its Gazebo model name, "x8" in the layout
# (trigger_model / vehicle_model): for another airframe the world is generated from a copy naming
# it - with "x8" the cart never started and the wind never blew (ctrack_x500_m360_ca_1)
LAYOUT_GZ=$LAYOUT
if [ "$AIRFRAME" != x8 ]; then
    LAYOUT_GZ=$OUT/layout_$AIRFRAME.json
    python3 - "$LAYOUT" "$LAYOUT_GZ" "$AIRFRAME" <<'PYEOF'
import json, os, sys
lay = json.load(open(sys.argv[1]))
# the copy lives in the run dir: files the layout names relative to itself (the Yard ground heightmap) by their
# absolute path, or gen_gazebo_world.py looks for them next to the copy (yard_x500_*_1017 never started)
for r in lay.get('relief', []):
    if r.get('heightmap') and not os.path.isabs(r['heightmap']):
        r['heightmap'] = os.path.join(os.path.dirname(os.path.abspath(sys.argv[1])), r['heightmap'])
for m in lay.get('moving_obstacles', []):
    if m.get('trigger_model') == 'x8':
        m['trigger_model'] = sys.argv[3]
for w in lay.get('wind_zones', []):
    if w.get('vehicle_model') == 'x8':
        w['vehicle_model'] = sys.argv[3]
json.dump(lay, open(sys.argv[2], 'w'), indent=1)
PYEOF
fi
"$CTRACK/layout/gen_gazebo_world.py" -l "$LAYOUT_GZ" -o "$CTRACK/gazebo/worlds/ctrack.world" \
    --mesh-root /ctrack-layout >/dev/null
ENTRY=$OUT/ctrack_sitl_entry.sh
sed -E "s/--model-name=\\\$\{MODEL\} -x 0 -y 0 -z 0 -Y 0/--model-name=\${MODEL} -x $SX -y $SY -z 0 -Y $SYAW/" \
    "$CTRACK/gazebo/ctrack_sitl_entry.sh" > "$ENTRY"
if ! grep -q -- "-x $SX -y $SY" "$ENTRY"; then
    echo "[ctrack] spawn line not found in $CTRACK/gazebo/ctrack_sitl_entry.sh"; exit 1
fi
SITL_ENV=()
if [ "$MODE" = pils ]; then
    # PX4 reads ROS_DOMAIN_ID as its DDS domain; its uXRCE client goes to the onboard computer: run PX4 on a
    # copy of the build's etc/ whose rcS names ONBOARD_IP (the read-only build keeps the SILS target)
    SITL_ENV=(-e ROS_DOMAIN_ID=$PILS_DOMAIN)
    LAUNCH='${PX4_BUILD}/bin/px4 -d -w /tmp/px4_rootfs ${PX4_BUILD}/etc'
    grep -qF "$LAUNCH" "$ENTRY" || { echo "[ctrack] PX4 launch line not found in the entry script"; exit 1; }
    SYNC_OFF=$([ "${PILS_LINK:-lan}" = wan ] && echo 'param set UXRCE_DDS_SYNCT 0; ')
    sed -i "s|\${PX4_BUILD}/bin/px4 -d -w /tmp/px4_rootfs \${PX4_BUILD}/etc|rm -rf /tmp/px4_etc; cp -r \${PX4_BUILD}/etc /tmp/px4_etc; sed -i 's/^uxrce_dds_client start .*/${SYNC_OFF}uxrce_dds_client start -t udp -h $ONBOARD_IP -p \$uxrce_dds_port \$uxrce_dds_ns/' /tmp/px4_etc/init.d-posix/rcS; \${PX4_BUILD}/bin/px4 -d -w /tmp/px4_rootfs /tmp/px4_etc|" "$ENTRY"
    grep -q "/tmp/px4_etc" "$ENTRY" || { echo "[ctrack] could not point PX4's DDS client at $ONBOARD_IP"; exit 1; }
fi
MIRROR=$(python3 -c "import json,sys; print(','.join(m['name'] for m in json.load(open(sys.argv[1])).get('moving_obstacles', [])))" "$LAYOUT")
mkdir -p "$OUT/ctrack_logs"
# GazeboDrone has to mirror the x8 into AirSim before anything flies: sometimes it connects but never gets
# a Gazebo pose (its log stops at "Connected!"; 4 of ~16 runs on 2026-10-13, none in the ~50 before), the
# AirSim vehicle stays on the spawn, the lidar keeps sending the start scene and the policy flies blind -
# into C1 at 5 m/s (ctrack_tau5_2). Restarting GazeboDrone inside the container did not help, a fresh
# container did (the next run was fine), so: wait for its pose lines ("local (n) x8::..."), else save what
# Gazebo publishes and start the container again, up to 3 times.
GD_LOG="$OUT/ctrack_logs/gazebodrone.log"
gd_ok=0
for attempt in 1 2 3; do
    docker run -d --rm --name ctrack-sitl --network host \
        -e SITL_AIRFRAME="$AIRFRAME" -e GUI= -e MIRROR_MODELS="$MIRROR" "${SITL_ENV[@]}" \
        -e DISPLAY="${DISPLAY:-:0}" -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v $W/PX4-Autopilot:/home/user/workspace:ro \
        -v "$CTRACK/gazebo":/ctrack:ro \
        -v "$(dirname "$(readlink -f "$LAYOUT")")":/ctrack-layout:ro \
        -v "$OUT/ctrack_logs":/ctrack-logs \
        -v "$ENTRY":/ctrack_entry.sh:ro \
        --entrypoint bash px4io/px4-dev-simulation-focal:latest /ctrack_entry.sh > /dev/null
    T_GD=$(date +%s)
    while [ $(( $(date +%s) - T_GD )) -lt 45 ]; do
        grep -qs "local (" "$GD_LOG" && { gd_ok=1; break; }
        sleep 1
    done
    [ $gd_ok = 1 ] && break
    echo "[ctrack] GazeboDrone sends no pose (AirSim frozen), attempt $attempt - starting the simulator container again"
    docker exec ctrack-sitl bash -c "source /usr/share/gazebo/setup.sh 2>/dev/null; timeout 10 gz topic -l; echo ---; ps -eo pid,etime,stat,cmd | grep -v ' ps '" \
        > "$OUT/ctrack_logs/freeze_diag_$attempt.txt" 2>&1
    cp "$GD_LOG" "$OUT/ctrack_logs/gazebodrone_frozen_$attempt.log" 2>/dev/null
    docker rm -f ctrack-sitl > /dev/null 2>&1
    sleep 3
done
if [ $gd_ok = 0 ]; then
    echo "[ctrack] $NAME: airsim_frozen (GazeboDrone sent no pose in 3 container starts) - not flying"
    echo airsim_frozen > "$OUT/result.txt"
    exit 1
fi

# 2) our ROS2 container alone, at the address PX4's DDS client targets
if ! docker network inspect ctrack-ros2 > /dev/null 2>&1; then
    if ! docker network create --subnet 172.70.0.0/16 --gateway 172.70.0.1 ctrack-ros2 > /dev/null; then
        echo "[ctrack] 172.70.0.0/16 is taken (A4VAI-SITL compose network left over? docker network ls)"; exit 1
    fi
fi
mkdir -p $W/ROS2/scripts/include
cp -r $REPO/scripts/ros2/* $W/ROS2/scripts/
cp -r $REPO/scripts/include/* $W/ROS2/scripts/include/
if [ "$MODE" = pils ]; then
    # the apps run on the onboard computer: hand it the app, its knobs and the goal (no goal file there)
    ONB_ENV=(ROS_DOMAIN_ID=$PILS_DOMAIN ROS2_APP=$APP SITL_AIRFRAME=$AIRFRAME NOCLIMB_GOAL=$GX,$GY NOCLIMB_ALT=$Z
             SUPER_PF_MAX_SPEED=$SPEED SUPER_PF_GOAL_HANDOVER_M=3.0 SUPER_PF_ALT=$Z SUPER_PF_VIRTUAL_GROUND=$GROUND
             SUPER_PF_VIRTUAL_CEIL=$CEIL SUPER_PF_CORRIDOR_Z_MARGIN=$ZMARGIN SUPER_PF_SPEED_GUARD=${CTRACK_GUARD:-1}
             SUPER_PF_HOLD_Z=$([ "${CTRACK_HOLD:-1}" = 1 ] && echo $Z)
             SUPER_PF_STATIC_MIN_Z=$([ "${CTRACK_HOLD:-1}" = 1 ] && python3 -c "print(round($Z - 0.5, 3))")
             AIRSIM_IP=${PILS_AIRSIM_IP:-$(ip route get "$ONBOARD_IP" | awk '{for (i = 1; i < NF; i++) if ($i == "src") print $(i + 1)}')})
    [ -n "${ONBOARD_IMAGE:-}" ] && ONB_ENV+=(ONBOARD_IMAGE=$ONBOARD_IMAGE)
    if [ "${PILS_LINK:-lan}" = wan ]; then
        CW=/home/user/workspace/ros2
        docker run -d --rm --name pils-airsim --network host -e ROS_DOMAIN_ID=88 -e AIRSIM_IP=127.0.0.1 \
            -v $W/ROS2:$CW -w /home/user/workspace --entrypoint bash "$IMAGE" -c \
            "$CW/scripts/airsim-bridge.sh > $CMNT/pc-airsim-bridge.log 2>&1 & source /opt/ros/humble/setup.bash; \
             python3 $CW/scripts/tools/topic_relay.py send $ONBOARD_IP:${PILS_RELAY_PORT:-9750} \
             /airsim_node/SimpleFlight/lidar/points/RPLIDAR_A3 /airsim_node/SimpleFlight/odom_local --dedupe \
             > $CMNT/pc-relay.log 2>&1" > /dev/null || { echo "[ctrack] pils-airsim did not start"; exit 1; }
        ONB_ENV+=(PILS_LINK=wan PILS_RELAY_PORT=${PILS_RELAY_PORT:-9750})
        echo "[ctrack] PILS over a routed link: PX4 time sync off, AirSim bridge here, relay to $ONBOARD_IP:${PILS_RELAY_PORT:-9750}"
    fi
    for kv in "${APP_ENV[@]}"; do [ "$kv" != -e ] && ONB_ENV+=("$kv"); done
    if [ -n "${ONBOARD_SSH:-}" ]; then
        ssh -o BatchMode=yes "$ONBOARD_SSH" "cd ${ONBOARD_REPO:-PX4-SITL-Runner} && env ${ONB_ENV[*]} scripts/dev/pils_onboard.sh $NAME" \
            || { echo "[ctrack] onboard start over ssh failed"; docker rm -f ctrack-sitl > /dev/null 2>&1; exit 1; }
    elif [ "$ONBOARD_IP" = 127.0.0.1 ]; then
        env "${ONB_ENV[@]}" "$REPO/scripts/dev/pils_onboard.sh" "$NAME" || { docker rm -f ctrack-sitl > /dev/null 2>&1; exit 1; }
    else
        echo "[ctrack] start the onboard side now: env ${ONB_ENV[*]} scripts/dev/pils_onboard.sh $NAME"
    fi
    T0=$(date +%s)
else
git -C $W/ROS2/ros2_ws/src show HEAD:realgazebo_fastplanner_adapters/config/goals_super_far.csv > "$OUT/goals_csv_backup" 2>/dev/null
printf 'x,y,z\n%s, %s, %s\n' "$GX" "$GY" "$Z" > "$GOALS"
docker run -d --rm --name ros2-env --hostname ros2-env --network ctrack-ros2 --ip 172.70.0.5 \
    --gpus all --privileged \
    -e ROS_DOMAIN_ID=0 -e AIRSIM_IP=172.70.0.1 -e PX4_IP=127.0.0.1 \
    -e ROS2_APP="$APP" "${APP_ENV[@]}" -e SITL_AIRFRAME="$AIRFRAME" \
    -e SUPER_PF_MAX_SPEED="$SPEED" -e SUPER_PF_GOAL_HANDOVER_M=3.0 \
    -e SUPER_PF_ALT="$Z" -e SUPER_PF_VIRTUAL_GROUND="$GROUND" -e SUPER_PF_VIRTUAL_CEIL="$CEIL" \
    -e SUPER_PF_CORRIDOR_Z_MARGIN="$ZMARGIN" -e SUPER_PF_SPEED_GUARD="${CTRACK_GUARD:-1}" \
    -e SUPER_PF_HOLD_Z="$([ "${CTRACK_HOLD:-1}" = 1 ] && echo $Z)" \
    -e SUPER_PF_STATIC_MIN_Z="$([ "${CTRACK_HOLD:-1}" = 1 ] && python3 -c "print(round($Z - 0.5, 3))")" \
    -v $W/ROS2:/home/user/workspace/ros2 \
    -w /home/user/workspace --entrypoint bash "$IMAGE" \
    /home/user/workspace/ros2/scripts/sitl-px4-airsim.sh > /dev/null

T0=$(date +%s)
until docker exec ros2-env test -f /home/user/workspace/ros2/ros2_ws/install/setup.bash 2>/dev/null; do
    sleep 2; [ $(( $(date +%s) - T0 )) -gt 120 ] && { echo "[ctrack] ros2-env did not come up"; exit 1; }
done
docker exec -d ros2-env bash -c "source /opt/ros/humble/setup.bash; source /home/user/workspace/ros2/ros2_ws/install/setup.bash; \
    python3 /home/user/workspace/ros2/scripts/tools/flight_recorder.py $CMNT > $CMNT/recorder.log 2>&1"
fi

# speed / height line on the Unreal screen while the run lasts (scripts/dev/airsim_hud.py; CTRACK_HUD=1 on).
# Off by default: in the 6 runs with it polling AirSim at 50 Hz, the land command got lost 2-3 times
# (60 s hover at the goal) and GazeboDrone froze once (ctrack_tau5_2) - none in ~50 runs without it.
HUD_PID=
if [ "${CTRACK_HUD:-0}" != 0 ]; then
    python3 "$REPO/scripts/dev/airsim_hud.py" > "$OUT/airsim_hud.log" 2>&1 &
    HUD_PID=$!
fi

# PILS: arm / land straight from PX4 (the recorder writing status.csv runs on the onboard computer)
PX4_BIN=/home/user/workspace/PX4-Autopilot/build/px4_sitl_default/bin
px4_arming() { docker exec ctrack-sitl $PX4_BIN/px4-listener vehicle_status -n 1 2>/dev/null | awk '$1 == "arming_state:" {print $2; exit}'; }
# the app's log: here, or on the onboard computer (PILS over ssh)
app_log_has() {
    if [ "$MODE" = pils ] && [ -n "${ONBOARD_SSH:-}" ]; then
        ssh -o BatchMode=yes -o ConnectTimeout=3 "$ONBOARD_SSH" \
            "grep -qs '$1' ${ONBOARD_WS_REMOTE:-Documents/A4VAI-SITL/ROS2}/logs/$APP.log"
    else
        grep -qs "$1" $W/ROS2/logs/$APP.log
    fi
}
armed_seen=0; result=timeout
while [ $(( $(date +%s) - T0 )) -lt "$TIMEOUT" ]; do
    sleep 3
    if [ "$MODE" = pils ]; then
        last=$(px4_arming); echo "$(date +%s.%N),$last" >> "$OUT/status_px4.csv"
    else
        last=$(tail -n 1 "$OUT/status.csv" 2>/dev/null | cut -d, -f2)
    fi
    [ "$last" = "2" ] && armed_seen=1
    if [ $armed_seen = 1 ] && [ "$last" = "1" ]; then result=landed; sleep 3; break; fi
    if [ $armed_seen = 0 ] && app_log_has "not taking off"; then result=no_lidar; break; fi
done
echo "[ctrack] $NAME: $result after $(( $(date +%s) - T0 ))s"
[ -n "$HUD_PID" ] && kill "$HUD_PID" 2>/dev/null

if [ "$MODE" = pils ]; then
    if [ -n "${ONBOARD_SSH:-}" ]; then
        ssh -o BatchMode=yes "$ONBOARD_SSH" "cd ${ONBOARD_REPO:-PX4-SITL-Runner} && scripts/dev/pils_onboard.sh --stop $NAME"
        rsync -a "$ONBOARD_SSH:${ONBOARD_WS_REMOTE:-Documents/A4VAI-SITL/ROS2}/test_runs/$NAME/" "$OUT/" \
            || echo "[ctrack] could not copy the onboard run dir"
    elif [ "$ONBOARD_IP" = 127.0.0.1 ]; then
        "$REPO/scripts/dev/pils_onboard.sh" --stop "$NAME"
    else
        echo "[ctrack] stop the onboard side and copy its test_runs/$NAME here"
    fi
else
docker exec ros2-env pkill -INT -f flight_recorder.py
for i in $(seq 1 30); do [ -f "$OUT/cloud_map.npz" ] && break; sleep 1; done
docker exec ros2-env bash -c "cp /home/user/workspace/ros2/logs/*.log /home/user/workspace/ros2/logs/*.pkl $CMNT/ 2>/dev/null; \
    cp /home/user/workspace/ros2/logs/noclimb_ca.csv /home/user/workspace/ros2/logs/noclimb_ca_dump.npz /home/user/workspace/ros2/logs/t1000_pf.csv $CMNT/ 2>/dev/null; \
    f=\$(ls -t /home/user/workspace/ros2/logs/[0-9]*.csv 2>/dev/null | head -1); [ -n \"\$f\" ] && cp \"\$f\" $CMNT/pf_log.csv; \
    f=\$(ls -t /home/user/workspace/ros2/logs/flight_driver_*.csv 2>/dev/null | head -1); [ -n \"\$f\" ] && cp \"\$f\" $CMNT/flight_driver.csv; \
    chmod -R a+rw $CMNT"
fi
# PX4 log from its rootfs inside ctrack-sitl (closed at disarm)
ULG=$(docker exec ctrack-sitl bash -c "ls -t /tmp/px4_rootfs/log/*/*.ulg 2>/dev/null | head -1")
[ -n "$ULG" ] && docker cp "ctrack-sitl:$ULG" "$OUT/px4_sitl.ulg" > /dev/null
docker rm -f ros2-env ctrack-sitl pils-airsim > /dev/null 2>&1
git -C $W/ROS2/ros2_ws/src checkout -- realgazebo_fastplanner_adapters/config/goals_super_far.csv
# a disarm is not a landing when flight_manager aborted (impact, estimate reset / dead reckoning,
# PF done far from the goal): it lands in place
abort=$(grep -ohE "Impact detected|Position estimate reset|dead-reckoning|PF done [0-9.]+ m from the goal" "$OUT/flight_manager.log" 2>/dev/null | head -1)
# flight_driver (NOCLIMB_DRIVER=flight_driver): its abort (landed in place) or yield (stopped commanding)
[ -z "$abort" ] && abort=$(grep -ohE "(ABORT - landing in place|YIELD \(no more commands\)): [^[:cntrl:]]*" "$OUT/flight_driver.log" 2>/dev/null \
    | head -1 | cut -c1-140)
[ "$result" = landed ] && [ -n "$abort" ] && result="aborted ($abort)"
[ -n "$abort" ] && echo "[ctrack] $NAME: flight FSM: $abort"
echo "$result" > "$OUT/result.txt"
