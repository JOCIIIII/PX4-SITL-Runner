#!/bin/bash
# Dev harness for tuning runs: start the full SITL stack with one goal, record the
# flight (scripts/ros2/tools/flight_recorder.py), wait for landing, stop the stack.
#
# usage: scripts/dev/fly_test.sh NAME GX GY GZ [TIMEOUT_S] [APP]
#   goal (GX,GY,GZ) is ENU metres relative to the PX4 origin (spawn point)
#   output: ~/Documents/A4VAI-SITL/ROS2/test_runs/NAME/ (CSVs, cloud_map.npz, node logs;
#           pf_log.csv = PathFollowing's own log, noclimb_ca.csv = noclimb_ca per-step log)
set -u
NAME=$1; GX=$2; GY=$3; GZ=$4; TIMEOUT=${5:-300}; APP=${6-super_pf}; SPEED=${7:-10.0}
REPO=$(dirname $(dirname $(dirname $(readlink -f "$0"))))
W=~/Documents/A4VAI-SITL
OUT=$W/ROS2/test_runs/$NAME
CMNT=/home/user/workspace/ros2/test_runs/$NAME
mkdir -p "$OUT"; rm -f "$OUT"/*

printf 'x,y,z\n%s, %s, %s\n' "$GX" "$GY" "$GZ" > $W/ROS2/ros2_ws/src/realgazebo_fastplanner_adapters/config/goals_super_far.csv
sed -i "s/^ROS2_APP=.*/ROS2_APP=\"$APP\"/; s/^SUPER_PF_MAX_SPEED=.*/SUPER_PF_MAX_SPEED=$SPEED/" $REPO/envs/ros2.env
# FLY_ENV="KEY=VAL ..." sets app knobs in envs/ros2.env for this run (appended if absent;
# e.g. FLY_ENV="NOCLIMB_POLICY=ca_noclimb_tau03-08 NOCLIMB_SCAN_MEMORY=0")
for kv in ${FLY_ENV:-}; do
    k=${kv%%=*}
    grep -q "^$k=" $REPO/envs/ros2.env && sed -i "s|^$k=.*|$kv|" $REPO/envs/ros2.env || printf '\n%s\n' "$kv" >> $REPO/envs/ros2.env
done
# FLY_OFFSCREEN=1 renders AirSim without a window (envs/airsim.env AIRSIM_RENDER_ARGS); it did
# not make start-up faster here (1-3 min either way when the host is cold). Restored at the end.
AIRSIM_ARGS=$([ "${FLY_OFFSCREEN:-0}" = 1 ] && echo -RenderOffScreen || echo -windowed)
sed -i "s/^AIRSIM_RENDER_ARGS=.*/AIRSIM_RENDER_ARGS=$AIRSIM_ARGS/" $REPO/envs/airsim.env

# FLY_PROFILE: compose profile (default gazebo-classic-airsim-sitl; gazebo-classic-sitl = no AirSim, e.g. a
# PathFollowing-only wind test with ROS2_APP pf_test)
PROFILE=${FLY_PROFILE:-gazebo-classic-airsim-sitl}
$REPO/scripts/stop.sh $PROFILE >/dev/null 2>&1
docker rm -f px4-env gz-sim airsim-binary ros2-env qgc-app >/dev/null 2>&1
nohup $REPO/scripts/run.sh $PROFILE run > "$OUT/stack.log" 2>&1 &

T0=$(date +%s)
until docker exec ros2-env test -f /home/user/workspace/ros2/ros2_ws/install/setup.bash 2>/dev/null; do
    sleep 2; [ $(( $(date +%s) - T0 )) -gt 120 ] && { echo "ros2-env did not come up"; exit 1; }
done
docker exec ros2-env pkill -f flight_recorder.py 2>/dev/null
docker exec -d ros2-env bash -c "source /opt/ros/humble/setup.bash; source /home/user/workspace/ros2/ros2_ws/install/setup.bash; \
    python3 /home/user/workspace/ros2/scripts/tools/flight_recorder.py $CMNT > $CMNT/recorder.log 2>&1"

armed_seen=0; result=timeout
while [ $(( $(date +%s) - T0 )) -lt "$TIMEOUT" ]; do
    sleep 3
    last=$(tail -n 1 "$OUT/status.csv" 2>/dev/null | cut -d, -f2)
    [ "$last" = "2" ] && armed_seen=1
    if [ $armed_seen = 1 ] && [ "$last" = "1" ]; then result=landed; sleep 3; break; fi
    # the app gave up waiting for AirSim (lidar gate) - nothing will fly in this run
    if [ $armed_seen = 0 ] && grep -qs "not taking off" $W/ROS2/logs/$APP.log; then result=no_lidar; break; fi
done
echo "[fly_test] $NAME: $result after $(( $(date +%s) - T0 ))s"

docker exec ros2-env pkill -INT -f flight_recorder.py
for i in $(seq 1 30); do [ -f "$OUT/cloud_map.npz" ] && break; sleep 1; done
docker exec ros2-env bash -c "cp /home/user/workspace/ros2/logs/*.log $CMNT/ 2>/dev/null; \
    cp /home/user/workspace/ros2/logs/noclimb_ca.csv /home/user/workspace/ros2/logs/noclimb_ca_dump.npz /home/user/workspace/ros2/logs/x8_step_test.csv $CMNT/ 2>/dev/null; \
    f=\$(ls -t /home/user/workspace/ros2/logs/[0-9]*.csv 2>/dev/null | head -1); [ -n \"\$f\" ] && cp \"\$f\" $CMNT/pf_log.csv; chmod -R a+rw $CMNT"
$REPO/scripts/stop.sh $PROFILE >/dev/null 2>&1
# the run's PX4 log (newest; PX4 has closed it once the stack is down)
ULG=$(ls -t $W/PX4-Autopilot/PX4-Autopilot/build/px4_sitl_default/rootfs/log/*/*.ulg 2>/dev/null | head -1)
[ -n "$ULG" ] && cp "$ULG" "$OUT/px4_sitl.ulg"
# AirSim must have followed the vehicle (GazeboDrone pose bridge); frozen at the spawn point the
# lidar showed one static scan all flight and the run says nothing about avoidance
FROZEN=$(python3 - "$OUT" <<'PYEOF'
import sys, numpy as np
d = sys.argv[1]
try:
    a = np.genfromtxt(f'{d}/airsim_odom.csv', delimiter=',', names=True)
    p = np.genfromtxt(f'{d}/px4.csv', delimiter=',', names=True)
    moved_airsim = np.hypot(a['x'] - a['x'][0], a['y'] - a['y'][0]).max()
    moved_px4 = np.hypot(p['x'] - p['x'][0], p['y'] - p['y'][0]).max()
    print('1' if moved_px4 > 10 and moved_airsim < 1 else '0')
except Exception:
    print('0')
PYEOF
)
[ "$FROZEN" = 1 ] && { result="$result airsim_frozen"; echo "[fly_test] $NAME: AirSim pose never moved (GazeboDrone) - avoidance not tested"; }
cp ~/Documents/A4VAI-SITL/Gazebo-Classic/logs/gazebodrone.log "$OUT/" 2>/dev/null
echo "$result" > "$OUT/result.txt"
sed -i "s/^AIRSIM_RENDER_ARGS=.*/AIRSIM_RENDER_ARGS=-windowed/" $REPO/envs/airsim.env
