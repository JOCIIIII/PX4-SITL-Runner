#!/bin/bash
# Dev harness for tuning runs: start the full SITL stack with one goal, record the
# flight (scripts/ros2/tools/flight_recorder.py), wait for landing, stop the stack.
#
# usage: scripts/dev/fly_test.sh NAME GX GY GZ [TIMEOUT_S] [APP]
#   goal (GX,GY,GZ) is ENU metres relative to the PX4 origin (spawn point)
#   output: ~/Documents/A4VAI-SITL/ROS2/test_runs/NAME/ (CSVs, cloud_map.npz, node logs)
set -u
NAME=$1; GX=$2; GY=$3; GZ=$4; TIMEOUT=${5:-300}; APP=${6-super_pf}; SPEED=${7:-10.0}
REPO=$(dirname $(dirname $(dirname $(readlink -f "$0"))))
W=~/Documents/A4VAI-SITL
OUT=$W/ROS2/test_runs/$NAME
CMNT=/home/user/workspace/ros2/test_runs/$NAME
mkdir -p "$OUT"; rm -f "$OUT"/*

printf 'x,y,z\n%s, %s, %s\n' "$GX" "$GY" "$GZ" > $W/ROS2/ros2_ws/src/realgazebo_fastplanner_adapters/config/goals_super_far.csv
sed -i "s/^ROS2_APP=.*/ROS2_APP=\"$APP\"/; s/^SUPER_PF_MAX_SPEED=.*/SUPER_PF_MAX_SPEED=$SPEED/" $REPO/envs/ros2.env
# FLY_OFFSCREEN=1 renders AirSim without a window (envs/airsim.env AIRSIM_RENDER_ARGS); it did
# not make start-up faster here (1-3 min either way when the host is cold). Restored at the end.
AIRSIM_ARGS=$([ "${FLY_OFFSCREEN:-0}" = 1 ] && echo -RenderOffScreen || echo -windowed)
sed -i "s/^AIRSIM_RENDER_ARGS=.*/AIRSIM_RENDER_ARGS=$AIRSIM_ARGS/" $REPO/envs/airsim.env

$REPO/scripts/stop.sh gazebo-classic-airsim-sitl >/dev/null 2>&1
docker rm -f px4-env gz-sim airsim-binary ros2-env qgc-app >/dev/null 2>&1
nohup $REPO/scripts/run.sh gazebo-classic-airsim-sitl run > "$OUT/stack.log" 2>&1 &

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
done
echo "[fly_test] $NAME: $result after $(( $(date +%s) - T0 ))s"

docker exec ros2-env pkill -INT -f flight_recorder.py
for i in $(seq 1 30); do [ -f "$OUT/cloud_map.npz" ] && break; sleep 1; done
docker exec ros2-env bash -c "cp /home/user/workspace/ros2/logs/*.log $CMNT/ 2>/dev/null; \
    f=\$(ls -t /home/user/workspace/ros2/logs/*.csv 2>/dev/null | head -1); [ -n \"\$f\" ] && cp \"\$f\" $CMNT/pf_log.csv; chmod -R a+rw $CMNT"
$REPO/scripts/stop.sh gazebo-classic-airsim-sitl >/dev/null 2>&1
echo "$result" > "$OUT/result.txt"
sed -i "s/^AIRSIM_RENDER_ARGS=.*/AIRSIM_RENDER_ARGS=-windowed/" $REPO/envs/airsim.env
