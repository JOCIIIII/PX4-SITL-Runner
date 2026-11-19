#!/bin/bash
# Replay a recorded SLAM run through lio_px4_bridge on this PC (shadow: nothing goes to any PX4) to see how the
# bridge behaves on FAST-LIO output holes, before and after a change. Plays RUN/odom_bag (/Odometry, /fast_lio/health)
# and RUN/px4_bag (PX4 attitude, local position) into a bridge built in this PC's slam_ws; with HOLES set, the two
# FAST-LIO topics pass through scripts/dev/slam/hole_relay.py, which drops them for the given windows (the Jetson's
# 3.2-3.5 s output gaps of 2026-10-31). Writes OUT/bridge.csv, bridge.log, status.log, relay.log. ROS2 image, no network.
#   scripts/dev/slam/replay_bridge.sh RUN_DIR OUT_DIR [BRIDGE_CONFIG=x500_mid360.yaml] [extra bridge -p args...]
#   env: RATE (default 1), START (s into the bags, default 0), DUR (s of the bags to play, default all),
#        HOLES ("every:40;hole:3.4;first:30" or "at:25:3.4,80:12" - keys separated by ";"; default none),
#        REPLAYS (play the bags this many times back to back, default 1: a second pass = FAST-LIO restarting at its origin)
#   e.g. HOLES="every:40;hole:3.4" DUR=150 scripts/dev/slam/replay_bridge.sh ~/Documents/A4VAI-SITL/ROS2/slam_ws/runs/m1_slam /tmp/m1_holes
# Then: python3 scripts/dev/slam/replay_report.py OUT_DIR
set -eu
RUN=$(readlink -f "$1"); OUT=$(readlink -f "$2"); CFG=${3:-x500_mid360.yaml}; shift $(( $# < 3 ? $# : 3 ))
RATE=${RATE:-1}; START=${START:-0}; DUR=${DUR:-0}; HOLES=${HOLES:-}; REPLAYS=${REPLAYS:-1}
WS=$HOME/Documents/A4VAI-SITL/ROS2
IMG=${ROS2_IMAGE:-jociiiii/a4vai:inhousesim-v0.9}
HERE=$(dirname "$(readlink -f "$0")")
[ -d "$RUN/odom_bag" ] && [ -d "$RUN/px4_bag" ] || { echo "need $RUN/odom_bag and px4_bag"; exit 1; }
mkdir -p "$OUT"
case "$RUN" in "$WS"/*) CRUN=/home/user/workspace/ros2/${RUN#$WS/} ;; *) echo "RUN must be under $WS"; exit 1 ;; esac
BAGDUR=$(python3 -c "import yaml; d=yaml.safe_load(open('$RUN/odom_bag/metadata.yaml'))['rosbag2_bagfile_information']; print(d['duration']['nanoseconds']/1e9)")
PLAY=$(python3 -c "d=$BAGDUR-$START; d=min(d, $DUR) if $DUR > 0 else d; print(max(1, int(d / $RATE) + 3))")
RELAY_ARGS=""
if [ -n "$HOLES" ]; then
    for kv in ${HOLES//;/ }; do case "$kv" in every:*) RELAY_ARGS="$RELAY_ARGS --every ${kv#every:}";; hole:*) RELAY_ARGS="$RELAY_ARGS --hole ${kv#hole:}";;
        first:*) RELAY_ARGS="$RELAY_ARGS --first ${kv#first:}";; at:*) RELAY_ARGS="$RELAY_ARGS --at ${kv#at:}";; esac; done
fi
echo "[replay] $RUN: start $START s, play $PLAY s at x$RATE, holes '${HOLES:-none}' -> $OUT"
docker run --rm --network none --entrypoint bash -e ROS_DOMAIN_ID=71 -e ROS_LOCALHOST_ONLY=1 -e HOST_IDS="$(id -u):$(id -g)" \
    -v "$WS":/home/user/workspace/ros2 -v "$OUT":/out -v "$HERE/hole_relay.py":/hole_relay.py:ro "$IMG" -c "
source /opt/ros/humble/setup.bash; source /home/user/workspace/ros2/ros2_ws/install/setup.bash; source /home/user/workspace/ros2/slam_ws/install/setup.bash
BR=\$(ros2 pkg prefix lio_px4_bridge)
CFGP=\$BR/share/lio_px4_bridge/config/$CFG; [ -f /home/user/workspace/ros2/slam_ws/config/$CFG ] && CFGP=/home/user/workspace/ros2/slam_ws/config/$CFG
\$BR/lib/lio_px4_bridge/bridge --ros-args --params-file \$CFGP -p send_to_px4:=false -p log_csv:=/out/bridge.csv $* > /out/bridge.log 2>&1 &
ros2 topic echo --full-length /lio_px4_bridge/status std_msgs/msg/String > /out/status.log 2>&1 &
REMAP=''
if [ -n '$RELAY_ARGS' ]; then python3 /hole_relay.py $RELAY_ARGS > /out/relay.log 2>&1 & REMAP='--remap /Odometry:=/Odometry_raw /fast_lio/health:=/fast_lio/health_raw'; fi
sleep 3
for pass in \$(seq $REPLAYS); do
    timeout $PLAY ros2 bag play --disable-keyboard-controls $CRUN/px4_bag -r $RATE --start-offset $START --topics /fmu/out/vehicle_attitude /fmu/out/vehicle_local_position > /out/play_px4_\$pass.log 2>&1 &
    PXP=\$!
    timeout $PLAY ros2 bag play --disable-keyboard-controls $CRUN/odom_bag -r $RATE --start-offset $START --topics /Odometry /fast_lio/health \$REMAP > /out/play_odom_\$pass.log 2>&1 || true
    wait \$PXP 2>/dev/null || true
done
sleep 2; kill -INT \$(jobs -p) 2>/dev/null; sleep 2
chown -R \$HOST_IDS /out
" 2>&1 | grep -vE '^\[INFO\] \[ros2bag|^$' || true
echo "[replay] done: $OUT"
