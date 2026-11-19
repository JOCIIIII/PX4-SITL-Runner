#!/bin/bash
# Put this PC's lio_px4_bridge (the 2026-10-31 hole-tolerant bridge_node.py) and the per-airframe bridge configs on
# the vehicle's Jetson and reinstall the package there (its slam_ws install is a copy, not a symlink, so a source
# edit alone changes nothing). Never while the vehicle flies.
#   scripts/dev/slam/deploy_bridge_jetson.sh jetson@HOST
# env: SLAM_WS_HOST (this PC's slam_ws, default ~/Documents/A4VAI-SITL/ROS2/slam_ws), REMOTE_SLAM_WS (default slam_ws in the
#      login's home), SLAM_IMAGE (default a4vai-onboard:humble)
set -eu
HOST=${1:?usage: deploy_bridge_jetson.sh user@host}
SRC=${SLAM_WS_HOST:-$HOME/Documents/A4VAI-SITL/ROS2/slam_ws}
RWS=${REMOTE_SLAM_WS:-slam_ws}
IMG=${SLAM_IMAGE:-a4vai-onboard:humble}
on() { ssh -o BatchMode=yes "$HOST" "$@"; }
if on "docker ps --format '{{.Names}}' | grep -qxE 'ros2-real|slam-ev'"; then
    echo "[deploy] a flight app or SLAM container is running on $HOST - not touching it"; exit 1
fi
on "cp -n $RWS/src/lio_px4_bridge/lio_px4_bridge/bridge_node.py $RWS/src/lio_px4_bridge/lio_px4_bridge/bridge_node.py.before_holes 2>/dev/null || true"
rsync -ai "$SRC/src/lio_px4_bridge/lio_px4_bridge/bridge_node.py" "$HOST:$RWS/src/lio_px4_bridge/lio_px4_bridge/bridge_node.py"
rsync -ai "$SRC/config/bridge_x500_mid360.yaml" "$SRC/config/bridge_x8_mid360.yaml" "$HOST:$RWS/config/"
# the container script (EV_PX4BAG light recording) and the launcher that passes it
HERE=$(dirname "$(readlink -f "$0")")
rsync -ai "$HERE/ground_ev_inner.sh" "$HOST:$RWS/scripts/ground_ev_inner.sh"
rsync -ai "$HERE/../onboard_slam.sh" "$HOST:PX4-SITL-Runner/scripts/dev/onboard_slam.sh"
echo "[deploy] reinstalling lio_px4_bridge in $IMG on $HOST"
on "docker run --rm --entrypoint bash -e HOST_IDS=\$(id -u):\$(id -g) -v \$HOME/$RWS:/home/user/workspace/slam_ws \
      -v \$HOME/Documents/A4VAI-SITL/ROS2:/home/user/workspace/ros2 $IMG -c '
    source /opt/ros/humble/setup.bash; source /home/user/workspace/ros2/ros2_ws/install/setup.bash
    cd /home/user/workspace/slam_ws && colcon build --packages-select lio_px4_bridge --event-handlers console_cohesion+ 2>&1 | tail -3
    chown -R \$HOST_IDS build install log 2>/dev/null; true'"
on "grep -c hole_max_s $RWS/install/lio_px4_bridge/lib/python3.10/site-packages/lio_px4_bridge/bridge_node.py" \
    | grep -q '^[1-9]' && echo "[deploy] installed bridge has the hole tolerance" || { echo "[deploy] install check FAILED"; exit 1; }
