#!/bin/bash
# Build the SLAM workspace (~/Documents/A4VAI-SITL/ROS2/slam_ws: FAST-LIO, lio_px4_bridge, the simulation adapter and the
# PC's Livox message stand-in) on THIS PC, inside the ROS2 image, mounted where the apps run it
# (/home/user/workspace/ros2/slam_ws - colcon keeps absolute paths). The image's pip setuptools (83) does not work with
# its apt 'packaging' (21.3) for ament_python packages, so a newer packaging goes into the build container first.
#   scripts/dev/slam/build_pc.sh [colcon args, e.g. --packages-select lio_px4_bridge]
# env: SLAM_WS_HOST (default ~/Documents/A4VAI-SITL/ROS2/slam_ws), ROS2_IMAGE (default envs/ros2.env ROS2_ENV_IMAGE)
set -eu
REPO=$(dirname $(dirname $(dirname $(dirname $(readlink -f "$0")))))
WS=$HOME/Documents/A4VAI-SITL/ROS2
SLAM=${SLAM_WS_HOST:-$WS/slam_ws}
IMG=${ROS2_IMAGE:-$(grep '^ROS2_ENV_IMAGE=' "$REPO/envs/ros2.env" | cut -d= -f2 | tr -d '"')}
[ -d "$SLAM/src" ] || { echo "[slam build] no $SLAM/src"; exit 1; }
[ "$(readlink -f "$SLAM")" = "$(readlink -f "$WS/slam_ws")" ] || echo "[slam build] note: $SLAM is mounted as /home/user/workspace/ros2/slam_ws"
# the container builds as root: its build / install / log come back owned by this user at the end
docker run --rm --entrypoint bash -e HOST_IDS="$(id -u):$(id -g)" \
    -v "$WS":/home/user/workspace/ros2 -v "$SLAM":/home/user/workspace/ros2/slam_ws "$IMG" -c "
pip3 install -q 'packaging>=22' 2>&1 | grep -v 'already satisfied' || true
source /opt/ros/humble/setup.bash
source /home/user/workspace/ros2/ros2_ws/install/setup.bash          # px4_msgs
cd /home/user/workspace/ros2/slam_ws
MAKEFLAGS=-j8 colcon build --symlink-install --event-handlers console_cohesion+ --cmake-args -DCMAKE_BUILD_TYPE=Release $*
rc=\$?
chown -R \$HOST_IDS build install log 2>/dev/null
exit \$rc
"
echo "[slam build] done: $SLAM/install (source it after ros2_ws: source $SLAM/install/setup.bash)"
