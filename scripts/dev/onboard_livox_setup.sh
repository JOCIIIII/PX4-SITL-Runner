#!/bin/bash
# The vehicle part of the onboard computer (after scripts/dev/pils_jetson_setup.sh): the Mid-360 driver.
#   1. image a4vai-onboard:humble = a4vai-pils:humble + Livox-SDK2 (in /usr/local, where livox_ros_driver2
#      looks for it)
#   2. livox_ros_driver2 built in its own workspace ~/Documents/A4VAI-SITL/ROS2/livox_ws, in that image, with the
#      workspace mounted where it runs (scripts/ros2/onboard-real.sh sources it)
# The sources are cloned here and copied over (GitHub from orin-nano-01 stalled). Idempotent.
#   scripts/dev/onboard_livox_setup.sh user@host
set -eu
HOST=${1:?usage: onboard_livox_setup.sh user@host (the Jetson)}
RWS=Documents/A4VAI-SITL/ROS2
BASE=${PILS_IMAGE:-a4vai-pils:humble}
IMG=${ONBOARD_REAL_IMAGE:-a4vai-onboard:humble}
MNT=/home/user/workspace/ros2
on() { ssh -o BatchMode=yes "$HOST" "$@"; }
T=$(mktemp -d); trap 'rm -rf "$T"' EXIT
git clone -q --depth 1 https://github.com/Livox-SDK/Livox-SDK2.git "$T/Livox-SDK2"
git clone -q --depth 1 https://github.com/Livox-SDK/livox_ros_driver2.git "$T/livox_ros_driver2"

echo "[livox_setup] 1/2 image $IMG ($BASE + Livox-SDK2)"
if ! on "docker image inspect $IMG > /dev/null 2>&1"; then
    on "rm -rf /tmp/onboard_image && mkdir -p /tmp/onboard_image"
    rsync -a --exclude .git "$T/Livox-SDK2" "$HOST:/tmp/onboard_image/"
    on "cat > /tmp/onboard_image/Dockerfile && docker build -q -t $IMG /tmp/onboard_image" <<EOF
FROM $BASE
COPY Livox-SDK2 /tmp/Livox-SDK2
RUN cd /tmp/Livox-SDK2 && mkdir -p build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release \\
 && make -j4 && make install && ldconfig && rm -rf /tmp/Livox-SDK2
EOF
fi

echo "[livox_setup] 2/2 livox_ros_driver2 -> $HOST:$RWS/livox_ws (log: $RWS/logs/livox_build.log)"
D=$RWS/livox_ws/src/livox_ros_driver2
on "mkdir -p $RWS/livox_ws/src $RWS/logs"
rsync -a --delete --exclude .git "$T/livox_ros_driver2" "$HOST:$RWS/livox_ws/src/"
# what its build.sh does for ROS2: the ROS2 manifest and launch files
on "cp -f $D/package_ROS2.xml $D/package.xml && rm -rf $D/launch && cp -r $D/launch_ROS2 $D/launch"
on "docker run --rm --runtime nvidia -v \$HOME/$RWS:$MNT --entrypoint bash $IMG -c 'source /opt/ros/humble/setup.bash; \
    cd $MNT/livox_ws && MAKEFLAGS=-j4 colcon build --event-handlers console_cohesion+ --cmake-args \
    -DROS_EDITION=ROS2 -DDISTRO_ROS=humble -DCMAKE_BUILD_TYPE=Release > $MNT/logs/livox_build.log 2>&1; rc=\$?; \
    chmod -R a+rwX $MNT/livox_ws; exit \$rc'" \
    && echo "[livox_setup] done - shadow test: scripts/dev/onboard_real.sh (on the Jetson)" \
    || { echo "[livox_setup] driver build failed, see $HOST:$RWS/logs/livox_build.log"; exit 1; }
