#!/bin/bash
# Runs inside ground-ros2-<name> (the ROS2 image, in the SITL container's network namespace): the uXRCE-DDS
# agent, then the command given as arguments (default: sleep). The overlay workspace /home/user/workspace/ground_ws
# (the ugv-usv branch of PathFollowing) is sourced on top of the shared ros2_ws when it is built.
export TERM=xterm-256color
source /opt/ros/humble/setup.bash
source /home/user/workspace/ros2/ros2_ws/install/setup.bash
[ -f /home/user/workspace/ground_ws/install/setup.bash ] && source /home/user/workspace/ground_ws/install/setup.bash
LOG=/ground-run/logs
mkdir -p ${LOG}
MicroXRCEAgent udp4 -p 8888 > ${LOG}/uxrce-dds.log 2>&1 &
for _i in $(seq 1 90); do
    ros2 topic list 2>/dev/null | grep -q "^/fmu/out/vehicle_local_position" && break
    sleep 1
done
ros2 topic list 2>/dev/null | grep -q "^/fmu/" && echo "[ground] /fmu topics up" || echo "[ground] TIMEOUT waiting for /fmu topics"
if [ $# -eq 0 ]; then exec sleep infinity; fi
exec "$@"
