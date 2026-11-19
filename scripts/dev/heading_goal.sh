#!/bin/bash
# THE VEHICLE (Jetson): the NOCLIMB_GOAL line DIST m (default 40) ahead along where the vehicle points now - stand it on
# the take-off point facing the course, then:
#   scripts/dev/heading_goal.sh [DIST]          -> heading, then NOCLIMB_GOAL=east,north for onboard_real.sh fly
# A throwaway container of the onboard image reads PX4's heading (the agent must be up and PX4 connected).
# env: ONBOARD_WS, ONBOARD_REAL_IMAGE, ONBOARD_DOMAIN (as onboard_real.sh)
set -u
REPO=$(dirname $(dirname $(dirname $(readlink -f "$0"))))
WS=${ONBOARD_WS:-$HOME/Documents/A4VAI-SITL/ROS2}
docker run --rm -i --network host --ipc host -e ROS_DOMAIN_ID=${ONBOARD_DOMAIN:-0} -v "$WS":/home/user/workspace/ros2 \
    --entrypoint bash "${ONBOARD_REAL_IMAGE:-a4vai-onboard:humble}" -c "source /opt/ros/humble/setup.bash; \
    source /home/user/workspace/ros2/ros2_ws/install/setup.bash; python3 - ${1:-40}" < "$REPO/scripts/dev/heading_goal.py"
