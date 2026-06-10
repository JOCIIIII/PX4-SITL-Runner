#! /bin/bash

# SCRIPT TO BUILD ROS2 PACKAGE INSIDE THE CONTAINER

# INITIAL STATEMENTS
# >>>----------------------------------------------------

# SET THE BASIC ENVIRONMENT VARIABLE
export TERM=xterm-256color

# SET THE BASE DIRECTORY
BASE_DIR=$(dirname $(readlink -f "$0"))
WORKSPACE_DIR=$(dirname $(dirname $(readlink -f "$0")))

# SOURCE THE ENVIRONMENT AND FUNCTION DEFINITIONS
for file in ${BASE_DIR}/include/*.sh; do
    source ${file}
done
# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


# MAIN STATEMENTS
# >>>----------------------------------------------------

source /opt/ros/${ROS_DISTRO}/setup.bash

# px4_msgs DDS types: prefer the prebuilt px4_ros workspace if it exists, otherwise
# fall back to ros2_ws (where px4_msgs is built). MicroXRCEAgent itself is a system
# binary (/usr/local/bin), so px4_ros is NOT required for the agent to run.
if [ -f ${WORKSPACE_DIR}/px4_ros/install/setup.bash ]; then
    source ${WORKSPACE_DIR}/px4_ros/install/setup.bash
elif [ -f ${WORKSPACE_DIR}/ros2_ws/install/setup.bash ]; then
    source ${WORKSPACE_DIR}/ros2_ws/install/setup.bash
fi

EchoYellow "[$(basename $0)] STARTING uXRCE DDS AGENT."
MicroXRCEAgent udp4 -p 8888

# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<