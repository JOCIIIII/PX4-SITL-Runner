#! /bin/bash

# SCRIPT TO BUILD ROS2 PACKAGE INSIDE THE CONTAINER

# INITIAL STATEMENTS
# >>>----------------------------------------------------

# SET THE BASIC ENVIRONMENT VARIABLE
export TERM=xterm-256color

# SET THE BASE DIRECTORY
BASE_DIR=$(dirname $(readlink -f "$0"))

# SOURCE THE ENVIRONMENT AND FUNCTION DEFINITIONS
source ${BASE_DIR}/include/commonFcn.sh
source ${BASE_DIR}/include/commonEnv.sh

# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


# MAIN STATEMENTS
# >>>----------------------------------------------------

CheckFileExists ${BASE_DIR}/uxrce-dds.sh
CheckFileExecutable ${BASE_DIR}/uxrce-dds.sh

CheckFileExists ${BASE_DIR}/airsim-bridge.sh
CheckFileExecutable ${BASE_DIR}/airsim-bridge.sh

${BASE_DIR}/uxrce-dds.sh &

${BASE_DIR}/airsim-bridge.sh &

# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


# USER-DEFINED SATEMENTS
# >>>----------------------------------------------------

# PLACE USER-DEFINED SHELL SCRIPTS/COMMANDS HERE
# FOR EXAMPLE FOR RUNNING:
#   algorithm1 build at /home/user/workspace/ros2/alg_ws
# PLACE THE FOLLOWING COMMAND OR CREATE A SHELL SCRIPT WITH THE COMMAND:
#   source /opt/ros/${ROS_DISTRO}/setup.bash
#   source /home/user/workspace/ros2/alg_ws/install/setup.bash
#   ros2 run algorithm1 algorithm1_node
# CHECK uxrce-dds.sh, airsim-bridge.sh, AND THIS SCRIPT FOR EXAMPLES



# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

sleep infinity