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

source ${BASE_DIR}/sourceAll.sh

# CREATE DIRECTORY FOR LOGS
CheckDirExists ${WORKSPACE_DIR}/logs create
rm -rf ${WORKSPACE_DIR}/logs/*

touch ${WORKSPACE_DIR}/logs/uxrce-dds.log

# RUN THE SHELL SCRIPTS AND LOG THE OUTPUT
${BASE_DIR}/uxrce-dds.sh 2>&1 | tee ${WORKSPACE_DIR}/logs/uxrce-dds.log &

CheckDirExists ${WORKSPACE_DIR}/ros2_ws/build/algorithm_test/algorithm_test/path_following_unit_test/log create
# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

# CREATE LOG FILES
touch ${WORKSPACE_DIR}/logs/node_MPPI_output.log
touch ${WORKSPACE_DIR}/logs/node_att_ctrl.log
touch ${WORKSPACE_DIR}/logs/algorithm_test.log
touch ${WORKSPACE_DIR}/logs/plot.log

# RUN THE ROS2 NODES
# ros2 run algorithm_test path_following_test 2>&1 | tee ${WORKSPACE_DIR}/logs/algorithm_test.log &
# ros2 run pathfollowing node_MPPI_output 2>&1 | tee ${WORKSPACE_DIR}/logs/node_MPPI_output.log &
# ros2 run pathfollowing node_att_ctrl 2>&1 | tee ${WORKSPACE_DIR}/logs/node_att_ctrl.log &

# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

sleep infinity
