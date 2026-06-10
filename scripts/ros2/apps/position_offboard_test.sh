#! /bin/bash

# ROS2 APP: position_offboard_test (offboard control example).
# This only RUNS the node. Build it beforehand with:
#   ./scripts/run.sh ros2 build ros2_ws position_offboard_test
# Runs inside the ros2-env container, alongside the uXRCE-DDS agent
# (same container => localhost DDS, so /fmu/* topics are directly visible).

# WORKSPACE_DIR is exported by the launcher; fall back if run standalone.
if [ -z "${WORKSPACE_DIR}" ]; then
    WORKSPACE_DIR=$(dirname $(dirname $(dirname $(readlink -f "$0"))))
fi

ROS2_WS=${WORKSPACE_DIR}/ros2_ws

# THE PACKAGE MUST ALREADY BE BUILT (run flow does NOT build).
if [ ! -d "${ROS2_WS}/install/position_offboard_test" ]; then
    echo "[$(basename "$0")] ERROR: position_offboard_test is not built."
    echo "[$(basename "$0")] Build it first: ./scripts/run.sh ros2 build ros2_ws position_offboard_test"
    exit 1
fi

# SOURCE THE WORKSPACE AND RUN THE NODE
source ${ROS2_WS}/install/setup.bash
ros2 run position_offboard_test offboard_control
