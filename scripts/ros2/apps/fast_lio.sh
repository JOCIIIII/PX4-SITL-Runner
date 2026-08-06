#! /bin/bash

# ROS2 APP: FAST-LIO2 odometry/mapping fed by the AirSim lidar + AirSim IMU.
# Use with the gazebo-classic-airsim-sitl profile (needs the airsim bridge topics).
# This only RUNS the node. Build it beforehand with:
#   ./scripts/run.sh ros2 build ros2_ws fast_lio
# Outputs: /Odometry (LIO odom), /cloud_registered (world-frame cloud, rog_map input).

# WORKSPACE_DIR is exported by the launcher; fall back if run standalone.
if [ -z "${WORKSPACE_DIR}" ]; then
    WORKSPACE_DIR=$(dirname $(dirname $(dirname $(readlink -f "$0"))))
fi

ROS2_WS=${WORKSPACE_DIR}/ros2_ws

# THE PACKAGE MUST ALREADY BE BUILT (run flow does NOT build).
if [ ! -d "${ROS2_WS}/install/fast_lio" ]; then
    echo "[$(basename "$0")] ERROR: fast_lio is not built."
    echo "[$(basename "$0")] Build it first: ./scripts/run.sh ros2 build ros2_ws fast_lio"
    exit 1
fi

# SOURCE THE WORKSPACE AND RUN WITH THE AIRSIM CONFIG.
# NOTE: no use_sim_time - nothing publishes /clock in this SITL, so sim time would
# freeze the node clock and FAST-LIO would never publish. Sensor sync uses message
# stamps, which the airsim bridge provides.
source ${ROS2_WS}/install/setup.bash
ros2 run fast_lio fastlio_mapping --ros-args \
    --params-file ${ROS2_WS}/src/FAST_LIO/config/airsim.yaml
