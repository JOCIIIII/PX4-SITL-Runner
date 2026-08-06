#!/bin/bash
# RUN THE FULL SUPER + PATHFOLLOWING PIPELINE (collision-avoidance flight).
# Goal comes from ros2_ws/src/realgazebo_fastplanner_adapters/config/goals_super_far.csv
BASE_DIR=$(dirname $(readlink -f "$0"))
REPO_DIR=$(dirname ${BASE_DIR})
sed -i 's/^ROS2_APP=.*/ROS2_APP="super_pf"/' ${REPO_DIR}/envs/ros2.env
exec ${BASE_DIR}/run.sh gazebo-classic-airsim-sitl run
