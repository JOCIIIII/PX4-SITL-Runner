#!/bin/bash
# RUN THE PATHFOLLOWING-ONLY TEST (no planner - fixed waypoints from
# ros2_ws/src/path_following_test/wp.csv; edit it and rerun, no rebuild needed).
BASE_DIR=$(dirname $(readlink -f "$0"))
REPO_DIR=$(dirname ${BASE_DIR})
sed -i 's/^ROS2_APP=.*/ROS2_APP="pf_test"/' ${REPO_DIR}/envs/ros2.env
exec ${BASE_DIR}/run.sh gazebo-classic-airsim-sitl run
