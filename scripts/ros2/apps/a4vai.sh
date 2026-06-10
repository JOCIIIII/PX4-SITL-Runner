#! /bin/bash

# ROS2 APP: A4VAI integrated stack (path planning + path following + collision
# avoidance + visualization). Intended for the gazebo-classic-airsim-sitl profile
# (uses AirSim sensors via the airsim bridge).
# NOTE: requires the A4VAI packages to be built first: ./scripts/run.sh ros2 build

# WORKSPACE_DIR / BASE_DIR are exported by the launcher; fall back if run standalone.
if [ -z "${WORKSPACE_DIR}" ]; then
    WORKSPACE_DIR=$(dirname $(dirname $(dirname $(readlink -f "$0"))))
fi

mkdir -p ${WORKSPACE_DIR}/ros2_ws/build/controller/controller/log
pip install transforms3d

ros2 run pathplanning Plan2WP 2>&1 | tee ${WORKSPACE_DIR}/logs/Plan2WP.log &

ros2 run pathfollowing node_MPPI_output 2>&1 | tee ${WORKSPACE_DIR}/logs/node_MPPI_output.log &
ros2 run pathfollowing node_att_ctrl 2>&1 | tee ${WORKSPACE_DIR}/logs/node_att_ctrl.log &

ros2 run rs_converter rs_converter 2>&1 | tee ${WORKSPACE_DIR}/logs/rs_converter.log &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 SimpleFlight/RPLIDAR_A3 base_link 2>&1 | tee ${WORKSPACE_DIR}/logs/tf2_ros.log &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link velodyne 2>&1 | tee ${WORKSPACE_DIR}/logs/tf2_ros.log &

ros2 run controller controller 2>&1 | tee ${WORKSPACE_DIR}/logs/controller.log &

ros2 run rs_converter point_cloud_filter_node 2>&1 | tee ${WORKSPACE_DIR}/logs/point_cloud_filter_node.log &
ros2 run rs_converter point_cloud_feature_extractor_cov 2>&1 | tee ${WORKSPACE_DIR}/logs/point_cloud_feature_extractor_cov.log &

python3 /home/user/workspace/ros2/ros2_ws/src/Python_version_easy/direct_infer.py --ros-args   -p model_path:=/home/user/workspace/ros2/ros2_ws/src/Python_version_easy/walid.onnx   -p vec_normalize_path:=/home/user/workspace/ros2/ros2_ws/src/Python_version_easy/vec_normalize.pkl   -p pointcloud_topic:=/pointcloud_features   -p cmd_topic:=/ca_vel_2_control  -p expected_points:=256 2>&1 | tee ${WORKSPACE_DIR}/logs/direct_infer.log &

ros2 run foxglove foxglove 2>&1 | tee ${WORKSPACE_DIR}/logs/foxglove.log &
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765 2>&1 | tee ${WORKSPACE_DIR}/logs/foxglove_bridge.log &

# RUN ROSBOARD FOR ROS2 TOPIC VISUALIZATION
${WORKSPACE_DIR}/rosboard/run
