#!/bin/bash

# SET THE BASE DIRECTORY SO THIS SCRIPT WORKS FROM ANY CWD
BASE_DIR=$(dirname $(readlink -f "$0"))

${BASE_DIR}/run.sh px4 clone
${BASE_DIR}/run.sh px4 build
${BASE_DIR}/run.sh px4 stop
mkdir -p ${HOME}/Documents/A4VAI-SITL/ROS2/ros2_ws/src
git clone https://github.com/JOCIIIII/A4VAI-Algorithms-ROS2.git ${HOME}/Documents/A4VAI-SITL/ROS2/ros2_ws/src
git -C ${HOME}/Documents/A4VAI-SITL/ROS2/ros2_ws/src submodule update --init --recursive
git clone https://github.com/JOCIIIII/A4VAI-ROS2-Util-Package.git ${HOME}/Documents/A4VAI-SITL/ROS2/ros2_ws/src/A4VAI-ROS2-Util-Package
cp -r ${HOME}/Documents/A4VAI-SITL/ROS2/ros2_ws/src/A4VAI-ROS2-Util-Package/plotter ${HOME}/Documents/A4VAI-SITL/ROS2/ros2_ws/src/plotter
rm -rf ${HOME}/Documents/A4VAI-SITL/ROS2/ros2_ws/src/A4VAI-ROS2-Util-Package
mkdir -p ${HOME}/Documents/A4VAI-SITL/ROS2/ros2_ws/src/pathplanning/pathplanning/model

chmod -R o+wrx ${HOME}/Documents/A4VAI-SITL/ROS2

${BASE_DIR}/run.sh ros2 build ros2_ws
${BASE_DIR}/run.sh ros2 stop
wget https://github.com/kestr31/PX4-SITL-Runner/releases/download/Resources/airsim.tar.gz -O ${HOME}/Documents/A4VAI-SITL/ROS2/airsim.tar.gz
# px4_ros prebuilt tarball removed: MicroXRCEAgent is a system binary and px4_msgs is
# built by `ros2 build ros2_ws` above, so the px4_ros workspace is not needed.
tar -zxvf ${HOME}/Documents/A4VAI-SITL/ROS2/airsim.tar.gz -C ${HOME}/Documents/A4VAI-SITL/ROS2
wget https://github.com/kestr31/PX4-SITL-Runner/releases/download/Resources/GazeboDrone -O ${HOME}/Documents/A4VAI-SITL/Gazebo-Classic/GazeboDrone
chmod +x ${HOME}/Documents/A4VAI-SITL/Gazebo-Classic/GazeboDrone
wget https://github.com/kestr31/PX4-SITL-Runner/releases/download/Resources/settings.json -O ${HOME}/Documents/A4VAI-SITL/AirSim/settings.json
