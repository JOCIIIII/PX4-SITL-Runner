#!/bin/bash

./scripts/run.sh px4 clone 
./scripts/run.sh px4 build 
./scripts/run.sh px4 stop 
mkdir -p ~/Documents/A4VAI-SITL/ROS2/ros2_ws/src 
git clone https://github.com/JOCIIIII/A4VAI-Algorithms-ROS2.git ~/Documents/A4VAI-SITL/ROS2/ros2_ws/src 
git -C ~/Documents/A4VAI-SITL/ROS2/ros2_ws/src submodule update --init --recursive 
git clone https://github.com/JOCIIIII/A4VAI-ROS2-Util-Package.git ~/Documents/A4VAI-SITL/ROS2/ros2_ws/src/A4VAI-ROS2-Util-Package 
cp -r ~/Documents/A4VAI-SITL/ROS2/ros2_ws/src/A4VAI-ROS2-Util-Package/plotter ~/Documents/A4VAI-SITL/ROS2/ros2_ws/src/plotter 
rm -rf ~/Documents/A4VAI-SITL/ROS2/ros2_ws/src/A4VAI-ROS2-Util-Package 
mkdir -p ~/Documents/A4VAI-SITL/ROS2/ros2_ws/src/pathplanning/pathplanning/model 
wget https://github.com/kestr31/PX4-SITL-Runner/releases/download/Resources/weight.onnx -O ~/Documents/A4VAI-SITL/ROS2/ros2_ws/src/pathplanning/pathplanning/model/weight.onnx 
chmod -R o+wrx ~/Documents/A4VAI-SITL/ROS2 
./scripts/run.sh ros2 make-tensorRT-engine.sh 
./scripts/run.sh ros2 build ros2_ws 
./scripts/run.sh ros2 stop 
wget https://github.com/kestr31/PX4-SITL-Runner/releases/download/Resources/airsim.tar.gz -O ~/Documents/A4VAI-SITL/ROS2/airsim.tar.gz 
wget https://github.com/JOCIIIII/PX4-SITL-Runner/releases/tag/resources/px4_ros-v1.16.0.tar.gz -O ~/Documents/A4VAI-SITL/ROS2/px4_ros.tar.gz 
tar -zxvf ~/Documents/A4VAI-SITL/ROS2/airsim.tar.gz -C ~/Documents/A4VAI-SITL/ROS2 
tar -zxvf ~/Documents/A4VAI-SITL/ROS2/px4_ros.tar.gz -C ~/Documents/A4VAI-SITL/ROS2 
git clone https://github.com/dheera/rosboard.git -b v1.3.1 ~/Documents/A4VAI-SITL/ROS2/rosboard 
wget https://github.com/kestr31/PX4-SITL-Runner/releases/download/Resources/GazeboDrone -O ~/Documents/A4VAI-SITL/Gazebo-Classic/GazeboDrone 
chmod +x ~/Documents/A4VAI-SITL/Gazebo-Classic/GazeboDrone 
wget https://github.com/kestr31/PX4-SITL-Runner/releases/download/Resources/settings.json -O ~/Documents/A4VAI-SITL/AirSim/settings.json


