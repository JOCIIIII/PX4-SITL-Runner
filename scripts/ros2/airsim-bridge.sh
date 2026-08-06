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

CheckDirExists ${WORKSPACE_DIR}/airsim
CheckDirEmpty ${WORKSPACE_DIR}/airsim

CheckDirExists ${WORKSPACE_DIR}/airsim/install
CheckFileExists ${WORKSPACE_DIR}/airsim/install/setup.bash

source /opt/ros/${ROS_DISTRO}/setup.bash
source ${WORKSPACE_DIR}/airsim/install/setup.bash

EchoYellow "[$(basename $0)] STARRTING AIRSIM ROS2 BRIDGE."

while ! grep -q "SimpleFlight" /home/user/workspace/airsim/log; do
    EchoYellow "[$(basename $0)] WAITING FOR THE AIRSIM TO START."
    sleep 0.5
done

sleep 2
# launch 파일 대신 직접 실행: world_frame_id 를 airsim_world 로 바꿔
# 어댑터의 world(ENU, PX4 원점) 프레임과의 이름 충돌을 제거한다.
# (world -> airsim_world 정적 TF 는 sitl-px4-airsim.sh 가 발행)
ros2 run airsim_ros_pkgs airsim_node --ros-args -r __node:=airsim_node \
    -p is_vulkan:=true \
    -p update_airsim_img_response_every_n_sec:=0.05 \
    -p update_airsim_control_every_n_sec:=0.01 \
    -p update_lidar_every_n_sec:=0.01 \
    -p update_gpulidar_every_n_sec:=0.01 \
    -p update_echo_every_n_sec:=0.01 \
    -p publish_clock:=false \
    -p host_ip:=${AIRSIM_IP} \
    -p host_port:=41451 \
    -p enable_api_control:=false \
    -p enable_object_transforms_list:=true \
    -p world_frame_id:=airsim_world
# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<