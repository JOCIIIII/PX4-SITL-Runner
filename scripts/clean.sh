#!/bin/bash

# SCRIPT TO CLEAN ASSETS FOR CONTAINER TESTING

# INITIAL STATEMENTS
# >>>----------------------------------------------------

# SET THE BASE DIRECTORY
BASE_DIR=$(dirname $(readlink -f "$0"))
REPO_DIR=$(dirname $(dirname $(readlink -f "$0")))

# SOURCE THE ENVIRONMENT AND FUNCTION DEFINITIONS
source ${BASE_DIR}/include/commonFcn.sh

# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


# DEFINE USAGE FUNCTION
# >>>----------------------------------------------------

usageState1(){
    EchoRed "Usage: $0 [all|airsim|px4|ros2|gazebo-classic|gazebo]"
    EchoRed "all: CLEAN ALL ASSETS"
    EchoRed "airsim: CLEAN AIRSIM ASSETS"
    EchoRed "px4: CLEAN PX4 ASSETS"
    EchoRed "ros2: CLEAN ROS2 ASSETS"
    EchoRed "gazebo-classic: CLEAN GAZEBO CLASSIC ASSETS"
    EchoRed "gazebo: CLEAN GAZEBO ASSETS"
    exit 1
}

# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


# CHECK IF ANY INPUT ARGUMENTS ARE PROVIDED
# >>>----------------------------------------------------

if [ $# -eq 0 ]; then
    usageState1 $0
else
    if [ "$1x" != "allx" ] && [ "$1x" != "airsimx" ] && [ "$1x" != "px4x" ] && [ "$1x" != "ros2x" ] && [ "$1x" != "gazebo-classicx" ] && [ "$1x" != "gazebox" ]; then
        EchoRed "[$(basename "$0")] INVALID INPUT. PLEASE USE ARGUMENT AMONG \"airsim\", \"px4\", \"ros2\", \"gazebo-classic\", \"gazebo\"."
        exit 1
    fi
fi

# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

# CHECK IF DIRECTORY .git EXISTS
if [ ! -d "${REPO_DIR}/.git" ]; then
    EchoRed "[$(basename "$0")] THIS IS NOT A GIT REPOSITORY."
    EchoRed "[$(basename "$0")] DID YOU DOWNLOAD THE REPOSITORY AS A ZIP FILE?"
    EchoRed "[$(basename "$0")] PLEASE CLONE THE REPOSITORY USING \"git clone\" COMMAND."
    exit 1
fi

# RUN PROCESS PER ARGUMENT
if [ "$1x" == "allx" ]; then
    EchoGreen "[$(basename "$0")] CLEAN ALL ASSETS"
    git -C ${REPO_DIR} clean --hard
    git -C ${REPO_DIR} clean -fdx
elif [ "$1x" == "airsimx" ]; then
    EchoGreen "[$(basename "$0")] CLEAN AIRSIM ASSETS"
    git -C ${REPO_DIR} checkout -- AirSim
    git -C ${REPO_DIR} clean -fdx AirSim
elif [ "$1x" == "px4x" ]; then
    EchoRed "[$(basename "$0")] NOT IMPLEMENTED YET"
elif [ "$1x" == "ros2x" ]; then
    EchoRed "[$(basename "$0")] NOT IMPLEMENTED YET"
elif [ "$1x" == "gazebo-classicx" ]; then
    EchoRed "[$(basename "$0")] NOT IMPLEMENTED YET"
fi