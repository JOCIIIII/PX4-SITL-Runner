#!/bin/bash

# SCRIPT TO RESET ASSETS FOR CONTAINER TESTING

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
    EchoRed "all: RESET ALL ASSETS"
    EchoRed "airsim: RESET AIRSIM ASSETS"
    EchoRed "px4: RESET PX4 ASSETS"
    EchoRed "ros2: RESET ROS2 ASSETS"
    EchoRed "gazebo-classic: RESET GAZEBO CLASSIC ASSETS"
    EchoRed "gazebo: RESET GAZEBO ASSETS"
    EchoRed "scripts: RESET SCRIPTS"
    exit 1
}

# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


# CHECK IF ANY INPUT ARGUMENTS ARE PROVIDED
# >>>----------------------------------------------------

if [ $# -eq 0 ]; then
    usageState1 $0
else
    if [ "$1x" != "allx" ] && \
       [ "$1x" != "airsimx" ] && \
       [ "$1x" != "px4x" ] && \
       [ "$1x" != "ros2x" ] && \
       [ "$1x" != "gazebo-classicx" ] && \
       [ "$1x" != "scriptsx" ]; then
        EchoRed "[$(basename "$0")] INVALID INPUT. PLEASE USE ARGUMENT AMONG \"airsim\", \"px4\", \"ros2\", \"gazebo-classic\" and \"scripts\"."
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
    EchoGreen "[$(basename "$0")] RESET ALL SOURCE FILES"
    git -C ${REPO_DIR} reset --hard
elif [ "$1x" == "airsimx" ]; then
    EchoGreen "[$(basename "$0")] RESET AIRSIM SOURCE FILES"
    git -C ${REPO_DIR} checkout -- AirSim
elif [ "$1x" == "px4x" ]; then
    EchoGreen "[$(basename "$0")] RESET PX4-AUTOPILOT SOURCE FILES"
    git -C ${REPO_DIR} checkout -- PX4-Autopilot
elif [ "$1x" == "ros2x" ]; then
    EchoRed "[$(basename "$0")] NOT IMPLEMENTED YET"
    exit 1
elif [ "$1x" == "gazebo-classicx" ]; then
    EchoGreen "[$(basename "$0")] RESET GAZEBO-CLASSIC SOURCE FILES"
    git -C ${REPO_DIR} checkout -- Gazebo-Classic
elif [ "$1x" == "scriptsx" ]; then
    EchoGreen "[$(basename "$0")] RESET SCRIPTS"
    git -C ${REPO_DIR} checkout -- scripts
fi