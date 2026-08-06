#!/bin/bash

# UNINSTALLER FOR THE A4VAI SITL SIMULATOR.
# Removes: running containers, the sitl network, the deployed workspace
# (~/Documents/A4VAI-SITL), and (optionally) the docker images.
# The cloned PX4-SITL-Runner repository itself is NOT removed.

BASE_DIR=$(dirname $(readlink -f "$0"))
REPO_DIR=$(dirname ${BASE_DIR})

source ${BASE_DIR}/include/commonFcn.sh
source ${BASE_DIR}/include/commonEnv.sh

EchoYellow "[uninstall.sh] THIS WILL REMOVE:"
EchoYellow "*   all SITL containers (px4-env, gz-sim, airsim-binary, ros2-env, qgc-app)"
EchoYellow "*   the ${SITL_NETWORK_NAME} docker network"
EchoYellow "*   the deployed workspace ${SITL_DEPLOY_DIR}"
EchoYellow "    (PX4 build, ROS2 workspaces, AirSim binary - EVERYTHING under it)"
WarnAction

# 1) CONTAINERS + NETWORK
EchoGreen "[uninstall.sh] STOPPING CONTAINERS"
docker rm -f px4-env gz-sim airsim-binary ros2-env qgc-app >/dev/null 2>&1
docker network rm ${SITL_NETWORK_NAME} >/dev/null 2>&1
EchoGreen "[uninstall.sh] CONTAINERS AND NETWORK REMOVED."

# 2) DEPLOYED WORKSPACE
if [ -d "${SITL_DEPLOY_DIR}" ]; then
    EchoGreen "[uninstall.sh] REMOVING ${SITL_DEPLOY_DIR}"
    # some build artifacts are root-owned (created inside containers)
    rm -rf "${SITL_DEPLOY_DIR}" 2>/dev/null || sudo rm -rf "${SITL_DEPLOY_DIR}"
    EchoGreen "[uninstall.sh] WORKSPACE REMOVED."
else
    EchoYellow "[uninstall.sh] ${SITL_DEPLOY_DIR} DOES NOT EXIST - SKIPPING."
fi

# 3) DOCKER IMAGES (OPTIONAL)
EchoYellow "[uninstall.sh] ALSO REMOVE THE DOCKER IMAGES (~20GB)?"
read -n1 -r -p $'\e[33mREMOVE IMAGES? [y/n]:\e[0m ' RM_IMG; echo
if [ "${RM_IMG}x" == "yx" ] || [ "${RM_IMG}x" == "Yx" ]; then
    for var in ROS2_ENV_IMAGE PX4_ENV_IMAGE GAZEBO_CLASSIC_ENV_IMAGE AIRSIM_BINARY_IMAGE QGC_ENV_IMAGE; do
        img=$(grep -h "^${var}=" ${REPO_DIR}/envs/*.env | cut -d= -f2)
        [ -n "${img}" ] && docker rmi "${img}" 2>/dev/null
    done
    docker rmi jociiiii/a4vai:devel 2>/dev/null
    EchoGreen "[uninstall.sh] IMAGES REMOVED."
else
    EchoYellow "[uninstall.sh] IMAGES KEPT."
fi

EchoGreen "[uninstall.sh] UNINSTALL COMPLETE."
EchoGreen "[uninstall.sh] (to remove this repository too: rm -rf ${REPO_DIR})"
