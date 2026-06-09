#!/bin/bash

# INITIAL STATEMENTS
# >>>----------------------------------------------------

# SET THE BASIC ENVIRONMENT VARIABLE
export TERM=xterm-256color

# SET THE BASE DIRECTORY
BASE_DIR=$(dirname $(readlink -f "$0"))

# SOURCE THE ENVIRONMENT AND FUNCTION DEFINITIONS
for file in ${BASE_DIR}/include/*.sh; do
    source ${file}
done
# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

# SET ENVIRONMENT VARIABLES
PX4_WORKSPACE=/home/user/workspace/px4
GAZEBO_WORKSPACE=/home/user/workspace/gazebo

PX4_SOURCE_DIR=${PX4_WORKSPACE}/PX4-Autopilot
PX4_SIM_DIR=${PX4_WORKSPACE}/PX4-Autopilot/Tools/simulation
PX4_BUILD_DIR=${PX4_WORKSPACE}/PX4-Autopilot/build/px4_sitl_default
PX4_BINARY_DIR=${PX4_WORKSPACE}/PX4-Autopilot/build/px4_sitl_default/bin

# APPLY WIND PARAMETERS FROM envs/gazebo-classic.env INTO THE WORLD FILE.
# Uses sed (built-in) instead of envsubst, so no apt install is needed at container start.
WINDY_WORLD=${PX4_SIM_DIR}/gazebo-classic/sitl_gazebo-classic/worlds/windy.world
for windVar in windVelocityMean windVelocityMax windVelocityVariance \
               windDirectionMean windDirectionVariance \
               windGustStart windGustDuration \
               windGustVelocityMean windGustVelocityMax windGustVelocityVariance \
               windGustDirectionMean windGustDirectionVariance; do
    sed -i "s|<${windVar}>[^<]*</${windVar}>|<${windVar}>${!windVar}</${windVar}>|" "${WINDY_WORLD}"
done

source /usr/share/gazebo/setup.bash
export GAZEBO_RESOURCE_PATH=${GAZEBO_RESOURCE_PATH}:${GAZEBO_WORKSPACE}/worlds

# CHECK IF THE DIRECTORIES EXIST
CheckDirExists ${PX4_SOURCE_DIR}
CheckDirExists ${PX4_SIM_DIR}
CheckDirExists ${PX4_BUILD_DIR}
CheckDirExists ${PX4_BINARY_DIR}

# CHECK IF STRING "MODIFIED NOT TO RUN GAZEBO ON SITL" EXISTS ON ${PX4_SIM_DIR}/gazebo-classic/sitl_run.sh
while ! grep -q "MODIFIED NOT TO RUN GAZEBO ON SITL" ${PX4_SIM_DIR}/gazebo-classic/sitl_run.sh; do
    EchoYellow "[$(basename $0)] MODIFIED NOT TO RUN GAZEBO ON SITL NOT FOUND."
    EchoYellow "[$(basename $0)] WAITING FOR THE SCRIPT TO BE MODIFIED."
    sleep 0.5s
done

# COPY GAZEBO RESOURCES IF THEY EXIST
if [ -d ${GAZEBO_WORKSPACE}/media ]; then
    if [ -z "$(ls -A ${GAZEBO_WORKSPACE}/media)" ]; then
        EchoYellow "[$(basename $0)] ${GAZEBO_WORKSPACE}/media IS EMPTY."
    else
        cp -r ${GAZEBO_WORKSPACE}/media/* /usr/share/gazebo-11/media
    fi
fi

# COPY GAZEBO WORLDS IF THEY EXIST
if [ -d ${GAZEBO_WORKSPACE}/worlds ]; then
    if [ ! -f "$(ls -A ${GAZEBO_WORKSPACE}/worlds/*.world)" ]; then
        EchoYellow "[$(basename $0)] ${GAZEBO_WORKSPACE}/worlds DOES NOT CONTAIN ANY .world FILES."
    else
        cp -r ${GAZEBO_WORKSPACE}/worlds/*.world ${PX4_WORKSPACE}/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds
    fi
fi

# RUN ADDITIONAL SETUP SCRIPT IF IT EXISTS
if [ -f ${GAZEBO_WORKSPACE}/worlds/${SITL_WORLD}.sh ]; then
    EchoYellow "[$(basename $0)] ADDITIONAL SETUP SCRIPT ${SITL_WORLD}.sh FOUND."
    EcgYellow "[$(basename $0)] RUNNING ${SITL_WORLD}.sh."
    ${GAZEBO_WORKSPACE}/worlds/${SITL_WORLD}.sh
fi

# CREATE DIRECTORY FOR LOGS
CheckDirExists ${GAZEBO_WORKSPACE}/logs create
rm -rf ${GAZEBO_WORKSPACE}/logs/*

# CREATE LOG FILES
touch ${GAZEBO_WORKSPACE}/logs/gazebo.log

# RUN GAZEBO SIDE OF THE PX4-SITL
(HEADLESS=$1 ${PX4_SIM_DIR}/gazebo-classic/sitl_run.sh \
    ${PX4_BINARY_DIR}/px4 \
    none \
    ${SITL_AIRFRAME} \
    ${SITL_WORLD} \
    ${PX4_SOURCE_DIR} \
    ${PX4_BUILD_DIR}) 2>&1 | tee ${GAZEBO_WORKSPACE}/logs/gazebo.log &

sleep infinity

# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<