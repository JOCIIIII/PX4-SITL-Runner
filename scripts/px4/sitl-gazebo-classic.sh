#!/bin/bash

# INITIAL STATEMENTS
# >>>----------------------------------------------------

# SET THE BASIC ENVIRONMENT VARIABLE
export TERM=xterm-256color
export NO_PXH=1
export PX4_SYS_AUTOSTART=2000

# SET THE BASE DIRECTORY
BASE_DIR=$(dirname $(readlink -f "$0"))

# SOURCE THE ENVIRONMENT AND FUNCTION DEFINITIONS
source ${BASE_DIR}/include/commonFcn.sh
source ${BASE_DIR}/include/commonEnv.sh

# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


# MAIN STATEMENTS
# >>>----------------------------------------------------

CheckDirExists "PX4-Autopilot"
CheckDirExists "PX4-Autopilot/Tools"

CheckFileExists "PX4-Autopilot/Tools/convert_ip.py"
CheckFileExists ${BASE_DIR}/separateFromGazebo.sh

${BASE_DIR}/separateFromGazebo.sh

${BASE_DIR}/setGCS.sh
${BASE_DIR}/setDDS.sh

PX4_SIM_HOST_ADDR=${GAZEBO_CLASSIC_IP} \
    PX4-Autopilot/build/px4_sitl_default/bin/px4 -d

sleep infinity

# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<