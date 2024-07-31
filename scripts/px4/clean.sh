#!/bin/bash

# INITIAL STATEMENTS
# >>>----------------------------------------------------

# SET THE BASIC ENVIRONMENT VARIABLE
export TERM=xterm-256color

# SET THE BASE DIRECTORY
BASE_DIR=$(dirname $(readlink -f "$0"))

# SOURCE THE ENVIRONMENT AND FUNCTION DEFINITIONS
source ${BASE_DIR}/include/commonFcn.sh
source ${BASE_DIR}/include/commonEnv.sh

# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


# MAIN STATEMENTS
# >>>----------------------------------------------------

# CHCEK IF DIRECTORY PX4-Autopilot AND build EXISTS
CheckDirExists "PX4-Autopilot"
CheckDirExists "PX4-Autopilot/build"

# CLEAN PX4-Autopilot
(cd PX4-Autopilot || exit 1; make distclean)

EchoGreen "[$(basename $0)] PX4-Autopilot cleaned successfully."
EchoBoxLine

# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<