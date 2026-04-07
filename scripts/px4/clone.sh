#!/bin/bash

# INITIAL STATEMENTS
# >>>----------------------------------------------------

# SET THE BASIC ENVIRONMENT VARIABLE
export TERM=xterm-256color
PX4_VERSION=v1.16.0

# SET THE BASE DIRECTORY
BASE_DIR=$(dirname $(readlink -f "$0"))

# SOURCE THE ENVIRONMENT AND FUNCTION DEFINITIONS
for file in ${BASE_DIR}/include/*.sh; do
    source ${file}
done
# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


# MAIN STATEMENTS
# >>>----------------------------------------------------

# CHCEK IF DIRECTORY PX4-Autopilot EXISTS

CheckDirExists "PX4-Autopilot" git https://github.com/PX4/PX4-Autopilot.git ${PX4_VERSION}

CheckDirExists "A4VAI-Custom-PX4-File" git https://github.com/JOCIIIII/A4VAI-Custom-PX4-File.git

chmod -R o+rwx $(dirname "$BASE_DIR")
# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<