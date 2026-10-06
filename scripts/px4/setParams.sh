#! /bin/bash

# SITL-only PX4 parameters for the ROS2 offboard flow (flight_manager / path_following_test).
# flight_manager sets these itself through a px4-param binary that does not exist in the
# ROS2 container, so they never took effect. Without them an intermittent manual-control
# signal (QGC) counts as RC loss in OFFBOARD -> Hold -> RTL failsafe mid-flight
# (failsafe_flags: manual_control_signal_lost). Inserted into the SITL rcS before
# 'commander start' (idempotent).

# INITIALIZE ENVIRONMENT VARIABLES
export TERM=xterm-256color

BASE_DIR=$(dirname $(readlink -f "$0"))
for file in ${BASE_DIR}/include/*.sh; do
    source ${file}
done

PX4_BUILD_DIR="PX4-Autopilot/build/px4_sitl_default"
PX4_INIT_SCRIPT="${PX4_BUILD_DIR}/etc/init.d-posix/rcS"
MARK="# A4VAI SITL offboard params"

CheckDirExists "PX4-Autopilot"
CheckDirExists "${PX4_BUILD_DIR}/etc/init.d-posix"
CheckFileExists "${PX4_INIT_SCRIPT}"

if grep -q "${MARK}" ${PX4_INIT_SCRIPT}; then
    EchoGreen "[$(basename $0)] SITL OFFBOARD PARAMS ALREADY IN rcS."
    exit 0
fi

EchoGreen "[$(basename $0)] ADDING SITL OFFBOARD PARAMS TO rcS."
LINE_NUMBER=$(grep -n "^commander start" ${PX4_INIT_SCRIPT} | head -1 | cut -d: -f1)
if [ -z "${LINE_NUMBER}" ]; then
    EchoRed "[$(basename $0)] 'commander start' NOT FOUND IN ${PX4_INIT_SCRIPT}."
    exit 1
fi
# NAV_DLL_ACT 0: no datalink-loss action / COM_RCL_EXCEPT 31: ignore RC loss in all modes
# incl. offboard / COM_RC_IN_MODE 4: manual control disabled (fully automated SITL)
sed -i "${LINE_NUMBER}i ${MARK}\nparam set NAV_DLL_ACT 0\nparam set COM_RCL_EXCEPT 31\nparam set COM_RC_IN_MODE 4" ${PX4_INIT_SCRIPT}
